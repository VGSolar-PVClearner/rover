#include "AP_SuctionCup.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Relay/AP_Relay.h>

/*
 * AP_SuctionCup 实现：升降 PWM 缓速 + 气阀/气泵 Relay 异步状态机。
 * 不解析 NCU 协议；时序参数通过 SCUP_* 地面站参数可调。
 * 单例由 Rover ParametersG2::suction_cup 构造。
 */

extern const AP_HAL::HAL &hal;

AP_SuctionCup *AP_SuctionCup::_singleton;

const AP_Param::GroupInfo AP_SuctionCup::var_info[] = {
    // @Param: LIFT_DLY_MS
    // @DisplayName: Lift settle time after ramp
    // @Description: Extra wait after lift PWM reaches target before next suction step
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
    AP_GROUPINFO("LIFT_DLY_MS", 1, AP_SuctionCup, _lift_delay_ms, 2000),

    // @Param: VAC_DLY_MS
    // @DisplayName: Vacuum build time
    // @Description: Time to wait after starting pump before suction is considered ready
    // @Units: ms
    // @Range: 100 10000
    // @User: Standard
    AP_GROUPINFO("VAC_DLY_MS", 2, AP_SuctionCup, _vacuum_delay_ms, 3000),

    // @Param: VENT_DLY_MS
    // @DisplayName: Vent delay before lift
    // @Description: Time to wait after opening valve before raising lift servo
    // @Units: ms
    // @Range: 100 5000
    // @User: Standard
    AP_GROUPINFO("VENT_DLY_MS", 3, AP_SuctionCup, _vent_delay_ms, 2000),

    // @Param: ACT_TOUT_MS
    // @DisplayName: Suction action timeout
    // @Description: Maximum time allowed for a lower or raise sequence before fault
    // @Units: ms
    // @Range: 1000 60000
    // @User: Standard
    AP_GROUPINFO("ACT_TOUT_MS", 4, AP_SuctionCup, _action_timeout_ms, 30000),

    // @Param: LIFT_PWM_R
    // @DisplayName: Lift PWM raised
    // @Description: PWM pulse width in microseconds for raised lift position
    // @Units: us
    // @Range: 1000 2000
    // @User: Advanced
    AP_GROUPINFO("LIFT_PWM_R", 5, AP_SuctionCup, _lift_pwm_raised, 1900),

    // @Param: LIFT_PWM_L
    // @DisplayName: Lift PWM lowered
    // @Description: PWM pulse width in microseconds for lowered lift position
    // @Units: us
    // @Range: 1000 2000
    // @User: Advanced
    AP_GROUPINFO("LIFT_PWM_L", 6, AP_SuctionCup, _lift_pwm_lowered, 1100),

    // @Param: LIFT_RATE
    // @DisplayName: Lift PWM slew rate
    // @Description: Rate of lift PWM change in microseconds per second. With default raised/lowered (1900/1100), 400 us/s is about 2 seconds.
    // @Units: us/s
    // @Range: 50 5000
    // @User: Standard
    AP_GROUPINFO("LIFT_RATE", 7, AP_SuctionCup, _lift_rate_us, 400),

    // @Param: VLV_RLY
    // @DisplayName: Air valve relay instance
    // @Description: Relay instance for air valve. 0=RELAY1. Relay ON=seal, OFF=vent. Set RELAYx_PIN/FUNCTION/INVERTED on the board.
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("VLV_RLY", 9, AP_SuctionCup, _valve_relay, 0),

    // @Param: PUMP_RLY
    // @DisplayName: Air pump relay instance
    // @Description: Relay instance for vacuum pump. 0=RELAY1. Relay ON=pump on, OFF=pump off.
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("PUMP_RLY", 10, AP_SuctionCup, _pump_relay, 1),

    AP_GROUPEND
};

AP_SuctionCup::AP_SuctionCup()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_SuctionCup must be singleton");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    _active = false;
    _frozen = false;
    _frozen_vacuum_ready = false;
    _state = State::RAISED;
    _phase = Phase::NONE;
    _phase_start_ms = 0;
    _sequence_start_ms = 0;

    _lift_target_pwm = 1900;
    _lift_slew = false;
    _lift_ramp_last_ms = 0;
    _lift_reached_ms = 0;

    _last_lift_pwm = 1900;
    _last_valve_on = false;
    _last_pump_on = false;
    _last_logged_state = State::INACTIVE;
    _last_logged_phase = Phase::NONE;
    _last_log_ms = 0;
}

void AP_SuctionCup::set_active(bool active)
{
    if (_active == active) {
        return;
    }

    _active = active;

    if (!_active) {
        // 离开 VGSL：回到安全空闲，不保留吸附
        _frozen = false;
        _phase = Phase::NONE;
        _state = State::RAISED;
        apply_safe_idle();
        return;
    }

    if (_state == State::FAULT) {
        return;
    }

    _frozen = false;
    if (_state == State::INACTIVE || _state == State::RAISED) {
        _state = State::RAISED;
        apply_raised_idle();
    }
}

void AP_SuctionCup::update()
{
    if (!_active) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    if (_state == State::FAULT) {
        return;
    }

    // 每周期推进升降缓速（目标由 request_lift 设定）
    service_lift(now);

    // 未解锁：禁止保压/吸附，推进释放到安全位
    if (!hal.util->get_soft_armed()) {
        _frozen = false;
        if (_state == State::RAISING) {
            update_raising(now);
        } else if (_state != State::RAISED && _state != State::INACTIVE) {
            emergency_release();
            if (_state == State::RAISING) {
                update_raising(now);
            }
        } else {
            apply_safe_idle();
        }
        log_status();
        return;
    }

    // 冻结态：不再推进状态机，仅维持关泵+密封（或 LOWERING 中途的部分阀位）
    if (_frozen) {
        apply_frozen_hold();
        log_status();
        return;
    }

    check_action_timeout(now);

    if (_state == State::FAULT) {
        return;
    }

    switch (_state) {
    case State::LOWERING:
        update_lowering(now);
        break;
    case State::RAISING:
        update_raising(now);
        break;
    case State::LOWERED:
        apply_lowered_hold();  // 转向阶段持续维持吸附（关泵保密封）
        break;
    default:
        break;
    }

    log_status();
}

// 对外动作 API（由 ModeVGSolar 调用）
bool AP_SuctionCup::lower()
{
    if (!_active || _frozen || !can_start_sequence()) {
        return false;
    }

    if (!hal.util->get_soft_armed()) {
        return false;
    }

    begin_lower();
    return true;
}

bool AP_SuctionCup::raise()
{
    if (!_active || _frozen || !can_start_sequence()) {
        return false;
    }

    if (_state != State::LOWERED && _state != State::FROZEN) {
        return false;
    }

    begin_raise();
    return true;
}

void AP_SuctionCup::freeze()
{
    if (!_active) {
        return;
    }

    _frozen = true;
    const State prev = _state;
    _phase = Phase::NONE;

    switch (prev) {
    case State::LOWERING:
        // 吸附未完成：记为未建立负压，unfreeze 后回 RAISED
        _frozen_vacuum_ready = false;
        _state = State::FROZEN;
        apply_frozen_hold();
        break;

    case State::LOWERED:
        _frozen_vacuum_ready = true;
        _state = State::FROZEN;
        apply_frozen_hold();
        break;

    case State::FROZEN:
        apply_frozen_hold();
        break;

    case State::RAISING:
        // 抬起中断：仍视为已吸附，保持密封防掉落
        _frozen_vacuum_ready = true;
        _state = State::FROZEN;
        apply_frozen_hold();
        break;

    default:
        break;
    }

    log_status(true);
}

void AP_SuctionCup::emergency_release()
{
    if (!_active) {
        apply_safe_idle();
        return;
    }

    _frozen = false;

    if (_state == State::RAISED || _state == State::INACTIVE) {
        apply_raised_idle();
        return;
    }

    // 已在释放中则不重启序列，避免阀/泵反复切换
    if (_state == State::RAISING) {
        return;
    }

    begin_raise();
}

void AP_SuctionCup::clear_fault()
{
    if (_state != State::FAULT) {
        return;
    }

    _frozen = false;
    _frozen_vacuum_ready = false;
    _phase = Phase::NONE;
    _state = State::RAISED;

    if (_active) {
        apply_raised_idle();
    } else {
        apply_safe_idle();
    }

    log_status(true);
}

void AP_SuctionCup::unfreeze()
{
    _frozen = false;

    if (_state != State::FROZEN) {
        return;
    }

    _phase = Phase::NONE;
    if (_frozen_vacuum_ready) {
        // 冻结前已吸附：恢复 LOWERED，可继续转弯或 raise
        _state = State::LOWERED;
        apply_lowered_hold();
    } else {
        // LOWERING 中途冻结：未完成负压，回到抬起空闲
        _state = State::RAISED;
        apply_raised_idle();
    }
}

// 状态查询
bool AP_SuctionCup::is_busy() const
{
    return _state == State::LOWERING || _state == State::RAISING;
}

bool AP_SuctionCup::is_lowered() const
{
    return _state == State::LOWERED || _state == State::FROZEN;
}

bool AP_SuctionCup::is_raised() const
{
    return _state == State::RAISED || _state == State::INACTIVE;
}

bool AP_SuctionCup::can_start_sequence() const
{
    if (_state == State::FAULT) {
        return false;
    }
    return !is_busy();
}

// 序列启动与故障
void AP_SuctionCup::begin_lower()
{
    const uint32_t now = AP_HAL::millis();

    _state = State::LOWERING;
    _phase = Phase::LOWER_WAIT_LIFT;
    _phase_start_ms = now;
    _sequence_start_ms = now;
    _lift_reached_ms = 0;

    // 吸附第一步：先放气、停泵，再缓速放下升降（避免带压硬顶光伏板）
    write_valve(false);
    write_pump(false);
    request_lift(uint16_t(_lift_pwm_lowered.get()), true);

    log_status(true);
}

void AP_SuctionCup::begin_raise()
{
    const uint32_t now = AP_HAL::millis();

    _state = State::RAISING;
    _phase = Phase::RAISE_STOP_PUMP;
    _phase_start_ms = now;
    _sequence_start_ms = now;
    _lift_reached_ms = 0;

    // 释放必须先关泵，再开阀放气，最后才能抬起
    write_pump(false);

    log_status(true);
}

void AP_SuctionCup::set_fault()
{
    _phase = Phase::NONE;
    _state = State::FAULT;
    // 故障安全：停泵并放气，避免泵空转或异常保压
    write_pump(false);
    write_valve(false);
    log_status(true);
}

// 保持态输出组合
void AP_SuctionCup::apply_safe_idle()
{
    write_pump(false);
    write_valve(false);
    request_lift(uint16_t(_lift_pwm_raised.get()), false);
}

void AP_SuctionCup::apply_raised_idle()
{
    apply_safe_idle();
    _state = State::RAISED;
    _phase = Phase::NONE;
}

void AP_SuctionCup::apply_lowered_hold()
{
    // 负压已建立：停泵省电，靠密封阀维持吸附
    write_pump(false);
    write_valve(true);
    request_lift(uint16_t(_lift_pwm_lowered.get()), false);
}

void AP_SuctionCup::apply_frozen_hold()
{
    if (_state == State::FROZEN || _state == State::LOWERED) {
        apply_lowered_hold();
        return;
    }

    if (_state == State::LOWERING) {
        write_pump(false);
        request_lift(uint16_t(_lift_pwm_lowered.get()), false);
        // 若已过密封步骤则保持密封，否则阀仍可能在放气位
        if (_phase >= Phase::LOWER_SEAL) {
            write_valve(true);
        }
    }
}

void AP_SuctionCup::request_lift(uint16_t pwm_us, bool slew)
{
    _lift_target_pwm = pwm_us;
    _lift_slew = slew;
    if (!slew) {
        write_lift_pwm(pwm_us);
        _lift_reached_ms = 0;
    }
}

void AP_SuctionCup::service_lift(uint32_t now)
{
    if (_last_lift_pwm == _lift_target_pwm) {
        _lift_ramp_last_ms = now;
        return;
    }

    if (!_lift_slew) {
        write_lift_pwm(_lift_target_pwm);
        _lift_ramp_last_ms = now;
        return;
    }

    if (_lift_ramp_last_ms == 0 || now < _lift_ramp_last_ms) {
        _lift_ramp_last_ms = now;
        return;
    }

    const uint32_t dt_ms = now - _lift_ramp_last_ms;
    _lift_ramp_last_ms = now;
    if (dt_ms == 0) {
        return;
    }

    const uint16_t rate = constrain_int16(_lift_rate_us.get(), 50, 5000);
    uint16_t step = uint16_t((uint32_t(rate) * dt_ms) / 1000U);
    if (step < 1) {
        step = 1;
    }

    uint16_t next = _last_lift_pwm;
    if (next < _lift_target_pwm) {
        const uint16_t room = _lift_target_pwm - next;
        next = (step >= room) ? _lift_target_pwm : uint16_t(next + step);
    } else {
        const uint16_t room = next - _lift_target_pwm;
        next = (step >= room) ? _lift_target_pwm : uint16_t(next - step);
    }
    write_lift_pwm(next);
}

void AP_SuctionCup::write_lift_pwm(uint16_t pwm_us)
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_vgsolar_suction_lift, pwm_us);
    _last_lift_pwm = pwm_us;
}

void AP_SuctionCup::write_valve(bool seal)
{
#if AP_RELAY_ENABLED
    AP_Relay *relay = AP::relay();
    if (relay != nullptr) {
        const uint8_t inst = constrain_int16(_valve_relay.get(), 0, AP_RELAY_NUM_RELAYS - 1);
        if (seal) {
            relay->on(inst);
        } else {
            relay->off(inst);
        }
    }
#endif
    _last_valve_on = seal;
}

void AP_SuctionCup::write_pump(bool on)
{
#if AP_RELAY_ENABLED
    AP_Relay *relay = AP::relay();
    if (relay != nullptr) {
        const uint8_t inst = constrain_int16(_pump_relay.get(), 0, AP_RELAY_NUM_RELAYS - 1);
        if (on) {
            relay->on(inst);
        } else {
            relay->off(inst);
        }
    }
#endif
    _last_pump_on = on;
}

// 吸附 / 释放子状态机（每 update() 推进一步）
void AP_SuctionCup::update_lowering(uint32_t now)
{
    switch (_phase) {
    case Phase::LOWER_WAIT_LIFT:
        if (!lift_at_target()) {
            _lift_reached_ms = 0;
            break;
        }
        if (_lift_reached_ms == 0) {
            _lift_reached_ms = now;
        }
        if (now - _lift_reached_ms >= uint32_t(_lift_delay_ms.get())) {
            _phase = Phase::LOWER_SEAL;
            _phase_start_ms = now;
            write_valve(true);
        }
        break;

    case Phase::LOWER_SEAL:
        // 密封后立即开泵（无额外等待）
        _phase = Phase::LOWER_START_PUMP;
        _phase_start_ms = now;
        write_pump(true);
        break;

    case Phase::LOWER_START_PUMP:
        _phase = Phase::LOWER_WAIT_VACUUM;
        _phase_start_ms = now;
        break;

    case Phase::LOWER_WAIT_VACUUM:
        if (now - _phase_start_ms >= uint32_t(_vacuum_delay_ms.get())) {
            _phase = Phase::NONE;
            _state = State::LOWERED;
            apply_lowered_hold();
            log_status(true);
        }
        break;

    default:
        break;
    }
}

void AP_SuctionCup::update_raising(uint32_t now)
{
    switch (_phase) {
    case Phase::RAISE_STOP_PUMP:
        _phase = Phase::RAISE_VENT;
        _phase_start_ms = now;
        write_valve(false);
        break;

    case Phase::RAISE_VENT:
        _phase = Phase::RAISE_WAIT_VENT;
        _phase_start_ms = now;
        break;

    case Phase::RAISE_WAIT_VENT:
        if (now - _phase_start_ms >= uint32_t(_vent_delay_ms.get())) {
            _phase = Phase::RAISE_LIFT;
            _phase_start_ms = now;
            _lift_reached_ms = 0;
            request_lift(uint16_t(_lift_pwm_raised.get()), true);
        }
        break;

    case Phase::RAISE_LIFT:
        _phase = Phase::RAISE_WAIT_LIFT;
        _phase_start_ms = now;
        break;

    case Phase::RAISE_WAIT_LIFT:
        if (!lift_at_target()) {
            _lift_reached_ms = 0;
            break;
        }
        if (_lift_reached_ms == 0) {
            _lift_reached_ms = now;
        }
        if (now - _lift_reached_ms >= uint32_t(_lift_delay_ms.get())) {
            _phase = Phase::NONE;
            _state = State::RAISED;
            log_status(true);
        }
        break;

    default:
        break;
    }
}

void AP_SuctionCup::check_action_timeout(uint32_t now)
{
    if (!is_busy()) {
        return;
    }

    if (now - _sequence_start_ms > uint32_t(_action_timeout_ms.get())) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SCUP: action timeout");
        set_fault();
    }
}

void AP_SuctionCup::log_status(bool force)
{
    const uint32_t now = AP_HAL::millis();
    const bool changed = _state != _last_logged_state || _phase != _last_logged_phase;
    if (!force && !changed) {
        if ((now - _last_log_ms) < LOG_INTERVAL_MS) {
            return;
        }
    }

    _last_logged_state = _state;
    _last_logged_phase = _phase;
    _last_log_ms = now;

}

namespace AP
{

AP_SuctionCup &suction_cup()
{
    return *AP_SuctionCup::get_singleton();
}

}
