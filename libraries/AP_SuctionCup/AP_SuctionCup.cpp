#include "AP_SuctionCup.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

AP_SuctionCup *AP_SuctionCup::_singleton;

const AP_Param::GroupInfo AP_SuctionCup::var_info[] = {
    // @Param: PUMP_PWR
    // @DisplayName: Suction pump power
    // @Description: Vacuum pump speed when attaching, expressed as percent. 0=stop (1000us), 100=max (2000us).
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_PWR", 0, AP_SuctionCup, _pump_pwr, 100),

    // @Param: LIFT_DLY_MS
    // @DisplayName: Lift servo settle time
    // @Description: Time to wait after commanding lift servo before next suction step
    // @Units: ms
    // @Range: 100 5000
    // @User: Standard
    AP_GROUPINFO("LIFT_DLY_MS", 1, AP_SuctionCup, _lift_delay_ms, 500),

    // @Param: VAC_DLY_MS
    // @DisplayName: Vacuum build time
    // @Description: Time to wait after starting pump before suction is considered ready
    // @Units: ms
    // @Range: 100 10000
    // @User: Standard
    AP_GROUPINFO("VAC_DLY_MS", 2, AP_SuctionCup, _vacuum_delay_ms, 800),

    // @Param: VENT_DLY_MS
    // @DisplayName: Vent delay before lift
    // @Description: Time to wait after opening valve before raising lift servo
    // @Units: ms
    // @Range: 100 5000
    // @User: Standard
    AP_GROUPINFO("VENT_DLY_MS", 3, AP_SuctionCup, _vent_delay_ms, 400),

    // @Param: ACT_TOUT_MS
    // @DisplayName: Suction action timeout
    // @Description: Maximum time allowed for a lower or raise sequence before fault
    // @Units: ms
    // @Range: 1000 30000
    // @User: Standard
    AP_GROUPINFO("ACT_TOUT_MS", 4, AP_SuctionCup, _action_timeout_ms, 5000),

    // @Param: LIFT_PWM_R
    // @DisplayName: Lift PWM raised
    // @Description: PWM pulse width in microseconds for raised lift position
    // @Units: us
    // @Range: 1000 2000
    // @User: Advanced
    AP_GROUPINFO("LIFT_PWM_R", 5, AP_SuctionCup, _lift_pwm_raised, 1000),

    // @Param: LIFT_PWM_L
    // @DisplayName: Lift PWM lowered
    // @Description: PWM pulse width in microseconds for lowered lift position
    // @Units: us
    // @Range: 1000 2000
    // @User: Advanced
    AP_GROUPINFO("LIFT_PWM_L", 6, AP_SuctionCup, _lift_pwm_lowered, 2000),

    // @Param: VLV_PWM_V
    // @DisplayName: Valve PWM vent
    // @Description: PWM pulse width in microseconds for venting (release vacuum)
    // @Units: us
    // @Range: 1000 2000
    // @User: Advanced
    AP_GROUPINFO("VLV_PWM_V", 7, AP_SuctionCup, _valve_pwm_vent, 1000),

    // @Param: VLV_PWM_S
    // @DisplayName: Valve PWM seal
    // @Description: PWM pulse width in microseconds for sealing vacuum line
    // @Units: us
    // @Range: 1000 2000
    // @User: Advanced
    AP_GROUPINFO("VLV_PWM_S", 8, AP_SuctionCup, _valve_pwm_seal, 2000),

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
    _state = State::RAISED;
    _phase = Phase::NONE;
    _phase_start_ms = 0;
    _sequence_start_ms = 0;

    _last_lift_pwm = PWM_STOP_US;
    _last_valve_pwm = PWM_STOP_US;
    _last_pump_pwm = PWM_STOP_US;
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

    // 冻结态：不再推进状态机，仅维持关泵+密封
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
    default:
        break;
    }

    log_status();
}

bool AP_SuctionCup::lower()
{
    if (!_active || _frozen || !can_start_sequence()) {
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
    _phase = Phase::NONE;

    if (_state == State::LOWERING || _state == State::LOWERED || _state == State::FROZEN) {
        _state = State::FROZEN;
        apply_frozen_hold();
        log_status(true);
        return;
    }

    if (_state == State::RAISING) {
        // 释放过程中遇超时/倾角：中止抬起，尽量保持吸附
        _state = State::FROZEN;
        apply_frozen_hold();
        log_status(true);
        return;
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

    begin_raise();
}

void AP_SuctionCup::clear_fault()
{
    if (_state != State::FAULT) {
        return;
    }

    _frozen = false;
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
}

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

void AP_SuctionCup::begin_lower()
{
    const uint32_t now = AP_HAL::millis();

    _state = State::LOWERING;
    _phase = Phase::LOWER_WAIT_LIFT;
    _phase_start_ms = now;
    _sequence_start_ms = now;

    // 吸附第一步：先放气、停泵，再放下升降（避免带压硬顶）
    write_valve(_valve_pwm_vent.get());
    write_pump(PWM_STOP_US);
    write_lift(_lift_pwm_lowered.get());

    log_status(true);
}

void AP_SuctionCup::begin_raise()
{
    const uint32_t now = AP_HAL::millis();

    _state = State::RAISING;
    _phase = Phase::RAISE_STOP_PUMP;
    _phase_start_ms = now;
    _sequence_start_ms = now;

    // 释放必须先关泵，再开阀放气，最后才能抬起
    write_pump(PWM_STOP_US);

    log_status(true);
}

void AP_SuctionCup::set_fault()
{
    _phase = Phase::NONE;
    _state = State::FAULT;
    // 故障安全：停泵并放气，避免泵空转或密封异常保压
    write_pump(PWM_STOP_US);
    write_valve(_valve_pwm_vent.get());
    log_status(true);
}

void AP_SuctionCup::apply_safe_idle()
{
    write_pump(PWM_STOP_US);
    write_valve(_valve_pwm_vent.get());
    write_lift(_lift_pwm_raised.get());
}

void AP_SuctionCup::apply_raised_idle()
{
    apply_safe_idle();
    _state = State::RAISED;
    _phase = Phase::NONE;
}

void AP_SuctionCup::apply_frozen_hold()
{
    write_pump(PWM_STOP_US);

    if (_state == State::FROZEN || _state == State::LOWERED) {
        // 已吸附或冻结：关泵省电，阀保持密封维持负压
        write_valve(_valve_pwm_seal.get());
        write_lift(_lift_pwm_lowered.get());
        return;
    }

    if (_state == State::LOWERING) {
        write_lift(_lift_pwm_lowered.get());
        if (_phase >= Phase::LOWER_SEAL) {
            write_valve(_valve_pwm_seal.get());
        }
    }
}

uint16_t AP_SuctionCup::calc_pump_pwm_us() const
{
    const uint8_t pwr = constrain_int16(_pump_pwr.get(), 0, 100);
    if (pwr == 0) {
        return PWM_STOP_US;
    }
    // 100% → 2000us，0% → 1000us，线性映射
    const float scaled = float(pwr) / 100.0f;
    return uint16_t(PWM_STOP_US + scaled * float(PWM_MAX_US - PWM_STOP_US));
}

void AP_SuctionCup::write_lift(uint16_t pwm_us)
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_vgsolar_suction_lift, pwm_us);
    _last_lift_pwm = pwm_us;
}

void AP_SuctionCup::write_valve(uint16_t pwm_us)
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_vgsolar_air_valve, pwm_us);
    _last_valve_pwm = pwm_us;
}

void AP_SuctionCup::write_pump(uint16_t pwm_us)
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_vgsolar_air_pump, pwm_us);
    _last_pump_pwm = pwm_us;
}

void AP_SuctionCup::update_lowering(uint32_t now)
{
    switch (_phase) {
    case Phase::LOWER_WAIT_LIFT:
        if (now - _phase_start_ms >= uint32_t(_lift_delay_ms.get())) {
            _phase = Phase::LOWER_SEAL;
            _phase_start_ms = now;
            write_valve(_valve_pwm_seal.get());
        }
        break;

    case Phase::LOWER_SEAL:
        _phase = Phase::LOWER_START_PUMP;
        _phase_start_ms = now;
        write_pump(calc_pump_pwm_us());
        break;

    case Phase::LOWER_START_PUMP:
        _phase = Phase::LOWER_WAIT_VACUUM;
        _phase_start_ms = now;
        break;

    case Phase::LOWER_WAIT_VACUUM:
        if (now - _phase_start_ms >= uint32_t(_vacuum_delay_ms.get())) {
            _phase = Phase::NONE;
            _state = State::LOWERED;
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
        write_valve(_valve_pwm_vent.get());
        break;

    case Phase::RAISE_VENT:
        _phase = Phase::RAISE_WAIT_VENT;
        _phase_start_ms = now;
        break;

    case Phase::RAISE_WAIT_VENT:
        if (now - _phase_start_ms >= uint32_t(_vent_delay_ms.get())) {
            _phase = Phase::RAISE_LIFT;
            _phase_start_ms = now;
            write_lift(_lift_pwm_raised.get());
        }
        break;

    case Phase::RAISE_LIFT:
        _phase = Phase::RAISE_WAIT_LIFT;
        _phase_start_ms = now;
        break;

    case Phase::RAISE_WAIT_LIFT:
        if (now - _phase_start_ms >= uint32_t(_lift_delay_ms.get())) {
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

    gcs().send_text(MAV_SEVERITY_INFO,
                    "VG_SCUP: st=%u ph=%u frz=%u lift=%u valve=%u pump=%u",
                    unsigned(_state), unsigned(_phase), unsigned(_frozen),
                    unsigned(_last_lift_pwm), unsigned(_last_valve_pwm), unsigned(_last_pump_pwm));
}

namespace AP {

AP_SuctionCup &suction_cup()
{
    return *AP_SuctionCup::get_singleton();
}

}
