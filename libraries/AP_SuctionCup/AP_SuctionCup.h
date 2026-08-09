#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

/*
 * VGSolar 吸盘控制：升降舵机（PWM 缓速）+ 气阀/气泵（Relay 开关）异步状态机。
 *
 * 职责边界：
 *   ModeVGSolar — 转弯序列调用 lower()/raise()；安全保持/超时/倾角调用 freeze()；
 *                 急停/退出调用 emergency_release()；周期 update() 推进子状态机
 *   本库       — 时序与输出；故障置 State::FAULT（Mode 上报 fault bit4）
 *
 * NCU 无吸盘专用协议；吸附/释放完全由 VGSL 模式内逻辑触发。
 *
 * 输出：
 *   SERVO FUNCTION 159 (k_vgsolar_suction_lift) — 升降舵机，目标 PWM 按 SCUP_LIFT_RATE 缓变
 *   RELAYx — 气阀 / 气泵（SCUP_VLV_RLY / SCUP_PUMP_RLY 指定实例；on=密封/开泵）
 *
 * 吸附序列 lower()：放气停泵 → 缓速放下 → 红外/延时到位 → 密封 → 开泵 → 等待负压 → LOWERED（开泵维持）
 * 释放序列 raise()：关泵 → 放气 → 等待泄压 → 缓速抬起 → 红外见铁片+延时/纯延时 → RAISED
 * freeze()：停泵+保持密封，不主动抬起（NCU 运动丢控超时 / 倾角过大）
 * 未 soft_armed：lower() 拒绝；update() 强制释放/安全位（抬起+放气+停泵）
 *
 * 升降到位：SCUP_IR_PIN>=0 时用槽型光电 GPIO（默认 98）。
 *   放下：缓速中可记「见过铁片」，PWM 到位后再等消失；抬起：见铁片后再等 LIFT_DLY_MS（且 PWM 到位）。
 *   PIN=-1 时：PWM 到位后再等 LIFT_DLY_MS。红外超时用 LIFT_TO_MS。
 * 红外传感实现：AP_SuctionCup_IR.cpp（库内拆分，非独立 AP_ 库）。
 *
 * 地面站参数前缀 SCUP_*；实例挂在 Rover ParametersG2::suction_cup。
 */
class AP_SuctionCup {
public:
    AP_SuctionCup();

    CLASS_NO_COPY(AP_SuctionCup);

    static AP_SuctionCup *get_singleton() { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    enum class State : uint8_t {
        INACTIVE = 0,
        RAISED,
        LOWERING,
        LOWERED,
        RAISING,
        FROZEN,
        FAULT,
    };

    void set_active(bool active);
    void update();
    bool lower();
    bool raise();
    void freeze();
    void emergency_release();
    void clear_fault();
    void unfreeze();

    bool is_busy() const;
    bool is_lowered() const;
    bool is_raised() const;
    bool is_frozen() const { return _frozen; }
    bool has_fault() const { return _state == State::FAULT; }
    State get_state() const { return _state; }

    uint16_t get_last_lift_pwm_us() const { return _last_lift_pwm; }
    bool get_last_valve_on() const { return _last_valve_on; }
    bool get_last_pump_on() const { return _last_pump_on; }
    uint8_t get_state_u8() const { return uint8_t(_state); }
    uint8_t get_phase_u8() const { return uint8_t(_phase); }

private:
    static AP_SuctionCup *_singleton;

    enum class Phase : uint8_t {
        NONE = 0,
        LOWER_WAIT_LIFT,    // 缓速放下；红外有→无 或 PWM 到位后 LIFT_DLY
        LOWER_SEAL,
        LOWER_START_PUMP,
        LOWER_WAIT_VACUUM,
        RAISE_STOP_PUMP,
        RAISE_VENT,
        RAISE_WAIT_VENT,
        RAISE_LIFT,
        RAISE_WAIT_LIFT,    // 缓速抬起；见铁片+LIFT_DLY 或 PWM 到位后 LIFT_DLY
    };

    enum class LiftWaitResult : uint8_t {
        Waiting = 0,
        Done,
        Faulted,
    };

    static constexpr uint32_t LOG_INTERVAL_MS = 2000;

    AP_Int16 _lift_delay_ms;     // 无红外：PWM 到位后等待；有红外抬起：见铁片后补行程
    AP_Int16 _vacuum_delay_ms;
    AP_Int16 _vent_delay_ms;
    AP_Int16 _action_timeout_ms;
    AP_Int16 _lift_pwm_raised;
    AP_Int16 _lift_pwm_lowered;
    AP_Int16 _lift_rate_us;
    AP_Int8 _valve_relay;
    AP_Int8 _pump_relay;

    AP_Int16 _ir_pin;
    AP_Int8  _ir_pol;
    AP_Int16 _ir_deb_ms;
    AP_Int16 _lift_timeout_ms;
    AP_Float _vacuum_pressure_kpa;
    AP_Int16 _vacuum_debounce_ms;
    AP_Float _vacuum_hysteresis_kpa;

    bool _active;
    bool _frozen;
    bool _frozen_vacuum_ready;
    State _state;
    Phase _phase;
    uint32_t _phase_start_ms;
    uint32_t _sequence_start_ms;

    // 升降缓速
    uint16_t _lift_target_pwm;
    bool _lift_slew;
    uint32_t _lift_ramp_last_ms;
    uint32_t _lift_reached_ms;

    // 红外
    int16_t _ir_pin_configured;
    uint8_t _ir_raw_last;
    uint8_t _ir_debounced;
    uint32_t _ir_debounce_start_ms;
    uint32_t _lift_ir_edge_ms;
    bool _lift_saw_blade;

    uint16_t _last_lift_pwm;
    bool _last_valve_on;
    bool _last_pump_on;
    State _last_logged_state;
    Phase _last_logged_phase;
    uint32_t _last_log_ms;
    uint32_t _vacuum_ok_start_ms;
    uint32_t _vacuum_loss_start_ms;
    bool _vacuum_loss_warned;

    bool can_start_sequence() const;
    void begin_lower();
    void begin_raise();
    void set_fault();

    void apply_safe_idle();
    void apply_raised_idle();
    void apply_lowered_hold();
    void apply_frozen_hold();

    void request_lift(uint16_t pwm_us, bool slew);
    void service_lift(uint32_t now);
    bool lift_at_target() const { return _last_lift_pwm == _lift_target_pwm; }
    void write_lift_pwm(uint16_t pwm_us);

    void write_valve(bool seal);
    void write_pump(bool on);

    bool ir_sensor_enabled() const;
    void ensure_ir_pin_setup();
    void update_ir_debounce(uint32_t now);
    bool ir_blade_present() const;
    bool ir_fully_lowered() const;
    LiftWaitResult check_lift_position_wait(uint32_t now, bool want_lowered);

    void update_lowering(uint32_t now);
    void update_raising(uint32_t now);
    void check_action_timeout(uint32_t now);
    void check_vacuum_loss(uint32_t now);

    void log_status(bool force = false);
};

namespace AP {
    AP_SuctionCup &suction_cup();
}
