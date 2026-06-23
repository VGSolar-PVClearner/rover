#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

/*
  VGSolar 吸盘控制：升降舵机 + 气阀 + 气泵三路 PWM。
  NCU 无吸盘协议，由 ModeVGSolar 通过本库对外 API 调用。
  硬件映射：SERVO159 升降 / 160 气阀 / 161 气泵。
  PWM 约定：升降/气阀两档；气泵 1000us 停，1000~2000us 线性调速。
 */
class AP_SuctionCup {
public:
    AP_SuctionCup();

    CLASS_NO_COPY(AP_SuctionCup);

    static AP_SuctionCup *get_singleton() { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    // 吸盘顶层状态（供 Mode 查询）
    enum class State : uint8_t {
        INACTIVE = 0,   // 未激活，不输出 PWM
        RAISED,         // 抬起空闲：阀放气、泵停
        LOWERING,       // 吸附序列进行中
        LOWERED,        // 已吸附，可转弯
        RAISING,        // 释放序列进行中
        FROZEN,         // NCU 超时/倾角过大：关泵保密封，不主动抬起
        FAULT,          // 动作超时等故障，供 fault bit4
    };

    // 进入/离开 VGSL 时调用；退出时输出安全空闲态（抬起+放气+停泵）
    void set_active(bool active);

    // 推进 lower/raise 子状态机，需周期调用（建议 10~100Hz）
    void update();

    // 开始异步吸附：放下 → 密封 → 开泵 → 等待负压
    bool lower();

    // 开始异步释放：关泵 → 放气 → 等待 → 抬起（仅 LOWERED/FROZEN 可调用）
    bool raise();

    // NCU 200ms 超时 / 倾角过大：冻结序列，已吸附则关泵+保持密封
    void freeze();

    // 急停 / 退出 VGSL：完整释放（关泵 → 放气 → 抬起）
    void emergency_release();

    void stop() { emergency_release(); }

    // 清除 FAULT，回到 RAISED（需人工确认安全后调用）
    void clear_fault();

    // 解除 freeze()（新转弯指令等场景）
    void unfreeze();

    bool is_active() const { return _active; }
    bool is_busy() const;
    bool is_lowered() const;
    bool is_raised() const;
    bool is_frozen() const { return _frozen; }
    bool has_fault() const { return _state == State::FAULT; }
    State get_state() const { return _state; }

private:
    static AP_SuctionCup *_singleton;

    // lower/raise 内部子步骤
    enum class Phase : uint8_t {
        NONE = 0,
        LOWER_WAIT_LIFT,    // 等待升降放下到位
        LOWER_SEAL,         // 气阀密封
        LOWER_START_PUMP,   // 开泵
        LOWER_WAIT_VACUUM,  // 等待建立负压
        RAISE_STOP_PUMP,    // 关泵
        RAISE_VENT,         // 气阀放气
        RAISE_WAIT_VENT,    // 等待泄压
        RAISE_LIFT,         // 升降抬起
        RAISE_WAIT_LIFT,    // 等待抬起到位
    };

    static constexpr uint16_t PWM_STOP_US = 1000;
    static constexpr uint16_t PWM_MAX_US = 2000;
    static constexpr uint32_t LOG_INTERVAL_MS = 2000;

    AP_Int8 _pump_pwr;           // SCUP_PUMP_PWR，地面站可调气泵转速 0~100%
    AP_Int16 _lift_delay_ms;
    AP_Int16 _vacuum_delay_ms;
    AP_Int16 _vent_delay_ms;
    AP_Int16 _action_timeout_ms;
    AP_Int16 _lift_pwm_raised;
    AP_Int16 _lift_pwm_lowered;
    AP_Int16 _valve_pwm_vent;
    AP_Int16 _valve_pwm_seal;

    bool _active;
    bool _frozen;
    State _state;
    Phase _phase;
    uint32_t _phase_start_ms;
    uint32_t _sequence_start_ms;

    uint16_t _last_lift_pwm;
    uint16_t _last_valve_pwm;
    uint16_t _last_pump_pwm;
    State _last_logged_state;
    Phase _last_logged_phase;
    uint32_t _last_log_ms;

    bool can_start_sequence() const;
    void begin_lower();
    void begin_raise();
    void set_fault();
    void apply_safe_idle();
    void apply_raised_idle();
    void apply_frozen_hold();

    uint16_t calc_pump_pwm_us() const;

    void write_lift(uint16_t pwm_us);
    void write_valve(uint16_t pwm_us);
    void write_pump(uint16_t pwm_us);

    void update_lowering(uint32_t now);
    void update_raising(uint32_t now);
    void check_action_timeout(uint32_t now);

    void log_status(bool force = false);
};

namespace AP {
    AP_SuctionCup &suction_cup();
}
