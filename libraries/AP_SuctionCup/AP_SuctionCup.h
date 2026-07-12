#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

/*
 * VGSolar 吸盘控制：升降舵机 + 气阀 + 气泵三路 PWM 异步状态机。
 *
 * 职责边界：
 *   ModeVGSolar — 转弯序列调用 lower()/raise()；安全保持/超时/倾角调用 freeze()；
 *                 急停/退出调用 emergency_release()；周期 update() 推进子状态机
 *   本库       — 时序与 PWM 输出；故障置 State::FAULT（Mode 上报 fault bit4）
 *
 * NCU 无吸盘专用协议；吸附/释放完全由 VGSL 模式内逻辑触发。
 *
 * 输出通道（SERVOx_FUNCTION）：
 *   k_vgsolar_suction_lift = 159  升降舵机（两档：抬起/放下）
 *   k_vgsolar_air_valve    = 160  气阀（两档：密封/放气）
 *   k_vgsolar_air_pump     = 161  气泵（1000 µs 停，1000~2000 µs 线性调速）
 *
 * 吸附序列 lower()：放气停泵 → 放下 → 密封 → 开泵 → 等待负压 → LOWERED（关泵保密封）
 * 释放序列 raise()：关泵 → 放气 → 等待泄压 → 抬起 → RAISED
 * freeze()：关泵+保持密封，不主动抬起（NCU 200ms 超时 / 倾角过大）
 *
 * 地面站参数前缀 SCUP_*；实例挂在 Rover ParametersG2::suction_cup。
 */
class AP_SuctionCup {
public:
    AP_SuctionCup();

    CLASS_NO_COPY(AP_SuctionCup);

    static AP_SuctionCup *get_singleton() { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    // 吸盘顶层状态（供 Mode 查询）
    enum class State : uint8_t {
        INACTIVE = 0,   // 未激活，不推进状态机
        RAISED,         // 抬起空闲：阀放气、泵停、升降在上位
        LOWERING,       // 吸附序列进行中
        LOWERED,        // 已吸附，可执行原地转弯
        RAISING,        // 释放序列进行中
        FROZEN,         // 安全冻结：关泵保密封，不主动抬起
        FAULT,          // 动作超时等故障；需 clear_fault() 人工恢复
    };

    // ModeVGSolar::_enter/_exit；退出时 apply_safe_idle（抬起+放气+停泵）
    void set_active(bool active);

    // 推进 lower/raise 子状态机；ModeVGSolar::update() 内周期调用
    void update();

    // 开始异步吸附；busy/frozen/fault 时返回 false
    bool lower();

    // 开始异步释放；仅 LOWERED/FROZEN 可调用
    bool raise();

    // NCU 心跳超时 / 倾角过大：冻结序列，已吸附则关泵+保持密封
    void freeze();

    // 急停 / 退出 VGSL：启动完整释放（关泵 → 放气 → 抬起）
    void emergency_release();

    void stop() { emergency_release(); }

    // 清除 FAULT 回到 RAISED；进入 VGSL 时 Mode 会调用
    void clear_fault();

    // 解除 freeze()；新转弯指令或安全恢复时由 Mode 调用
    void unfreeze();

    bool is_active() const { return _active; }
    bool is_busy() const;      // LOWERING 或 RAISING
    bool is_lowered() const;   // LOWERED 或 FROZEN（视为仍吸附）
    bool is_raised() const;    // RAISED 或 INACTIVE
    bool is_frozen() const { return _frozen; }
    bool has_fault() const { return _state == State::FAULT; }
    State get_state() const { return _state; }

    // 最近一次 write_* 输出的 PWM
    uint16_t get_last_lift_pwm_us() const { return _last_lift_pwm; }
    uint16_t get_last_valve_pwm_us() const { return _last_valve_pwm; }
    uint16_t get_last_pump_pwm_us() const { return _last_pump_pwm; }
    uint8_t get_state_u8() const { return uint8_t(_state); }
    uint8_t get_phase_u8() const { return uint8_t(_phase); }

private:
    static AP_SuctionCup *_singleton;

    // lower/raise 内部子步骤（由 update_lowering / update_raising 推进）
    enum class Phase : uint8_t {
        NONE = 0,
        LOWER_WAIT_LIFT,    // 等待升降放下到位（LIFT_DLY_MS）
        LOWER_SEAL,         // 切换气阀密封
        LOWER_START_PUMP,   // 按 SCUP_PUMP_PWR 开泵
        LOWER_WAIT_VACUUM,  // 等待建立负压（VAC_DLY_MS）
        RAISE_STOP_PUMP,    // 关泵
        RAISE_VENT,         // 气阀放气
        RAISE_WAIT_VENT,    // 等待泄压（VENT_DLY_MS）
        RAISE_LIFT,         // 升降抬起
        RAISE_WAIT_LIFT,    // 等待抬起到位（LIFT_DLY_MS）
    };

    static constexpr uint16_t PWM_STOP_US = 1000;
    static constexpr uint16_t PWM_MAX_US = 2000;
    static constexpr uint32_t LOG_INTERVAL_MS = 2000;

    AP_Int8 _pump_pwr;           // SCUP_PUMP_PWR：建负压时气泵功率 0~100%
    AP_Int16 _lift_delay_ms;     // SCUP_LIFT_DLY_MS：升降动作后等待
    AP_Int16 _vacuum_delay_ms;   // SCUP_VAC_DLY_MS：开泵后等待负压建立
    AP_Int16 _vent_delay_ms;     // SCUP_VENT_DLY_MS：放气后等待泄压
    AP_Int16 _action_timeout_ms; // SCUP_ACT_TOUT_MS：单次 lower/raise 总超时 → FAULT
    AP_Int16 _lift_pwm_raised;   // SCUP_LIFT_PWM_R：抬起位置 PWM µs
    AP_Int16 _lift_pwm_lowered;  // SCUP_LIFT_PWM_L：放下位置 PWM µs
    AP_Int16 _valve_pwm_vent;    // SCUP_VLV_PWM_V：放气 PWM µs
    AP_Int16 _valve_pwm_seal;    // SCUP_VLV_PWM_S：密封 PWM µs

    bool _active;
    bool _frozen;                // freeze() 置位；unfreeze() 清除
    bool _frozen_vacuum_ready;   // freeze 时是否已完成负压（原 LOWERED/RAISING 中断）
    State _state;
    Phase _phase;
    uint32_t _phase_start_ms;    // 当前子步骤起始时刻
    uint32_t _sequence_start_ms; // 整次 lower/raise 起始时刻（超时判定）

    // 日志去重 + 最近一次 PWM 缓存
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

    // 安全/空闲/保持态 PWM 组合
    void apply_safe_idle();      // 抬起 + 放气 + 停泵（最安全）
    void apply_raised_idle();    // safe_idle + State::RAISED
    void apply_lowered_hold();   // 放下 + 密封 + 停泵（转向时维持吸附）
    void apply_frozen_hold();    // 冻结态输出；LOWERING 中途按阶段决定阀位

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
