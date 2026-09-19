#pragma once

#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>

/*
 * VGSolar 前后滚刷 PWM 输出驱动。
 *
 * 职责边界：
 *   AP_CompanionComputer — 缓存 NCU 运行参数 0x0101~0x0104，调用 update()/stop_all()
 *   ModeVGSolar          — 进入/退出 VGSL 时 set_active(true/false)
 *   本库                 — 开关/功率 → PWM 微秒值，写 SRV_Channel 辅助通道
 *
 * 输出通道（需在地面站配置 SERVOx_FUNCTION）：
 *   k_vgsolar_brush_front = 157  前滚刷
 *   k_vgsolar_brush_rear  = 158  后滚刷
 *
 * 电调约定（地面站可调，默认前后方向相反）：
 *   BRUSH_F_STOP / BRUSH_F_MAX：前刷停转 / 满速 µs（默认 1500 → 2000）
 *   BRUSH_R_STOP / BRUSH_R_MAX：后刷停转 / 满速 µs（默认 1500 → 1000）
 *   power_pct 1~100% 在各自 [STOP, MAX] 间线性映射（MAX 可小于 STOP）。
 * 安全：仅 _active=true（VGSL 模式内）且已 soft_armed 时才输出非停转 PWM；
 *       非活动或未解锁时 update()/set_active 只停刷或更新期望。
 *
 * 地面站参数前缀 BRUSH_*；实例挂在 Rover ParametersG2::brush。
 */
class AP_Brush
{
public:
    AP_Brush();

    /* Do not allow copies */
    AP_Brush(const AP_Brush &other) = delete;
    AP_Brush &operator=(const AP_Brush&) = delete;

    static AP_Brush *get_singleton()
    {
        return _singleton;
    }

    static const struct AP_Param::GroupInfo var_info[];

    // ModeVGSolar::_enter/_exit 调用；false 时立即输出各刷停转 PWM
    void set_active(bool active);

    // 更新期望开关与功率；仅 active 时写 PWM（CompanionComputer 参数写入后调用）
    void update(bool front_on, bool rear_on, uint8_t power_pct);

    // 清零期望状态并强制停转 PWM；急停/低电压/心跳超时/退出 VGSL 时由上层调用
    void stop_all();

private:
    static AP_Brush *_singleton;

    bool _active;       // VGSL 模式是否激活
    bool _front_on;     // 期望前刷开关（0x0101 / 0x0103）
    bool _rear_on;      // 期望后刷开关（0x0102 / 0x0103）
    uint8_t _power_pct; // 期望功率档位 0~100（0x0104）

    // 日志去重：状态变化立即打；仅 PWM 微变则按 LOG_INTERVAL_MS 限速
    uint16_t _last_front_pwm;
    uint16_t _last_rear_pwm;
    bool _last_front_on;
    bool _last_rear_on;
    uint8_t _last_power_pct;
    uint32_t _last_log_ms;

    static constexpr uint32_t LOG_INTERVAL_MS = 2000;

    // 前/后刷：停转与满速 PWM（µs），地面站 BRUSH_F_*/BRUSH_R_*
    AP_Int16 _front_pwm_stop;
    AP_Int16 _front_pwm_max;
    AP_Int16 _rear_pwm_stop;
    AP_Int16 _rear_pwm_max;

    // on=false 或 power=0 → STOP；否则线性插值 [STOP, MAX]（MAX 可小于 STOP）
    static uint16_t calc_pwm_us(bool on, uint8_t power_pct, uint16_t stop_us, uint16_t max_us);
    static uint16_t clamp_pwm_us(int16_t pwm_us);
    void write_outputs(bool front_on, bool rear_on, uint8_t power_pct);
    void log_brush_status(bool front_on, bool rear_on, uint8_t power_pct, uint16_t front_pwm, uint16_t rear_pwm);
};

namespace AP
{
AP_Brush &brush();
}
