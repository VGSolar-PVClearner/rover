#include "AP_Brush.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

/*
 * AP_Brush 实现：滚刷 PWM 换算与 SRV_Channel 输出。
 * 参数来源见 AP_CompanionComputer::apply_brush_runtime_params()。
 */

extern const AP_HAL::HAL &hal;

AP_Brush *AP_Brush::_singleton;

const AP_Param::GroupInfo AP_Brush::var_info[] = {

    // @Param: F_STOP
    // @DisplayName: Front brush stop PWM
    // @Description: Front roller brush PWM at stop (µs). Power maps from F_STOP toward F_MAX.
    // @Range: 1000 2000
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("F_STOP", 1, AP_Brush, _front_pwm_stop, 1500),

    // @Param: F_MAX
    // @DisplayName: Front brush max PWM
    // @Description: Front roller brush PWM at 100% power (µs). May be above or below F_STOP.
    // @Range: 1000 2000
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("F_MAX", 2, AP_Brush, _front_pwm_max, 2000),

    // @Param: R_STOP
    // @DisplayName: Rear brush stop PWM
    // @Description: Rear roller brush PWM at stop (µs). Power maps from R_STOP toward R_MAX.
    // @Range: 1000 2000
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("R_STOP", 3, AP_Brush, _rear_pwm_stop, 1500),

    // @Param: R_MAX
    // @DisplayName: Rear brush max PWM
    // @Description: Rear roller brush PWM at 100% power (µs). May be above or below R_STOP (e.g. reverse direction).
    // @Range: 1000 2000
    // @Units: us
    // @User: Standard
    AP_GROUPINFO("R_MAX", 4, AP_Brush, _rear_pwm_max, 1000),

    AP_GROUPEND
};

AP_Brush::AP_Brush()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Brush must be singleton");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    _active = false;
    _front_on = false;
    _rear_on = false;
    _power_pct = 0;
    _last_front_pwm = 1500;
    _last_rear_pwm = 1500;
    _last_front_on = false;
    _last_rear_on = false;
    _last_power_pct = 0;
    _last_log_ms = 0;
}

void AP_Brush::set_active(bool active)
{
    _active = active;
    if (!_active) {
        // 离开 VGSL：立即停刷，不保留上一周期 PWM
        write_outputs(false, false, 0);
        return;
    }

    // 进入 VGSL：未解锁只停刷；已解锁才按缓存期望恢复
    if (!hal.util->get_soft_armed()) {
        write_outputs(false, false, 0);
    } else {
        write_outputs(_front_on, _rear_on, _power_pct);
    }
}

void AP_Brush::update(bool front_on, bool rear_on, uint8_t power_pct)
{
    _front_on = front_on;
    _rear_on = rear_on;
    _power_pct = power_pct;

    if (!_active) {
        // 非 VGSL：只记期望，不写电调（防止其他模式误触滚刷）
        return;
    }

    // 未解锁：保留期望，强制停转输出
    if (!hal.util->get_soft_armed()) {
        write_outputs(false, false, 0);
        return;
    }

    write_outputs(_front_on, _rear_on, _power_pct);
}

void AP_Brush::stop_all()
{
    _front_on = false;
    _rear_on = false;
    _power_pct = 0;
    // 无论 active 与否都写各刷停转 PWM，确保硬件停转
    write_outputs(false, false, 0);
}

uint16_t AP_Brush::clamp_pwm_us(int16_t pwm_us)
{
    return uint16_t(constrain_int16(pwm_us, 1000, 2000));
}

uint16_t AP_Brush::calc_pwm_us(bool on, uint8_t power_pct, uint16_t stop_us, uint16_t max_us)
{
    if (!on || power_pct == 0) {
        return stop_us;
    }

    const float scaled = constrain_float(float(power_pct) / 100.0f, 0.0f, 1.0f);
    // max 可小于 stop（后刷反向）：功率↑ 时 PWM 从 stop 向 max 插值
    return uint16_t(lroundf(float(stop_us) + scaled * float(int32_t(max_us) - int32_t(stop_us))));
}

void AP_Brush::write_outputs(bool front_on, bool rear_on, uint8_t power_pct)
{
    const uint16_t front_stop = clamp_pwm_us(_front_pwm_stop);
    const uint16_t front_max = clamp_pwm_us(_front_pwm_max);
    const uint16_t rear_stop = clamp_pwm_us(_rear_pwm_stop);
    const uint16_t rear_max = clamp_pwm_us(_rear_pwm_max);

    const uint16_t front_pwm = calc_pwm_us(front_on, power_pct, front_stop, front_max);
    const uint16_t rear_pwm = calc_pwm_us(rear_on, power_pct, rear_stop, rear_max);

    SRV_Channels::set_output_pwm(SRV_Channel::k_vgsolar_brush_front, front_pwm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_vgsolar_brush_rear, rear_pwm);

    log_brush_status(front_on, rear_on, power_pct, front_pwm, rear_pwm);

    _last_front_pwm = front_pwm;
    _last_rear_pwm = rear_pwm;
}

void AP_Brush::log_brush_status(bool front_on, bool rear_on, uint8_t power_pct,
                                uint16_t front_pwm, uint16_t rear_pwm)
{
    const uint32_t now = AP_HAL::millis();
    const bool state_changed =
        front_on != _last_front_on ||
        rear_on != _last_rear_on ||
        power_pct != _last_power_pct;
    const bool pwm_changed =
        front_pwm != _last_front_pwm ||
        rear_pwm != _last_rear_pwm;

    if (!state_changed && !pwm_changed) {
        return;
    }
    // 开关/功率未变、仅 PWM 抖动时 2s 限速，避免刷屏
    if (!state_changed && (now - _last_log_ms) < LOG_INTERVAL_MS) {
        return;
    }

    _last_front_on = front_on;
    _last_rear_on = rear_on;
    _last_power_pct = power_pct;
    _last_log_ms = now;

    gcs().send_text(MAV_SEVERITY_INFO,
                    "VG_BRUSH: front=%u rear=%u pwr=%u pwm=%u/%u",
                    unsigned(front_on), unsigned(rear_on), unsigned(power_pct),
                    unsigned(front_pwm), unsigned(rear_pwm));
}

namespace AP
{

AP_Brush &brush()
{
    return *AP_Brush::get_singleton();
}

}
