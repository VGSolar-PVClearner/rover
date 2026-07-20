#include "AP_SuctionCup.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

/*
 * 升降红外到位检测（库内传感层）。
 *
 * 槽型光电 + 铁片约定（SCUP_IR_POL=0 时）：
 *   电平高 / “有铁片” → 未完全放下（抬起或运动中）
 *   电平低 / “无铁片” → 完全放下
 * POL=1 时反相。
 *
 * 放下：本阶段须先见过「有铁片」，再等到「无铁片」（防断线/极性误判秒完成）。
 * 抬起：先等到有铁片，再等 LIFT_DLY_MS；见过后抖动丢信号不重计时。
 */

extern const AP_HAL::HAL &hal;

bool AP_SuctionCup::ir_sensor_enabled() const
{
    return _ir_pin.get() >= 0;
}

void AP_SuctionCup::ensure_ir_pin_setup()
{
    const int16_t pin = _ir_pin.get();
    if (pin < 0) {
        _ir_pin_configured = -1;
        return;
    }

    if (_ir_pin_configured == pin) {
        return;
    }

    if (!hal.gpio->valid_pin((uint8_t)pin)) {
        // -2：已判定无效并告警过，避免每周期刷屏
        if (_ir_pin_configured != -2) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SCUP: invalid IR_PIN %d, using LIFT_DLY", (int)pin);
            _ir_pin_configured = -2;
        }
        return;
    }

    // 对齐 VGSolar hwdef：INPUT + PULLDOWN（ChibiOS 下对输入脚 write(0)=下拉）
    hal.gpio->pinMode((uint8_t)pin, HAL_GPIO_INPUT);
    hal.gpio->write((uint8_t)pin, 0);

    _ir_pin_configured = pin;
    _ir_raw_last = hal.gpio->read((uint8_t)pin) ? 1 : 0;
    _ir_debounced = _ir_raw_last;
    _ir_debounce_start_ms = AP_HAL::millis();
}

void AP_SuctionCup::update_ir_debounce(uint32_t now)
{
    ensure_ir_pin_setup();
    if (_ir_pin_configured < 0) {
        return;
    }

    const uint8_t raw = hal.gpio->read((uint8_t)_ir_pin_configured) ? 1 : 0;
    if (raw != _ir_raw_last) {
        _ir_raw_last = raw;
        _ir_debounce_start_ms = now;
        return;
    }

    const uint32_t deb_ms = MAX(0, (int)_ir_deb_ms.get());
    if (raw != _ir_debounced && (now - _ir_debounce_start_ms) >= deb_ms) {
        _ir_debounced = raw;
    }
}

bool AP_SuctionCup::ir_blade_present() const
{
    // POL 0：高=有铁片（未完全放下）；POL 非 0：反相
    const bool high_means_present = (_ir_pol.get() == 0);
    const bool is_high = (_ir_debounced != 0);
    return high_means_present ? is_high : !is_high;
}

bool AP_SuctionCup::ir_fully_lowered() const
{
    return !ir_blade_present();
}

AP_SuctionCup::LiftWaitResult AP_SuctionCup::check_lift_position_wait(uint32_t now, bool want_lowered)
{
    if (ir_sensor_enabled()) {
        update_ir_debounce(now);
        if (_ir_pin_configured < 0) {
            // 无效脚：回退延时，避免卡死
            if (now - _phase_start_ms >= uint32_t(_lift_delay_ms.get())) {
                return LiftWaitResult::Done;
            }
            return LiftWaitResult::Waiting;
        }

        if (want_lowered) {
            // 放下：须先见过有铁片，再等到无铁片（有→无）
            if (ir_blade_present()) {
                _lift_saw_blade = true;
            }
            if (_lift_saw_blade && ir_fully_lowered()) {
                return LiftWaitResult::Done;
            }
            if (now - _phase_start_ms >= uint32_t(_lift_timeout_ms.get())) {
                gcs().send_text(MAV_SEVERITY_WARNING, "VG_SCUP: lift lower timeout");
                set_fault();
                return LiftWaitResult::Faulted;
            }
            return LiftWaitResult::Waiting;
        }

        // 抬起：先等到有铁片，再等 LIFT_DLY_MS（见过后不再因抖动清零）
        if (_lift_ir_edge_ms == 0) {
            if (!ir_blade_present()) {
                if (now - _phase_start_ms >= uint32_t(_lift_timeout_ms.get())) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "VG_SCUP: lift raise IR timeout");
                    set_fault();
                    return LiftWaitResult::Faulted;
                }
                return LiftWaitResult::Waiting;
            }
            _lift_ir_edge_ms = now;
        }

        if (now - _lift_ir_edge_ms >= uint32_t(_lift_delay_ms.get())) {
            return LiftWaitResult::Done;
        }
        return LiftWaitResult::Waiting;
    }

    // 无红外：沿用 LIFT_DLY_MS
    if (now - _phase_start_ms >= uint32_t(_lift_delay_ms.get())) {
        return LiftWaitResult::Done;
    }
    return LiftWaitResult::Waiting;
}
