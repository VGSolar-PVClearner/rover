#include "AP_SuctionPressure.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include <cstring>

extern const AP_HAL::HAL &hal;

static bool float_exactly_equal(const float value, const float expected)
{
    return memcmp(&value, &expected, sizeof(value)) == 0;
}

// ==================== 板级默认值与接入约束 ====================
// VGSolar 由 hwdef 提供 PC0 对应的 ADC 逻辑引脚 10；其他板默认禁用。
// R95 不安装后，压力输出仍按电压有效性窗口判断，但不能可靠判断所有开路。
#ifdef HAL_SUCTION_PRESSURE_PIN
#define AP_SUCTION_PRESSURE_DEFAULT_PIN HAL_SUCTION_PRESSURE_PIN
#else
#define AP_SUCTION_PRESSURE_DEFAULT_PIN -1
#endif

AP_SuctionPressure *AP_SuctionPressure::_singleton;
constexpr float AP_SuctionPressure::SAMPLE_RATE_HZ;
constexpr float AP_SuctionPressure::VOLTAGE_MARGIN_V;

// ==================== SPRESS_ 参数定义 ====================
// V_MIN/P_MIN 和 V_MAX/P_MAX 是同一条线性标定曲线的两个端点；默认值
// 对应 MCP-H10-P 3.3V 负压表压模块：0.1V=0kPa，3.1V=-100kPa。
// FILT_HZ 只影响滤波，不改变端点标定。

const AP_Param::GroupInfo AP_SuctionPressure::var_info[] = {
    // @Param: PIN
    // @DisplayName: Suction pressure analog pin
    // @Description: Analog pin for the suction pressure sensor. -1 disables pressure sensing.
    // @Values: -1:Disabled,10:VGSolar PC0
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("PIN", 1, AP_SuctionPressure, _pin, AP_SUCTION_PRESSURE_DEFAULT_PIN),

    // @Param: V_MIN
    // @DisplayName: Minimum sensor voltage
    // @Description: MCP-H10 lower calibration voltage corresponding to SPRESS_P_MIN
    // @Units: V
    // @Range: 0 3.3
    // @User: Advanced
    AP_GROUPINFO("V_MIN", 2, AP_SuctionPressure, _voltage_min_v, 0.1f),

    // @Param: V_MAX
    // @DisplayName: Maximum sensor voltage
    // @Description: MCP-H10 upper calibration voltage corresponding to SPRESS_P_MAX
    // @Units: V
    // @Range: 0 3.3
    // @User: Advanced
    AP_GROUPINFO("V_MAX", 3, AP_SuctionPressure, _voltage_max_v, 3.1f),

    // @Param: P_MIN
    // @DisplayName: Pressure at minimum voltage
    // @Description: MCP-H10 pressure corresponding to SPRESS_V_MIN
    // @Units: kPa
    // @Range: -150 0
    // @User: Advanced
    AP_GROUPINFO("P_MIN", 4, AP_SuctionPressure, _pressure_min_kpa, 0.0f),

    // @Param: P_MAX
    // @DisplayName: Pressure at maximum voltage
    // @Description: MCP-H10 pressure corresponding to SPRESS_V_MAX
    // @Units: kPa
    // @Range: -100 50
    // @User: Advanced
    AP_GROUPINFO("P_MAX", 5, AP_SuctionPressure, _pressure_max_kpa, -100.0f),

    // @Param: FILT_HZ
    // @DisplayName: Pressure low-pass filter frequency
    // @Description: First-order low-pass cutoff frequency applied to suction pressure
    // @Units: Hz
    // @Range: 0.1 20
    // @User: Advanced
    AP_GROUPINFO("FILT_HZ", 6, AP_SuctionPressure, _filter_hz, 5.0f),

    AP_GROUPEND
};

// ==================== 构造与初始化 ====================

AP_SuctionPressure::AP_SuctionPressure()
    : _source(nullptr)
    , _status(Status::DISABLED)
    , _voltage_v(NAN)
    , _pressure_kpa(NAN)
    , _last_update_ms(0)
    , _valid_start_ms(0)
    , _invalid_start_ms(0)
    , _last_reported_healthy(false)
    , _health_report_initialized(false)
{
    // 压力模块由 ParametersG2 构造为单例，重复实例会导致 AP::suction_pressure()
    // 指向不确定对象，因此在构造阶段直接拒绝重复创建。
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_SuctionPressure must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_SuctionPressure::migrate_legacy_calibration()
{
    if (!enabled() ||
        !float_exactly_equal(_voltage_min_v.get(), 0.2f) ||
        !float_exactly_equal(_voltage_max_v.get(), 2.7f) ||
        !float_exactly_equal(_pressure_min_kpa.get(), -100.0f) ||
        !float_exactly_equal(_pressure_max_kpa.get(), 0.0f)) {
        return false;
    }

    _voltage_min_v.set_and_save(0.1f);
    _voltage_max_v.set_and_save(3.1f);
    _pressure_min_kpa.set_and_save(0.0f);
    _pressure_max_kpa.set_and_save(-100.0f);
    return true;
}

void AP_SuctionPressure::init()
{
    // 调度频率固定为 50Hz，滤波截止频率由 SPRESS_FILT_HZ 配置；初始化时
    // 清空旧的滤波和健康计时，避免重启或重新初始化后沿用历史数据。
    _pressure_filter.set_cutoff_frequency(SAMPLE_RATE_HZ, MAX(_filter_hz.get(), 0.1f));
    _pressure_filter.reset();
    _last_update_ms = 0;
    _valid_start_ms = 0;
    _invalid_start_ms = 0;

    if (!enabled()) {
        // PIN=-1 明确表示禁用，不申请 HAL 模拟源。
        _source = nullptr;
        set_status(Status::DISABLED);
        return;
    }

    // 使用原始平均电压，避免将 3.3V 模拟输出错误地当作 5V 比率式传感器。
    // 这里只申请 ADC 源，不在初始化阶段假定传感器已达到有效压力。
    _source = hal.analogin->channel(_pin.get());
    set_status(_source == nullptr ? Status::NO_SOURCE : Status::WARMING_UP);
}

bool AP_SuctionPressure::enabled() const
{
    return _pin.get() >= 0;
}

// ==================== 电压有效性判断 ====================

bool AP_SuctionPressure::voltage_valid(const float voltage_v) const
{
    // 先检查输入和标定参数，防止 NaN、无穷值或反向端点进入压力换算。
    if (!isfinite(voltage_v)) {
        return false;
    }

    const float voltage_min_v = _voltage_min_v.get();
    const float voltage_max_v = _voltage_max_v.get();
    const float pressure_min_kpa = _pressure_min_kpa.get();
    const float pressure_max_kpa = _pressure_max_kpa.get();
    if (!isfinite(voltage_min_v) || !isfinite(voltage_max_v) ||
        voltage_max_v <= voltage_min_v ||
        !isfinite(pressure_min_kpa) || !isfinite(pressure_max_kpa)) {
        return false;
    }

    // 仅拒绝明显超出标称电气范围的值；R95 不安装时不能据此可靠判断开路。
    return voltage_v >= MAX(0.0f, voltage_min_v - VOLTAGE_MARGIN_V) &&
           voltage_v <= MIN(3.3f, voltage_max_v + VOLTAGE_MARGIN_V);
}

// ==================== 50Hz ADC 采样与状态更新 ====================

void AP_SuctionPressure::update()
{
    if (!enabled()) {
        set_status(Status::DISABLED);
        return;
    }
    if (_source == nullptr) {
        set_status(Status::NO_SOURCE);
        return;
    }

    const uint32_t now = AP_HAL::millis();
    // voltage_average() 返回 ADC 累计平均值，单位 V；AP_HAL 会在读取后
    // 清理累计窗口，因此该函数每次 update() 消费一个新的平均采样窗口。
    const float voltage_v = _source->voltage_average();
    _last_update_ms = now;
    _voltage_v = voltage_v;

    if (!voltage_valid(voltage_v)) {
        // 无效电压进入消抖窗口后立即从上层健康数据中撤销；连续约 200ms
        // 后保留 INVALID_VOLTAGE 状态。这样单次毛刺不会污染滤波值，但也不会
        // 让上层在无效采样期间继续使用上一帧压力。
        _valid_start_ms = 0;
        if (_invalid_start_ms == 0) {
            _invalid_start_ms = now;
        }
        if (now - _invalid_start_ms >= INVALID_DELAY_MS) {
            set_status(Status::INVALID_VOLTAGE);
        }
        return;
    }

    _invalid_start_ms = 0;
    // 线性标定：V_MIN/P_MIN 与 V_MAX/P_MAX 定义两个换算端点，结果单位 kPa。
    const float pressure_kpa = _pressure_min_kpa.get() +
        (voltage_v - _voltage_min_v.get()) *
        (_pressure_max_kpa.get() - _pressure_min_kpa.get()) /
        (_voltage_max_v.get() - _voltage_min_v.get());
    // 上层状态机、遥测和日志统一使用滤波后的压力，单位 kPa；原始电压
    // 仍单独保留在 _voltage_v 中，便于诊断 ADC 和传感器输出。
    _pressure_kpa = _pressure_filter.apply(pressure_kpa);

    // 连续有效约 100ms 后才对上层发布 HEALTHY，避免上电瞬间的单个有效
    // 样本直接被吸盘状态机当成可信数据。
    if (_valid_start_ms == 0) {
        _valid_start_ms = now;
        set_status(Status::WARMING_UP);
    } else if (now - _valid_start_ms >= HEALTHY_DELAY_MS) {
        set_status(Status::HEALTHY);
    }
}

// ==================== 状态与数据读取接口 ====================

AP_SuctionPressure::Status AP_SuctionPressure::status() const
{
    // 无效采样消抖期间也不能继续对外报告 HEALTHY；否则吸盘可能读取到
    // 上一帧有效压力并错误地完成吸附判定。
    if (enabled() && _invalid_start_ms != 0) {
        return Status::INVALID_VOLTAGE;
    }

    // 即使内部最近状态为 HEALTHY，调度停止超过 250ms 后也必须报告 STALE；
    // 这是动态返回值，不修改 _status，恢复采样后由 update() 重新推进状态。
    if (enabled() && _source != nullptr && _last_update_ms != 0 &&
        AP_HAL::millis() - _last_update_ms > STALE_TIMEOUT_MS) {
        return Status::STALE;
    }
    return _status;
}

bool AP_SuctionPressure::healthy() const
{
    return status() == Status::HEALTHY;
}

bool AP_SuctionPressure::get_pressure_kpa(float &pressure_kpa) const
{
    // 禁止上层继续使用无效或过期的旧压力值。
    if (!healthy()) {
        return false;
    }
    pressure_kpa = _pressure_kpa;
    return true;
}

bool AP_SuctionPressure::get_voltage_v(float &voltage_v) const
{
    // 与压力接口保持一致，仅发布健康且未过期的采样值。
    if (!healthy()) {
        return false;
    }
    voltage_v = _voltage_v;
    return true;
}

void AP_SuctionPressure::set_status(const Status status)
{
    _status = status;
}

// ==================== 10Hz DataFlash 日志接口 ====================

void AP_SuctionPressure::log()
{
#if HAL_LOGGING_ENABLED
    // SPRS.Volt 为 V，SPRS.Press 数值为 kPa，Health/Status 用于判定有效性。
    // 即使当前状态无效，也记录最近一次原始值和换算值，便于分析断线、
    // 越界电压、供电波动和压力建立过程。
    AP::logger().WriteStreaming("SPRS", "TimeUS,Volt,Press,Health,Status",
                                "sv---", "F0000", "QffBB",
                                AP_HAL::micros64(),
                                static_cast<double>(_voltage_v),
                                static_cast<double>(_pressure_kpa),
                                healthy() ? 1U : 0U,
                                uint8_t(status()));
#endif
}

// ==================== 2Hz Mission Planner Status 接口 ====================

void AP_SuctionPressure::send_status()
{
    // 无健康压力时发送 NaN，避免 SUCT_PKPA 显示为仍然有效的历史值；
    // SUCT_HLT 同时提供明确的 1/0 健康标志，供 Mission Planner Status 页面
    // 和后续上层监控逻辑使用。
    float pressure_kpa = NAN;
    const bool pressure_healthy = get_pressure_kpa(pressure_kpa);
    gcs().send_named_float("SUCT_PKPA", pressure_healthy ? pressure_kpa : NAN);
    gcs().send_named_float("SUCT_HLT", pressure_healthy ? 1.0f : 0.0f);

    if (_health_report_initialized && _last_reported_healthy && !pressure_healthy) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Suction pressure sensor unhealthy");
    }
    _last_reported_healthy = pressure_healthy;
    _health_report_initialized = true;
}

namespace AP {

// 全局访问入口，供 AP_SuctionCup 及未来上层业务读取压力模块；调用方仍须
// 通过 get_pressure_kpa() 检查数据健康，不应直接暴露内部缓存值。
AP_SuctionPressure &suction_pressure()
{
    return *AP_SuctionPressure::get_singleton();
}

}
