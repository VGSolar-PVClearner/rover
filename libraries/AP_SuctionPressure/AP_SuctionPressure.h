#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter.h>

/**
 * @brief VGSolar 吸盘模拟压力传感器前端。
 *
 * 负责 ADC 采样、电压有效性判断、压力换算、低通滤波、健康状态维护、
 * DataFlash 日志和 Mission Planner Status 遥测。该压力不接入 AP_Baro，
 * 不参与 EKF 或高度估算。当前硬件为 PC0/ADC1/analog pin 10，默认量程
 * 为 MCP-H10-P 3.3V 负压表压模块的 0~-100kPa、输出 0.1~3.1V。
 * R95 上拉电阻不安装，因此有效电压不能等价地表示传感器已连接，软件
 * 只能检测明显无效、过期或越界数据。
 */
class AP_SuctionPressure {
public:
    AP_SuctionPressure();

    CLASS_NO_COPY(AP_SuctionPressure);

    // ==================== 状态定义 ====================

    /**
     * @brief 压力传感器运行状态。
     *
     * status() 会根据最近一次采样时间动态返回 STALE；调用方不应仅
     * 通过内部历史状态推断数据仍然有效。
     */
    enum class Status : uint8_t {
        DISABLED = 0,   ///< SPRESS_PIN=-1，压力功能已禁用
        NO_SOURCE,      ///< 已配置引脚，但 HAL 未提供模拟输入源
        WARMING_UP,     ///< 正在累计连续有效样本，尚未达到健康时间
        HEALTHY,        ///< 数据连续有效且最后更新时间未超时
        INVALID_VOLTAGE, ///< 电压或标定参数连续无效达到判定时间
        STALE,          ///< 超过规定时间没有新的 ADC 更新
    };

    /**
     * @brief 获取模块单例指针。
     * @return Rover ParametersG2 持有的唯一实例；调用方不应自行创建第二个实例。
     */
    static AP_SuctionPressure *get_singleton() { return _singleton; }

    /** AP_Param 参数元数据，由 Rover 以 SPRESS_ 前缀注册。 */
    static const struct AP_Param::GroupInfo var_info[];

    // ==================== 生命周期与调度接口 ====================

    /**
     * @brief 初始化模拟输入源、滤波器和健康状态。
     * @note 由 Rover::init_ardupilot() 在系统启动阶段调用一次，且应在
     * AP_HAL AnalogIn 初始化后调用。该函数不等待传感器达到有效压力。
     */
    void init();

    /**
     * @brief 将完全匹配旧 XGZP 默认端点的已保存参数迁移到 MCP-H10。
     * @return true 表示四个端点均已保存为 MCP-H10 默认值；用户自定义标定
     * 或压力模块禁用时返回 false，且不修改任何参数。
     */
    bool migrate_legacy_calibration();

    /**
     * @brief 读取 ADC、换算并滤波压力，同时更新健康状态。
     * @note 由 Rover 调度器以 50Hz 调用。函数使用 ADC 平均电压，完成
     * 有效性判断、线性换算和健康计时，不直接改变吸盘输出。
     */
    void update();

    /**
     * @brief 写入 SPRS DataFlash 日志。
     * @note 由 Rover 调度器以 10Hz 调用；Volt 单位 V，Press 单位 kPa，
     * Health 和 Status 用于回放时判断采样是否可用。
     */
    void log();

    /**
     * @brief 发送 SUCT_PKPA 和 SUCT_HLT 到 Mission Planner Status。
     * @note 由 Rover 调度器以 2Hz 调用，字段出现在 Mission Planner
     * Flight Data -> Status 列表；数据无效或过期时 SUCT_PKPA 发送 NaN，
     * SUCT_HLT 发送 0。
     */
    void send_status();

    // ==================== 上层数据读取接口 ====================

    /**
     * @brief 判断压力功能是否已通过 SPRESS_PIN 启用。
     * @return true 表示已配置非负 ADC 引脚，不代表数据健康。
     */
    bool enabled() const;

    /**
     * @brief 判断当前压力数据是否健康且未过期。
     * @return true 仅对应 Status::HEALTHY；DISABLED、NO_SOURCE、
     * WARMING_UP、INVALID_VOLTAGE 和 STALE 均返回 false。
     */
    bool healthy() const;

    /**
     * @brief 获取当前滤波后的吸盘压力。
     * @param[out] pressure_kpa 输出压力，单位 kPa。
     * @return 仅在数据健康且未过期时返回 true；失败时不修改输出参数，
     * 上层业务不得把上一次历史值当成新压力使用。
     */
    bool get_pressure_kpa(float &pressure_kpa) const;

    /**
     * @brief 获取最近一次 ADC 采样得到的原始平均电压。
     * @param[out] voltage_v 输出电压，单位 V。
     * @return 仅在数据健康且未过期时返回 true；失败时不修改输出参数。
     */
    bool get_voltage_v(float &voltage_v) const;

    /**
     * @brief 获取当前运行状态。
     * @return 包含动态超时判断后的状态；更新超过 250ms 时返回 STALE。
     */
    Status status() const;

    /**
     * @brief 获取最近一次执行 ADC 读取的系统时间。
     * @return AP_HAL 毫秒时间戳，单位 ms；尚未更新时为 0。该时间表示
     * 最近一次采样尝试，不保证采样值有效；有效性应通过 healthy() 或
     * get_pressure_kpa() 判断。
     */
    uint32_t last_update_ms() const { return _last_update_ms; }

private:
    // ==================== 单例与状态判定常量 ====================

    static AP_SuctionPressure *_singleton;

    static constexpr uint32_t HEALTHY_DELAY_MS = 100;
    static constexpr uint32_t INVALID_DELAY_MS = 200;
    static constexpr uint32_t STALE_TIMEOUT_MS = 250;
    static constexpr float SAMPLE_RATE_HZ = 50.0f;
    static constexpr float VOLTAGE_MARGIN_V = 0.1f;

    // ==================== SPRESS_ 参数 ====================

    AP_Int16 _pin;
    AP_Float _voltage_min_v;
    AP_Float _voltage_max_v;
    AP_Float _pressure_min_kpa;
    AP_Float _pressure_max_kpa;
    AP_Float _filter_hz;

    // ==================== 采样、滤波与健康状态 ====================

    AP_HAL::AnalogSource *_source;
    LowPassFilterConstDtFloat _pressure_filter;
    Status _status;
    float _voltage_v;
    float _pressure_kpa;
    uint32_t _last_update_ms;
    uint32_t _valid_start_ms;
    uint32_t _invalid_start_ms;
    bool _last_reported_healthy;
    bool _health_report_initialized;

    // ==================== 内部辅助接口 ====================

    /**
     * @brief 检查 ADC 电压和标定范围是否可用于压力换算。
     * @note R95 不安装后，本检查不能可靠识别所有传感器开路情况。
     */
    bool voltage_valid(float voltage_v) const;

    /** @brief 更新内部健康状态。 */
    void set_status(Status status);
};

namespace AP {
    /**
     * @brief 获取全局吸盘压力模块引用。
     * @return Rover ParametersG2 中构造的 AP_SuctionPressure 单例。
     */
    AP_SuctionPressure &suction_pressure();
}
