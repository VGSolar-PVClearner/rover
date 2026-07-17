#include "AP_ESC_2BLD6010.h"

#if AP_ESC_2BLD6010_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

/*
 * AP_ESC_2BLD6010 实现：在同一条 RS-485/Modbus RTU 总线上轮询最多四台电调。
 * 配置参数先整体校验，再一次性写入运行状态；任一启用参数非法时驱动不会部分启动。
 * 每台电调使用固定的 AP_ESC_Telem 槽位，并独立维护遥测、健康状态和通信统计。
 */

// BESC_* 参数表。以下 @Param 元数据保持 ArduPilot 标准英文格式，便于参数文档工具解析。
const AP_Param::GroupInfo AP_ESC_2BLD6010::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: 2BLD6010 telemetry enable
    // @Description: Enable read-only Modbus telemetry from 2BLD6010 motor controllers. Only 0 and 1 are valid. Reboot required after changing.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_ESC_2BLD6010, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: NUM
    // @DisplayName: Number of 2BLD6010 controllers
    // @Description: Number of uniquely addressed motor controllers on the RS-485 bus. All enabled addresses must be valid and unique.
    // @Range: 1 4
    // @Increment: 1
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("NUM", 2, AP_ESC_2BLD6010, _num_escs, 1),

    // @Param: ADDR1
    // @DisplayName: First 2BLD6010 Modbus address
    // @Description: Modbus unicast slave address mapped to the first ESC telemetry slot.
    // @Range: 1 247
    // @Increment: 1
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("ADDR1", 3, AP_ESC_2BLD6010, _address_params[0], 2),

    // @Param: ADDR2
    // @DisplayName: Second 2BLD6010 Modbus address
    // @Description: Modbus unicast slave address mapped to the second ESC telemetry slot. It must differ from all other enabled addresses.
    // @Range: 1 247
    // @Increment: 1
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("ADDR2", 4, AP_ESC_2BLD6010, _address_params[1], 1),

    // @Param: OFS
    // @DisplayName: ESC telemetry index offset
    // @Description: First AP_ESC_Telem index used by the 2BLD6010 telemetry instances. The complete configured range must fit in AP_ESC_Telem.
    // @Range: 0 31
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("OFS", 5, AP_ESC_2BLD6010, _esc_offset, 0),

    // @Param: RATE
    // @DisplayName: 2BLD6010 polling rate
    // @Description: Requested telemetry polling rate for each configured motor controller. Actual rate is limited by the scheduler and device response time.
    // @Units: Hz
    // @Range: 1 20
    // @Increment: 1
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("RATE", 6, AP_ESC_2BLD6010, _rate_hz, 10),

    // @Param: TIMEOUT
    // @DisplayName: 2BLD6010 health timeout
    // @Description: Maximum age of the last valid telemetry response before an instance is unhealthy. This does not change the fixed Modbus response timeout.
    // @Units: ms
    // @Range: 100 5000
    // @Increment: 10
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT", 7, AP_ESC_2BLD6010, _timeout_ms, 500),

    // @Param: ADDR3
    // @DisplayName: Third 2BLD6010 Modbus address
    // @Description: Modbus unicast slave address mapped to the third ESC telemetry slot. It must differ from all other enabled addresses.
    // @Range: 1 247
    // @Increment: 1
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("ADDR3", 8, AP_ESC_2BLD6010, _address_params[2], 3),

    // @Param: ADDR4
    // @DisplayName: Fourth 2BLD6010 Modbus address
    // @Description: Modbus unicast slave address mapped to the fourth ESC telemetry slot. It must differ from all other enabled addresses.
    // @Range: 1 247
    // @Increment: 1
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("ADDR4", 9, AP_ESC_2BLD6010, _address_params[3], 4),

    AP_GROUPEND
};

// 构造驱动对象并加载 BESC_* 参数默认值，实际串口初始化由 init() 完成。
AP_ESC_2BLD6010::AP_ESC_2BLD6010() :
    _uart(nullptr),
    _configured_count(0),
    _validated_esc_offset(0),
    _validated_rate_hz(0),
    _health_timeout_ms(0),
    _poll_interval_ms(0),
    _config_error(ConfigError::NONE),
    _current_instance(0),
    _requested_address(0),
    _state(State::IDLE),
    _request_sent_ms(0),
    _next_request_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// 返回 BESC_ENABLE 是否为合法启用值 1。
bool AP_ESC_2BLD6010::enabled() const
{
    return _enable.get() == 1;
}

// 返回通过完整配置验证并已写入运行态的电调数量。
uint8_t AP_ESC_2BLD6010::configured_count() const
{
    return _configured_count;
}

// 返回最近一次 init() 保存的配置校验结果，供诊断接口读取。
AP_ESC_2BLD6010::ConfigError AP_ESC_2BLD6010::config_error() const
{
    return _config_error;
}

// 停止当前轮询并清空所有实例运行状态，为禁用、失败或重复初始化建立安全起点。
void AP_ESC_2BLD6010::reset_runtime_state()
{
    // init() 允许重复调用：先停止上一轮轮询并清除全部运行态，但不修改 AP_Param 参数值。
    if (_uart != nullptr) {
        _uart->discard_input();
    }
    _uart = nullptr;
    _configured_count = 0;
    _validated_esc_offset = 0;
    _validated_rate_hz = 0;
    _health_timeout_ms = 0;
    _poll_interval_ms = 0;
    _current_instance = 0;
    _requested_address = 0;
    _state = State::IDLE;
    _request_sent_ms = 0;
    _next_request_ms = 0;
    _response_stream.reset();
    for (InstanceState &instance : _instances) {
        instance = {};
    }
}

// 将 AP_Param 原始值复制到宽类型配置结构，供统一范围和重复性校验。
AP_ESC_2BLD6010::RawConfig AP_ESC_2BLD6010::get_raw_config() const
{
    // 先提升为 int32_t，再执行范围校验，避免负数提前转换成无符号值或窄整数溢出。
    RawConfig raw {};
    raw.enable = int32_t(_enable.get());
    raw.count = int32_t(_num_escs.get());
    raw.esc_offset = int32_t(_esc_offset.get());
    raw.rate_hz = int32_t(_rate_hz.get());
    raw.health_timeout_ms = int32_t(_timeout_ms.get());
    for (uint8_t i = 0; i < MAX_ESC_COUNT; i++) {
        raw.addresses[i] = int32_t(_address_params[i].get());
    }
    return raw;
}

// 将已验证配置一次性应用到四实例运行数组，不接受部分有效配置。
void AP_ESC_2BLD6010::apply_config(const ValidatedConfig &config)
{
    // 只有 validate_config() 全部通过后才调用；一次性清空并提交四个实例的运行配置。
    for (InstanceState &instance : _instances) {
        instance = {};
    }
    _configured_count = config.count;
    _validated_esc_offset = config.esc_offset;
    _validated_rate_hz = config.rate_hz;
    _health_timeout_ms = config.health_timeout_ms;
    _poll_interval_ms = config.poll_interval_ms;
    for (uint8_t i = 0; i < _configured_count; i++) {
        _instances[i].address = config.addresses[i];
    }
}

// 在初始化阶段向地面站发送一次配置错误编号，避免高频路径重复提示。
void AP_ESC_2BLD6010::report_config_error() const
{
    if (_config_error == ConfigError::NONE) {
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "2BLD6010: invalid config (%u)", unsigned(_config_error));
}

// 验证全部 BESC_* 参数、应用实例配置，并在最后启动 protocol=51 的串口。
void AP_ESC_2BLD6010::init()
{
    // 初始化顺序固定为：安全停止 -> 完整验证 -> 原子应用 -> 最后启动 UART。
    reset_runtime_state();
    _config_error = ConfigError::NONE;

    const RawConfig raw = get_raw_config();
    ValidatedConfig config {};
    _config_error = validate_config(raw, ESC_TELEM_MAX_ESCS, config);
    if (_config_error != ConfigError::NONE) {
        // 配置错误仅在初始化阶段提示一次，update() 高频路径不会重复打印。
        report_config_error();
        return;
    }
    if (!config.enabled) {
        // ENABLE=0 是正常禁用，不属于配置错误。
        return;
    }

    apply_config(config);

    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ESC_2BLD6010, 0);
    if (_uart == nullptr) {
        return;
    }

    uint32_t baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_ESC_2BLD6010, 0);
    if (baudrate == 0) {
        baudrate = 115200;
    }
    _uart->begin(baudrate, 128, 64);
    _uart->discard_input();
    const uint32_t now_ms = AP_HAL::millis();
    _next_request_ms = now_ms;
    for (uint8_t i = 0; i < _configured_count; i++) {
        _instances[i].health_check_start_ms = now_ms;
    }
}

// 根据指定实例最后一帧合法遥测的时间判断当前通信健康状态。
bool AP_ESC_2BLD6010::healthy(uint8_t instance) const
{
    if (!enabled() || _config_error != ConfigError::NONE || _uart == nullptr || instance >= _configured_count) {
        return false;
    }
    const InstanceState &state = _instances[instance];
    // 请求成功发送不代表健康；必须至少收到一帧合法遥测，且数据未超过 BESC_TIMEOUT。
    return data_is_healthy(AP_HAL::millis(), state.data.last_update_ms, _health_timeout_ms, state.has_valid_data);
}

// 在实例通信健康时判断设备原始故障码是否非零。
bool AP_ESC_2BLD6010::has_fault(uint8_t instance) const
{
    return healthy(instance) && _instances[instance].data.fault_code != 0;
}

// 复制指定实例最近一次合法遥测；尚无有效数据或索引越界时返回 false。
bool AP_ESC_2BLD6010::get_data(uint8_t instance, Data &out) const
{
    if (instance >= _configured_count || !_instances[instance].has_valid_data) {
        return false;
    }
    out = _instances[instance].data;
    return true;
}

// 由 Rover 100Hz 调度器调用，推进非阻塞发送、接收、超时和健康检查状态机。
void AP_ESC_2BLD6010::update()
{
    if (!enabled() || _config_error != ConfigError::NONE || _uart == nullptr || _configured_count == 0) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    // 每台设备独立检查健康状态，某一实例离线不会改变其他实例的时间戳和统计。
    for (uint8_t i = 0; i < _configured_count; i++) {
        update_health_log(i, now_ms);
    }
    if (_state == State::IDLE) {
        // IDLE 仅在轮询期限到达时发送，不会在每次 100Hz 调度中无条件查询。
        if (time_reached(now_ms, _next_request_ms)) {
            send_request(now_ms);
        }
        return;
    }

    read_response(now_ms);
    if (_state == State::WAITING_RESPONSE && now_ms - _request_sent_ms >= RESPONSE_TIMEOUT_MS) {
        // 固定 40ms 是单次 Modbus 响应超时，与 BESC_TIMEOUT 遥测健康超时相互独立。
        _instances[_current_instance].data.timeout_count++;
        update_health_log(_current_instance, now_ms);
        advance_instance();
    }
}

// 向当前 round-robin 实例发送一次只读 Modbus 请求并建立 outstanding request。
void AP_ESC_2BLD6010::send_request(uint32_t now_ms)
{
    if (_current_instance >= _configured_count) {
        _current_instance = 0;
    }

    uint8_t request[REQUEST_LENGTH];
    InstanceState &instance = _instances[_current_instance];
    // outstanding request 绑定当前数组槽位和 expected_address，响应不能转发给其他实例。
    _requested_address = instance.address;
    build_request(_requested_address, request);

    if (_uart->txspace() < REQUEST_LENGTH) {
        _next_request_ms = now_ms + MIN_POLL_INTERVAL_MS;
        return;
    }

    _uart->discard_input();
    _response_stream.reset();
    // 仅在新请求发送前清理残留 RX；等待响应期间不会清空缓存，因此可接收跨周期半帧。
    if (_uart->write(request, REQUEST_LENGTH) != REQUEST_LENGTH) {
        instance.data.bad_response_count++;
        _next_request_ms = now_ms + _poll_interval_ms;
        advance_instance();
        return;
    }

    instance.last_request_ms = now_ms;
    _request_sent_ms = now_ms;
    // 下一请求期限从本次发送起点计算，避免把设备响应耗时重复叠加到轮询间隔。
    _next_request_ms = now_ms + _poll_interval_ms;
    _state = State::WAITING_RESPONSE;
}

// 非阻塞读取 UART 可用字节，将数据追加到流式缓存并尝试解析完整响应。
void AP_ESC_2BLD6010::read_response(uint32_t now_ms)
{
    uint8_t bytes[RX_BUFFER_SIZE];
    uint8_t length = 0;
    uint16_t available = MIN(_uart->available(), uint32_t(RX_BUFFER_SIZE - _response_stream.buffered_length()));
    while (available-- > 0 && length < ARRAY_SIZE(bytes)) {
        const int16_t value = _uart->read();
        if (value < 0) {
            break;
        }
        bytes[length++] = uint8_t(value);
    }
    if (length > 0 && !_response_stream.append(bytes, length)) {
        _instances[_current_instance].data.bad_response_count++;
        advance_instance();
        return;
    }
    process_rx_buffer(now_ms);
}

// 处理当前实例的缓存帧，按解析结果更新统计并决定继续等待或切换实例。
bool AP_ESC_2BLD6010::process_rx_buffer(uint32_t now_ms)
{
    InstanceState &instance = _instances[_current_instance];
    // 一次只处理当前 outstanding request；所有计数均归属当前实例。
    while (true) {
        const ParseResult result = _response_stream.next(_requested_address, instance.data);
        switch (result) {
        case ParseResult::VALID:
            // 只有完整合法响应才能刷新遥测、成功时间和健康状态。
            handle_valid_response(_current_instance, now_ms);
            advance_instance();
            return true;
        case ParseResult::MODBUS_EXCEPTION:
            // 异常帧、CRC 错误和确定的结构错误均结束本轮，避免故障设备长期占用总线。
            instance.data.modbus_exception_count++;
            advance_instance();
            return true;
        case ParseResult::CRC_ERROR:
            instance.data.crc_error_count++;
            advance_instance();
            return true;
        case ParseResult::BAD_RESPONSE:
            instance.data.bad_response_count++;
            advance_instance();
            return true;
        case ParseResult::ADDRESS_MISMATCH:
            // 迟到帧或其他设备帧只丢弃，不切换实例，继续等待当前 expected_address。
            instance.data.bad_response_count++;
            break;
        case ParseResult::INCOMPLETE:
            return false;
        }
    }
}

// 提交合法响应，刷新实例成功状态并发布标准 AP_ESC_Telem 遥测。
void AP_ESC_2BLD6010::handle_valid_response(uint8_t instance_index, uint32_t now_ms)
{
    if (instance_index >= _configured_count) {
        return;
    }
    InstanceState &instance = _instances[instance_index];
    Data &data = instance.data;
    data.last_update_ms = now_ms;
    data.success_count++;
    instance.has_valid_data = true;

    AP_ESC_Telem_Backend::TelemetryData telem {};
    telem.temperature_cdeg = data.temperature_cdeg;
    telem.voltage = data.voltage_v;
    telem.current = data.current_a;
    const uint16_t mask = AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
                          AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
                          AP_ESC_Telem_Backend::TelemetryType::CURRENT;
    // 映射由参数槽位决定：ADDRn 固定发布到 BESC_OFS + (n-1)，不按 Modbus 地址排序。
    const uint8_t esc_index = _validated_esc_offset + instance_index;
    update_telem_data(esc_index, telem, mask);
    update_rpm(esc_index, data.rpm, 0.0f);
    update_health_log(instance_index, now_ms);
    write_log(instance_index, now_ms, true, false);
}

// 结束当前 outstanding request，并按固定参数槽位切换到下一实例。
void AP_ESC_2BLD6010::advance_instance()
{
    // 所有成功和失败出口都通过这里集中执行 round-robin，防止不同分支切换行为不一致。
    _current_instance = next_instance(_current_instance, _configured_count);
    _requested_address = 0;
    _request_sent_ms = 0;
    _state = State::IDLE;
    _response_stream.reset();
}

// 检查单个实例的健康状态变化，并在首次状态或状态翻转时记录日志。
void AP_ESC_2BLD6010::update_health_log(uint8_t instance_index, uint32_t now_ms)
{
    if (instance_index >= _configured_count) {
        return;
    }
    InstanceState &instance = _instances[instance_index];
    // 首帧到达前保持不健康，但等待一个 BESC_TIMEOUT 周期后才记录首次不健康状态。
    if (!instance.has_valid_data && now_ms - instance.health_check_start_ms < _health_timeout_ms) {
        return;
    }
    const bool healthy_state = data_is_healthy(now_ms, instance.data.last_update_ms, _health_timeout_ms, instance.has_valid_data);
    if (!health_transition_due(healthy_state, instance.logged_health_valid, instance.last_logged_healthy)) {
        return;
    }
    write_log(instance_index, now_ms, healthy_state, true);
}

// 写入低频 BESC 通信日志，记录地址、健康状态、设备状态和各类计数。
void AP_ESC_2BLD6010::write_log(uint8_t instance_index, uint32_t now_ms, bool healthy_state, bool force)
{
#if HAL_LOGGING_ENABLED
    if (instance_index >= _configured_count) {
        return;
    }
    InstanceState &instance = _instances[instance_index];
    // 正常遥测日志按 100ms 限频；健康状态变化通过 force 立即记录一次。
    if (!force && now_ms - instance.last_log_ms < 100) {
        return;
    }
    instance.last_log_ms = now_ms;
    const Data &data = instance.data;
    AP::logger().WriteStreaming("BESC", "TimeUS,Inst,Addr,Healthy,Dir,Fault,Hall,Success,CRCErr,Timeout,BadResp,Except",
                                "QBBBBHIIIIII",
                                AP_HAL::micros64(),
                                instance_index,
                                instance.address,
                                uint8_t(healthy_state),
                                data.direction,
                                data.fault_code,
                                data.hall_count,
                                data.success_count,
                                data.crc_error_count,
                                data.timeout_count,
                                data.bad_response_count,
                                data.modbus_exception_count);
#else
    (void)instance_index;
    (void)now_ms;
    (void)healthy_state;
    (void)force;
#endif
}

#endif
