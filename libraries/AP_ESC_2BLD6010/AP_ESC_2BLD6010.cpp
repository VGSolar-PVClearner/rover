#include "AP_ESC_2BLD6010.h"

#if AP_ESC_2BLD6010_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

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

bool AP_ESC_2BLD6010::enabled() const
{
    return _enable.get() == 1;
}

uint8_t AP_ESC_2BLD6010::configured_count() const
{
    return _configured_count;
}

AP_ESC_2BLD6010::ConfigError AP_ESC_2BLD6010::config_error() const
{
    return _config_error;
}

void AP_ESC_2BLD6010::reset_runtime_state()
{
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

AP_ESC_2BLD6010::RawConfig AP_ESC_2BLD6010::get_raw_config() const
{
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

void AP_ESC_2BLD6010::apply_config(const ValidatedConfig &config)
{
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

void AP_ESC_2BLD6010::report_config_error() const
{
    if (_config_error == ConfigError::NONE) {
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "2BLD6010: invalid config (%u)", unsigned(_config_error));
}

void AP_ESC_2BLD6010::init()
{
    reset_runtime_state();
    _config_error = ConfigError::NONE;

    const RawConfig raw = get_raw_config();
    ValidatedConfig config {};
    _config_error = validate_config(raw, ESC_TELEM_MAX_ESCS, config);
    if (_config_error != ConfigError::NONE) {
        report_config_error();
        return;
    }
    if (!config.enabled) {
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

bool AP_ESC_2BLD6010::healthy(uint8_t instance) const
{
    if (!enabled() || _config_error != ConfigError::NONE || _uart == nullptr || instance >= _configured_count) {
        return false;
    }
    const InstanceState &state = _instances[instance];
    return data_is_healthy(AP_HAL::millis(), state.data.last_update_ms, _health_timeout_ms, state.has_valid_data);
}

bool AP_ESC_2BLD6010::has_fault(uint8_t instance) const
{
    return healthy(instance) && _instances[instance].data.fault_code != 0;
}

bool AP_ESC_2BLD6010::get_data(uint8_t instance, Data &out) const
{
    if (instance >= _configured_count || !_instances[instance].has_valid_data) {
        return false;
    }
    out = _instances[instance].data;
    return true;
}

void AP_ESC_2BLD6010::update()
{
    if (!enabled() || _config_error != ConfigError::NONE || _uart == nullptr || _configured_count == 0) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    for (uint8_t i = 0; i < _configured_count; i++) {
        update_health_log(i, now_ms);
    }
    if (_state == State::IDLE) {
        if (time_reached(now_ms, _next_request_ms)) {
            send_request(now_ms);
        }
        return;
    }

    read_response(now_ms);
    if (_state == State::WAITING_RESPONSE && now_ms - _request_sent_ms >= RESPONSE_TIMEOUT_MS) {
        _instances[_current_instance].data.timeout_count++;
        update_health_log(_current_instance, now_ms);
        advance_instance();
    }
}

void AP_ESC_2BLD6010::send_request(uint32_t now_ms)
{
    if (_current_instance >= _configured_count) {
        _current_instance = 0;
    }

    uint8_t request[REQUEST_LENGTH];
    InstanceState &instance = _instances[_current_instance];
    _requested_address = instance.address;
    build_request(_requested_address, request);

    if (_uart->txspace() < REQUEST_LENGTH) {
        _next_request_ms = now_ms + MIN_POLL_INTERVAL_MS;
        return;
    }

    _uart->discard_input();
    _response_stream.reset();
    if (_uart->write(request, REQUEST_LENGTH) != REQUEST_LENGTH) {
        instance.data.bad_response_count++;
        _next_request_ms = now_ms + _poll_interval_ms;
        advance_instance();
        return;
    }

    instance.last_request_ms = now_ms;
    _request_sent_ms = now_ms;
    _next_request_ms = now_ms + _poll_interval_ms;
    _state = State::WAITING_RESPONSE;
}

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

bool AP_ESC_2BLD6010::process_rx_buffer(uint32_t now_ms)
{
    InstanceState &instance = _instances[_current_instance];
    while (true) {
        const ParseResult result = _response_stream.next(_requested_address, instance.data);
        switch (result) {
        case ParseResult::VALID:
            handle_valid_response(_current_instance, now_ms);
            advance_instance();
            return true;
        case ParseResult::MODBUS_EXCEPTION:
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
            instance.data.bad_response_count++;
            break;
        case ParseResult::INCOMPLETE:
            return false;
        }
    }
}

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
    const uint8_t esc_index = _validated_esc_offset + instance_index;
    update_telem_data(esc_index, telem, mask);
    update_rpm(esc_index, data.rpm, 0.0f);
    update_health_log(instance_index, now_ms);
    write_log(instance_index, now_ms, true, false);
}

void AP_ESC_2BLD6010::advance_instance()
{
    _current_instance = next_instance(_current_instance, _configured_count);
    _requested_address = 0;
    _request_sent_ms = 0;
    _state = State::IDLE;
    _response_stream.reset();
}

void AP_ESC_2BLD6010::update_health_log(uint8_t instance_index, uint32_t now_ms)
{
    if (instance_index >= _configured_count) {
        return;
    }
    InstanceState &instance = _instances[instance_index];
    if (!instance.has_valid_data && now_ms - instance.health_check_start_ms < _health_timeout_ms) {
        return;
    }
    const bool healthy_state = data_is_healthy(now_ms, instance.data.last_update_ms, _health_timeout_ms, instance.has_valid_data);
    if (!health_transition_due(healthy_state, instance.logged_health_valid, instance.last_logged_healthy)) {
        return;
    }
    write_log(instance_index, now_ms, healthy_state, true);
}

void AP_ESC_2BLD6010::write_log(uint8_t instance_index, uint32_t now_ms, bool healthy_state, bool force)
{
#if HAL_LOGGING_ENABLED
    if (instance_index >= _configured_count) {
        return;
    }
    InstanceState &instance = _instances[instance_index];
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
