#include "AP_2BLD6010.h"

#if AP_2BLD6010_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>

const AP_Param::GroupInfo AP_2BLD6010::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: 2BLD6010 telemetry enable
    // @Description: Enable read-only Modbus telemetry from 2BLD6010 motor controllers. Reboot required after changing.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_2BLD6010, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: NUM
    // @DisplayName: Number of 2BLD6010 controllers
    // @Description: Number of uniquely addressed motor controllers on the RS-485 bus.
    // @Range: 1 2
    // @Increment: 1
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("NUM", 2, AP_2BLD6010, _num_escs, 1),

    // @Param: ADDR1
    // @DisplayName: First 2BLD6010 Modbus address
    // @Description: Modbus slave address of the first motor controller.
    // @Range: 1 247
    // @Increment: 1
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("ADDR1", 3, AP_2BLD6010, _address1, 2),

    // @Param: ADDR2
    // @DisplayName: Second 2BLD6010 Modbus address
    // @Description: Modbus slave address of the second motor controller. It must differ from ADDR1.
    // @Range: 1 247
    // @Increment: 1
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("ADDR2", 4, AP_2BLD6010, _address2, 1),

    // @Param: OFS
    // @DisplayName: ESC telemetry index offset
    // @Description: First AP_ESC_Telem index used by the 2BLD6010 telemetry instances.
    // @Range: 0 31
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("OFS", 5, AP_2BLD6010, _esc_offset, 0),

    // @Param: RATE
    // @DisplayName: 2BLD6010 polling rate
    // @Description: Requested telemetry polling rate for each configured motor controller.
    // @Units: Hz
    // @Range: 1 20
    // @Increment: 1
    // @RebootRequired: False
    // @User: Standard
    AP_GROUPINFO("RATE", 6, AP_2BLD6010, _rate_hz, 10),

    // @Param: TIMEOUT
    // @DisplayName: 2BLD6010 health timeout
    // @Description: Maximum age of the last valid response before an instance is unhealthy.
    // @Units: ms
    // @Range: 100 5000
    // @Increment: 10
    // @RebootRequired: False
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT", 7, AP_2BLD6010, _timeout_ms, 500),

    AP_GROUPEND
};

AP_2BLD6010::AP_2BLD6010() :
    _uart(nullptr),
    _configured_count(0),
    _current_instance(0),
    _requested_address(0),
    _state(State::IDLE),
    _request_sent_ms(0),
    _next_request_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_2BLD6010::enabled() const
{
    return _enable.get() != 0;
}

uint8_t AP_2BLD6010::configured_count() const
{
    return _configured_count;
}

void AP_2BLD6010::configure()
{
    _configured_count = constrain_int16(_num_escs.get(), 1, MAX_INSTANCES);
    _addresses[0] = constrain_int16(_address1.get(), 1, 247);
    _addresses[1] = constrain_int16(_address2.get(), 1, 247);

    if (_configured_count == 2 && _addresses[0] == _addresses[1]) {
        _configured_count = 1;
    }

    const uint8_t offset = constrain_int16(_esc_offset.get(), 0, ESC_TELEM_MAX_ESCS - 1);
    if (offset + _configured_count > ESC_TELEM_MAX_ESCS) {
        _configured_count = ESC_TELEM_MAX_ESCS - offset;
    }
}

void AP_2BLD6010::init()
{
    if (!enabled()) {
        return;
    }

    configure();
    if (_configured_count == 0) {
        return;
    }

    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_2BLD6010, 0);
    if (_uart == nullptr) {
        return;
    }

    uint32_t baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_2BLD6010, 0);
    if (baudrate == 0) {
        baudrate = 115200;
    }
    _uart->begin(baudrate, 128, 64);
    _uart->discard_input();
    const uint32_t now_ms = AP_HAL::millis();
    _next_request_ms = now_ms;
    for (uint8_t i = 0; i < _configured_count; i++) {
        _health_check_start_ms[i] = now_ms;
    }
}

bool AP_2BLD6010::healthy(uint8_t instance) const
{
    if (!enabled() || _uart == nullptr || instance >= _configured_count) {
        return false;
    }
    const uint32_t timeout_ms = constrain_int16(_timeout_ms.get(), 100, 5000);
    return data_is_healthy(AP_HAL::millis(), _data[instance].last_update_ms, timeout_ms, _has_valid_data[instance]);
}

bool AP_2BLD6010::has_fault(uint8_t instance) const
{
    return healthy(instance) && _data[instance].fault_code != 0;
}

bool AP_2BLD6010::get_data(uint8_t instance, Data &out) const
{
    if (instance >= _configured_count || !_has_valid_data[instance]) {
        return false;
    }
    out = _data[instance];
    return true;
}

void AP_2BLD6010::update()
{
    if (!enabled() || _uart == nullptr || _configured_count == 0) {
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
        _data[_current_instance].timeout_count++;
        _response_stream.reset();
        update_health_log(_current_instance, now_ms);
        advance_instance(now_ms);
    }
}

void AP_2BLD6010::send_request(uint32_t now_ms)
{
    uint8_t request[REQUEST_LENGTH];
    _requested_address = _addresses[_current_instance];
    build_request(_requested_address, request);

    if (_uart->txspace() < REQUEST_LENGTH) {
        _next_request_ms = now_ms + 1;
        return;
    }

    _uart->discard_input();
    _response_stream.reset();
    if (_uart->write(request, REQUEST_LENGTH) != REQUEST_LENGTH) {
        _data[_current_instance].bad_response_count++;
        advance_instance(now_ms);
        return;
    }

    _request_sent_ms = now_ms;
    _state = State::WAITING_RESPONSE;
}

void AP_2BLD6010::read_response(uint32_t now_ms)
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
    if (length > 0) {
        _response_stream.append(bytes, length);
    }
    process_rx_buffer(now_ms);
}

bool AP_2BLD6010::process_rx_buffer(uint32_t now_ms)
{
    while (true) {
        const ParseResult result = _response_stream.next(_requested_address, _data[_current_instance]);
        switch (result) {
        case ParseResult::VALID:
            handle_valid_response(_current_instance, now_ms);
            advance_instance(now_ms);
            return true;
        case ParseResult::MODBUS_EXCEPTION:
            _data[_current_instance].modbus_exception_count++;
            advance_instance(now_ms);
            return true;
        case ParseResult::CRC_ERROR:
            _data[_current_instance].crc_error_count++;
            break;
        case ParseResult::BAD_RESPONSE:
            _data[_current_instance].bad_response_count++;
            break;
        case ParseResult::INCOMPLETE:
            return false;
        }
    }
}

void AP_2BLD6010::handle_valid_response(uint8_t instance, uint32_t now_ms)
{
    Data &data = _data[instance];
    data.last_update_ms = now_ms;
    data.success_count++;
    _has_valid_data[instance] = true;

    AP_ESC_Telem_Backend::TelemetryData telem {};
    telem.temperature_cdeg = data.temperature_cdeg;
    telem.voltage = data.voltage_v;
    telem.current = data.current_a;
    const uint16_t mask = AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
                          AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
                          AP_ESC_Telem_Backend::TelemetryType::CURRENT;
    const uint8_t esc_index = constrain_int16(_esc_offset.get(), 0, ESC_TELEM_MAX_ESCS - 1) + instance;
    update_telem_data(esc_index, telem, mask);
    update_rpm(esc_index, data.rpm, 0.0f);
    update_health_log(instance, now_ms);
    write_log(instance, now_ms, true, false);
}

void AP_2BLD6010::advance_instance(uint32_t now_ms)
{
    _current_instance = next_instance(_current_instance, _configured_count);
    _state = State::IDLE;
    _response_stream.reset();
    const uint8_t rate_hz = constrain_int16(_rate_hz.get(), 1, 20);
    uint16_t interval_ms = 1000U / (uint16_t(rate_hz) * _configured_count);
    if (interval_ms == 0) {
        interval_ms = 1;
    }
    _next_request_ms = now_ms + interval_ms;
}

void AP_2BLD6010::update_health_log(uint8_t instance, uint32_t now_ms)
{
    if (instance >= _configured_count) {
        return;
    }
    const uint32_t timeout_ms = constrain_int16(_timeout_ms.get(), 100, 5000);
    if (!_has_valid_data[instance] && now_ms - _health_check_start_ms[instance] < timeout_ms) {
        return;
    }
    const bool healthy_state = data_is_healthy(now_ms, _data[instance].last_update_ms, timeout_ms, _has_valid_data[instance]);
    if (!health_transition_due(healthy_state, _logged_health_valid[instance], _last_logged_healthy[instance])) {
        return;
    }
    write_log(instance, now_ms, healthy_state, true);
}

void AP_2BLD6010::write_log(uint8_t instance, uint32_t now_ms, bool healthy_state, bool force)
{
#if HAL_LOGGING_ENABLED
    if (!force && now_ms - _last_log_ms[instance] < 100) {
        return;
    }
    _last_log_ms[instance] = now_ms;
    const Data &data = _data[instance];
    AP::logger().WriteStreaming("BESC", "TimeUS,Inst,Addr,Healthy,Dir,Fault,Hall,Success,CRCErr,Timeout,BadResp,Except",
                                "QBBBBHIIIIII",
                                AP_HAL::micros64(),
                                instance,
                                _addresses[instance],
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
    (void)instance;
    (void)now_ms;
    (void)healthy_state;
    (void)force;
#endif
}

#endif
