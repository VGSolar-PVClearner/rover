#include "AP_2BLD6010.h"

#if AP_2BLD6010_ENABLED

#include <AP_Math/crc.h>

#include <string.h>

void AP_2BLD6010::build_request(uint8_t address, uint8_t request[REQUEST_LENGTH])
{
    request[0] = address;
    request[1] = 0x03;
    request[2] = 0x00;
    request[3] = 0x20;
    request[4] = 0x00;
    request[5] = 0x08;
    const uint16_t crc = calc_crc_modbus(request, REQUEST_LENGTH - 2);
    request[6] = uint8_t(crc);
    request[7] = uint8_t(crc >> 8);
}

AP_2BLD6010::ParseResult AP_2BLD6010::parse_response(const uint8_t *frame, uint8_t frame_len, uint8_t expected_address, Data &data)
{
    if (frame == nullptr || frame_len < EXCEPTION_LENGTH) {
        return ParseResult::INCOMPLETE;
    }
    if (frame[0] != expected_address) {
        return ParseResult::BAD_RESPONSE;
    }

    if ((frame[1] & 0x80U) != 0) {
        const uint16_t received_crc = uint16_t(frame[3]) | (uint16_t(frame[4]) << 8);
        if (received_crc != calc_crc_modbus(frame, 3)) {
            return ParseResult::CRC_ERROR;
        }
        if (frame[1] != 0x83) {
            return ParseResult::BAD_RESPONSE;
        }
        return ParseResult::MODBUS_EXCEPTION;
    }

    if (frame[1] != 0x03 || frame[2] != 0x10) {
        return ParseResult::BAD_RESPONSE;
    }
    if (frame_len < RESPONSE_LENGTH) {
        return ParseResult::INCOMPLETE;
    }

    const uint16_t received_crc = uint16_t(frame[19]) | (uint16_t(frame[20]) << 8);
    if (received_crc != calc_crc_modbus(frame, RESPONSE_LENGTH - 2)) {
        return ParseResult::CRC_ERROR;
    }

    uint16_t registers[8];
    for (uint8_t i = 0; i < ARRAY_SIZE(registers); i++) {
        registers[i] = (uint16_t(frame[3 + i * 2]) << 8) | frame[4 + i * 2];
    }

    const uint16_t fault_code = registers[0];
    const float current_a = registers[1] * 0.01f;
    const float rpm = registers[2];
    const int16_t temperature_c = int16_t(registers[3]);
    if (temperature_c < -40 || temperature_c > 150) {
        return ParseResult::BAD_RESPONSE;
    }
    const float voltage_v = registers[4] * 0.1f;
    if (registers[5] > 3) {
        return ParseResult::BAD_RESPONSE;
    }
    const uint8_t direction = uint8_t(registers[5]);
    const uint32_t hall_count = (uint32_t(registers[6]) << 16) | uint32_t(registers[7]);

    data.fault_code = fault_code;
    data.current_a = current_a;
    data.rpm = rpm;
    data.temperature_cdeg = temperature_c * 100;
    data.voltage_v = voltage_v;
    data.direction = direction;
    data.hall_count = hall_count;
    return ParseResult::VALID;
}

bool AP_2BLD6010::time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return int32_t(now_ms - deadline_ms) >= 0;
}

bool AP_2BLD6010::data_is_healthy(uint32_t now_ms, uint32_t last_update_ms, uint32_t timeout_ms, bool has_valid_data)
{
    return has_valid_data && now_ms - last_update_ms <= timeout_ms;
}

bool AP_2BLD6010::health_transition_due(bool healthy_state, bool &logged_valid, bool &last_logged_healthy)
{
    if (logged_valid && last_logged_healthy == healthy_state) {
        return false;
    }
    logged_valid = true;
    last_logged_healthy = healthy_state;
    return true;
}

uint8_t AP_2BLD6010::next_instance(uint8_t current_instance, uint8_t configured_count)
{
    if (configured_count == 0) {
        return 0;
    }
    return (current_instance + 1) % configured_count;
}

bool AP_2BLD6010::ResponseStream::append(const uint8_t *bytes, uint8_t length)
{
    if (bytes == nullptr || length > sizeof(_buffer) - _length) {
        return false;
    }
    memcpy(&_buffer[_length], bytes, length);
    _length += length;
    return true;
}

AP_2BLD6010::ParseResult AP_2BLD6010::ResponseStream::next(uint8_t expected_address, Data &data)
{
    if (_length < EXCEPTION_LENGTH) {
        return ParseResult::INCOMPLETE;
    }
    if (_buffer[0] != expected_address) {
        consume(1);
        return ParseResult::BAD_RESPONSE;
    }

    uint8_t expected_length;
    if ((_buffer[1] & 0x80U) != 0) {
        expected_length = EXCEPTION_LENGTH;
    } else if (_buffer[1] == 0x03) {
        if (_buffer[2] != 0x10) {
            consume(1);
            return ParseResult::BAD_RESPONSE;
        }
        expected_length = RESPONSE_LENGTH;
    } else {
        consume(1);
        return ParseResult::BAD_RESPONSE;
    }

    if (_length < expected_length) {
        return ParseResult::INCOMPLETE;
    }

    const ParseResult result = parse_response(_buffer, expected_length, expected_address, data);
    consume(result == ParseResult::VALID || result == ParseResult::MODBUS_EXCEPTION ? expected_length : 1);
    return result;
}

void AP_2BLD6010::ResponseStream::reset()
{
    _length = 0;
}

void AP_2BLD6010::ResponseStream::consume(uint8_t count)
{
    if (count >= _length) {
        _length = 0;
        return;
    }
    memmove(_buffer, &_buffer[count], _length - count);
    _length -= count;
}

#endif
