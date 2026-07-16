#include "AP_ESC_2BLD6010.h"

#if AP_ESC_2BLD6010_ENABLED

#include <AP_Math/crc.h>

#include <string.h>

namespace {

constexpr uint8_t MODBUS_READ_HOLDING_REGISTERS = 0x03;
constexpr uint8_t MODBUS_EXCEPTION_MASK = 0x80;
constexpr uint8_t MODBUS_READ_EXCEPTION = MODBUS_READ_HOLDING_REGISTERS | MODBUS_EXCEPTION_MASK;
constexpr uint16_t TELEMETRY_START_REGISTER = 0x0020;
constexpr uint16_t TELEMETRY_REGISTER_COUNT = 8;
constexpr uint8_t TELEMETRY_BYTE_COUNT = TELEMETRY_REGISTER_COUNT * 2;

constexpr uint8_t ADDRESS_INDEX = 0;
constexpr uint8_t FUNCTION_INDEX = 1;
constexpr uint8_t BYTE_COUNT_INDEX = 2;
constexpr uint8_t DATA_START_INDEX = 3;
constexpr uint8_t REQUEST_START_HIGH_INDEX = 2;
constexpr uint8_t REQUEST_START_LOW_INDEX = 3;
constexpr uint8_t REQUEST_COUNT_HIGH_INDEX = 4;
constexpr uint8_t REQUEST_COUNT_LOW_INDEX = 5;
constexpr uint8_t REQUEST_CRC_LOW_INDEX = 6;
constexpr uint8_t REQUEST_CRC_HIGH_INDEX = 7;
constexpr uint8_t RESPONSE_CRC_LOW_INDEX = 19;
constexpr uint8_t RESPONSE_CRC_HIGH_INDEX = 20;
constexpr uint8_t EXCEPTION_CRC_LOW_INDEX = 3;
constexpr uint8_t EXCEPTION_CRC_HIGH_INDEX = 4;
constexpr uint8_t MODBUS_CRC_LENGTH = 2;
constexpr uint8_t MODBUS_EXCEPTION_CRC_INPUT_LENGTH = 3;

constexpr float CURRENT_SCALE_A = 0.01f;
constexpr float VOLTAGE_SCALE_V = 0.1f;
constexpr int16_t TEMPERATURE_SCALE_CDEG = 100;
constexpr int16_t MIN_TEMPERATURE_C = -40;
constexpr int16_t MAX_TEMPERATURE_C = 150;
constexpr uint16_t MAX_DIRECTION = 3;

enum RegisterIndex : uint8_t {
    REGISTER_FAULT = 0,
    REGISTER_CURRENT,
    REGISTER_RPM,
    REGISTER_TEMPERATURE,
    REGISTER_VOLTAGE,
    REGISTER_DIRECTION,
    REGISTER_HALL_HIGH,
    REGISTER_HALL_LOW,
};

uint16_t get_u16_be(const uint8_t *bytes)
{
    return (uint16_t(bytes[0]) << 8) | bytes[1];
}

uint16_t get_crc_le(const uint8_t *frame, uint8_t low_index, uint8_t high_index)
{
    return uint16_t(frame[low_index]) | (uint16_t(frame[high_index]) << 8);
}

}

/*
 * 2BLD6010 read-only Modbus RTU request (8 bytes)
 *
 * | Byte | Field                  | Value                     |
 * |------|------------------------|---------------------------|
 * | 0    | Slave address          | 1..247                    |
 * | 1    | Function               | 0x03                      |
 * | 2..3 | Start register, BE     | 0x0020                    |
 * | 4..5 | Register count, BE     | 0x0008                    |
 * | 6    | CRC low byte           | Modbus CRC of bytes 0..5  |
 * | 7    | CRC high byte          | Modbus CRC of bytes 0..5  |
 *
 * Normal response (21 bytes)
 *
 * | Byte  | Field                 | Value                     |
 * |-------|-----------------------|---------------------------|
 * | 0     | Slave address         | Requested address         |
 * | 1     | Function              | 0x03                      |
 * | 2     | Data byte count       | 0x10                      |
 * | 3..18 | Eight registers, BE   | Register table below      |
 * | 19    | CRC low byte          | Modbus CRC of bytes 0..18 |
 * | 20    | CRC high byte         | Modbus CRC of bytes 0..18 |
 *
 * Exception response (5 bytes)
 *
 * | Byte | Field                  | Value                    |
 * |------|------------------------|--------------------------|
 * | 0    | Slave address          | Requested address        |
 * | 1    | Exception function     | 0x83                     |
 * | 2    | Exception code         | Raw Modbus exception     |
 * | 3    | CRC low byte           | Modbus CRC of bytes 0..2 |
 * | 4    | CRC high byte          | Modbus CRC of bytes 0..2 |
 *
 * Telemetry registers
 *
 * | Address | Field       | Encoding | Conversion / meaning                     |
 * |---------|-------------|----------|------------------------------------------|
 * | 0x0020  | Fault       | U16      | Raw fault code                           |
 * | 0x0021  | Current     | U16      | raw * 0.01 A                             |
 * | 0x0022  | RPM         | U16      | Absolute RPM                             |
 * | 0x0023  | Temperature | S16      | Degrees C, published as centi-degrees    |
 * | 0x0024  | Voltage     | U16      | raw * 0.1 V                              |
 * | 0x0025  | Direction   | U16      | 0 stop, 1 forward, 2 reverse, 3 brake   |
 * | 0x0026  | Hall high   | U16      | Hall counter bits 31..16                 |
 * | 0x0027  | Hall low    | U16      | Hall counter bits 15..0                  |
 */
void AP_ESC_2BLD6010::build_request(uint8_t address, uint8_t request[REQUEST_LENGTH])
{
    request[ADDRESS_INDEX] = address;
    request[FUNCTION_INDEX] = MODBUS_READ_HOLDING_REGISTERS;
    request[REQUEST_START_HIGH_INDEX] = uint8_t(TELEMETRY_START_REGISTER >> 8);
    request[REQUEST_START_LOW_INDEX] = uint8_t(TELEMETRY_START_REGISTER);
    request[REQUEST_COUNT_HIGH_INDEX] = uint8_t(TELEMETRY_REGISTER_COUNT >> 8);
    request[REQUEST_COUNT_LOW_INDEX] = uint8_t(TELEMETRY_REGISTER_COUNT);
    const uint16_t crc = calc_crc_modbus(request, REQUEST_LENGTH - MODBUS_CRC_LENGTH);
    request[REQUEST_CRC_LOW_INDEX] = uint8_t(crc);
    request[REQUEST_CRC_HIGH_INDEX] = uint8_t(crc >> 8);
}

AP_ESC_2BLD6010::ParseResult AP_ESC_2BLD6010::parse_response(const uint8_t *frame, uint8_t frame_len, uint8_t expected_address, Data &data)
{
    if (frame == nullptr || frame_len == 0) {
        return ParseResult::INCOMPLETE;
    }
    if (frame[ADDRESS_INDEX] != expected_address) {
        return ParseResult::ADDRESS_MISMATCH;
    }
    if (frame_len <= FUNCTION_INDEX) {
        return ParseResult::INCOMPLETE;
    }

    const uint8_t function = frame[FUNCTION_INDEX];
    if ((function & MODBUS_EXCEPTION_MASK) != 0) {
        if (function != MODBUS_READ_EXCEPTION) {
            return ParseResult::BAD_RESPONSE;
        }
        if (frame_len < EXCEPTION_LENGTH) {
            return ParseResult::INCOMPLETE;
        }
        const uint16_t received_crc = get_crc_le(frame, EXCEPTION_CRC_LOW_INDEX, EXCEPTION_CRC_HIGH_INDEX);
        if (received_crc != calc_crc_modbus(frame, MODBUS_EXCEPTION_CRC_INPUT_LENGTH)) {
            return ParseResult::CRC_ERROR;
        }
        return ParseResult::MODBUS_EXCEPTION;
    }

    if (function != MODBUS_READ_HOLDING_REGISTERS) {
        return ParseResult::BAD_RESPONSE;
    }
    if (frame_len <= BYTE_COUNT_INDEX) {
        return ParseResult::INCOMPLETE;
    }
    if (frame[BYTE_COUNT_INDEX] != TELEMETRY_BYTE_COUNT) {
        return ParseResult::BAD_RESPONSE;
    }
    if (frame_len < RESPONSE_LENGTH) {
        return ParseResult::INCOMPLETE;
    }

    const uint16_t received_crc = get_crc_le(frame, RESPONSE_CRC_LOW_INDEX, RESPONSE_CRC_HIGH_INDEX);
    if (received_crc != calc_crc_modbus(frame, RESPONSE_LENGTH - MODBUS_CRC_LENGTH)) {
        return ParseResult::CRC_ERROR;
    }

    uint16_t registers[TELEMETRY_REGISTER_COUNT];
    for (uint8_t i = 0; i < ARRAY_SIZE(registers); i++) {
        registers[i] = get_u16_be(&frame[DATA_START_INDEX + i * 2]);
    }

    const int16_t temperature_c = int16_t(registers[REGISTER_TEMPERATURE]);
    if (temperature_c < MIN_TEMPERATURE_C || temperature_c > MAX_TEMPERATURE_C) {
        return ParseResult::BAD_RESPONSE;
    }
    if (registers[REGISTER_DIRECTION] > MAX_DIRECTION) {
        return ParseResult::BAD_RESPONSE;
    }

    Data parsed = data;
    parsed.fault_code = registers[REGISTER_FAULT];
    parsed.current_a = registers[REGISTER_CURRENT] * CURRENT_SCALE_A;
    parsed.rpm = registers[REGISTER_RPM];
    parsed.temperature_cdeg = temperature_c * TEMPERATURE_SCALE_CDEG;
    parsed.voltage_v = registers[REGISTER_VOLTAGE] * VOLTAGE_SCALE_V;
    parsed.direction = uint8_t(registers[REGISTER_DIRECTION]);
    parsed.hall_count = (uint32_t(registers[REGISTER_HALL_HIGH]) << 16) | registers[REGISTER_HALL_LOW];
    data = parsed;
    return ParseResult::VALID;
}

bool AP_ESC_2BLD6010::time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return int32_t(now_ms - deadline_ms) >= 0;
}

bool AP_ESC_2BLD6010::data_is_healthy(uint32_t now_ms, uint32_t last_update_ms, uint32_t timeout_ms, bool has_valid_data)
{
    return has_valid_data && now_ms - last_update_ms <= timeout_ms;
}

bool AP_ESC_2BLD6010::health_transition_due(bool healthy_state, bool &logged_valid, bool &last_logged_healthy)
{
    if (logged_valid && last_logged_healthy == healthy_state) {
        return false;
    }
    logged_valid = true;
    last_logged_healthy = healthy_state;
    return true;
}

uint8_t AP_ESC_2BLD6010::next_instance(uint8_t current_instance, uint8_t configured_count)
{
    if (configured_count == 0) {
        return 0;
    }
    return (current_instance + 1) % configured_count;
}

bool AP_ESC_2BLD6010::valid_modbus_address(int16_t address)
{
    return address >= MODBUS_MIN_SLAVE_ADDRESS && address <= MODBUS_MAX_SLAVE_ADDRESS;
}

AP_ESC_2BLD6010::ConfigError AP_ESC_2BLD6010::validate_config(const RawConfig &raw, uint8_t max_esc_instances, ValidatedConfig &validated)
{
    validated = {};
    if (raw.enable != 0 && raw.enable != 1) {
        return ConfigError::INVALID_ENABLE;
    }
    if (raw.enable == 0) {
        return ConfigError::NONE;
    }
    if (raw.count < 1 || raw.count > MAX_ESC_COUNT) {
        return ConfigError::INVALID_COUNT;
    }
    if (raw.esc_offset < 0) {
        return ConfigError::INVALID_ESC_OFFSET;
    }
    if (raw.rate_hz < MIN_RATE_HZ || raw.rate_hz > MAX_RATE_HZ) {
        return ConfigError::INVALID_RATE;
    }
    if (raw.health_timeout_ms < MIN_HEALTH_TIMEOUT_MS || raw.health_timeout_ms > MAX_HEALTH_TIMEOUT_MS) {
        return ConfigError::INVALID_TIMEOUT;
    }

    for (int32_t i = 0; i < raw.count; i++) {
        if (raw.addresses[i] < MODBUS_MIN_SLAVE_ADDRESS || raw.addresses[i] > MODBUS_MAX_SLAVE_ADDRESS) {
            return ConfigError::INVALID_ADDRESS;
        }
    }
    for (int32_t i = 0; i < raw.count; i++) {
        for (int32_t j = i + 1; j < raw.count; j++) {
            if (raw.addresses[i] == raw.addresses[j]) {
                return ConfigError::DUPLICATE_ADDRESS;
            }
        }
    }

    const int32_t final_index_exclusive = raw.esc_offset + raw.count;
    if (final_index_exclusive > max_esc_instances) {
        return ConfigError::ESC_INDEX_RANGE;
    }

    validated.enabled = true;
    validated.count = uint8_t(raw.count);
    validated.esc_offset = uint8_t(raw.esc_offset);
    validated.rate_hz = uint8_t(raw.rate_hz);
    validated.health_timeout_ms = uint16_t(raw.health_timeout_ms);
    validated.poll_interval_ms = polling_interval_ms(validated.rate_hz, validated.count);
    for (uint8_t i = 0; i < validated.count; i++) {
        validated.addresses[i] = uint8_t(raw.addresses[i]);
    }
    return ConfigError::NONE;
}

uint16_t AP_ESC_2BLD6010::polling_interval_ms(uint8_t rate_hz, uint8_t configured_count)
{
    if (rate_hz == 0 || configured_count == 0) {
        return 0;
    }
    const uint32_t denominator = uint32_t(rate_hz) * uint32_t(configured_count);
    const uint32_t rounded_interval_ms = (1000U + denominator - 1U) / denominator;
    return uint16_t(rounded_interval_ms < MIN_POLL_INTERVAL_MS ? MIN_POLL_INTERVAL_MS : rounded_interval_ms);
}

bool AP_ESC_2BLD6010::ResponseStream::append(const uint8_t *bytes, uint8_t length)
{
    if (bytes == nullptr || length > sizeof(_buffer) - _length) {
        return false;
    }
    memcpy(&_buffer[_length], bytes, length);
    _length += length;
    return true;
}

AP_ESC_2BLD6010::ParseResult AP_ESC_2BLD6010::ResponseStream::next(uint8_t expected_address, Data &data)
{
    if (_length == 0) {
        return ParseResult::INCOMPLETE;
    }
    if (_buffer[ADDRESS_INDEX] != expected_address) {
        if (_length <= FUNCTION_INDEX) {
            return ParseResult::INCOMPLETE;
        }
        uint8_t mismatched_frame_length;
        const uint8_t function = _buffer[FUNCTION_INDEX];
        if (function == MODBUS_READ_EXCEPTION) {
            mismatched_frame_length = EXCEPTION_LENGTH;
        } else if (function == MODBUS_READ_HOLDING_REGISTERS) {
            if (_length <= BYTE_COUNT_INDEX) {
                return ParseResult::INCOMPLETE;
            }
            if (_buffer[BYTE_COUNT_INDEX] != TELEMETRY_BYTE_COUNT) {
                consume(1);
                return ParseResult::ADDRESS_MISMATCH;
            }
            mismatched_frame_length = RESPONSE_LENGTH;
        } else {
            consume(1);
            return ParseResult::ADDRESS_MISMATCH;
        }
        if (_length < mismatched_frame_length) {
            return ParseResult::INCOMPLETE;
        }
        consume(mismatched_frame_length);
        return ParseResult::ADDRESS_MISMATCH;
    }
    if (_length <= FUNCTION_INDEX) {
        return ParseResult::INCOMPLETE;
    }

    uint8_t expected_length;
    const uint8_t function = _buffer[FUNCTION_INDEX];
    if ((function & MODBUS_EXCEPTION_MASK) != 0) {
        if (function != MODBUS_READ_EXCEPTION) {
            consume(1);
            return ParseResult::BAD_RESPONSE;
        }
        expected_length = EXCEPTION_LENGTH;
    } else if (function == MODBUS_READ_HOLDING_REGISTERS) {
        if (_length <= BYTE_COUNT_INDEX) {
            return ParseResult::INCOMPLETE;
        }
        if (_buffer[BYTE_COUNT_INDEX] != TELEMETRY_BYTE_COUNT) {
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

void AP_ESC_2BLD6010::ResponseStream::reset()
{
    _length = 0;
}

void AP_ESC_2BLD6010::ResponseStream::consume(uint8_t count)
{
    if (count >= _length) {
        _length = 0;
        return;
    }
    memmove(_buffer, &_buffer[count], _length - count);
    _length -= count;
}

#endif
