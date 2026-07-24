#include "AP_ESC_Telem_2BLD6010.h"

#if AP_ESC_TELEM_2BLD6010_ENABLED

#include <AP_Math/crc.h>

#include <string.h>

/*
 * 2BLD6010 Modbus RTU 协议处理：构造只读查询、解析遥测响应、验证运行参数，
 * 并通过固定长度缓存支持半帧、粘包、迟到帧和噪声后的非阻塞重同步。
 */

namespace {

// Modbus 功能码及遥测寄存器范围。
constexpr uint8_t MODBUS_READ_HOLDING_REGISTERS = 0x03;
constexpr uint8_t MODBUS_EXCEPTION_MASK = 0x80;
constexpr uint8_t MODBUS_READ_EXCEPTION = MODBUS_READ_HOLDING_REGISTERS | MODBUS_EXCEPTION_MASK;
constexpr uint16_t TELEMETRY_START_REGISTER = 0x0020;
constexpr uint16_t TELEMETRY_REGISTER_COUNT = 8;
constexpr uint8_t TELEMETRY_BYTE_COUNT = TELEMETRY_REGISTER_COUNT * 2;

// 请求帧和响应帧中的固定字段下标。
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

// 原始寄存器到 AP_ESC_Telem 单位的换算系数及物理范围。
constexpr float CURRENT_SCALE_A = 0.01f;
constexpr float VOLTAGE_SCALE_V = 0.1f;
constexpr int16_t TEMPERATURE_SCALE_CDEG = 100;
constexpr int16_t MIN_TEMPERATURE_C = -40;
constexpr int16_t MAX_TEMPERATURE_C = 150;
constexpr uint16_t MAX_DIRECTION = 3;

// 0x0020～0x0027 在响应数据区中的顺序，与设备寄存器地址一一对应。
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

// 从两个大端字节读取一个 16 位无符号寄存器值。
uint16_t get_u16_be(const uint8_t *bytes)
{
    // 设备寄存器按大端传输：高字节在前、低字节在后。
    return (uint16_t(bytes[0]) << 8) | bytes[1];
}

// 按 Modbus 低字节在前的顺序，从指定位置读取帧内 CRC。
uint16_t get_crc_le(const uint8_t *frame, uint8_t low_index, uint8_t high_index)
{
    // Modbus RTU 在线路上先发送 CRC 低字节，再发送 CRC 高字节。
    return uint16_t(frame[low_index]) | (uint16_t(frame[high_index]) << 8);
}

}

/*
 * 2BLD6010 只读 Modbus RTU 请求帧（8 字节）
 *
 * | 字节  | 字段                 | 数值/含义                  |
 * |-------|----------------------|----------------------------|
 * | 0     | 从站地址             | 1～247                     |
 * | 1     | 功能码               | 0x03                       |
 * | 2..3  | 起始寄存器，大端     | 0x0020                     |
 * | 4..5  | 寄存器数量，大端     | 0x0008                     |
 * | 6     | CRC 低字节           | 字节 0..5 的 Modbus CRC    |
 * | 7     | CRC 高字节           | 字节 0..5 的 Modbus CRC    |
 *
 * 正常响应帧（21 字节）
 *
 * | 字节  | 字段                 | 数值/含义                  |
 * |-------|----------------------|----------------------------|
 * | 0     | 从站地址             | 本次请求的地址             |
 * | 1     | 功能码               | 0x03                       |
 * | 2     | 数据字节数           | 0x10                       |
 * | 3..18 | 8 个寄存器，大端     | 见下方寄存器表             |
 * | 19    | CRC 低字节           | 字节 0..18 的 Modbus CRC   |
 * | 20    | CRC 高字节           | 字节 0..18 的 Modbus CRC   |
 *
 * 异常响应帧（5 字节）
 *
 * | 字节 | 字段                  | 数值/含义                  |
 * |------|-----------------------|----------------------------|
 * | 0    | 从站地址              | 本次请求的地址             |
 * | 1    | 异常功能码            | 0x83                       |
 * | 2    | 异常码                | 原始 Modbus 异常码         |
 * | 3    | CRC 低字节            | 字节 0..2 的 Modbus CRC    |
 * | 4    | CRC 高字节            | 字节 0..2 的 Modbus CRC    |
 *
 * 遥测寄存器
 *
 * | 地址    | 字段       | 编码 | 换算/含义                              |
 * |---------|------------|------|----------------------------------------|
 * | 0x0020  | 故障码     | U16  | 保留设备原始故障码                     |
 * | 0x0021  | 电流       | U16  | 原始值 × 0.01 A                        |
 * | 0x0022  | 转速       | U16  | 绝对 RPM，方向单独保存                 |
 * | 0x0023  | 温度       | S16  | 摄氏度，发布时转换为 0.01°C            |
 * | 0x0024  | 电压       | U16  | 原始值 × 0.1 V                         |
 * | 0x0025  | 方向       | U16  | 0 停止、1 正转、2 反转、3 制动         |
 * | 0x0026  | 霍尔高字   | U16  | 霍尔计数位 31..16                      |
 * | 0x0027  | 霍尔低字   | U16  | 霍尔计数位 15..0                       |
 */
// 构造读取 0x0020～0x0027 八个遥测寄存器的 8 字节 Modbus RTU 请求。
void AP_ESC_Telem_2BLD6010::build_request(uint8_t address, uint8_t request[REQUEST_LENGTH])
{
    // 查询固定的 0x0020～0x0027 寄存器；本驱动不会构造任何 Modbus 写请求。
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

// 校验并解析单个正常或异常响应帧，只有完整合法帧才更新 DiagnosticData。
AP_ESC_Telem_2BLD6010::ParseResult AP_ESC_Telem_2BLD6010::parse_response(const uint8_t *frame, uint8_t frame_len, uint8_t expected_address, DiagnosticData &data)
{
    // 校验顺序由帧头到数据区逐步推进，任何失败都不会修改调用方原有 DiagnosticData。
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
        // 异常响应固定为 5 字节，且必须是读保持寄存器对应的 0x83。
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

    // 温度寄存器是有符号 16 位；先转换符号，再检查设备允许的物理范围。
    const int16_t temperature_c = int16_t(registers[REGISTER_TEMPERATURE]);
    if (temperature_c < MIN_TEMPERATURE_C || temperature_c > MAX_TEMPERATURE_C) {
        return ParseResult::BAD_RESPONSE;
    }
    if (registers[REGISTER_DIRECTION] > MAX_DIRECTION) {
        return ParseResult::BAD_RESPONSE;
    }

    // 使用临时副本集中提交解析结果，保证范围校验失败时不会留下半更新数据。
    DiagnosticData parsed = data;
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

// 判断毫秒时间是否到达期限，并正确处理 32 位计数器回绕。
bool AP_ESC_Telem_2BLD6010::time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    // 有符号差值比较可正确处理 millis() 的 32 位回绕。
    return int32_t(now_ms - deadline_ms) >= 0;
}

// 根据是否收到过合法数据及最后更新时间判断实例是否仍在健康超时范围内。
bool AP_ESC_Telem_2BLD6010::data_is_healthy(uint32_t now_ms, uint32_t last_update_ms, uint32_t timeout_ms, bool has_valid_data)
{
    return has_valid_data && now_ms - last_update_ms <= timeout_ms;
}

// 判断健康状态是否需要首次记录或变化记录，并同步日志去重状态。
bool AP_ESC_Telem_2BLD6010::health_transition_due(bool healthy_state, bool &logged_valid, bool &last_logged_healthy)
{
    // 仅首次状态和健康状态变化需要输出日志，避免周期性重复记录。
    if (logged_valid && last_logged_healthy == healthy_state) {
        return false;
    }
    logged_valid = true;
    last_logged_healthy = healthy_state;
    return true;
}

// 计算固定 round-robin 顺序中的下一实例下标。
uint8_t AP_ESC_Telem_2BLD6010::next_instance(uint8_t current_instance, uint8_t configured_count)
{
    // 参数槽位顺序即轮询顺序，地址数值大小不会影响实例编号。
    if (configured_count == 0) {
        return 0;
    }
    return (current_instance + 1) % configured_count;
}

// 判断地址是否属于 Modbus 单播从站范围 1～247。
bool AP_ESC_Telem_2BLD6010::valid_modbus_address(int16_t address)
{
    return address >= MODBUS_MIN_SLAVE_ADDRESS && address <= MODBUS_MAX_SLAVE_ADDRESS;
}

// 严格校验全部启用参数，并在全部通过后生成可原子应用的运行配置。
AP_ESC_Telem_2BLD6010::ConfigError AP_ESC_Telem_2BLD6010::validate_config(const RawConfig &raw, uint8_t max_esc_instances, ValidatedConfig &validated)
{
    // 每次先清空输出，验证失败时调用方不会获得上一轮残留的部分配置。
    validated = {};
    if (raw.enable != 0 && raw.enable != 1) {
        return ConfigError::INVALID_ENABLE;
    }
    if (raw.enable == 0) {
        // ENABLE=0 是正常禁用；其他参数即使非法也不参与本次初始化。
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
    // 使用宽有符号类型完成 OFS+NUM，避免 8 位加法在比较前溢出。
    if (final_index_exclusive > max_esc_instances) {
        return ConfigError::ESC_INDEX_RANGE;
    }

    // 仅在全部校验通过后一次性生成可应用配置，实现“全有或全无”。
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

// 根据每台期望频率和实例数计算向上取整且不小于 10ms 的总线请求间隔。
uint16_t AP_ESC_Telem_2BLD6010::polling_interval_ms(uint8_t rate_hz, uint8_t configured_count)
{
    if (rate_hz == 0 || configured_count == 0) {
        return 0;
    }
    const uint32_t denominator = uint32_t(rate_hz) * uint32_t(configured_count);
    // 整数向上取整，避免实际查询频率高于参数要求；10ms 下限对应 Rover 100Hz 调度周期。
    const uint32_t rounded_interval_ms = (1000U + denominator - 1U) / denominator;
    return uint16_t(rounded_interval_ms < MIN_POLL_INTERVAL_MS ? MIN_POLL_INTERVAL_MS : rounded_interval_ms);
}

// 将新接收字节追加到固定缓存，空指针或容量不足时拒绝写入。
bool AP_ESC_Telem_2BLD6010::ResponseStream::append(const uint8_t *bytes, uint8_t length)
{
    // 固定缓存不动态分配；空间不足时拒绝追加，由上层结束当前请求并计错。
    if (bytes == nullptr || length > sizeof(_buffer) - _length) {
        return false;
    }
    memcpy(&_buffer[_length], bytes, length);
    _length += length;
    return true;
}

// 从流式缓存提取下一帧，处理半帧、粘包、迟到帧和噪声重同步。
AP_ESC_Telem_2BLD6010::ParseResult AP_ESC_Telem_2BLD6010::ResponseStream::next(uint8_t expected_address, DiagnosticData &data)
{
    if (_length == 0) {
        return ParseResult::INCOMPLETE;
    }
    if (_buffer[ADDRESS_INDEX] != expected_address) {
        // 对可识别的完整迟到帧整体丢弃，防止其数据被误认成下一实例的响应。
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
            // 非法功能码更可能是噪声，只消费一个字节后继续寻找下一帧起点。
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
    // 合法帧和异常帧整体消费；CRC/结构错误只移一字节，便于独立使用时继续重同步。
    consume(result == ParseResult::VALID || result == ParseResult::MODBUS_EXCEPTION ? expected_length : 1);
    return result;
}

// 丢弃当前缓存内容，在开始新 outstanding request 时建立干净接收状态。
void AP_ESC_Telem_2BLD6010::ResponseStream::reset()
{
    _length = 0;
}

// 从缓存头部消费指定字节数，并前移保留的未解析数据。
void AP_ESC_Telem_2BLD6010::ResponseStream::consume(uint8_t count)
{
    if (count >= _length) {
        _length = 0;
        return;
    }
    // 保留尚未解析的数据，支持粘包以及跨多个 update() 到达的半帧。
    memmove(_buffer, &_buffer[count], _length - count);
    _length -= count;
}

#endif
