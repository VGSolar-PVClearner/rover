#include <AP_gtest.h>

#include <AP_ESC_2BLD6010/AP_ESC_2BLD6010.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Math/crc.h>

#include <cstring>

#if AP_ESC_2BLD6010_ENABLED

static constexpr uint8_t NORMAL_FRAME_LENGTH = 21;
static constexpr uint8_t EXCEPTION_FRAME_LENGTH = 5;
static constexpr uint8_t NORMAL_CRC_LOW_INDEX = 19;
static constexpr uint8_t NORMAL_CRC_HIGH_INDEX = 20;

static void set_register(uint8_t frame[NORMAL_FRAME_LENGTH], uint8_t index, uint16_t value)
{
    frame[3 + index * 2] = uint8_t(value >> 8);
    frame[4 + index * 2] = uint8_t(value);
}

static void update_crc(uint8_t frame[NORMAL_FRAME_LENGTH])
{
    const uint16_t crc = calc_crc_modbus(frame, NORMAL_FRAME_LENGTH - 2);
    frame[NORMAL_CRC_LOW_INDEX] = uint8_t(crc);
    frame[NORMAL_CRC_HIGH_INDEX] = uint8_t(crc >> 8);
}

static void make_valid_frame(uint8_t frame[NORMAL_FRAME_LENGTH], uint8_t address = 2)
{
    memset(frame, 0, NORMAL_FRAME_LENGTH);
    frame[0] = address;
    frame[1] = 0x03;
    frame[2] = 0x10;
    set_register(frame, 0, 5);
    set_register(frame, 1, 117);
    set_register(frame, 2, 3456);
    set_register(frame, 3, 29);
    set_register(frame, 4, 229);
    set_register(frame, 5, 1);
    set_register(frame, 6, 0x1234);
    set_register(frame, 7, 0x5678);
    update_crc(frame);
}

static void make_exception_frame(uint8_t frame[EXCEPTION_FRAME_LENGTH], uint8_t address = 2, uint8_t exception_code = 2)
{
    frame[0] = address;
    frame[1] = 0x83;
    frame[2] = exception_code;
    const uint16_t crc = calc_crc_modbus(frame, 3);
    frame[3] = uint8_t(crc);
    frame[4] = uint8_t(crc >> 8);
}

static AP_ESC_2BLD6010::Data sentinel_data()
{
    AP_ESC_2BLD6010::Data data {};
    data.fault_code = 77;
    data.current_a = 12.5f;
    data.rpm = 6543.0f;
    data.temperature_cdeg = 4321;
    data.voltage_v = 48.5f;
    data.direction = 3;
    data.hall_count = 0x89ABCDEF;
    data.last_update_ms = 123456;
    data.success_count = 11;
    data.crc_error_count = 12;
    data.timeout_count = 13;
    data.bad_response_count = 14;
    data.modbus_exception_count = 15;
    return data;
}

static void expect_data_unchanged(const AP_ESC_2BLD6010::Data &data)
{
    EXPECT_EQ(77, data.fault_code);
    EXPECT_FLOAT_EQ(12.5f, data.current_a);
    EXPECT_FLOAT_EQ(6543.0f, data.rpm);
    EXPECT_EQ(4321, data.temperature_cdeg);
    EXPECT_FLOAT_EQ(48.5f, data.voltage_v);
    EXPECT_EQ(3, data.direction);
    EXPECT_EQ(0x89ABCDEFU, data.hall_count);
    EXPECT_EQ(123456U, data.last_update_ms);
    EXPECT_EQ(11U, data.success_count);
    EXPECT_EQ(12U, data.crc_error_count);
    EXPECT_EQ(13U, data.timeout_count);
    EXPECT_EQ(14U, data.bad_response_count);
    EXPECT_EQ(15U, data.modbus_exception_count);
}

static AP_ESC_2BLD6010::RawConfig valid_raw_config(int32_t count = 4)
{
    AP_ESC_2BLD6010::RawConfig raw {};
    raw.enable = 1;
    raw.count = count;
    raw.esc_offset = 0;
    raw.rate_hz = 10;
    raw.health_timeout_ms = 500;
    raw.addresses[0] = 4;
    raw.addresses[1] = 2;
    raw.addresses[2] = 8;
    raw.addresses[3] = 1;
    return raw;
}

static void expect_invalid_config(const AP_ESC_2BLD6010::RawConfig &raw, AP_ESC_2BLD6010::ConfigError expected_error)
{
    AP_ESC_2BLD6010::ValidatedConfig validated {};
    validated.enabled = true;
    validated.count = 4;
    validated.poll_interval_ms = 123;
    memset(validated.addresses, 42, sizeof(validated.addresses));
    EXPECT_EQ(expected_error, AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
    EXPECT_FALSE(validated.enabled);
    EXPECT_EQ(0, validated.count);
    EXPECT_EQ(0, validated.poll_interval_ms);
    for (uint8_t address : validated.addresses) {
        EXPECT_EQ(0, address);
    }
}

TEST(AP_ESC_2BLD6010, ValidatesEnableStrictly)
{
    AP_ESC_2BLD6010::RawConfig raw = valid_raw_config();
    raw.enable = -1;
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::INVALID_ENABLE);
    raw.enable = 2;
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::INVALID_ENABLE);

    raw = valid_raw_config();
    raw.enable = 0;
    raw.count = -1;
    AP_ESC_2BLD6010::ValidatedConfig validated {};
    EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
              AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
    EXPECT_FALSE(validated.enabled);
    EXPECT_EQ(0, validated.count);

    raw = valid_raw_config();
    EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
              AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
    EXPECT_TRUE(validated.enabled);
    EXPECT_EQ(4, validated.count);
}

TEST(AP_ESC_2BLD6010, ValidatesCountWithoutClamping)
{
    for (const int32_t count : {-1, 0, 5}) {
        AP_ESC_2BLD6010::RawConfig raw = valid_raw_config(count);
        expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::INVALID_COUNT);
    }
    for (int32_t count = 1; count <= AP_ESC_2BLD6010::MAX_ESC_COUNT; count++) {
        AP_ESC_2BLD6010::RawConfig raw = valid_raw_config(count);
        AP_ESC_2BLD6010::ValidatedConfig validated {};
        EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
                  AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
        EXPECT_EQ(count, validated.count);
    }
}

TEST(AP_ESC_2BLD6010, ValidatesEscTelemetryIndexRangeWithWideArithmetic)
{
    AP_ESC_2BLD6010::RawConfig raw = valid_raw_config(1);
    raw.esc_offset = -1;
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::INVALID_ESC_OFFSET);

    raw = valid_raw_config(1);
    raw.esc_offset = ESC_TELEM_MAX_ESCS - 1;
    AP_ESC_2BLD6010::ValidatedConfig validated {};
    EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
              AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
    EXPECT_EQ(ESC_TELEM_MAX_ESCS - 1, validated.esc_offset);

    raw = valid_raw_config(2);
    raw.esc_offset = ESC_TELEM_MAX_ESCS - 1;
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::ESC_INDEX_RANGE);

    raw = valid_raw_config(4);
    raw.esc_offset = ESC_TELEM_MAX_ESCS - 4;
    EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
              AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));

    raw.esc_offset = ESC_TELEM_MAX_ESCS - 3;
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::ESC_INDEX_RANGE);
}

TEST(AP_ESC_2BLD6010, ValidatesRateAndComputesCeilingInterval)
{
    for (const int32_t rate : {-1, 0, 21}) {
        AP_ESC_2BLD6010::RawConfig raw = valid_raw_config();
        raw.rate_hz = rate;
        expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::INVALID_RATE);
    }

    EXPECT_EQ(0, AP_ESC_2BLD6010::polling_interval_ms(0, 4));
    EXPECT_EQ(0, AP_ESC_2BLD6010::polling_interval_ms(10, 0));
    EXPECT_EQ(1000, AP_ESC_2BLD6010::polling_interval_ms(1, 1));
    EXPECT_EQ(25, AP_ESC_2BLD6010::polling_interval_ms(10, 4));
    EXPECT_EQ(13, AP_ESC_2BLD6010::polling_interval_ms(20, 4));

    AP_ESC_2BLD6010::RawConfig raw = valid_raw_config(4);
    raw.rate_hz = 20;
    AP_ESC_2BLD6010::ValidatedConfig validated {};
    EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
              AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
    EXPECT_EQ(13, validated.poll_interval_ms);
}

TEST(AP_ESC_2BLD6010, ValidatesHealthTimeoutSeparatelyFromResponseTimeout)
{
    for (const int32_t timeout : {-1, 0, 99, 5001}) {
        AP_ESC_2BLD6010::RawConfig raw = valid_raw_config();
        raw.health_timeout_ms = timeout;
        expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::INVALID_TIMEOUT);
    }
    for (const int32_t timeout : {100, 5000}) {
        AP_ESC_2BLD6010::RawConfig raw = valid_raw_config();
        raw.health_timeout_ms = timeout;
        AP_ESC_2BLD6010::ValidatedConfig validated {};
        EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
                  AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
        EXPECT_EQ(timeout, validated.health_timeout_ms);
    }
}

TEST(AP_ESC_2BLD6010, RequiresAllEnabledAddressesToBeValidAndUnique)
{
    AP_ESC_2BLD6010::RawConfig raw = valid_raw_config();
    raw.addresses[2] = raw.addresses[1];
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::DUPLICATE_ADDRESS);

    raw = valid_raw_config();
    raw.addresses[3] = 0;
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::INVALID_ADDRESS);

    raw = valid_raw_config();
    raw.addresses[1] = 248;
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::INVALID_ADDRESS);

    raw = valid_raw_config();
    raw.addresses[3] = raw.addresses[0];
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::DUPLICATE_ADDRESS);
}

TEST(AP_ESC_2BLD6010, IgnoresDisabledAddressParameters)
{
    AP_ESC_2BLD6010::RawConfig raw = valid_raw_config(2);
    raw.addresses[2] = 0;
    raw.addresses[3] = raw.addresses[0];
    AP_ESC_2BLD6010::ValidatedConfig validated {};
    EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
              AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
    EXPECT_EQ(2, validated.count);
    EXPECT_EQ(4, validated.addresses[0]);
    EXPECT_EQ(2, validated.addresses[1]);
    EXPECT_EQ(0, validated.addresses[2]);
    EXPECT_EQ(0, validated.addresses[3]);
}

TEST(AP_ESC_2BLD6010, ValidationOutputIsAtomicAcrossReconfiguration)
{
    AP_ESC_2BLD6010::ValidatedConfig validated {};
    AP_ESC_2BLD6010::RawConfig raw = valid_raw_config(4);
    EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
              AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
    EXPECT_EQ(4, validated.count);
    EXPECT_EQ(8, validated.addresses[2]);

    raw = valid_raw_config(2);
    raw.addresses[2] = 0;
    raw.addresses[3] = 0;
    EXPECT_EQ(AP_ESC_2BLD6010::ConfigError::NONE,
              AP_ESC_2BLD6010::validate_config(raw, ESC_TELEM_MAX_ESCS, validated));
    EXPECT_EQ(2, validated.count);
    EXPECT_EQ(0, validated.addresses[2]);
    EXPECT_EQ(0, validated.addresses[3]);

    raw.addresses[1] = raw.addresses[0];
    expect_invalid_config(raw, AP_ESC_2BLD6010::ConfigError::DUPLICATE_ADDRESS);
}

TEST(AP_ESC_2BLD6010, BuildsReadRequestForFourAddresses)
{
    for (uint8_t address = 1; address <= AP_ESC_2BLD6010::MAX_ESC_COUNT; address++) {
        uint8_t request[8] {};
        AP_ESC_2BLD6010::build_request(address, request);
        EXPECT_EQ(address, request[0]);
        EXPECT_EQ(0x03, request[1]);
        EXPECT_EQ(0x00, request[2]);
        EXPECT_EQ(0x20, request[3]);
        EXPECT_EQ(0x00, request[4]);
        EXPECT_EQ(0x08, request[5]);
        const uint16_t crc = calc_crc_modbus(request, 6);
        EXPECT_EQ(uint8_t(crc), request[6]);
        EXPECT_EQ(uint8_t(crc >> 8), request[7]);
    }
}

TEST(AP_ESC_2BLD6010, ParsesAllTelemetryFieldsForFourAddresses)
{
    for (uint8_t address = 1; address <= AP_ESC_2BLD6010::MAX_ESC_COUNT; address++) {
        uint8_t frame[NORMAL_FRAME_LENGTH];
        make_valid_frame(frame, address);
        AP_ESC_2BLD6010::Data data {};
        EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::VALID,
                  AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), address, data));
        EXPECT_EQ(5, data.fault_code);
        EXPECT_FLOAT_EQ(1.17f, data.current_a);
        EXPECT_FLOAT_EQ(3456.0f, data.rpm);
        EXPECT_EQ(2900, data.temperature_cdeg);
        EXPECT_FLOAT_EQ(22.9f, data.voltage_v);
        EXPECT_EQ(1, data.direction);
        EXPECT_EQ(0x12345678U, data.hall_count);
    }
}

TEST(AP_ESC_2BLD6010, ParsesSignedTemperatureScalesAndBoundaries)
{
    uint8_t frame[NORMAL_FRAME_LENGTH];
    make_valid_frame(frame);
    set_register(frame, 1, 1);
    set_register(frame, 2, 65535);
    set_register(frame, 4, 1);
    set_register(frame, 5, 3);
    AP_ESC_2BLD6010::Data data {};

    set_register(frame, 3, uint16_t(int16_t(-40)));
    update_crc(frame);
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::VALID,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    EXPECT_FLOAT_EQ(0.01f, data.current_a);
    EXPECT_FLOAT_EQ(65535.0f, data.rpm);
    EXPECT_EQ(-4000, data.temperature_cdeg);
    EXPECT_FLOAT_EQ(0.1f, data.voltage_v);
    EXPECT_EQ(3, data.direction);

    set_register(frame, 3, uint16_t(int16_t(-1)));
    update_crc(frame);
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::VALID,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    EXPECT_EQ(-100, data.temperature_cdeg);

    set_register(frame, 3, 150);
    update_crc(frame);
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::VALID,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    EXPECT_EQ(15000, data.temperature_cdeg);
}

TEST(AP_ESC_2BLD6010, IncompleteFramesNeverModifyData)
{
    uint8_t frame[NORMAL_FRAME_LENGTH];
    make_valid_frame(frame);
    for (uint8_t length = 0; length < NORMAL_FRAME_LENGTH; length++) {
        AP_ESC_2BLD6010::Data data = sentinel_data();
        EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::INCOMPLETE,
                  AP_ESC_2BLD6010::parse_response(frame, length, 2, data));
        expect_data_unchanged(data);
    }

    AP_ESC_2BLD6010::Data data = sentinel_data();
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::INCOMPLETE,
              AP_ESC_2BLD6010::parse_response(nullptr, 0, 2, data));
    expect_data_unchanged(data);
}

TEST(AP_ESC_2BLD6010, RejectsHeaderAndCrcErrorsWithoutModifyingData)
{
    uint8_t frame[NORMAL_FRAME_LENGTH];
    make_valid_frame(frame);
    AP_ESC_2BLD6010::Data data = sentinel_data();

    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::ADDRESS_MISMATCH,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 1, data));
    expect_data_unchanged(data);

    frame[1] = 0x04;
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);

    make_valid_frame(frame);
    frame[2] = 0x0E;
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);

    make_valid_frame(frame);
    frame[NORMAL_CRC_LOW_INDEX] ^= 0x01;
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::CRC_ERROR,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);

    make_valid_frame(frame);
    frame[NORMAL_CRC_HIGH_INDEX] ^= 0x01;
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::CRC_ERROR,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);
}

TEST(AP_ESC_2BLD6010, HandlesExceptionFramesAndPhysicalRanges)
{
    uint8_t exception[EXCEPTION_FRAME_LENGTH];
    make_exception_frame(exception);
    AP_ESC_2BLD6010::Data data = sentinel_data();
    for (uint8_t length = 0; length < EXCEPTION_FRAME_LENGTH; length++) {
        EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::INCOMPLETE,
                  AP_ESC_2BLD6010::parse_response(exception, length, 2, data));
        expect_data_unchanged(data);
    }
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::MODBUS_EXCEPTION,
              AP_ESC_2BLD6010::parse_response(exception, sizeof(exception), 2, data));
    expect_data_unchanged(data);

    exception[3] ^= 0x01;
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::CRC_ERROR,
              AP_ESC_2BLD6010::parse_response(exception, sizeof(exception), 2, data));

    uint8_t frame[NORMAL_FRAME_LENGTH];
    make_valid_frame(frame);
    set_register(frame, 3, uint16_t(int16_t(-41)));
    update_crc(frame);
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);

    make_valid_frame(frame);
    set_register(frame, 3, 151);
    update_crc(frame);
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));

    make_valid_frame(frame);
    set_register(frame, 5, 4);
    update_crc(frame);
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_ESC_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
}

TEST(AP_ESC_2BLD6010, RoundRobinSupportsOneThroughFourInstances)
{
    for (uint8_t count = 1; count <= AP_ESC_2BLD6010::MAX_ESC_COUNT; count++) {
        uint8_t instance = 0;
        for (uint8_t i = 0; i < count; i++) {
            EXPECT_EQ(i, instance);
            instance = AP_ESC_2BLD6010::next_instance(instance, count);
        }
        EXPECT_EQ(0, instance);
    }
    EXPECT_EQ(0, AP_ESC_2BLD6010::next_instance(3, 0));
}

TEST(AP_ESC_2BLD6010, HealthStartsInvalidTimesOutAndRecovers)
{
    EXPECT_FALSE(AP_ESC_2BLD6010::data_is_healthy(100, 0, 500, false));
    EXPECT_TRUE(AP_ESC_2BLD6010::data_is_healthy(100, 100, 500, true));
    EXPECT_TRUE(AP_ESC_2BLD6010::data_is_healthy(600, 100, 500, true));
    EXPECT_FALSE(AP_ESC_2BLD6010::data_is_healthy(601, 100, 500, true));
    EXPECT_TRUE(AP_ESC_2BLD6010::data_is_healthy(700, 700, 500, true));
    EXPECT_TRUE(AP_ESC_2BLD6010::data_is_healthy(0x00000005U, 0xFFFFFFF0U, 30, true));
    EXPECT_FALSE(AP_ESC_2BLD6010::data_is_healthy(0x00000020U, 0xFFFFFFF0U, 30, true));
}

TEST(AP_ESC_2BLD6010, TimeAndHealthTransitionsHandleWrap)
{
    EXPECT_TRUE(AP_ESC_2BLD6010::time_reached(100, 99));
    EXPECT_FALSE(AP_ESC_2BLD6010::time_reached(99, 100));
    EXPECT_TRUE(AP_ESC_2BLD6010::time_reached(0x00000005U, 0xFFFFFFF0U));
    EXPECT_FALSE(AP_ESC_2BLD6010::time_reached(0xFFFFFFF0U, 0x00000005U));

    bool logged_valid = false;
    bool last_healthy = false;
    EXPECT_TRUE(AP_ESC_2BLD6010::health_transition_due(false, logged_valid, last_healthy));
    EXPECT_FALSE(AP_ESC_2BLD6010::health_transition_due(false, logged_valid, last_healthy));
    EXPECT_TRUE(AP_ESC_2BLD6010::health_transition_due(true, logged_valid, last_healthy));
    EXPECT_FALSE(AP_ESC_2BLD6010::health_transition_due(true, logged_valid, last_healthy));
}

TEST(AP_ESC_2BLD6010, ResponseStreamRetainsPartialFrame)
{
    uint8_t frame[NORMAL_FRAME_LENGTH];
    make_valid_frame(frame);
    AP_ESC_2BLD6010::ResponseStream stream;
    AP_ESC_2BLD6010::Data data = sentinel_data();

    ASSERT_TRUE(stream.append(frame, 8));
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::INCOMPLETE, stream.next(2, data));
    expect_data_unchanged(data);
    EXPECT_EQ(8, stream.buffered_length());

    ASSERT_TRUE(stream.append(&frame[8], sizeof(frame) - 8));
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::VALID, stream.next(2, data));
    EXPECT_EQ(0, stream.buffered_length());
    EXPECT_EQ(2900, data.temperature_cdeg);
}

TEST(AP_ESC_2BLD6010, ResponseStreamDropsCompleteLateFrameWithoutCrossPublishing)
{
    uint8_t late_frame[NORMAL_FRAME_LENGTH];
    uint8_t current_frame[NORMAL_FRAME_LENGTH];
    make_valid_frame(late_frame, 1);
    make_valid_frame(current_frame, 2);
    set_register(late_frame, 2, 111);
    set_register(current_frame, 2, 222);
    update_crc(late_frame);
    update_crc(current_frame);

    AP_ESC_2BLD6010::ResponseStream stream;
    AP_ESC_2BLD6010::Data current_data = sentinel_data();
    ASSERT_TRUE(stream.append(late_frame, sizeof(late_frame)));
    ASSERT_TRUE(stream.append(current_frame, sizeof(current_frame)));
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::ADDRESS_MISMATCH, stream.next(2, current_data));
    expect_data_unchanged(current_data);
    EXPECT_EQ(NORMAL_FRAME_LENGTH, stream.buffered_length());
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::VALID, stream.next(2, current_data));
    EXPECT_FLOAT_EQ(222.0f, current_data.rpm);
}

TEST(AP_ESC_2BLD6010, ResponseStreamDropsLateExceptionAndRecoversFromGarbage)
{
    uint8_t late_exception[EXCEPTION_FRAME_LENGTH];
    uint8_t valid[NORMAL_FRAME_LENGTH];
    make_exception_frame(late_exception, 3);
    make_valid_frame(valid, 4);
    const uint8_t garbage[] = {0xAA, 0x55, 0x00};

    AP_ESC_2BLD6010::ResponseStream stream;
    AP_ESC_2BLD6010::Data data {};
    ASSERT_TRUE(stream.append(late_exception, sizeof(late_exception)));
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::ADDRESS_MISMATCH, stream.next(4, data));
    EXPECT_EQ(0, stream.buffered_length());

    ASSERT_TRUE(stream.append(garbage, sizeof(garbage)));
    ASSERT_TRUE(stream.append(valid, sizeof(valid)));
    AP_ESC_2BLD6010::ParseResult result = AP_ESC_2BLD6010::ParseResult::INCOMPLETE;
    for (uint8_t i = 0; i < 8 && result != AP_ESC_2BLD6010::ParseResult::VALID; i++) {
        result = stream.next(4, data);
    }
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::VALID, result);
}

TEST(AP_ESC_2BLD6010, ResponseStreamRejectsOverflowAndClassifiesCertainFailures)
{
    AP_ESC_2BLD6010::ResponseStream stream;
    uint8_t oversized[65] {};
    EXPECT_FALSE(stream.append(nullptr, 1));
    EXPECT_FALSE(stream.append(oversized, sizeof(oversized)));

    uint8_t frame[NORMAL_FRAME_LENGTH];
    make_valid_frame(frame, 2);
    frame[NORMAL_CRC_LOW_INDEX] ^= 0x01;
    AP_ESC_2BLD6010::Data data = sentinel_data();
    ASSERT_TRUE(stream.append(frame, sizeof(frame)));
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::CRC_ERROR, stream.next(2, data));
    expect_data_unchanged(data);

    stream.reset();
    make_valid_frame(frame, 2);
    frame[2] = 0x0E;
    ASSERT_TRUE(stream.append(frame, sizeof(frame)));
    EXPECT_EQ(AP_ESC_2BLD6010::ParseResult::BAD_RESPONSE, stream.next(2, data));
    expect_data_unchanged(data);
}

#endif

AP_GTEST_MAIN()
