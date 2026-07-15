#include <AP_gtest.h>

#include <AP_2BLD6010/AP_2BLD6010.h>
#include <AP_Math/crc.h>

#include <cstring>

#if AP_2BLD6010_ENABLED

static void set_register(uint8_t frame[21], uint8_t index, uint16_t value)
{
    frame[3 + index * 2] = uint8_t(value >> 8);
    frame[4 + index * 2] = uint8_t(value);
}

static void update_crc(uint8_t frame[21])
{
    const uint16_t crc = calc_crc_modbus(frame, 19);
    frame[19] = uint8_t(crc);
    frame[20] = uint8_t(crc >> 8);
}

static void make_valid_frame(uint8_t frame[21], uint8_t address = 2)
{
    const uint8_t base[] = {
        address, 0x03, 0x10, 0x00, 0x00, 0x00, 0x75,
        0x00, 0x00, 0x00, 0x1D, 0x00, 0xE5, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    memcpy(frame, base, sizeof(base));
    update_crc(frame);
}

static AP_2BLD6010::Data sentinel_data()
{
    AP_2BLD6010::Data data {};
    data.fault_code = 77;
    data.current_a = 12.5f;
    data.rpm = 3456.0f;
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

static void expect_data_unchanged(const AP_2BLD6010::Data &data)
{
    EXPECT_EQ(77, data.fault_code);
    EXPECT_FLOAT_EQ(12.5f, data.current_a);
    EXPECT_FLOAT_EQ(3456.0f, data.rpm);
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

TEST(AP_2BLD6010, BuildRequest)
{
    uint8_t request[8];
    AP_2BLD6010::build_request(2, request);
    const uint8_t expected[] = {0x02, 0x03, 0x00, 0x20, 0x00, 0x08, 0x45, 0xF5};
    EXPECT_EQ(0, memcmp(request, expected, sizeof(expected)));

    AP_2BLD6010::build_request(1, request);
    EXPECT_EQ(0x01, request[0]);
    EXPECT_NE(0, memcmp(request, expected, sizeof(expected)));
}

TEST(AP_2BLD6010, ParseKnownResponse)
{
    uint8_t frame[21];
    make_valid_frame(frame);
    AP_2BLD6010::Data data {};
    EXPECT_EQ(AP_2BLD6010::ParseResult::VALID,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    EXPECT_EQ(0, data.fault_code);
    EXPECT_FLOAT_EQ(1.17f, data.current_a);
    EXPECT_FLOAT_EQ(0.0f, data.rpm);
    EXPECT_EQ(2900, data.temperature_cdeg);
    EXPECT_FLOAT_EQ(22.9f, data.voltage_v);
    EXPECT_EQ(0, data.direction);
    EXPECT_EQ(0U, data.hall_count);
}

TEST(AP_2BLD6010, RejectsIncompleteAndCorruptFrames)
{
    uint8_t frame[21];
    make_valid_frame(frame);
    AP_2BLD6010::Data data = sentinel_data();
    EXPECT_EQ(AP_2BLD6010::ParseResult::INCOMPLETE,
              AP_2BLD6010::parse_response(frame, 12, 2, data));
    expect_data_unchanged(data);
    EXPECT_EQ(AP_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 1, data));
    expect_data_unchanged(data);

    frame[20] ^= 0x01;
    EXPECT_EQ(AP_2BLD6010::ParseResult::CRC_ERROR,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);
}

TEST(AP_2BLD6010, ParsesSignedTemperatureAndHallCount)
{
    uint8_t frame[21];
    make_valid_frame(frame);
    set_register(frame, 0, 5);
    set_register(frame, 1, 100);
    set_register(frame, 2, 1234);
    set_register(frame, 3, uint16_t(int16_t(-5)));
    set_register(frame, 4, 230);
    set_register(frame, 5, 2);
    set_register(frame, 6, 0x1234);
    set_register(frame, 7, 0x5678);
    update_crc(frame);

    AP_2BLD6010::Data data {};
    EXPECT_EQ(AP_2BLD6010::ParseResult::VALID,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    EXPECT_EQ(5, data.fault_code);
    EXPECT_FLOAT_EQ(1.0f, data.current_a);
    EXPECT_FLOAT_EQ(1234.0f, data.rpm);
    EXPECT_EQ(-500, data.temperature_cdeg);
    EXPECT_EQ(2, data.direction);
    EXPECT_EQ(0x12345678U, data.hall_count);
}

TEST(AP_2BLD6010, HandlesModbusException)
{
    uint8_t frame[] = {0x02, 0x83, 0x02, 0x00, 0x00};
    const uint16_t crc = calc_crc_modbus(frame, 3);
    frame[3] = uint8_t(crc);
    frame[4] = uint8_t(crc >> 8);

    AP_2BLD6010::Data data {};
    EXPECT_EQ(AP_2BLD6010::ParseResult::MODBUS_EXCEPTION,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    frame[4] ^= 0x01;
    EXPECT_EQ(AP_2BLD6010::ParseResult::CRC_ERROR,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));

    frame[1] = 0x84;
    const uint16_t other_crc = calc_crc_modbus(frame, 3);
    frame[3] = uint8_t(other_crc);
    frame[4] = uint8_t(other_crc >> 8);
    EXPECT_EQ(AP_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
}

TEST(AP_2BLD6010, RejectsOutOfRangeTemperatureWithoutPollution)
{
    uint8_t frame[21];
    make_valid_frame(frame);
    AP_2BLD6010::Data data = sentinel_data();

    set_register(frame, 3, 151);
    update_crc(frame);
    EXPECT_EQ(AP_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);

    set_register(frame, 3, uint16_t(int16_t(-41)));
    update_crc(frame);
    EXPECT_EQ(AP_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);
}

TEST(AP_2BLD6010, AcceptsTemperatureBoundaries)
{
    uint8_t frame[21];
    make_valid_frame(frame);
    AP_2BLD6010::Data data {};

    set_register(frame, 3, uint16_t(int16_t(-40)));
    update_crc(frame);
    EXPECT_EQ(AP_2BLD6010::ParseResult::VALID,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    EXPECT_EQ(-4000, data.temperature_cdeg);

    set_register(frame, 3, 150);
    update_crc(frame);
    EXPECT_EQ(AP_2BLD6010::ParseResult::VALID,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    EXPECT_EQ(15000, data.temperature_cdeg);
}

TEST(AP_2BLD6010, RejectsInvalidDirectionWithoutPollution)
{
    uint8_t frame[21];
    make_valid_frame(frame);
    AP_2BLD6010::Data data = sentinel_data();

    set_register(frame, 5, 4);
    update_crc(frame);
    EXPECT_EQ(AP_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);

    set_register(frame, 5, 0x0100);
    update_crc(frame);
    EXPECT_EQ(AP_2BLD6010::ParseResult::BAD_RESPONSE,
              AP_2BLD6010::parse_response(frame, sizeof(frame), 2, data));
    expect_data_unchanged(data);
}

TEST(AP_2BLD6010, TimeComparisonHandlesMillisWrap)
{
    EXPECT_TRUE(AP_2BLD6010::time_reached(100, 99));
    EXPECT_FALSE(AP_2BLD6010::time_reached(99, 100));
    EXPECT_TRUE(AP_2BLD6010::time_reached(0x00000005U, 0xFFFFFFF0U));
    EXPECT_FALSE(AP_2BLD6010::time_reached(0xFFFFFFF0U, 0x00000005U));
    EXPECT_TRUE(AP_2BLD6010::data_is_healthy(0x00000005U, 0xFFFFFFF0U, 30, true));
    EXPECT_FALSE(AP_2BLD6010::data_is_healthy(0x00000020U, 0xFFFFFFF0U, 30, true));
    EXPECT_FALSE(AP_2BLD6010::data_is_healthy(100, 100, 500, false));
}

TEST(AP_2BLD6010, HealthTransitionsAreLoggedOnce)
{
    bool logged_valid = false;
    bool last_healthy = false;
    EXPECT_TRUE(AP_2BLD6010::health_transition_due(false, logged_valid, last_healthy));
    EXPECT_FALSE(AP_2BLD6010::health_transition_due(false, logged_valid, last_healthy));
    EXPECT_TRUE(AP_2BLD6010::health_transition_due(true, logged_valid, last_healthy));
    EXPECT_FALSE(AP_2BLD6010::health_transition_due(true, logged_valid, last_healthy));
    EXPECT_TRUE(AP_2BLD6010::health_transition_due(false, logged_valid, last_healthy));
}

TEST(AP_2BLD6010, ResponseStreamRetainsPartialFrame)
{
    uint8_t frame[21];
    make_valid_frame(frame);
    AP_2BLD6010::ResponseStream stream;
    AP_2BLD6010::Data data = sentinel_data();

    ASSERT_TRUE(stream.append(frame, 8));
    EXPECT_EQ(AP_2BLD6010::ParseResult::INCOMPLETE, stream.next(2, data));
    expect_data_unchanged(data);
    EXPECT_EQ(8, stream.buffered_length());

    ASSERT_TRUE(stream.append(&frame[8], sizeof(frame) - 8));
    EXPECT_EQ(AP_2BLD6010::ParseResult::VALID, stream.next(2, data));
    EXPECT_EQ(0, stream.buffered_length());
    EXPECT_EQ(2900, data.temperature_cdeg);
    EXPECT_EQ(AP_2BLD6010::ParseResult::INCOMPLETE, stream.next(2, data));
}

TEST(AP_2BLD6010, ResponseStreamRecoversFromGarbage)
{
    uint8_t frame[21];
    make_valid_frame(frame);
    const uint8_t garbage[] = {0xAA, 0x55, 0x00};
    AP_2BLD6010::ResponseStream stream;
    AP_2BLD6010::Data data {};
    ASSERT_TRUE(stream.append(garbage, sizeof(garbage)));
    ASSERT_TRUE(stream.append(frame, sizeof(frame)));

    uint8_t bad_responses = 0;
    AP_2BLD6010::ParseResult result;
    do {
        result = stream.next(2, data);
        if (result == AP_2BLD6010::ParseResult::BAD_RESPONSE) {
            bad_responses++;
        }
    } while (result == AP_2BLD6010::ParseResult::BAD_RESPONSE);
    EXPECT_EQ(3, bad_responses);
    EXPECT_EQ(AP_2BLD6010::ParseResult::VALID, result);
}

TEST(AP_2BLD6010, ResponseStreamRecoversAfterCrcError)
{
    uint8_t corrupt[21];
    uint8_t valid[21];
    make_valid_frame(corrupt);
    make_valid_frame(valid);
    corrupt[20] ^= 0x01;
    AP_2BLD6010::ResponseStream stream;
    AP_2BLD6010::Data data {};
    ASSERT_TRUE(stream.append(corrupt, sizeof(corrupt)));
    ASSERT_TRUE(stream.append(valid, sizeof(valid)));

    bool saw_crc_error = false;
    AP_2BLD6010::ParseResult result = AP_2BLD6010::ParseResult::INCOMPLETE;
    for (uint8_t i = 0; i < 32 && result != AP_2BLD6010::ParseResult::VALID; i++) {
        result = stream.next(2, data);
        saw_crc_error |= result == AP_2BLD6010::ParseResult::CRC_ERROR;
    }
    EXPECT_TRUE(saw_crc_error);
    EXPECT_EQ(AP_2BLD6010::ParseResult::VALID, result);
}

TEST(AP_2BLD6010, ResponseStreamKeepsAddressesIsolated)
{
    uint8_t address1_frame[21];
    uint8_t address2_frame[21];
    make_valid_frame(address1_frame, 1);
    make_valid_frame(address2_frame, 2);
    set_register(address1_frame, 2, 111);
    set_register(address2_frame, 2, 222);
    update_crc(address1_frame);
    update_crc(address2_frame);

    AP_2BLD6010::ResponseStream stream;
    AP_2BLD6010::Data data = sentinel_data();
    ASSERT_TRUE(stream.append(address1_frame, sizeof(address1_frame)));
    AP_2BLD6010::ParseResult result;
    do {
        result = stream.next(2, data);
    } while (result == AP_2BLD6010::ParseResult::BAD_RESPONSE);
    EXPECT_EQ(AP_2BLD6010::ParseResult::INCOMPLETE, result);
    expect_data_unchanged(data);
    stream.reset();
    ASSERT_TRUE(stream.append(address2_frame, sizeof(address2_frame)));
    EXPECT_EQ(AP_2BLD6010::ParseResult::VALID, stream.next(2, data));
    EXPECT_FLOAT_EQ(222.0f, data.rpm);

    EXPECT_FALSE(AP_2BLD6010::data_is_healthy(1000, 100, 500, true));
    EXPECT_TRUE(AP_2BLD6010::data_is_healthy(1000, 900, 500, true));
    EXPECT_EQ(1, AP_2BLD6010::next_instance(0, 2));
    EXPECT_EQ(0, AP_2BLD6010::next_instance(1, 2));
    EXPECT_EQ(0, AP_2BLD6010::next_instance(0, 1));
}

#endif

AP_GTEST_MAIN()
