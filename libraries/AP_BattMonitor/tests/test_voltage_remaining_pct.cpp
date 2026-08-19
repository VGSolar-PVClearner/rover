#include <AP_gtest.h>

#include <AP_BattMonitor/AP_BattMonitor_Backend.h>

TEST(AP_BATTMONITOR_VOLTAGE_PERCENTAGE, test_voltage_range)
{
    uint8_t percentage = 0;

    EXPECT_TRUE(AP_BattMonitor_Backend::calculate_voltage_remaining_pct(25.2f, 19.8f, 25.2f, percentage));
    EXPECT_EQ(100, percentage);

    EXPECT_TRUE(AP_BattMonitor_Backend::calculate_voltage_remaining_pct(22.5f, 19.8f, 25.2f, percentage));
    EXPECT_EQ(50, percentage);

    EXPECT_TRUE(AP_BattMonitor_Backend::calculate_voltage_remaining_pct(19.8f, 19.8f, 25.2f, percentage));
    EXPECT_EQ(0, percentage);

    EXPECT_TRUE(AP_BattMonitor_Backend::calculate_voltage_remaining_pct(28.0f, 19.8f, 25.2f, percentage));
    EXPECT_EQ(100, percentage);

    EXPECT_TRUE(AP_BattMonitor_Backend::calculate_voltage_remaining_pct(18.0f, 19.8f, 25.2f, percentage));
    EXPECT_EQ(0, percentage);
}

TEST(AP_BATTMONITOR_VOLTAGE_PERCENTAGE, test_invalid_voltage_range)
{
    uint8_t percentage = 0;

    EXPECT_FALSE(AP_BattMonitor_Backend::calculate_voltage_remaining_pct(22.5f, 25.2f, 19.8f, percentage));
    EXPECT_FALSE(AP_BattMonitor_Backend::calculate_voltage_remaining_pct(22.5f, 22.5f, 22.5f, percentage));
    EXPECT_FALSE(AP_BattMonitor_Backend::calculate_voltage_remaining_pct(0.0f, 19.8f, 25.2f, percentage));
}

AP_GTEST_MAIN()
