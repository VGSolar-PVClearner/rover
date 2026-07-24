#pragma once

#include "AP_ESC_Telem_config.h"

#if AP_ESC_TELEM_2BLD6010_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_ESC_Telem_2BLD6010 : public AP_ESC_Telem_Backend {
public:
    static constexpr uint8_t MAX_ESC_COUNT = 4;

    struct DiagnosticData {
        uint16_t fault_code;
        float current_a;
        float rpm;
        int16_t temperature_cdeg;
        float voltage_v;
        uint8_t direction;
        uint32_t hall_count;
        uint32_t last_update_ms;
        uint32_t success_count;
        uint32_t crc_error_count;
        uint32_t timeout_count;
        uint32_t bad_response_count;
        uint32_t modbus_exception_count;
    };

    enum class ParseResult : uint8_t {
        INCOMPLETE,
        VALID,
        CRC_ERROR,
        BAD_RESPONSE,
        ADDRESS_MISMATCH,
        MODBUS_EXCEPTION,
    };

    enum class ConfigError : uint8_t {
        NONE,
        INVALID_ENABLE,
        INVALID_COUNT,
        INVALID_ADDRESS,
        DUPLICATE_ADDRESS,
        INVALID_ESC_OFFSET,
        ESC_INDEX_RANGE,
        INVALID_RATE,
        INVALID_TIMEOUT,
    };

    struct RawConfig {
        int32_t enable;
        int32_t count;
        int32_t esc_offset;
        int32_t rate_hz;
        int32_t health_timeout_ms;
        int32_t addresses[MAX_ESC_COUNT];
    };

    struct ValidatedConfig {
        bool enabled;
        uint8_t count;
        uint8_t esc_offset;
        uint8_t rate_hz;
        uint16_t health_timeout_ms;
        uint16_t poll_interval_ms;
        uint8_t addresses[MAX_ESC_COUNT];
    };

    class ResponseStream {
    public:
        bool append(const uint8_t *bytes, uint8_t length);
        ParseResult next(uint8_t expected_address, DiagnosticData &data);
        void reset();
        uint8_t buffered_length() const { return _length; }

    private:
        void consume(uint8_t count);

        uint8_t _buffer[64] {};
        uint8_t _length {};
    };

    AP_ESC_Telem_2BLD6010();

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    bool enabled() const;
    uint8_t configured_count() const;
    ConfigError config_error() const;
    bool healthy(uint8_t instance) const;
    bool has_fault(uint8_t instance) const;
    bool get_diagnostics(uint8_t instance, DiagnosticData &out) const;

    static void build_request(uint8_t address, uint8_t request[8]);
    static ParseResult parse_response(const uint8_t *frame, uint8_t frame_len, uint8_t expected_address, DiagnosticData &data);
    static bool time_reached(uint32_t now_ms, uint32_t deadline_ms);
    static bool data_is_healthy(uint32_t now_ms, uint32_t last_update_ms, uint32_t timeout_ms, bool has_valid_data);
    static bool health_transition_due(bool healthy_state, bool &logged_valid, bool &last_logged_healthy);
    static uint8_t next_instance(uint8_t current_instance, uint8_t configured_count);
    static bool valid_modbus_address(int16_t address);
    static ConfigError validate_config(const RawConfig &raw, uint8_t max_esc_instances, ValidatedConfig &validated);
    static uint16_t polling_interval_ms(uint8_t rate_hz, uint8_t configured_count);

private:
    static constexpr uint8_t REQUEST_LENGTH = 8;
    static constexpr uint8_t RESPONSE_LENGTH = 21;
    static constexpr uint8_t EXCEPTION_LENGTH = 5;
    static constexpr uint8_t RX_BUFFER_SIZE = 64;
    static constexpr uint16_t RESPONSE_TIMEOUT_MS = 40;
    static constexpr uint16_t MIN_POLL_INTERVAL_MS = 10;
    static constexpr uint8_t MIN_RATE_HZ = 1;
    static constexpr uint8_t MAX_RATE_HZ = 20;
    static constexpr uint16_t MIN_HEALTH_TIMEOUT_MS = 100;
    static constexpr uint16_t MAX_HEALTH_TIMEOUT_MS = 5000;
    static constexpr uint8_t MODBUS_MIN_SLAVE_ADDRESS = 1;
    static constexpr uint8_t MODBUS_MAX_SLAVE_ADDRESS = 247;

    enum class State : uint8_t {
        IDLE,
        WAITING_RESPONSE,
    };

    struct InstanceState {
        uint8_t address;
        DiagnosticData data;
        uint32_t last_request_ms;
        uint32_t health_check_start_ms;
        bool has_valid_data;
        bool logged_health_valid;
        bool last_logged_healthy;
        uint32_t last_log_ms;
    };

    void reset_runtime_state();
    RawConfig get_raw_config() const;
    void apply_config(const ValidatedConfig &config);
    void report_config_error() const;
    void send_request(uint32_t now_ms);
    void read_response(uint32_t now_ms);
    bool process_rx_buffer(uint32_t now_ms);
    void handle_valid_response(uint8_t instance, uint32_t now_ms);
    void advance_instance();
    void update_health_log(uint8_t instance, uint32_t now_ms);
    void write_log(uint8_t instance, uint32_t now_ms, bool healthy_state, bool force);

    AP_Int8 _enable;
    AP_Int8 _num_escs;
    AP_Int16 _address_params[MAX_ESC_COUNT];
    AP_Int8 _esc_offset;
    AP_Int8 _rate_hz;
    AP_Int16 _timeout_ms;

    AP_HAL::UARTDriver *_uart;
    InstanceState _instances[MAX_ESC_COUNT] {};
    uint8_t _configured_count;
    uint8_t _validated_esc_offset;
    uint8_t _validated_rate_hz;
    uint16_t _health_timeout_ms;
    uint16_t _poll_interval_ms;
    ConfigError _config_error;
    uint8_t _current_instance;
    uint8_t _requested_address;
    State _state;
    uint32_t _request_sent_ms;
    uint32_t _next_request_ms;
    ResponseStream _response_stream;
};

#endif
