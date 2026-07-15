#pragma once

#include "AP_2BLD6010_config.h"

#if AP_2BLD6010_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_2BLD6010 : public AP_ESC_Telem_Backend {
public:
    struct Data {
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
        MODBUS_EXCEPTION,
    };

    class ResponseStream {
    public:
        bool append(const uint8_t *bytes, uint8_t length);
        ParseResult next(uint8_t expected_address, Data &data);
        void reset();
        uint8_t buffered_length() const { return _length; }

    private:
        void consume(uint8_t count);

        uint8_t _buffer[64] {};
        uint8_t _length {};
    };

    AP_2BLD6010();

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    bool enabled() const;
    uint8_t configured_count() const;
    bool healthy(uint8_t instance) const;
    bool has_fault(uint8_t instance) const;
    bool get_data(uint8_t instance, Data &out) const;

    static void build_request(uint8_t address, uint8_t request[8]);
    static ParseResult parse_response(const uint8_t *frame, uint8_t frame_len, uint8_t expected_address, Data &data);
    static bool time_reached(uint32_t now_ms, uint32_t deadline_ms);
    static bool data_is_healthy(uint32_t now_ms, uint32_t last_update_ms, uint32_t timeout_ms, bool has_valid_data);
    static bool health_transition_due(bool healthy_state, bool &logged_valid, bool &last_logged_healthy);
    static uint8_t next_instance(uint8_t current_instance, uint8_t configured_count);

private:
    static constexpr uint8_t MAX_INSTANCES = 2;
    static constexpr uint8_t REQUEST_LENGTH = 8;
    static constexpr uint8_t RESPONSE_LENGTH = 21;
    static constexpr uint8_t EXCEPTION_LENGTH = 5;
    static constexpr uint8_t RX_BUFFER_SIZE = 64;
    static constexpr uint16_t RESPONSE_TIMEOUT_MS = 40;

    enum class State : uint8_t {
        IDLE,
        WAITING_RESPONSE,
    };

    void configure();
    void send_request(uint32_t now_ms);
    void read_response(uint32_t now_ms);
    bool process_rx_buffer(uint32_t now_ms);
    void handle_valid_response(uint8_t instance, uint32_t now_ms);
    void advance_instance(uint32_t now_ms);
    void update_health_log(uint8_t instance, uint32_t now_ms);
    void write_log(uint8_t instance, uint32_t now_ms, bool healthy_state, bool force);

    AP_Int8 _enable;
    AP_Int8 _num_escs;
    AP_Int16 _address1;
    AP_Int16 _address2;
    AP_Int8 _esc_offset;
    AP_Int8 _rate_hz;
    AP_Int16 _timeout_ms;

    AP_HAL::UARTDriver *_uart;
    Data _data[MAX_INSTANCES] {};
    uint8_t _addresses[MAX_INSTANCES] {};
    uint8_t _configured_count;
    uint8_t _current_instance;
    uint8_t _requested_address;
    State _state;
    uint32_t _request_sent_ms;
    uint32_t _next_request_ms;
    uint32_t _health_check_start_ms[MAX_INSTANCES] {};
    ResponseStream _response_stream;
    bool _has_valid_data[MAX_INSTANCES] {};
    bool _logged_health_valid[MAX_INSTANCES] {};
    bool _last_logged_healthy[MAX_INSTANCES] {};
    uint32_t _last_log_ms[MAX_INSTANCES] {};
};

#endif
