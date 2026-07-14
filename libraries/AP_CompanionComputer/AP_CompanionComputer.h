#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include "AP_CompanionComputer_config.h"

/*
 * VGSolar NCU（导引单元，0xAA）↔ FCU（飞控，0xBB）串口通讯库。
 *
 * 职责边界：
 *   本库：帧收发、解析、ACK/参数反馈、状态帧填充、滚刷运行参数缓存。
 *   ModeVGSolar：运动控制、子模式状态机、心跳超时、吸盘/导航/转弯执行。
 *
 * 调度（Rover.cpp）：
 *   50Hz  receive_companion_computer() → update()        收 NCU 帧
 *   10Hz  send2_companion_computer()   → publish_*()     Mode 写入模式侧字段
 *                                       → send_data()     0xBB 0x01 状态反馈
 *                                       → send_nav_data() 0xBB 0x04 导航状态  （导航现在用不到）
 *
 * 上行发送时机：
 *   事件帧 0x02/0x03 — parse_* 或 Mode 调用时立即 send_frame(EVENT)
 *   周期帧 0x01/0x04 — 10Hz 任务内 send_frame(PERIODIC)
 */
class AP_CompanionComputer
{
public:
    AP_CompanionComputer();

    /* Do not allow copies */
    AP_CompanionComputer(const AP_CompanionComputer &other) = delete;
    AP_CompanionComputer &operator=(const AP_CompanionComputer&) = delete;

    static AP_CompanionComputer *get_singleton()
    {
        return _singleton;
    }

    void init();   // 初始化串口
    void update(); // 解析NCU数据
    void send_data(); // 发送FCU状态

    // ModeVGSolar 导航 ACK（0xBB 0x02，cmd_type=NCU_CMD_POSITION）
    void send_position_ack(uint8_t status);
    // ModeVGSolar 导航期间更新；send_nav=true 时本周期发送 0xBB 0x04
    void set_nav_status(const NavStatusData &data, bool send_nav);
    void send_nav_data();

    // ModeVGSolar 每周期更新：控制模式、急停/转弯标志、模式侧故障位
    void update_mode_status(uint8_t control_mode, bool estop, bool turning, uint16_t fault_bits);
    // 离开 VGSOLAR 模式时清零模式侧反馈字段
    void reset_mode_status();

    // --- NCU 指令缓存（parse 写入，ModeVGSolar 通过 is_new_* 消费）---
    // _new_cmd_flags: bit0=SPEED bit1=TURN bit2=POSITION bit3=SYSTEM
    const SpeedCtrlData& get_latest_speed_ctrl() const
    {
        return _latest_speed_ctrl;
    }
    const TurnData& get_latest_turn() const
    {
        return _latest_turn;
    }
    const PositionData& get_latest_position() const
    {
        return _latest_position;
    }
    const SystemCtrlData& get_latest_system_ctrl() const
    {
        return _latest_system_ctrl;
    }

    bool is_new_speed_ctrl() const
    {
        return _new_cmd_flags & (1<<0);
    }
    bool is_new_turn() const
    {
        return _new_cmd_flags & (1<<1);
    }
    bool is_new_position() const
    {
        return _new_cmd_flags & (1<<2);
    }
    bool is_new_system_ctrl() const
    {
        return _new_cmd_flags & (1<<3);
    }

    void clear_new_speed_flag()
    {
        _new_cmd_flags &= ~(1<<0);
    }
    void clear_new_turn_flag()
    {
        _new_cmd_flags &= ~(1<<1);
    }
    void clear_new_position_flag()
    {
        _new_cmd_flags &= ~(1<<2);
    }
    void clear_new_system_flag()
    {
        _new_cmd_flags &= ~(1<<3);
    }

    // 急停锁存；parse_system_ctrl 置位，Mode 层执行停车/关刷/吸盘
    bool is_estop_active() const
    {
        return _estop_active;
    }

    // 滚刷运行参数停止：PWM 回中位并清零内存状态
    void stop_brushes();

    // 主电池(instance 0)电量有效且 ≤ LOW_BATT_PCT_THRESHOLD；无有效读数时返回 false
    bool is_low_battery() const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_CompanionComputer *_singleton;

    enum class TxPriority : uint8_t {
        EVENT,    // 0x02/0x03：按需即时；tx 满时仍尝试写并计 _tx_drop_event
        PERIODIC, // 0x01/0x04：10Hz；tx 满时丢弃本帧并计 _tx_drop_periodic
    };

    // Parameters
    AP_Int8 _enable;
    AP_Int8 _port_index;

    AP_HAL::UARTDriver *_uart;

    // NCU → FCU 接收状态机
    enum class RxState {
        WAITING_HEADER1,
        WAITING_HEADER2,
        WAITING_SOURCE,
        WAITING_TYPE,
        WAITING_LENGTH,
        RECEIVING_DATA
    } _rx_state;

    std::array<uint8_t, COMPANION_RECV_TOTAL_LENGTH> _rx_buffer;
    uint8_t _rx_count;
    uint32_t _rx_start_time;
    uint32_t _last_sent_ms;  // send_data 10Hz 限速
    uint16_t _tx_drop_event;
    uint16_t _tx_drop_periodic;

    uint8_t _cmd_type;
    uint8_t _data_len;

    // NCU 指令缓存；bit0=SPEED bit1=TURN bit2=POSITION bit3=SYSTEM
    SpeedCtrlData   _latest_speed_ctrl;
    TurnData        _latest_turn;
    PositionData    _latest_position;
    SystemCtrlData  _latest_system_ctrl;
    uint8_t _new_cmd_flags;
    bool _estop_active;

    // Mode 写入的控制模式/急停/转弯/模式侧故障；与 collect_sensor_faults() 在 send_data 中合并
    uint8_t _fb_control_mode;
    bool _fb_estop;
    bool _fb_turning;
    uint16_t _fb_fault_bits;      // bit7 超时、bit10 导航失败、bit4 吸盘等（Mode 侧）
    bool _fb_mode_status_valid;   // 非 VGSL 或未 publish 时为 false，control_mode 回退 STANDBY

    NavStatusData _nav_status;    // Mode publish_nav_status_feedback() 写入
    bool _nav_status_send;        // true 时本周期 send_nav_data() 发一帧后清零

    void process_received_data(uint8_t oneByte);
    void parse_speed_ctrl();
    void parse_turn();
    void parse_param_write();
    void parse_param_read();
    void parse_system_ctrl();
    void parse_position();

    // 校验和: byte2..byte(n-2) 累加和低 8 位（帧头不参与）
    uint8_t calculate_checksum(const uint8_t *data, uint8_t len) const;
    bool validate_packet() const;

    // 组 FCU→NCU 帧到 out；成功返回整帧长度，失败返回 0
    size_t build_frame(uint8_t cmd_content, const uint8_t *body, uint8_t body_len,
                       uint8_t *out, size_t out_size) const;
    bool send_frame(const uint8_t *data, size_t len, TxPriority pri);

    void send_response(uint8_t cmd_type, uint8_t status);
    void send_param_feedback(const ParamFeedbackData &data);

    bool write_runtime_param(uint16_t param_index, uint8_t param_type, uint32_t param_value,
                             ParamFeedbackData &feedback_out);
    bool read_runtime_param(uint16_t param_index, ParamFeedbackData &feedback_out);
    void apply_brush_runtime_params();

    uint32_t _brush_front_on;
    uint32_t _brush_rear_on;
    uint32_t _brush_power_pct;

    // 传感器侧故障（IMU/GPS/轮速计/倾角/低电压）；与 _fb_fault_bits 按位或后上报
    uint16_t collect_sensor_faults() const;
    // 根据 fault/estop/turning/velocity 计算 motion_state；FAULT 优先于 ESTOP
    static uint8_t compute_motion_state(int16_t velocity_cms, bool estop, bool turning, uint16_t fault_code);
};

namespace AP
{
AP_CompanionComputer &companioncomputer();
};
