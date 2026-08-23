#include "AP_CompanionComputer.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_WheelEncoder/AP_WheelEncoder.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_config.h>
#include <AP_Math/AP_Math.h>
#include <AP_Brush/AP_Brush.h>

// AP_CompanionComputer 实现：串口收发、NCU 指令解析、FCU 反馈组帧。
// DataFlash 通信日志见 AP_CompanionComputer_Logging.cpp。
// 运动/导航/转弯/急停执行逻辑在 Rover/mode_vgsolar.cpp。

const AP_Param::GroupInfo AP_CompanionComputer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Companion computer
    // @Description: Enable communication with companion computer
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_CompanionComputer, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PORT
    // @DisplayName: instance of companion computer Serial Port
    // @Description: The nth serial port instance found starting from serial0 that uses serial protocol 50
    // @Values: 0:instance0, 1:instance1
    // @User: Advanced
    AP_GROUPINFO("PORT", 2, AP_CompanionComputer, _port_index, 0),

    // @Param: LOG
    // @DisplayName: NCU communication DataFlash logging
    // @Description: 0:disabled, 1:NCLK+NSPD(change/5Hz)+NTRN+NEVT, 2:same with NSPD at up to 10Hz
    // @Values: 0:Disabled, 1:Default, 2:VerboseSpeed
    // @User: Advanced
    AP_GROUPINFO("LOG", 3, AP_CompanionComputer, _log, 1),

    AP_GROUPEND
};

AP_CompanionComputer::AP_CompanionComputer() :
    _rx_state(RxState::WAITING_HEADER1),
    _rx_count(0),
    _uart(nullptr),
    _last_sent_ms(0),
    _tx_drop_event(0),
    _tx_drop_periodic(0),
    // NCLK 秒窗计数 / NSPD 降频状态
    _last_rx_ok_ms(0),
    _last_nclk_ms(0),
    _rx_ok_sec(0),
    _bad_checksum_sec(0),
    _bad_length_sec(0),
    _last_nspd_ms(0),
    _last_nspd_vel_mode(0),
    _last_nspd_lin_vel(0),
    _last_nspd_yaw_data(0),
    _last_nspd_accepted(0),
    _last_nspd_reject_reason(0),
    _last_nspd_valid(false),
    _new_cmd_flags(0),
    _estop_active(false),
    _fb_control_mode(0),
    _fb_estop(false),
    _fb_turning(false),
    _fb_fault_bits(0),
    _fb_mode_status_valid(false),
    _nav_status_send(false),
    _brush_front_on(0),
    _brush_rear_on(0),
    _brush_power_pct(0)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    _rx_buffer.fill(0);
}

void AP_CompanionComputer::init()
{
    if (!_enable) {
        return;
    }

    // 查找第 CC_PORT 个 protocol=50 的串口
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_2CC, _port_index.get());
    if (_uart != nullptr) {
        _uart->begin(115200, 512, 128);
    }
}

void AP_CompanionComputer::update()
{
    if (!_enable || _uart == nullptr) {
        return;
    }

    while (_uart->available() > 0) {
        uint8_t byte;
        if (_uart->read(&byte, 1)) {
            process_received_data(byte);
        }
    }

    // CC_LOG>0 时约 1Hz 写链路健康 NCLK
    maybe_write_nclk();
}

// 逐字节状态机；校验失败静默丢弃；合法帧按 _cmd_type 分发 parse_*
// 缓冲区布局：byte0~1 帧头，2 源，3 类型，4 长度，5.. 数据体，末2 校验+0xFF
void AP_CompanionComputer::process_received_data(uint8_t oneByte)
{
    const uint32_t now = AP_HAL::millis();

    switch (_rx_state) {
    case RxState::WAITING_HEADER1:
        if (oneByte == COMPANION_FRAME_HEADER1) {
            _rx_state = RxState::WAITING_HEADER2;
            _rx_start_time = now;
            _rx_count = 0;
        }
        break;

    case RxState::WAITING_HEADER2:
        if (oneByte == COMPANION_FRAME_HEADER2) {
            _rx_state = RxState::WAITING_SOURCE;
            _rx_buffer[_rx_count++] = COMPANION_FRAME_HEADER1;
            _rx_buffer[_rx_count++] = COMPANION_FRAME_HEADER2;
        } else {
            _rx_state = RxState::WAITING_HEADER1;
        }
        break;

    case RxState::WAITING_SOURCE:
        if (oneByte != COMPANION_CMD_SOURCE_NCU) {
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }
        _rx_state = RxState::WAITING_TYPE;
        _rx_buffer[_rx_count++] = oneByte;
        break;

    case RxState::WAITING_TYPE:
        if (oneByte <= NCU_CMD_POSITION) {
            _rx_state = RxState::WAITING_LENGTH;
            _rx_buffer[_rx_count++] = oneByte;
            _cmd_type = oneByte;
        } else {
            _rx_state = RxState::WAITING_HEADER1;
        }
        break;

    case RxState::WAITING_LENGTH:
        if (oneByte > NCU_RX_MAX_DATA_LEN) {
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }
        _rx_state = RxState::RECEIVING_DATA;
        _data_len = oneByte;
        _rx_buffer[_rx_count++] = oneByte;
        break;

    case RxState::RECEIVING_DATA:
        if (now - _rx_start_time > PACKET_TIMEOUT_MS) {
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }

        if (_rx_count >= _rx_buffer.size()) {
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }

        _rx_buffer[_rx_count++] = oneByte;

        // 整帧长度 = DATA_LENGTH + 7（含帧头、校验和、结束符 0xFF）
        if (_rx_count >= (_data_len + 7)) {
            if (validate_packet()) {
                // 长度与协议表一致才分发，降低串口噪声被误解析成速度指令的概率
                // 同时累计 NCLK 的 BadLength / RxPerSec
                const uint8_t expect_len = expected_ncu_data_len(_cmd_type);
                if (expect_len == 0) {
                    // 未知类型：校验已过但无法解析，不计坏长度
                } else if (_data_len != expect_len) {
                    note_bad_length();
                } else {
                    note_rx_ok();
                    switch (_cmd_type) {
                    case NCU_CMD_SPEED_CTRL:
                        parse_speed_ctrl();
                        break;
                    case NCU_CMD_TURN:
                        parse_turn();
                        break;
                    case NCU_CMD_PARAM_WRITE:
                        parse_param_write();
                        break;
                    case NCU_CMD_PARAM_READ:
                        parse_param_read();
                        break;
                    case NCU_CMD_SYSTEM_CTRL:
                        parse_system_ctrl();
                        break;
                    case NCU_CMD_POSITION:
                        parse_position();
                        break;
                    default:
                        break;
                    }
                }
            } else {
                note_bad_checksum();  // 供 NCLK.BadChecksum
            }
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
        }
        break;
    }
}

// NCU 指令解析（写缓存 + 按需 ACK；运动执行由 ModeVGSolar 消费 _new_cmd_flags）
void AP_CompanionComputer::parse_speed_ctrl()
{
    _latest_speed_ctrl = PacketBuilder::deserialize<SpeedCtrlData>(_rx_buffer.data() + 5);
    _new_cmd_flags |= (1<<0);
    // 速度控制 10Hz 发送，协议规定不需要 ACK
}

void AP_CompanionComputer::parse_turn()
{
    _latest_turn = PacketBuilder::deserialize<TurnData>(_rx_buffer.data() + 5);
    _new_cmd_flags |= (1<<1);
    send_response(NCU_CMD_TURN, CMD_ACK_SUCCESS);
}

void AP_CompanionComputer::parse_param_write()
{
    // 成功：0x02 ACK + 0x03 回传；失败：仅 0x02 FAILED
    const ParamWriteData cmd = PacketBuilder::deserialize<ParamWriteData>(_rx_buffer.data() + 5);

    ParamFeedbackData feedback {};
    if (write_runtime_param(cmd.param_index, cmd.param_type, cmd.param_value, feedback)) {
        send_response(NCU_CMD_PARAM_WRITE, CMD_ACK_SUCCESS);
        send_param_feedback(feedback);
        apply_brush_runtime_params();
    } else {
        send_response(NCU_CMD_PARAM_WRITE, CMD_ACK_FAILED);
    }
}

void AP_CompanionComputer::parse_param_read()
{
    const ParamReadData cmd = PacketBuilder::deserialize<ParamReadData>(_rx_buffer.data() + 5);

    ParamFeedbackData feedback {};
    if (read_runtime_param(cmd.param_index, feedback)) {
        send_param_feedback(feedback);
    }
    // 未知索引：只回 0xBB 0x03，无 ACK 帧
}

void AP_CompanionComputer::parse_system_ctrl()
{
    // 仅缓存 + ACK + _estop_active；SYS_CMD_REBOOT/SHUTDOWN 由 ModeVGSolar 执行
    _latest_system_ctrl = PacketBuilder::deserialize<SystemCtrlData>(_rx_buffer.data() + 5);
    _new_cmd_flags |= (1<<3);

    if (_latest_system_ctrl.command == SYS_CMD_ESTOP) {
        _estop_active = true;
    } else if (_latest_system_ctrl.command == SYS_CMD_ESTOP_CLEAR) {
        _estop_active = false;
    }

    send_response(NCU_CMD_SYSTEM_CTRL, CMD_ACK_SUCCESS);
}

void AP_CompanionComputer::parse_position()
{
    _latest_position = PacketBuilder::deserialize<PositionData>(_rx_buffer.data() + 5);
    _new_cmd_flags |= (1<<2);
    // 导航 ACK 由 ModeVGSolar 通过 send_position_ack() 发送
}

void AP_CompanionComputer::send_position_ack(uint8_t status)
{
    send_response(NCU_CMD_POSITION, status);
}

void AP_CompanionComputer::set_nav_status(const NavStatusData &data, bool send_nav)
{
    _nav_status = data;
    _nav_status_send = send_nav;
}

// 校验和：byte2（源地址）..byte(末-2) 累加低 8 位；帧头 0xA5 0x5A 不参与
size_t AP_CompanionComputer::build_frame(uint8_t cmd_content, const uint8_t *body, uint8_t body_len,
        uint8_t *out, size_t out_size) const
{
    const size_t frame_len = size_t(body_len) + FRAME_OVERHEAD;
    if (out_size < frame_len || body_len > FCU_TX_MAX_DATA_LEN) {
        return 0;
    }

    out[0] = COMPANION_FRAME_HEADER1;
    out[1] = COMPANION_FRAME_HEADER2;
    out[2] = COMPANION_CMD_SOURCE_FC;
    out[3] = cmd_content;
    out[4] = body_len;
    if (body_len > 0 && body != nullptr) {
        memcpy(out + 5, body, body_len);
    }
    out[frame_len - 2] = calculate_checksum(out, frame_len - 2);
    out[frame_len - 1] = COMPANION_END_SIGN;
    return frame_len;
}

// EVENT：tx 满仍尝试写（可能阻塞/部分写），计 _tx_drop_event
// PERIODIC：tx 满直接丢帧，计 _tx_drop_periodic，下周期再发
bool AP_CompanionComputer::send_frame(const uint8_t *data, size_t len, TxPriority pri)
{
    if (!_enable || _uart == nullptr || data == nullptr || len == 0) {
        return false;
    }

    if (_uart->txspace() < len) {
        if (pri == TxPriority::PERIODIC) {
            _tx_drop_periodic++;
            return false;
        }
        _tx_drop_event++;
    }

    return _uart->write(data, len) == len;
}

void AP_CompanionComputer::send_nav_data()
{
    if (!_nav_status_send) {
        return;
    }

    uint8_t packet[COMPANION_SEND_NAV_LENGTH];
    const size_t frame_len = build_frame(FCU_FB_NAV_STATUS,
                                         reinterpret_cast<const uint8_t *>(&_nav_status),
                                         sizeof(NavStatusData),
                                         packet, sizeof(packet));
    if (frame_len == 0) {
        return;
    }

    if (send_frame(packet, frame_len, TxPriority::PERIODIC)) {
        _nav_status_send = false;
    }
}

// 需 BATT_MONITOR + BATT_CAPACITY + 电流监测；无有效读数返回 false（台架不误报）
bool AP_CompanionComputer::is_low_battery() const
{
    uint8_t batt_pct = 0;
    return AP::battery().capacity_remaining_pct(batt_pct, 0)
           && batt_pct <= LOW_BATT_PCT_THRESHOLD;
}

bool AP_CompanionComputer::write_runtime_param(uint16_t param_index, uint8_t param_type, uint32_t param_value,
        ParamFeedbackData &feedback_out)
{
    // 指南滚刷参数均为 uint32
    if (param_type != PARAM_TYPE_UINT32) {
        return false;
    }

    feedback_out.param_index = param_index;
    feedback_out.param_type = PARAM_TYPE_UINT32;

    // 低电压：拒绝开启滚刷；关刷与功率档位仍允许
    if (is_low_battery() && param_value != 0) {
        switch (param_index) {
        case PARAM_IDX_BRUSH_FRONT:
        case PARAM_IDX_BRUSH_REAR:
        case PARAM_IDX_BRUSH_BOTH:
            return false;
        default:
            break;
        }
    }

    switch (param_index) {
    case PARAM_IDX_BRUSH_FRONT:
        _brush_front_on = param_value ? 1U : 0U;
        feedback_out.param_value = _brush_front_on;
        return true;

    case PARAM_IDX_BRUSH_REAR:
        _brush_rear_on = param_value ? 1U : 0U;
        feedback_out.param_value = _brush_rear_on;
        return true;

    case PARAM_IDX_BRUSH_BOTH:
        _brush_front_on = param_value ? 1U : 0U;
        _brush_rear_on = param_value ? 1U : 0U;
        feedback_out.param_value = param_value ? 1U : 0U;
        return true;

    case PARAM_IDX_BRUSH_POWER:
        _brush_power_pct = constrain_uint32(param_value, 0U, 100U);
        feedback_out.param_value = _brush_power_pct;
        return true;

    default:
        return false;
    }
}

bool AP_CompanionComputer::read_runtime_param(uint16_t param_index, ParamFeedbackData &feedback_out)
{
    feedback_out.param_index = param_index;
    feedback_out.param_type = PARAM_TYPE_UINT32;

    switch (param_index) {
    case PARAM_IDX_BRUSH_FRONT:
        feedback_out.param_value = _brush_front_on;
        return true;
    case PARAM_IDX_BRUSH_REAR:
        feedback_out.param_value = _brush_rear_on;
        return true;
    case PARAM_IDX_BRUSH_BOTH:
        feedback_out.param_value = (_brush_front_on && _brush_rear_on) ? 1U : 0U;
        return true;
    case PARAM_IDX_BRUSH_POWER:
        feedback_out.param_value = _brush_power_pct;
        return true;
    default:
        return false;
    }
}

// 功率档位 0x0104 单独写不输出 PWM；需 0x0101/2/3 开刷后 apply_brush_runtime_params 才写 PWM
void AP_CompanionComputer::apply_brush_runtime_params()
{
    AP::brush().update(_brush_front_on != 0, _brush_rear_on != 0, uint8_t(_brush_power_pct));
}

// 清零内存参数并 AP::brush().stop_all()；急停/超时/低电压/退出 VGSL 时由 Mode 调用
void AP_CompanionComputer::stop_brushes()
{
    _brush_front_on = 0;
    _brush_rear_on = 0;
    _brush_power_pct = 0;
    AP::brush().stop_all();
}

void AP_CompanionComputer::send_response(uint8_t cmd_type, uint8_t status)
{
    const CmdAckData ack { cmd_type, status };
    uint8_t packet[COMPANION_SEND_RESP_LENGTH];
    const size_t frame_len = build_frame(FCU_FB_CMD_ACK,
                                         reinterpret_cast<const uint8_t *>(&ack),
                                         sizeof(CmdAckData),
                                         packet, sizeof(packet));
    if (frame_len == 0) {
        return;
    }
    send_frame(packet, frame_len, TxPriority::EVENT);
}

void AP_CompanionComputer::send_param_feedback(const ParamFeedbackData &data)
{
    uint8_t packet[COMPANION_SEND_PARAM_LENGTH];
    const size_t frame_len = build_frame(FCU_FB_PARAM,
                                         reinterpret_cast<const uint8_t *>(&data),
                                         sizeof(ParamFeedbackData),
                                         packet, sizeof(packet));
    if (frame_len == 0) {
        return;
    }
    send_frame(packet, frame_len, TxPriority::EVENT);
}

bool AP_CompanionComputer::validate_packet() const
{
    const uint8_t calculated_checksum = calculate_checksum(_rx_buffer.data(), _rx_count - 2);
    return (calculated_checksum == _rx_buffer[_rx_count - 2]) &&
           (_rx_buffer[_rx_count - 1] == COMPANION_END_SIGN);
}

uint8_t AP_CompanionComputer::calculate_checksum(const uint8_t *data, uint8_t len) const
{
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

void AP_CompanionComputer::update_mode_status(uint8_t control_mode, bool estop, bool turning, uint16_t fault_bits)
{
    _fb_control_mode = control_mode;
    _fb_estop = estop;
    _fb_turning = turning;
    _fb_fault_bits = fault_bits;
    _fb_mode_status_valid = true;
}

void AP_CompanionComputer::reset_mode_status()
{
    _fb_control_mode = uint8_t(ControlMode::STANDBY);
    _fb_estop = false;
    _fb_turning = false;
    _fb_fault_bits = 0;
    _fb_mode_status_valid = false;
    _nav_status_send = false;
}

void AP_CompanionComputer::clear_pending_motion_commands()
{
    // bit0=SPEED bit1=TURN bit2=POSITION；bit3=SYSTEM（急停）保留
    _new_cmd_flags &= ~uint8_t((1 << 0) | (1 << 1) | (1 << 2));
    _latest_speed_ctrl = {};
    _latest_turn = {};
    _latest_position = {};
}

// bit8 低电压：置位后 motion_state 可能为 0x05（FAULT_MOTION_MASK 含 FAULT_LOW_VOLTAGE）
uint16_t AP_CompanionComputer::collect_sensor_faults() const
{
    uint16_t faults = 0;
    const AP_AHRS &ahrs = AP::ahrs();

    if (!ahrs.healthy()) {
        faults |= FAULT_IMU;
    }

    const int16_t roll_cd = constrain_int16(int16_t(lroundf(degrees(ahrs.get_roll()) * 100.0f)), -32767, 32767);
    const int16_t pitch_cd = constrain_int16(int16_t(lroundf(degrees(ahrs.get_pitch()) * 100.0f)), -32767, 32767);
    if (abs(roll_cd) > 3000 || abs(pitch_cd) > 3000) {
        faults |= FAULT_TILT;  // |roll| 或 |pitch| > 30°
    }

#if AP_GPS_ENABLED
    if (AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        faults |= FAULT_GPS_NO_SIGNAL;
    }
#endif

    // WENC=左履带, WENC2=右履带
    AP_WheelEncoder *wenc = AP::wheelencoder();
    if (wenc != nullptr) {
        if (wenc->num_sensors() > 0 && wenc->enabled(0) && !wenc->healthy(0)) {
            faults |= FAULT_LEFT_MOTOR;
        }
        if (wenc->num_sensors() > 1 && wenc->enabled(1) && !wenc->healthy(1)) {
            faults |= FAULT_RIGHT_MOTOR;
        }
    }

    if (is_low_battery()) {
        faults |= FAULT_LOW_VOLTAGE;
    }

    return faults;
}

// 速度死区 ±5 cm/s 内视为 STOPPED；|velocity|>5 判前进/后退
uint8_t AP_CompanionComputer::compute_motion_state(int16_t velocity_cms, bool estop, bool turning, uint16_t fault_code)
{
    constexpr uint16_t FAULT_MOTION_MASK =
        FAULT_IMU | FAULT_LOW_VOLTAGE | FAULT_LEFT_MOTOR | FAULT_RIGHT_MOTOR |
        FAULT_NAV_FAILED | FAULT_TILT |
        FAULT_SUCTION_CUP;

    if ((fault_code & FAULT_MOTION_MASK) != 0) {
        return uint8_t(MotionState::FAULT);
    }
    if (estop) {
        return uint8_t(MotionState::ESTOP);
    }
    if (turning) {
        return uint8_t(MotionState::TURNING);
    }
    if (velocity_cms > 5) {
        return uint8_t(MotionState::FORWARD);
    }
    if (velocity_cms < -5) {
        return uint8_t(MotionState::BACKWARD);
    }
    return uint8_t(MotionState::STOPPED);
}

// AP_GPS 枚举与 protocol（0=无定位, 1=2D, 2=3D, 3=RTK固定）不同，上报前需映射
static uint8_t map_gps_status_to_protocol(AP_GPS::GPS_Status status)
{
    switch (status) {
    case AP_GPS::GPS_OK_FIX_2D:
        return 1;
    case AP_GPS::GPS_OK_FIX_3D:
    case AP_GPS::GPS_OK_FIX_3D_DGPS:
    case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:  // 浮点 RTK 仍视为 3D
        return 2;
    case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
        return 3;
    case AP_GPS::NO_GPS:
    case AP_GPS::NO_FIX:
    default:
        return 0;
    }
}

#if AP_RANGEFINDER_ENABLED
// 按 RNGFNDx_ORIENT 取距离；无实例或非 Good → RANGE_INVALID_CM
static uint16_t range_cm_for_orient(enum Rotation orientation)
{
    const RangeFinder *rfnd = RangeFinder::get_singleton();
    if (rfnd == nullptr || !rfnd->has_orientation(orientation)) {
        return RANGE_INVALID_CM;
    }
    if (rfnd->status_orient(orientation) != RangeFinder::Status::Good) {
        return RANGE_INVALID_CM;
    }
    return rfnd->distance_cm_orient(orientation);
}
#endif

// 10Hz 状态反馈 0xBB 0x01；须在 VGSL 下先 publish_status_feedback() 再调用
void AP_CompanionComputer::send_data()
{
    if (!_enable || _uart == nullptr) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - _last_sent_ms < 100) {  // 10Hz
        return;
    }

    // FCU → NCU 状态反馈帧 0xBB 0x01
    StatusFeedbackData status_data {};
    const AP_AHRS &ahrs = AP::ahrs();
    const AP_BattMonitor &battery = AP::battery();

    // 电池电量 (%)
    uint8_t percentage = 0;
    if (battery.capacity_remaining_pct(percentage, 0)) {
        status_data.battery_percent = percentage;
    }

    // 经纬度 (度 × 1e7)
    Location loc;
    if (ahrs.get_location(loc)) {
        status_data.longitude = loc.lng;
        status_data.latitude = loc.lat;
    }

    // 航向 (0.01°, 0~36000)；yaw_sensor 已为同单位厘度 [0, 36000)
    status_data.heading = (uint16_t)ahrs.yaw_sensor;

    // 线速度 (cm/s)
    const int16_t velocity_cms = constrain_int16(int16_t(lroundf(ahrs.groundspeed() * 100.0f)), -32767, 32767);
    status_data.velocity = velocity_cms;

    // 左右履带速度 (cm/s)
    AP_WheelEncoder *wenc = AP::wheelencoder();
    if (wenc != nullptr) {
        for (uint8_t i = 0; i < MIN(2U, wenc->num_sensors()); i++) {
            if (!wenc->enabled(i) || !wenc->healthy(i)) {
                continue;
            }
            const float rate_mps = wenc->get_rate(i) * wenc->get_wheel_radius(i);
            const int16_t vel_cms = constrain_int16(int16_t(lroundf(rate_mps * 100.0f)), -32767, 32767);
            if (i == 0) {
                status_data.left_track_vel = vel_cms;
            } else {
                status_data.right_track_vel = vel_cms;
            }
        }
    }

    // 横滚 / 俯仰 (0.01°)
    status_data.roll = constrain_int16(int16_t(lroundf(degrees(ahrs.get_roll()) * 100.0f)), -32767, 32767);
    status_data.pitch = constrain_int16(int16_t(lroundf(degrees(ahrs.get_pitch()) * 100.0f)), -32767, 32767);

    // 控制模式 / 运动状态 / 故障码
    const uint8_t control_mode = _fb_mode_status_valid ? _fb_control_mode : uint8_t(ControlMode::STANDBY);
    const bool estop = _fb_estop || _estop_active;
    const bool turning = _fb_turning;
    const uint16_t fault_code = _fb_fault_bits | collect_sensor_faults();

    status_data.control_mode = control_mode;
    status_data.motion_state = compute_motion_state(velocity_cms, estop, turning, fault_code);
    status_data.fault_code = fault_code;

#if AP_GPS_ENABLED
    // 映射到 protocol 0~3，不可直接传 AP_GPS::status() 枚举
    status_data.gps_status = map_gps_status_to_protocol(AP::gps().status());
#endif

    // 车头四路：LEFT_OUT/RIGHT_OUT 读传感器；LEFT_IN/RIGHT_IN 同侧拷贝；无效 0xFFFF
#if AP_RANGEFINDER_ENABLED
    status_data.range_left_out_cm = range_cm_for_orient(ROTATION_YAW_315);
    status_data.range_right_out_cm = range_cm_for_orient(ROTATION_YAW_45);
    status_data.range_left_in_cm = status_data.range_left_out_cm;
    status_data.range_right_in_cm = status_data.range_right_out_cm;
#else
    status_data.range_left_out_cm = RANGE_INVALID_CM;
    status_data.range_left_in_cm = RANGE_INVALID_CM;
    status_data.range_right_in_cm = RANGE_INVALID_CM;
    status_data.range_right_out_cm = RANGE_INVALID_CM;
#endif

    uint8_t packet[COMPANION_SEND_TOTAL_LENGTH];
    const size_t frame_len = build_frame(FCU_FB_STATUS,
                                         reinterpret_cast<const uint8_t *>(&status_data),
                                         sizeof(StatusFeedbackData),
                                         packet, sizeof(packet));
    if (frame_len == 0) {
        return;
    }

    if (send_frame(packet, frame_len, TxPriority::PERIODIC)) {
        _last_sent_ms = now;
    }
}

// 各 NCU 指令类型对应的 DATA_LENGTH；未知返回 0（供收帧长度校验）
uint8_t AP_CompanionComputer::expected_ncu_data_len(uint8_t cmd_type)
{
    switch (cmd_type) {
    case NCU_CMD_SPEED_CTRL:
        return NCU_DATA_LEN_SPEED_CTRL;
    case NCU_CMD_TURN:
        return NCU_DATA_LEN_TURN;
    case NCU_CMD_PARAM_WRITE:
        return NCU_DATA_LEN_PARAM_WRITE;
    case NCU_CMD_PARAM_READ:
        return NCU_DATA_LEN_PARAM_READ;
    case NCU_CMD_SYSTEM_CTRL:
        return NCU_DATA_LEN_SYSTEM_CTRL;
    case NCU_CMD_POSITION:
        return NCU_DATA_LEN_POSITION;
    default:
        return 0;
    }
}

AP_CompanionComputer *AP_CompanionComputer::_singleton;

namespace AP
{

AP_CompanionComputer &companioncomputer()
{
    return *AP_CompanionComputer::get_singleton();
}

}
