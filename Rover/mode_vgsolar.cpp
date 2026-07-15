#include "Rover.h"
#include <AP_Math/AP_Math.h>
#include <AP_Brush/AP_Brush.h>
#include <AP_SuctionCup/AP_SuctionCup.h>

#if MODE_VGSOLAR_ENABLED

/*
 * ModeVGSolar（模式17）— VGSolar 光伏清洗机器人主控模式。
 *
 * 继承 ModeGuided，复用其航向/角速度/航点导航底层能力。
 *
 * 架构：
 *   NCU UART → AP_CompanionComputer（解析、ACK、状态帧）
 *            → 本文件 read_companion_commands() 消费指令
 *            → VGSubMode 子状态机（STANDBY/YAW/YAWRATE/TURN/NAV/ESTOP）
 *            → AP_SuctionCup / AP_Brush / ModeGuided 执行
 *
 * Rover 调度：
 *   50Hz  companion_computer.update()     收 NCU 帧
 *   主循环 mode_vgsolar.update()           本文件主逻辑
 *   10Hz  publish_*() + send_data()       状态/导航反馈上行
 *
 * NCU 指令优先级（read_companion_commands）：系统控制 > 转弯 > 导航 > 速度
 * 安全：倾角>30° 或 NCU 200ms 无合法帧（含参数）→ freeze 吸盘 + safety_hold；恢复后 raise
 *
 */

// ---------------------------------------------------------------------------
// 地面站参数 VGS_*
// ---------------------------------------------------------------------------

const AP_Param::GroupInfo ModeVGSolar::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: VG Solar mode enable
    // @Description: Enable VG Solar cleaning robot mode
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, ModeVGSolar, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: KP_YAW
    // @DisplayName: VG Solar yaw P gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("KP_YAW", 2, ModeVGSolar, _kp_yaw, 1.0f),

    // @Param: KP_SPEED
    // @DisplayName: VG Solar speed P gain
    // @Range: 0.1 5.0
    // @User: Advanced
    AP_GROUPINFO("KP_SPEED", 3, ModeVGSolar, _kp_speed, 0.5f),

    // @Param: CRUISE_SPD
    // @DisplayName: VG Solar default cruise speed
    // @Description: Default cruise speed for position navigation (m/s)
    // @Range: 0.1 5.0
    // @User: Advanced
    AP_GROUPINFO("CRUISE_SPD", 4, ModeVGSolar, _cruise_speed_default, 1.0f),

    // @Param: TURN_TO
    // @DisplayName: VG Solar turn timeout
    // @Description: Maximum time for turn sequence in seconds
    // @Range: 5 60
    // @User: Advanced
    AP_GROUPINFO("TURN_TO", 5, ModeVGSolar, _turn_timeout, 15.0f),

    // @Param: TURN_SPD
    // @DisplayName: VG Solar turn speed
    // @Description: Max speed during moving turn (m/s)
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("TURN_SPD", 6, ModeVGSolar, _turn_max_speed, 0.3f),

    AP_GROUPEND
};

// 子模式/转弯/导航相关运行时状态初始化
ModeVGSolar::ModeVGSolar(void) :
    _vg_submode(VGSubMode::STANDBY),
    _submode_before_turn(VGSubMode::STANDBY),
    _turn_phase(TurnPhase::IDLE),
    _target_speed_ms(0.0f),
    _target_yaw_cd(0.0f),
    _target_yaw_rate_cds(0.0f),
    _cruise_speed_ms(1.0f),
    _turn_direction(TURN_DIR_LEFT),
    _turn_mode_type(TURN_MODE_SPOT),
    _turn_target_angle_deg(0.0f),
    _turn_angular_vel_dps(30.0f),
    _turn_start_yaw_deg(0.0f),
    _turn_accumulated_deg(0.0f),
    _last_turn_yaw_deg(0.0f),
    _nav_coord_mode(0),
    _fault_flags(0),
    _nav_report_state(NavReportState::NONE),
    _nav_phase(NavPhase::CRUISE),
    _ned_origin_valid(false),
    _arrival_radius_m(0.0f),
    _arrival_yaw_required(false),
    _arrival_yaw_raw_cd(0),
    _arrival_yaw_target_cd(0.0f),
    _turn_phase_start_ms(0),
    _turn_frozen(false),
    _safety_hold_mask(0),
    _last_turn_pwm_gcs_ms(0),
    _last_turn_gcs_phase(TurnPhase::IDLE)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeVGSolar::_enter()
{
    if (!_enabled) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: mode disabled");
        return false;
    }

    if (!ModeGuided::_enter()) {
        return false;
    }

    // 子模式与故障标志复位
    _vg_submode = VGSubMode::STANDBY;
    _turn_phase = TurnPhase::IDLE;
    _target_speed_ms = 0.0f;
    _fault_flags = 0;
    clear_safety_hold_mask();
    _nav_phase = NavPhase::CRUISE;
    // 记录 NED 全局原点，供后续 NED 导航换算经纬度
    capture_ned_origin();

    // 外设激活：滚刷/吸盘仅在 VGSL 内输出 PWM
    rover.companion_computer.reset_ncu_rx_heartbeat();
    rover.companion_computer.stop_brushes();
    AP::brush().set_active(true);
    AP::suction_cup().clear_fault();
    AP::suction_cup().set_active(true);

    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: entered");
    return true;
}

void ModeVGSolar::_exit()
{
    stop_vehicle();
    _vg_submode = VGSubMode::STANDBY;
    _turn_phase = TurnPhase::IDLE;
    _turn_frozen = false;

    AP::suction_cup().emergency_release();
    AP::suction_cup().set_active(false);
    rover.companion_computer.stop_brushes();
    AP::brush().set_active(false);
    rover.companion_computer.reset_mode_status();

    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: exited");
}

void ModeVGSolar::update()
{
    // 低电压：强制关刷；无有效电量读数时不触发（台架安全）
    if (rover.companion_computer.is_low_battery()) {
        rover.companion_computer.stop_brushes();
    }

    AP::suction_cup().update();
    check_tilt_safety();
    sync_suction_fault_flags();

    if (_vg_submode == VGSubMode::TURN && AP::suction_cup().has_fault()) {
        abort_turn_suction_fault();
    }

    read_companion_commands();
    // 先消费本周期 NCU 指令，再判超时，避免「先超时停车、后收到新帧」的竞态
    check_ncu_timeout();
    try_recover_safety_hold();  // 倾角/NCU 条件满足后自动抬吸盘

    if (_vg_submode == VGSubMode::ESTOP) {
        update_estop();
        return;
    }

    // 安全保持期间禁止 NAV/YAW/YAWRATE 继续运动；TURN 仅 _turn_frozen 路径
    if (_safety_hold_mask != 0) {
        stop_vehicle();
        if (_vg_submode == VGSubMode::TURN) {
            update_turn();
        } else {
            update_standby();
        }
        return;
    }

    switch (_vg_submode) {
    case VGSubMode::STANDBY:
        update_standby();
        break;
    case VGSubMode::YAW:
        update_yaw();
        break;
    case VGSubMode::YAWRATE:
        update_yawrate();
        break;
    case VGSubMode::TURN:
        update_turn();
        break;
    case VGSubMode::NAV:
        update_nav();
        break;
    default:
        break;
    }
}

// NCU 状态上报（Rover.cpp 10Hz：publish_* → companion_computer.send_data/send_nav_data）
void ModeVGSolar::publish_status_feedback()
{
    uint8_t control_mode = uint8_t(ControlMode::STANDBY);
    switch (_vg_submode) {
    case VGSubMode::STANDBY:
        control_mode = uint8_t(ControlMode::STANDBY);
        break;
    case VGSubMode::YAW:
        control_mode = uint8_t(ControlMode::YAW);
        break;
    case VGSubMode::YAWRATE:
        control_mode = uint8_t(ControlMode::YAWRATE);
        break;
    case VGSubMode::TURN:
        control_mode = uint8_t(ControlMode::TURN);
        break;
    case VGSubMode::NAV:
        control_mode = (_nav_coord_mode == NAV_MODE_BODY)
                       ? uint8_t(ControlMode::NAV_BODY)
                       : uint8_t(ControlMode::NAV_GPS);
        break;
    case VGSubMode::ESTOP:
        control_mode = uint8_t(ControlMode::STANDBY);
        break;
    }

    const bool estop = (_vg_submode == VGSubMode::ESTOP);
    // motion_state=0x03 仅吸盘已吸附且尚未抬起期间
    const bool turning = (_vg_submode == VGSubMode::TURN)
                         && AP::suction_cup().is_lowered()
                         && !AP::suction_cup().is_raised()
                         && !_turn_frozen
                         && !AP::suction_cup().is_frozen();

    sync_suction_fault_flags();
    rover.companion_computer.update_mode_status(control_mode, estop, turning, _fault_flags);
}

void ModeVGSolar::publish_nav_status_feedback()
{
    // 终态（到达/失败/取消）发单帧；导航中 10Hz 持续上报距离与航向误差
    NavStatusData data {};
    bool send_nav = false;

    if (_nav_report_state == NavReportState::ARRIVED) {
        data.nav_state = NAV_STATE_ARRIVED;
        data.coord_mode = _nav_coord_mode;
        send_nav = true;
        _nav_report_state = NavReportState::NONE;
    } else if (_nav_report_state == NavReportState::FAILED) {
        data.nav_state = NAV_STATE_FAILED;
        data.coord_mode = _nav_coord_mode;
        send_nav = true;
        _nav_report_state = NavReportState::NONE;
    } else if (_nav_report_state == NavReportState::CANCELLED) {
        data.nav_state = NAV_STATE_CANCELLED;
        data.coord_mode = _nav_coord_mode;
        send_nav = true;
        _nav_report_state = NavReportState::NONE;
    } else if (_vg_submode == VGSubMode::NAV) {
        data.nav_state = NAV_STATE_ACTIVE;
        data.coord_mode = _nav_coord_mode;

        // protocol 5.4: 距目标 uint32 cm
        const float dist_m = get_distance_to_destination();
        data.distance_to_target = (uint32_t)constrain_float(dist_m * 100.0f, 0.0f, 4294967295.0f);

        const float yaw_deg = ahrs.yaw_sensor * 0.01f;
        float err_deg;
        if (_nav_phase == NavPhase::YAW_ALIGN) {
            // 对航向阶段：偏差相对 arrival_yaw，而非航点方位
            err_deg = wrap_180(yaw_deg - _arrival_yaw_target_cd * 0.01f);
        } else {
            const float bearing_deg = wp_bearing();
            // 航向偏差，正值=目标在左侧
            err_deg = wrap_180(yaw_deg - bearing_deg);
        }
        data.heading_error = constrain_int16(int16_t(lroundf(err_deg * 100.0f)), -32767, 32767);

        send_nav = true;
    }

    rover.companion_computer.set_nav_status(data, send_nav);
}

// NCU 指令消费（心跳由 CompanionComputer 在任意合法帧上刷新 last_ncu_frame_ms）
// 优先级：系统控制 > 转弯 > 导航 > 速度控制
void ModeVGSolar::read_companion_commands()
{
    auto &cc = rover.companion_computer;

    if (cc.is_new_system_ctrl()) {
        cc.clear_new_system_flag();
        const SystemCtrlData &cmd = cc.get_latest_system_ctrl();
        _fault_flags &= ~FAULT_COMM_TIMEOUT;

        switch (cmd.command) {
        case SYS_CMD_ESTOP:
            // 导航中急停：停车并上报 nav_state=已取消
            if (_vg_submode == VGSubMode::NAV) {
                stop_vehicle();
                g2.wp_nav.set_reversed(false);
                _nav_phase = NavPhase::CRUISE;
                _nav_report_state = NavReportState::CANCELLED;
            }
            _vg_submode = VGSubMode::ESTOP;
            _turn_phase = TurnPhase::IDLE;
            _turn_frozen = false;
            clear_safety_hold_mask();
            cc.stop_brushes();
            AP::suction_cup().emergency_release();
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: ESTOP");
            break;
        case SYS_CMD_ESTOP_CLEAR:
            if (_vg_submode == VGSubMode::ESTOP) {
                _vg_submode = VGSubMode::STANDBY;
                gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: ESTOP cleared");
            }
            try_recover_safety_hold();
            break;
        case SYS_CMD_REBOOT:
            hal.scheduler->reboot(false);
            break;
        // SYS_CMD_SHUTDOWN(0x04) 关机未实现
        default:
            break;
        }
    }

    if (_vg_submode == VGSubMode::ESTOP) {
        return;
    }

    if (cc.is_new_turn()) {
        cc.clear_new_turn_flag();
        const TurnData &cmd = cc.get_latest_turn();
        start_turn(cmd);
        _fault_flags &= ~FAULT_COMM_TIMEOUT;
        return;
    }

    if (cc.is_new_position()) {
        cc.clear_new_position_flag();
        const PositionData &cmd = cc.get_latest_position();
        _fault_flags &= ~FAULT_COMM_TIMEOUT;

        if (cmd.nav_mode == NAV_MODE_CANCEL) {
            cancel_navigation();
            cc.send_position_ack(CMD_ACK_SUCCESS);
            return;
        }

        Location target_loc;
        Location current_loc;
        if (!ahrs.get_location(current_loc)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: no position, nav rejected");
            _nav_coord_mode = cmd.nav_mode;
            _nav_report_state = NavReportState::FAILED;
            cc.send_position_ack(CMD_ACK_FAILED);
            return;
        }

        switch (cmd.nav_mode) {
        case NAV_MODE_GPS:
            target_loc.lat = cmd.target_y;
            target_loc.lng = cmd.target_x;
            break;

        case NAV_MODE_NED: {
            // 以进入 VGSL 时记录的全局原点为基准，非当前位置
            if (!_ned_origin_valid) {
                capture_ned_origin();
            }
            if (!_ned_origin_valid) {
                gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: no NED origin, nav rejected");
                _nav_coord_mode = cmd.nav_mode;
                _nav_report_state = NavReportState::FAILED;
                cc.send_position_ack(CMD_ACK_FAILED);
                return;
            }
            const float north_m = cmd.target_x * 0.01f;
            const float east_m = cmd.target_y * 0.01f;
            target_loc = _ned_origin;
            target_loc.offset(north_m, east_m);
            break;
        }

        case NAV_MODE_BODY: {
            // 以指令接收时刻的当前位置+航向为基准，换算为 NED 偏移
            const float yaw_rad = radians(ahrs.yaw_sensor * 0.01f);
            const float forward_cm = cmd.target_x;
            const float right_cm = cmd.target_y;
            const float north_m = (forward_cm * cosf(yaw_rad) - right_cm * sinf(yaw_rad)) * 0.01f;
            const float east_m = (forward_cm * sinf(yaw_rad) + right_cm * cosf(yaw_rad)) * 0.01f;
            target_loc = current_loc;
            target_loc.offset(north_m, east_m);
            break;
        }

        default:
            cc.send_position_ack(CMD_ACK_FAILED);
            return;
        }

        if (!set_desired_location(target_loc)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: set destination failed");
            _nav_coord_mode = cmd.nav_mode;
            _nav_report_state = NavReportState::FAILED;
            cc.send_position_ack(CMD_ACK_FAILED);
            _fault_flags |= FAULT_NAV_FAILED;
            return;
        }
        // ModeGuided 不初始化 _distance_to_destination；wp_nav 在设点后已有正确距离
        _distance_to_destination = g2.wp_nav.get_distance_to_destination();

        _nav_coord_mode = cmd.nav_mode;
        _nav_report_state = NavReportState::NONE;
        _nav_phase = NavPhase::CRUISE;
        _fault_flags &= ~FAULT_NAV_FAILED;

        // 巡航速度：0 用参数默认；负值倒车（见 apply_nav_speed / set_reversed）
        if (cmd.cruise_speed != 0) {
            _cruise_speed_ms = cmd.cruise_speed * 0.01f;
        } else {
            _cruise_speed_ms = _cruise_speed_default;
        }

        // 到达半径：NCU 指定则优先；0 则回退 WP_RADIUS
        _arrival_radius_m = (cmd.arrival_radius > 0) ? cmd.arrival_radius * 0.01f : 0.0f;

        // 到达航向：0xFFFF 不指定；Body 模式在到达位置后再换算为绝对航向
        const uint16_t arrival_yaw_u16 = (uint16_t)cmd.arrival_yaw;
        if (arrival_yaw_u16 == NAV_YAW_UNSPECIFIED) {
            _arrival_yaw_required = false;
        } else {
            _arrival_yaw_required = true;
            _arrival_yaw_raw_cd = arrival_yaw_u16;
        }

        g2.wp_nav.set_reversed(is_negative(_cruise_speed_ms));
        apply_nav_speed();

        _vg_submode = VGSubMode::NAV;
        cc.send_position_ack(CMD_ACK_SUCCESS);

        gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV start, mode=%d", cmd.nav_mode);
        return;
    }

    if (cc.is_new_speed_ctrl()) {
        cc.clear_new_speed_flag();
        const SpeedCtrlData &cmd = cc.get_latest_speed_ctrl();
        _fault_flags &= ~FAULT_COMM_TIMEOUT;

        if (_vg_submode == VGSubMode::NAV) {
            return;
        }

        // 吸盘 lower/raise 序列进行中不接受速度，避免带吸盘移动
        if (AP::suction_cup().is_busy()) {
            return;
        }

        // 转弯冻结：NCU 恢复通信后走统一安全恢复（倾角仍超限则继续等待）
        if (_vg_submode == VGSubMode::TURN && _turn_frozen) {
            try_recover_safety_hold();
            return;
        }

        // 其它安全保持期间只刷新通信心跳，不切换控制子模式
        if (_safety_hold_mask != 0) {
            return;
        }

        _target_speed_ms = cmd.velocity * 0.01f;

        if (cmd.control_mode == SPEED_MODE_YAW) {
            _vg_submode = VGSubMode::YAW;
            _target_yaw_cd = cmd.yaw_data;
        } else if (cmd.control_mode == SPEED_MODE_YAWRATE) {
            if (_vg_submode == VGSubMode::YAW) {
                // 从航向角模式切到角速度模式，清掉角速度 PID 积分避免残留打舵
                attitude_control.relax_I();
            }
            _vg_submode = VGSubMode::YAWRATE;
            _target_yaw_rate_cds = cmd.yaw_data;
        }
    }
}

// 子模式执行：待机 / 航向角 / 偏航速率 / 导航
void ModeVGSolar::update_standby()
{
    stop_vehicle();
}

void ModeVGSolar::update_yaw()
{
    // // 复用 ModeGuided 的 HeadingAndSpeed 能力：
    // set_desired_heading_and_speed(_target_yaw_cd, _target_speed_ms);
    // ModeGuided::update();

    // 台架/无轮速计：闭环速度 PID 无反馈，油门恒为 0，仅转向差速有输出。
    // 开环：线速度指令 → 油门百分比，航向仍用 calc_steering_to_heading。
    calc_steering_to_heading(_target_yaw_cd);

    const float speed_max = calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);
    float throttle_pct = 100.0f * (_target_speed_ms / speed_max);
    throttle_pct = constrain_float(throttle_pct, -100.0f, 100.0f);
    g2.motors.set_throttle(throttle_pct);
}

void ModeVGSolar::update_yawrate()
{
    // NCU 协议角速度正=左转；ArduPilot get_steering_out_rate 正=右转
    const float turn_rate_cds = -_target_yaw_rate_cds;

    // NCU 线速度/角速度均为 0：清转向积分并停车，避免目标突然变成0后，车还有小幅度转向
    if (is_zero(_target_speed_ms) && is_zero(turn_rate_cds)) {
        attitude_control.relax_I();
        stop_vehicle();
        return;
    }

    set_desired_turn_rate_and_speed(turn_rate_cds, _target_speed_ms);
    ModeGuided::update();

}

void ModeVGSolar::update_nav()
{
    // 两阶段导航：CRUISE 沿航点行驶 →（可选）YAW_ALIGN 原地对 arrival_yaw
    if (_nav_phase == NavPhase::YAW_ALIGN) {
        calc_steering_to_heading(_arrival_yaw_target_cd);
        g2.motors.set_throttle(0.0f);

        const float err_deg = fabsf(wrap_180(_arrival_yaw_target_cd * 0.01f - ahrs.yaw_sensor * 0.01f));
        if (err_deg <= 2.0f) {  // 2° 容差内视为对航向完成
            complete_nav_arrived();
        }
        return;
    }

    //沿航点导航至目标区域（巡航速度在收到导航指令时已设置）
    ModeGuided::update();

    if (nav_position_reached()) {
        if (_arrival_yaw_required) {
            stop_vehicle();
            if (_nav_coord_mode == NAV_MODE_BODY) {
                // Body 模式：arrival_yaw 相对车头，到达时刻叠加当前航向
                _arrival_yaw_target_cd = wrap_360_cd(ahrs.yaw_sensor + _arrival_yaw_raw_cd);
            } else {
                // GPS/NED：arrival_yaw 为相对正北的绝对航向
                _arrival_yaw_target_cd = _arrival_yaw_raw_cd;
            }
            _nav_phase = NavPhase::YAW_ALIGN;
            gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV position reached, aligning yaw");
            return;
        }
        complete_nav_arrived();
    }
}

void ModeVGSolar::capture_ned_origin()
{
    // 优先用当前 GPS 位置；无定位时退回到 Home 点
    Location loc;
    if (ahrs.get_location(loc)) {
        _ned_origin = loc;
        _ned_origin_valid = true;
        return;
    }

    if (ahrs.home_is_set()) {
        _ned_origin = ahrs.get_home();
        _ned_origin_valid = true;
    }
}

bool ModeVGSolar::nav_position_reached() const
{
    if (_arrival_radius_m > 0.0f) {
        // NCU 下发的到达半径优先于 WP_RADIUS
        return get_distance_to_destination() <= _arrival_radius_m;
    }
    return reached_destination();
}

void ModeVGSolar::complete_nav_arrived()
{
    stop_vehicle();
    g2.wp_nav.set_reversed(false);
    _nav_phase = NavPhase::CRUISE;
    _nav_report_state = NavReportState::ARRIVED;  // 下一周期 publish 一次 0xBB 0x04
    _vg_submode = VGSubMode::STANDBY;
    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV arrived");
}

void ModeVGSolar::apply_nav_speed()
{
    // set_desired_speed 只接受正值；倒车方向由 wp_nav.set_reversed 控制
    set_desired_speed(fabsf(_cruise_speed_ms));
}

void ModeVGSolar::update_estop()
{
    stop_vehicle();
    rover.companion_computer.stop_brushes();
}

// 吸盘故障同步与转弯中止
void ModeVGSolar::sync_suction_fault_flags()
{
    if (AP::suction_cup().has_fault()) {
        _fault_flags |= FAULT_SUCTION_CUP;
    } else {
        _fault_flags &= ~FAULT_SUCTION_CUP;
    }
}

void ModeVGSolar::abort_turn_suction_fault()
{
    if (_vg_submode != VGSubMode::TURN && _turn_phase == TurnPhase::IDLE) {
        return;
    }

    _fault_flags |= FAULT_SUCTION_CUP;
    stop_vehicle();
    _vg_submode = VGSubMode::STANDBY;
    _turn_phase = TurnPhase::IDLE;
    _turn_frozen = false;
    clear_safety_hold_mask();
    AP::suction_cup().emergency_release();
    gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: suction fault, turn aborted");
}

void ModeVGSolar::complete_turn()
{
    _vg_submode = _submode_before_turn;
    _turn_phase = TurnPhase::IDLE;
    _turn_frozen = false;
    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: TURN complete, restored");
}

// 安全保持：倾角过大 / NCU 通信超时 → freeze 吸盘，条件满足后自动恢复
void ModeVGSolar::check_tilt_safety()
{
    // |roll|/|pitch|>30° 且已吸附：freeze 吸盘；bit9 由 collect_sensor_faults 上报
    if (!AP::suction_cup().is_lowered()) {
        return;
    }

    if (tilt_within_limit()) {
        return;
    }

    stop_vehicle();
    enter_safety_hold(SAFETY_HOLD_TILT);
    AP::suction_cup().freeze();
    if (_vg_submode == VGSubMode::TURN) {
        _turn_frozen = true;
    }
}

bool ModeVGSolar::tilt_within_limit() const
{
    // 30° = 3000 cd，与 FAULT_TILT 阈值一致
    const int16_t roll_cd = constrain_int16(int16_t(lroundf(degrees(ahrs.get_roll()) * 100.0f)), -32767, 32767);
    const int16_t pitch_cd = constrain_int16(int16_t(lroundf(degrees(ahrs.get_pitch()) * 100.0f)), -32767, 32767);
    return abs(roll_cd) <= 3000 && abs(pitch_cd) <= 3000;
}

void ModeVGSolar::enter_safety_hold(uint8_t reason_bit)
{
    if ((_safety_hold_mask & reason_bit) == 0) {
        if (reason_bit == SAFETY_HOLD_TILT) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: tilt hold, suction frozen");
        } else if (reason_bit == SAFETY_HOLD_NCU_COMM) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: NCU hold, suction frozen");
        }
    }
    _safety_hold_mask |= reason_bit;
}

void ModeVGSolar::clear_safety_hold_mask()
{
    _safety_hold_mask = 0;
}

void ModeVGSolar::abort_motion_for_safety_recovery()
{
    // 恢复前中止 NAV/TURN/YAW，避免带吸盘继续运动
    switch (_vg_submode) {
    case VGSubMode::NAV:
        stop_vehicle();
        g2.wp_nav.set_reversed(false);
        _nav_phase = NavPhase::CRUISE;
        _nav_report_state = NavReportState::CANCELLED;
        _vg_submode = VGSubMode::STANDBY;
        gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV cancelled by safety recovery");
        break;
    case VGSubMode::TURN:
        _vg_submode = VGSubMode::STANDBY;
        _turn_phase = TurnPhase::IDLE;
        break;
    case VGSubMode::YAW:
    case VGSubMode::YAWRATE:
        _vg_submode = VGSubMode::STANDBY;
        break;
    default:
        break;
    }
    _turn_frozen = false;
}

void ModeVGSolar::release_safety_hold_suction()
{
    // unfreeze 后异步 raise，抬起完成前 is_busy 阻塞速度指令
    auto &scup = AP::suction_cup();
    if (scup.is_frozen() || scup.get_state() == AP_SuctionCup::State::FROZEN) {
        scup.unfreeze();
    }
    if (scup.is_lowered()) {
        scup.raise();
    }
}

void ModeVGSolar::try_recover_safety_hold()
{
    // TILT：倾角回限；NCU_COMM：bit7 清除；ESTOP 期间不自动恢复
    if (_safety_hold_mask == 0) {
        return;
    }

    if (_vg_submode == VGSubMode::ESTOP) {
        return;
    }

    if (AP::suction_cup().is_busy()) {
        return;
    }

    if ((_safety_hold_mask & SAFETY_HOLD_TILT) != 0 && !tilt_within_limit()) {
        return;
    }

    if ((_safety_hold_mask & SAFETY_HOLD_NCU_COMM) != 0 &&
        (_fault_flags & FAULT_COMM_TIMEOUT) != 0) {
        return;
    }

    const uint8_t recovered_mask = _safety_hold_mask;
    abort_motion_for_safety_recovery();
    release_safety_hold_suction();
    clear_safety_hold_mask();

    if ((recovered_mask & SAFETY_HOLD_TILT) != 0 &&
        (recovered_mask & SAFETY_HOLD_NCU_COMM) != 0) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "VG_SOLAR: tilt+NCU hold cleared, releasing suction");
    } else if ((recovered_mask & SAFETY_HOLD_TILT) != 0) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "VG_SOLAR: tilt hold cleared, releasing suction");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "VG_SOLAR: NCU hold cleared, releasing suction");
    }
}

// 转弯序列（NCU 0x02）
// 阶段：停车 → 等稳 500ms → lower 吸盘 → 原地/行进转弯 → raise 吸盘 → 恢复原子模式
void ModeVGSolar::start_turn(const TurnData &cmd)
{
    // 安全保持/吸盘 busy/故障时拒绝新转弯
    if (_safety_hold_mask != 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: TURN rejected, safety hold active");
        return;
    }

    if (AP::suction_cup().is_busy()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: TURN rejected, suction busy");
        return;
    }

    if (AP::suction_cup().has_fault()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: TURN rejected, suction fault");
        return;
    }
    AP::suction_cup().unfreeze();
    _turn_frozen = false;

    _submode_before_turn = (_vg_submode == VGSubMode::TURN || _vg_submode == VGSubMode::ESTOP)
                           ? VGSubMode::STANDBY : _vg_submode;

    _vg_submode = VGSubMode::TURN;
    _turn_direction = cmd.direction;
    _turn_mode_type = cmd.turn_mode;
    _turn_target_angle_deg = cmd.target_angle * 0.01f;
    _turn_angular_vel_dps = MAX(cmd.angular_vel * 0.01f, 1.0f);
    _turn_phase = TurnPhase::STOPPING;
    _turn_accumulated_deg = 0.0f;

    gcs().send_text(MAV_SEVERITY_INFO,
                    "VG_SOLAR: TURN start dir=%d mode=%d angle=%.1f",
                    _turn_direction, _turn_mode_type, _turn_target_angle_deg);
    send_turn_pwm_gcs(true);
}

void ModeVGSolar::send_turn_pwm_gcs(bool force)
{
    // 阶段切换立即上报，同阶段最多 1Hz
    const uint32_t now = AP_HAL::millis();
    const bool phase_changed = _turn_phase != _last_turn_gcs_phase;
    if (!force && !phase_changed &&
        (now - _last_turn_pwm_gcs_ms) < TURN_PWM_GCS_INTERVAL_MS) {
        return;
    }

    _last_turn_pwm_gcs_ms = now;
    _last_turn_gcs_phase = _turn_phase;

    const auto &scup = AP::suction_cup();
    gcs().send_text(MAV_SEVERITY_INFO,
                    "VG_SOLAR TURN pwm: tph=%u scup_st=%u scup_ph=%u lift=%u valve=%u pump=%u",
                    unsigned(_turn_phase),
                    unsigned(scup.get_state_u8()),
                    unsigned(scup.get_phase_u8()),
                    unsigned(scup.get_last_lift_pwm_us()),
                    unsigned(scup.get_last_valve_pwm_us()),
                    unsigned(scup.get_last_pump_pwm_us()));
}

void ModeVGSolar::update_turn()
{
    if (_turn_frozen) {
        // 倾角/NCU 安全保持：只停车，等 try_recover_safety_hold 恢复
        stop_vehicle();
        send_turn_pwm_gcs();
        return;
    }

    auto &scup = AP::suction_cup();
    const uint32_t now = AP_HAL::millis();

    switch (_turn_phase) {

    case TurnPhase::STOPPING: {
        // 减速至零速后进入等待
        const bool stopped = stop_vehicle();
        if (stopped) {
            _turn_phase = TurnPhase::WAIT_STOPPED;
            _turn_phase_start_ms = now;
        }
        break;
    }

    case TurnPhase::WAIT_STOPPED: {
        // 停稳 500ms 后再放吸盘，避免惯性滑动
        if (now - _turn_phase_start_ms > 500) {
            _turn_phase = TurnPhase::LOWER_SUCTION;
            _turn_phase_start_ms = now;
        }
        break;
    }

    case TurnPhase::LOWER_SUCTION: {
        // 异步 lower；is_lowered() 后进入 TURNING（motion_state=0x03）
        if (!scup.is_busy() && !scup.is_lowered()) {
            if (!scup.lower()) {
                abort_turn_suction_fault();
                break;
            }
        }
        if (scup.has_fault()) {
            abort_turn_suction_fault();
        } else if (scup.is_lowered()) {
            _turn_phase = TurnPhase::TURNING;
            _turn_phase_start_ms = now;
            _turn_start_yaw_deg = wrap_180(degrees(ahrs.get_yaw()));
            _last_turn_yaw_deg = _turn_start_yaw_deg;
            _turn_accumulated_deg = 0.0f;
        }
        break;
    }

    case TurnPhase::TURNING: {
        // 累计转角达目标或超时 → 停车并进入 RAISE_SUCTION
        const float current_yaw_deg = wrap_180(degrees(ahrs.get_yaw()));
        const float step_deg = wrap_180(current_yaw_deg - _last_turn_yaw_deg);
        _turn_accumulated_deg += fabsf(step_deg);
        _last_turn_yaw_deg = current_yaw_deg;

        if (_turn_accumulated_deg >= _turn_target_angle_deg) {
            stop_vehicle();
            _turn_phase = TurnPhase::RAISE_SUCTION;
            _turn_phase_start_ms = now;
        } else if (now - _turn_phase_start_ms > _turn_timeout * 1000.0f) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: TURN angle timeout");
            stop_vehicle();
            if (scup.is_lowered()) {
                _turn_phase = TurnPhase::RAISE_SUCTION;
                _turn_phase_start_ms = now;
            } else {
                complete_turn();
            }
        } else {
            // 协议左转=正；ArduPilot 角速度正=右转
            const float dir_sign = (_turn_direction == TURN_DIR_LEFT) ? -1.0f : 1.0f;
            const float turn_rate_cds = _turn_angular_vel_dps * 100.0f * dir_sign;
            const float speed_ms = (_turn_mode_type == TURN_MODE_SPOT) ? 0.0f : _turn_max_speed;
            set_desired_turn_rate_and_speed(turn_rate_cds, speed_ms);
            ModeGuided::update();
        }
        break;
    }

    case TurnPhase::RAISE_SUCTION: {
        // 异步 raise；is_raised() 后 complete_turn() 恢复 _submode_before_turn
        if (!scup.is_busy() && !scup.is_raised()) {
            if (scup.is_lowered() || scup.is_frozen()) {
                if (!scup.raise()) {
                    abort_turn_suction_fault();
                    break;
                }
            } else {
                complete_turn();
                break;
            }
        }
        if (scup.has_fault()) {
            abort_turn_suction_fault();
        } else if (scup.is_raised()) {
            complete_turn();
        }
        break;
    }

    case TurnPhase::IDLE:
    default:
        break;
    }

    send_turn_pwm_gcs();
}

void ModeVGSolar::cancel_navigation()
{
    stop_vehicle();
    g2.wp_nav.set_reversed(false);
    _nav_phase = NavPhase::CRUISE;
    _nav_report_state = NavReportState::CANCELLED;
    _vg_submode = VGSubMode::STANDBY;
    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV cancelled");
}

// NCU 心跳超时 200ms：以 companion.last_ncu_frame_ms 为准（任意合法帧，含参数写/读）
void ModeVGSolar::check_ncu_timeout()
{
    const uint32_t last_ncu_ms = rover.companion_computer.last_ncu_frame_ms();
    if (last_ncu_ms == 0) {
        // 进入 VGSL 后尚未收到任何合法帧：不判超时
        return;
    }

    // 转弯/导航执行期豁免 NCU 心跳（协议为单次长指令）；TURN 仅 _turn_frozen 后再判超时
    if ((_vg_submode == VGSubMode::TURN && !_turn_frozen) ||
        _vg_submode == VGSubMode::NAV) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_ncu_ms <= NCU_HEARTBEAT_TIMEOUT_MS) {
        // 任意帧（含参数）恢复通讯后清除 bit7，便于 try_recover_safety_hold
        _fault_flags &= ~FAULT_COMM_TIMEOUT;
        return;
    }

    _fault_flags |= FAULT_COMM_TIMEOUT;
    rover.companion_computer.stop_brushes();

    switch (_vg_submode) {
    case VGSubMode::TURN:
        stop_vehicle();
        enter_safety_hold(SAFETY_HOLD_NCU_COMM);
        AP::suction_cup().freeze();
        _turn_frozen = true;
        return;

    case VGSubMode::NAV:
    case VGSubMode::ESTOP:
        stop_vehicle();
        enter_safety_hold(SAFETY_HOLD_NCU_COMM);
        AP::suction_cup().freeze();
        return;

    case VGSubMode::STANDBY:
        stop_vehicle();
        if (AP::suction_cup().is_lowered()) {
            enter_safety_hold(SAFETY_HOLD_NCU_COMM);
            AP::suction_cup().freeze();
        }
        return;

    default:
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: NCU timeout, auto stop");
        stop_vehicle();
        _vg_submode = VGSubMode::STANDBY;
        _turn_phase = TurnPhase::IDLE;
        _turn_frozen = false;
        if (AP::suction_cup().is_lowered()) {
            enter_safety_hold(SAFETY_HOLD_NCU_COMM);
            AP::suction_cup().freeze();
        }
        break;
    }
}

// 距离查询与滚刷钩子（滚刷主路径走 CompanionComputer 参数 0x0101~0x0104）
float ModeVGSolar::get_distance_to_destination() const
{
    if (_vg_submode == VGSubMode::NAV) {
        // 始终以 wp_nav 为准，避免 Mode 层 _distance_to_destination 滞后
        return g2.wp_nav.get_distance_to_destination();
    }
    return 0.0f;
}

void ModeVGSolar::set_brush_control(uint8_t brush_id, bool turn_on)
{
    // Rover 框架回调；VGSolar 滚刷由 NCU 参数控制，此处仅处理关刷
    (void)brush_id;
    if (!turn_on) {
        rover.companion_computer.stop_brushes();
    }
}

#endif  // MODE_VGSOLAR_ENABLED
