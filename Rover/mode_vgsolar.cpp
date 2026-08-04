#include "Rover.h"
#include <AP_Math/AP_Math.h>
#include <AP_Brush/AP_Brush.h>
#include <AP_SuctionCup/AP_SuctionCup.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <climits>

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
 * 安全：倾角>30° 或 NCU 200ms 无帧 → freeze 吸盘 + safety_hold；条件恢复后 raise
 *       未解锁 → 滚刷/气泵/气阀/吸盘强制安全位，禁止运动类 NCU 指令
 *       LEFT_OUT/RIGHT_OUT 超 VGS_RF_MIN~MAX 或无效 → ESTOP（需 NCU 解除）
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

    // @Param: RF_MIN
    // @DisplayName: VG Solar rangefinder safe min
    // @Description: Inclusive safe-band lower limit (cm) for LEFT_OUT/RIGHT_OUT; outside or invalid triggers ESTOP
    // @Range: 1 450
    // @Units: cm
    // @User: Standard
    AP_GROUPINFO("RF_MIN", 7, ModeVGSolar, _rf_safe_min_cm, 5),

    // @Param: RF_MAX
    // @DisplayName: VG Solar rangefinder safe max
    // @Description: Inclusive safe-band upper limit (cm) for LEFT_OUT/RIGHT_OUT
    // @Range: 1 450
    // @Units: cm
    // @User: Standard
    AP_GROUPINFO("RF_MAX", 8, ModeVGSolar, _rf_safe_max_cm, 15),

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
    _last_ncu_cmd_ms(0),
    _turn_phase_start_ms(0),
    _turn_frozen(false),
    _await_ncu_after_lost_motion(false),
    _turn_timeout_aborted(false),
    _safety_hold_mask(0),
    _last_turn_pwm_gcs_ms(0),
    _last_turn_gcs_phase(TurnPhase::IDLE),
    _actuators_were_armed(false),
    _last_disarmed_actuator_gcs_ms(0),
    _range_unsafe_since_ms(0)
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
    _target_yaw_rate_cds = 0;
    _last_ncu_cmd_ms = 0;
    _await_ncu_after_lost_motion = false;
    _turn_timeout_aborted = false;
    _fault_flags = 0;
    clear_safety_hold_mask();
    _nav_phase = NavPhase::CRUISE;
    // 记录 NED 全局原点，供后续 NED 导航换算经纬度
    capture_ned_origin();

    // 其它模式期间 UART 仍会解析 NCU；丢弃堆积的运动指令，避免重进后突然跟旧速度
    rover.companion_computer.clear_pending_motion_commands();

    // 外设激活：滚刷/吸盘仅在 VGSL 内输出；未 soft_armed 时库内仍强制安全位
    rover.companion_computer.stop_brushes();
    AP::brush().set_active(true);
    AP::suction_cup().clear_fault();
    AP::suction_cup().set_active(true);
    _actuators_were_armed = false;
    _last_disarmed_actuator_gcs_ms = 0;
    _range_unsafe_since_ms = 0;

    rover.companion_computer.log_nevt(NCULog::EVT_ENTER_VGSL);
    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: entered");
    return true;
}

void ModeVGSolar::_exit()
{
    stop_vehicle();
    _vg_submode = VGSubMode::STANDBY;
    _turn_phase = TurnPhase::IDLE;
    _turn_frozen = false;
    _target_speed_ms = 0.0f;
    _target_yaw_rate_cds = 0;

    AP::suction_cup().emergency_release();
    AP::suction_cup().set_active(false);
    rover.companion_computer.stop_brushes();
    AP::brush().set_active(false);
    rover.companion_computer.clear_pending_motion_commands();
    rover.companion_computer.reset_mode_status();

    rover.companion_computer.log_nevt(NCULog::EVT_EXIT_VGSL);
    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: exited");
}

void ModeVGSolar::update()
{
    // 与履带/外设一致：需 arm 且安全开关允许（soft_armed）
    const bool armed = hal.util->get_soft_armed();

    // 低电压：强制关刷；无有效电量读数时不触发（台架安全）
    if (rover.companion_computer.is_low_battery()) {
        rover.companion_computer.stop_brushes();
    }

    // 未解锁：外设回安全位，不跑运动子模式（仍推进吸盘释放序列）
    if (!armed) {
        apply_disarmed_actuator_safety();
        AP::suction_cup().update();
        sync_suction_fault_flags();
        read_companion_commands();  // 仅急停等系统指令生效；运动指令在函数内拒绝
        apply_disarmed_actuator_safety();
        // 未解锁时不要跑 stop_vehicle() 速度环：编码器噪声会把刹车 I 项顶成反转，
        // 一解锁电机真正出力就会往后拱一下。
        clear_speed_motion_state();
        if (_vg_submode == VGSubMode::ESTOP) {
            update_estop();
        }
        return;
    }

    if (!_actuators_were_armed) {
        // 解锁边沿：丢弃上锁期间串口堆积的运动指令，避免一解锁就跟旧速度
        _actuators_were_armed = true;
        rover.companion_computer.clear_pending_motion_commands();
        clear_speed_motion_state();
        if (_vg_submode == VGSubMode::YAW || _vg_submode == VGSubMode::YAWRATE) {
            _vg_submode = VGSubMode::STANDBY;
        }
        rover.companion_computer.log_nevt(NCULog::EVT_ARM_CLEAR);
        gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: armed, actuators enabled");
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

    if (_vg_submode != VGSubMode::ESTOP) {
        check_rangefinder_safety();
    }

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

void ModeVGSolar::apply_disarmed_actuator_safety()
{
    const bool disarm_edge = _actuators_were_armed;
    _actuators_were_armed = false;

    if (disarm_edge) {
        if (_vg_submode == VGSubMode::TURN && _turn_phase != TurnPhase::IDLE) {
            log_turn_event(NCULog::TURN_ACTION_ABORT, 1, NCULog::REJECT_NONE);
        }
        rover.companion_computer.log_nevt(NCULog::EVT_DISARM);
    }

    rover.companion_computer.stop_brushes();
    AP::suction_cup().emergency_release();

    // 中止依赖吸盘/运动的子模式，回到待机（保留 ESTOP）
    if (_vg_submode == VGSubMode::TURN) {
        _turn_phase = TurnPhase::IDLE;
        _turn_frozen = false;
        _vg_submode = VGSubMode::STANDBY;
        clear_safety_hold_mask();
        clear_speed_motion_state();
    } else if (_vg_submode == VGSubMode::NAV) {
        cancel_navigation();
    } else if (_vg_submode == VGSubMode::YAW || _vg_submode == VGSubMode::YAWRATE) {
        _vg_submode = VGSubMode::STANDBY;
        clear_speed_motion_state();
    }

    rover.companion_computer.clear_pending_motion_commands();

    const uint32_t now = AP_HAL::millis();
    if (disarm_edge ||
        (_last_disarmed_actuator_gcs_ms == 0) ||
        (now - _last_disarmed_actuator_gcs_ms) >= DISARMED_ACTUATOR_GCS_INTERVAL_MS) {
        _last_disarmed_actuator_gcs_ms = now;
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: disarmed, actuators safe");
    }
}

void ModeVGSolar::clear_speed_motion_state()
{
    _target_speed_ms = 0.0f;
    _target_yaw_rate_cds = 0;
    _desired_speed = 0.0f;
    have_attitude_target = false;
    start_stop();
    // 速度环 I 项停车时必须清掉，否则仍可能往前走
    attitude_control.relax_I();
    g2.motors.set_throttle(0.0f);
    g2.motors.set_steering(0.0f);
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

// NCU 指令消费；仅非零速度帧打开运动看门狗（_last_ncu_cmd_ms），不置 bit7
// 优先级：系统控制 > 转弯 > 导航 > 速度控制
void ModeVGSolar::read_companion_commands()
{
    auto &cc = rover.companion_computer;

    if (cc.is_new_system_ctrl()) {
        cc.clear_new_system_flag();
        const SystemCtrlData &cmd = cc.get_latest_system_ctrl();
        _last_ncu_cmd_ms = 0;  // 系统控制关闭运动看门狗
        _await_ncu_after_lost_motion = false;
        _fault_flags &= ~FAULT_COMM_TIMEOUT;  // 清除历史残留 bit7

        switch (cmd.command) {
        case SYS_CMD_ESTOP:
            enter_estop("VG_SOLAR: ESTOP");
            break;
        case SYS_CMD_ESTOP_CLEAR:
            if (_vg_submode == VGSubMode::ESTOP) {
                _vg_submode = VGSubMode::STANDBY;
                _fault_flags &= ~FAULT_RANGE_SAFE;
                _range_unsafe_since_ms = 0;
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
        // 急停期间丢弃运动指令并记拒因，避免 ESTOP 解除后旧指令突然生效
        if (cc.is_new_turn()) {
            cc.clear_new_turn_flag();
            const TurnData &cmd = cc.get_latest_turn();
            cc.log_ntrn(NCULog::TURN_ACTION_CMD, cmd.turn_mode, cmd.direction,
                        cmd.target_angle, cmd.angular_vel, 0, 0, NCULog::REJECT_ESTOP);
        }
        if (cc.is_new_speed_ctrl()) {
            cc.clear_new_speed_flag();
            const SpeedCtrlData &cmd = cc.get_latest_speed_ctrl();
            cc.log_nspd(cmd.control_mode, cmd.velocity, cmd.yaw_data, 0, NCULog::REJECT_ESTOP);
        }
        return;
    }

    const bool armed = hal.util->get_soft_armed();

    if (cc.is_new_turn()) {
        cc.clear_new_turn_flag();
        const TurnData &cmd = cc.get_latest_turn();
        if (!armed) {
            cc.log_ntrn(NCULog::TURN_ACTION_CMD, cmd.turn_mode, cmd.direction,
                        cmd.target_angle, cmd.angular_vel, 0, 0, NCULog::REJECT_DISARMED);
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: turn rejected, disarmed");
            return;
        }
        _last_ncu_cmd_ms = 0;  // TURN 豁免看门狗
        _await_ncu_after_lost_motion = false;
        start_turn(cmd);
        _fault_flags &= ~FAULT_COMM_TIMEOUT;
        return;
    }

    if (cc.is_new_position()) {
        cc.clear_new_position_flag();
        const PositionData &cmd = cc.get_latest_position();
        _last_ncu_cmd_ms = 0;  // NAV 豁免看门狗
        _await_ncu_after_lost_motion = false;
        _fault_flags &= ~FAULT_COMM_TIMEOUT;

        if (cmd.nav_mode == NAV_MODE_CANCEL) {
            cancel_navigation();
            cc.send_position_ack(CMD_ACK_SUCCESS);
            return;
        }

        if (!armed) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: nav rejected, disarmed");
            cc.send_position_ack(CMD_ACK_FAILED);
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
        if (!armed) {
            cc.log_nspd(cmd.control_mode, cmd.velocity, cmd.yaw_data, 0, NCULog::REJECT_DISARMED);
            return;
        }
        _await_ncu_after_lost_motion = false;
        _fault_flags &= ~FAULT_COMM_TIMEOUT;

        if (_vg_submode == VGSubMode::NAV) {
            _last_ncu_cmd_ms = 0;
            cc.log_nspd(cmd.control_mode, cmd.velocity, cmd.yaw_data, 0, NCULog::REJECT_NAV_ACTIVE);
            return;
        }

        // 吸盘 lower/raise 序列进行中不接受速度，避免带吸盘移动
        if (AP::suction_cup().is_busy()) {
            cc.log_nspd(cmd.control_mode, cmd.velocity, cmd.yaw_data, 0, NCULog::REJECT_SUCTION_BUSY);
            return;
        }

        // 转弯冻结：NCU 恢复通信后走统一安全恢复（倾角仍超限则继续等待）
        if (_vg_submode == VGSubMode::TURN && _turn_frozen) {
            _last_ncu_cmd_ms = 0;
            try_recover_safety_hold();
            cc.log_nspd(cmd.control_mode, cmd.velocity, cmd.yaw_data, 0, NCULog::REJECT_SAFETY_HOLD);
            return;
        }

        // 其它安全保持期间不跟速度、不打开看门狗
        if (_safety_hold_mask != 0) {
            _last_ncu_cmd_ms = 0;
            cc.log_nspd(cmd.control_mode, cmd.velocity, cmd.yaw_data, 0, NCULog::REJECT_SAFETY_HOLD);
            return;
        }

        // 零速：正常停车，关闭看门狗（静默不再判丢控）
        const bool yawrate_idle = (cmd.control_mode == SPEED_MODE_YAWRATE) &&
                                  (cmd.yaw_data == 0);
        const bool yaw_idle = (cmd.control_mode == SPEED_MODE_YAW);
        if (cmd.velocity == 0 && (yawrate_idle || yaw_idle)) {
            clear_speed_motion_state();
            _vg_submode = VGSubMode::STANDBY;
            _last_ncu_cmd_ms = 0;
            cc.log_nspd(cmd.control_mode, cmd.velocity, cmd.yaw_data, 1, NCULog::REJECT_NONE);
            return;
        }

        // 非零运动：打开 200ms 丢控看门狗
        _last_ncu_cmd_ms = AP_HAL::millis();
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
        cc.log_nspd(cmd.control_mode, cmd.velocity, cmd.yaw_data, 1, NCULog::REJECT_NONE);
    }
}

// 子模式执行：待机 / 航向角 / 偏航速率 / 导航
void ModeVGSolar::update_standby()
{
    // 待机：强制清速度环并停车，避免 YAWRATE 残留 I 项在超时切回来后继续加油门
    if (!is_zero(_target_speed_ms) || !is_zero(_desired_speed) || have_attitude_target) {
        clear_speed_motion_state();
    }
    stop_vehicle();
}

void ModeVGSolar::update_yaw()
{
    // 复用 ModeGuided 的 HeadingAndSpeed 能力：
    set_desired_heading_and_speed(_target_yaw_cd, _target_speed_ms);
    ModeGuided::update();
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

void ModeVGSolar::enter_estop(const char *gcs_msg)
{
    if (_vg_submode == VGSubMode::NAV) {
        stop_vehicle();
        g2.wp_nav.set_reversed(false);
        _nav_phase = NavPhase::CRUISE;
        _nav_report_state = NavReportState::CANCELLED;
    }
    _vg_submode = VGSubMode::ESTOP;
    _turn_phase = TurnPhase::IDLE;
    _turn_frozen = false;
    _last_ncu_cmd_ms = 0;
    _await_ncu_after_lost_motion = false;
    clear_safety_hold_mask();
    rover.companion_computer.stop_brushes();
    AP::suction_cup().emergency_release();
    gcs().send_text(MAV_SEVERITY_WARNING, "%s", gcs_msg != nullptr ? gcs_msg : "VG_SOLAR: ESTOP");
}

void ModeVGSolar::check_rangefinder_safety()
{
    if (_vg_submode == VGSubMode::ESTOP) {
        _range_unsafe_since_ms = 0;
        return;
    }

#if AP_RANGEFINDER_ENABLED
    const auto channel_unsafe = [this](enum Rotation orientation) -> bool {
        const RangeFinder *rfnd = RangeFinder::get_singleton();
        if (rfnd == nullptr || !rfnd->has_orientation(orientation)) {
            return true;  // 无效 / 未配置
        }
        if (rfnd->status_orient(orientation) != RangeFinder::Status::Good) {
            return true;
        }
        const uint16_t dist_cm = rfnd->distance_cm_orient(orientation);
        const int16_t min_cm = _rf_safe_min_cm;
        const int16_t max_cm = _rf_safe_max_cm;
        if (min_cm > max_cm) {
            return true;
        }
        return dist_cm < (uint16_t)min_cm || dist_cm > (uint16_t)max_cm;
    };

    // 与状态帧一致：LEFT_OUT=YAW_315，RIGHT_OUT=YAW_45
    const bool unsafe = channel_unsafe(ROTATION_YAW_315) || channel_unsafe(ROTATION_YAW_45);
    const uint32_t now = AP_HAL::millis();
    if (!unsafe) {
        _range_unsafe_since_ms = 0;
        return;
    }
    if (_range_unsafe_since_ms == 0) {
        _range_unsafe_since_ms = now;
        return;
    }
    if (now - _range_unsafe_since_ms < RANGE_SAFE_DEBOUNCE_MS) {
        return;
    }

    _fault_flags |= FAULT_RANGE_SAFE;
    enter_estop("VG_SOLAR: ESTOP rangefinder");
#else
    _fault_flags |= FAULT_RANGE_SAFE;
    enter_estop("VG_SOLAR: ESTOP rangefinder");
#endif
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

    log_turn_event(NCULog::TURN_ACTION_ABORT, 1, NCULog::REJECT_SUCTION_FAULT);

    _fault_flags |= FAULT_SUCTION_CUP;
    stop_vehicle();
    _vg_submode = VGSubMode::STANDBY;
    _turn_phase = TurnPhase::IDLE;
    _turn_frozen = false;
    _turn_timeout_aborted = false;
    clear_safety_hold_mask();
    AP::suction_cup().emergency_release();
    gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: suction fault, turn aborted");
}

void ModeVGSolar::complete_turn()
{
    if (!_turn_timeout_aborted) {
        log_turn_event(NCULog::TURN_ACTION_DONE, 1, NCULog::REJECT_NONE);
    }
    _turn_timeout_aborted = false;

    // 回待机并清零旧速度；不再恢复转弯前的 YAWRATE 目标（避免突然前冲）
    _vg_submode = VGSubMode::STANDBY;
    _turn_phase = TurnPhase::IDLE;
    _turn_frozen = false;
    clear_speed_motion_state();
    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: TURN complete, standby");
}

void ModeVGSolar::set_turn_phase(TurnPhase phase)
{
    if (_turn_phase == phase) {
        return;
    }
    _turn_phase = phase;
    log_turn_event(NCULog::TURN_ACTION_PHASE, 1, NCULog::REJECT_NONE);
}

void ModeVGSolar::log_turn_event(uint8_t action, uint8_t accepted, uint8_t reject_reason) const
{
    const uint16_t angle_cd = uint16_t(constrain_int32(lroundf(_turn_target_angle_deg * 100.0f), 0, 65535));
    const uint16_t ang_vel_cds = uint16_t(constrain_int32(lroundf(_turn_angular_vel_dps * 100.0f), 0, 65535));
    rover.companion_computer.log_ntrn(action, _turn_mode_type, _turn_direction,
                                      angle_cd, ang_vel_cds, uint8_t(_turn_phase),
                                      accepted, reject_reason);
}

// 安全保持：倾角过大 / 运动丢控（已吸附）→ freeze 吸盘，条件满足后自动恢复
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
        log_turn_event(NCULog::TURN_ACTION_ABORT, 1, NCULog::REJECT_NONE);
        _vg_submode = VGSubMode::STANDBY;
        _turn_phase = TurnPhase::IDLE;
        break;
    case VGSubMode::YAW:
    case VGSubMode::YAWRATE:
        _vg_submode = VGSubMode::STANDBY;
        clear_speed_motion_state();
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
    // TILT：倾角回限；NCU_COMM：需再收到 NCU 指令（_await_ncu_after_lost_motion 已清）；ESTOP 不自动恢复
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
        _await_ncu_after_lost_motion) {
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
    auto &cc = rover.companion_computer;

    // 安全保持/吸盘 busy/故障时拒绝新转弯
    if (_safety_hold_mask != 0) {
        cc.log_ntrn(NCULog::TURN_ACTION_CMD, cmd.turn_mode, cmd.direction,
                    cmd.target_angle, cmd.angular_vel, uint8_t(_turn_phase),
                    0, NCULog::REJECT_SAFETY_HOLD);
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: TURN rejected, safety hold active");
        return;
    }

    if (AP::suction_cup().is_busy()) {
        cc.log_ntrn(NCULog::TURN_ACTION_CMD, cmd.turn_mode, cmd.direction,
                    cmd.target_angle, cmd.angular_vel, uint8_t(_turn_phase),
                    0, NCULog::REJECT_SUCTION_BUSY);
        gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: TURN rejected, suction busy");
        return;
    }

    if (AP::suction_cup().has_fault()) {
        cc.log_ntrn(NCULog::TURN_ACTION_CMD, cmd.turn_mode, cmd.direction,
                    cmd.target_angle, cmd.angular_vel, uint8_t(_turn_phase),
                    0, NCULog::REJECT_SUCTION_FAULT);
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
    _turn_timeout_aborted = false;

    cc.log_ntrn(NCULog::TURN_ACTION_CMD, cmd.turn_mode, cmd.direction,
                cmd.target_angle, cmd.angular_vel, uint8_t(_turn_phase),
                1, NCULog::REJECT_NONE);

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
                    "VG_SOLAR TURN out: tph=%u scup_st=%u scup_ph=%u lift=%u valve=%u pump=%u",
                    unsigned(_turn_phase),
                    unsigned(scup.get_state_u8()),
                    unsigned(scup.get_phase_u8()),
                    unsigned(scup.get_last_lift_pwm_us()),
                    unsigned(scup.get_last_valve_on()),
                    unsigned(scup.get_last_pump_on()));
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
            set_turn_phase(TurnPhase::WAIT_STOPPED);
            _turn_phase_start_ms = now;
        }
        break;
    }

    case TurnPhase::WAIT_STOPPED: {
        // 停稳 500ms 后再放吸盘，避免惯性滑动
        if (now - _turn_phase_start_ms > 500) {
            set_turn_phase(TurnPhase::LOWER_SUCTION);
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
            set_turn_phase(TurnPhase::TURNING);
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
            set_turn_phase(TurnPhase::RAISE_SUCTION);
            _turn_phase_start_ms = now;
        } else if (now - _turn_phase_start_ms > _turn_timeout * 1000.0f) {
            gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: TURN angle timeout");
            stop_vehicle();
            if (scup.is_lowered()) {
                // 超时后仍抬盘；记 Abort，抬完不再写 Done
                log_turn_event(NCULog::TURN_ACTION_ABORT, 1, NCULog::REJECT_NONE);
                set_turn_phase(TurnPhase::RAISE_SUCTION);
                _turn_phase_start_ms = now;
                _turn_timeout_aborted = true;
            } else {
                log_turn_event(NCULog::TURN_ACTION_ABORT, 1, NCULog::REJECT_NONE);
                _vg_submode = VGSubMode::STANDBY;
                _turn_phase = TurnPhase::IDLE;
                _turn_frozen = false;
                clear_speed_motion_state();
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
        // 异步 raise；is_raised() 后 complete_turn() 回待机
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

// 导航取消与运动丢控看门狗（仅非零速度后 200ms 无新速度帧则停车，不置 bit7）
void ModeVGSolar::cancel_navigation()
{
    stop_vehicle();
    g2.wp_nav.set_reversed(false);
    _nav_phase = NavPhase::CRUISE;
    _nav_report_state = NavReportState::CANCELLED;
    _vg_submode = VGSubMode::STANDBY;
    gcs().send_text(MAV_SEVERITY_INFO, "VG_SOLAR: NAV cancelled");
}

void ModeVGSolar::check_ncu_timeout()
{
    // 看门狗未打开（待机/零速/系统控制/TURN/NAV 等）
    if (_last_ncu_cmd_ms == 0) {
        return;
    }

    // 转弯/导航执行期豁免（协议为单次长指令）
    if ((_vg_submode == VGSubMode::TURN && !_turn_frozen) ||
        _vg_submode == VGSubMode::NAV) {
        return;
    }

    const bool ncu_timed_out = AP_HAL::millis() - _last_ncu_cmd_ms > NCU_HEARTBEAT_TIMEOUT_MS;
    if (!ncu_timed_out) {
        return;
    }

    const int32_t submode_before = int32_t(_vg_submode);
    // Param1 与 NCLK.SinceRx 同源（合法 RX 年龄）；过大时钳到 INT32_MAX
    const uint32_t since_rx = rover.companion_computer.since_rx_ms();
    const int32_t since_rx_i32 = (since_rx > uint32_t(INT32_MAX)) ? INT32_MAX : int32_t(since_rx);

    // 运动中丢控：停车关刷，不上报 FAULT_COMM_TIMEOUT
    _last_ncu_cmd_ms = 0;
    rover.companion_computer.stop_brushes();
    gcs().send_text(MAV_SEVERITY_WARNING, "VG_SOLAR: motion cmd lost, auto stop");
    rover.companion_computer.log_nevt(NCULog::EVT_TIMEOUT, since_rx_i32, submode_before);

    clear_speed_motion_state();
    stop_vehicle();
    _vg_submode = VGSubMode::STANDBY;
    _turn_phase = TurnPhase::IDLE;
    _turn_frozen = false;

    if (AP::suction_cup().is_lowered()) {
        _await_ncu_after_lost_motion = true;
        enter_safety_hold(SAFETY_HOLD_NCU_COMM);
        AP::suction_cup().freeze();
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
