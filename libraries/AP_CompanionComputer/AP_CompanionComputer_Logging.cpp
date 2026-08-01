#include "AP_CompanionComputer.h"
#include <AP_Logger/AP_Logger.h>
#include <climits>

/*
 * DataFlash NCU 通信日志（CC_LOG）：NCLK / NSPD / NTRN / NEVT。
 * 字段约定见 docs/VGSolar/NCU通信日志设计.md。
 * 收发与解析仍在 AP_CompanionComputer.cpp；Mode 通过 log_nspd/log_ntrn/log_nevt 打点。
 */

// 将 CC_LOG 钳到 0（关）/1（默认）/2（速度约 10Hz，测试是否丢帧）
uint8_t AP_CompanionComputer::log_level() const
{
    const int8_t v = _log.get();
    if (v <= 0) {
        return 0;
    }
    if (v >= 2) {
        return 2;
    }
    return 1;
}

// 距上一帧合法 RX 的毫秒数；从未收到过则为 UINT32_MAX
uint32_t AP_CompanionComputer::since_rx_ms() const
{
    if (_last_rx_ok_ms == 0) {
        return UINT32_MAX;
    }
    return AP_HAL::millis() - _last_rx_ok_ms;
}

void AP_CompanionComputer::note_rx_ok()
{
    _last_rx_ok_ms = AP_HAL::millis();
    if (_rx_ok_sec < UINT16_MAX) {
        _rx_ok_sec++;
    }
}

void AP_CompanionComputer::note_bad_checksum()
{
    if (_bad_checksum_sec < UINT16_MAX) {
        _bad_checksum_sec++;
    }
}

void AP_CompanionComputer::note_bad_length()
{
    if (_bad_length_sec < UINT16_MAX) {
        _bad_length_sec++;
    }
}

// 约 1Hz：写 NCLK；RxPerSec/Bad* 为本秒增量（写后清零）；Drop* 为累计
void AP_CompanionComputer::maybe_write_nclk()
{
    if (log_level() == 0) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (_last_nclk_ms != 0 && (now - _last_nclk_ms) < NCULog::NCLK_INTERVAL_MS) {
        return;
    }
    _last_nclk_ms = now;

#if HAL_LOGGING_ENABLED
    const uint8_t control_mode = _fb_mode_status_valid ? _fb_control_mode : uint8_t(ControlMode::STANDBY);
    const uint16_t fault_code = _fb_fault_bits | collect_sensor_faults();

    AP::logger().WriteStreaming(
        "NCLK",
        "TimeUS,SinceRx,RxN,BadCRC,BadLen,DropP,DropE,CMode,Fault",
        "QIHHHHHBH",
        AP_HAL::micros64(),
        since_rx_ms(),
        _rx_ok_sec,
        _bad_checksum_sec,
        _bad_length_sec,
        _tx_drop_periodic,
        _tx_drop_event,
        control_mode,
        fault_code);
#endif

    // 秒窗计数清零；Drop* 保持累计不在此清
    _rx_ok_sec = 0;
    _bad_checksum_sec = 0;
    _bad_length_sec = 0;
}

// Mode 在接受/拒绝速度后调用。CC_LOG=1：速度或接受/拒因变化立刻记，未变则最短间隔 200ms；=2：每条都记
void AP_CompanionComputer::log_nspd(uint8_t vel_mode, int16_t lin_vel_cms, int16_t yaw_data,
                                   uint8_t accepted, uint8_t reject_reason)
{
    if (log_level() == 0) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (log_level() == 1) {
        const bool changed = !_last_nspd_valid ||
                             vel_mode != _last_nspd_vel_mode ||
                             lin_vel_cms != _last_nspd_lin_vel ||
                             yaw_data != _last_nspd_yaw_data ||
                             accepted != _last_nspd_accepted ||
                             reject_reason != _last_nspd_reject_reason;
        if (!changed && (now - _last_nspd_ms) < NCULog::NSPD_MIN_INTERVAL_MS) {
            return;
        }
    }

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming(
        "NSPD",
        "TimeUS,VelMode,LinVel,YawData,Acc,RRej",
        "QBhhBB",
        AP_HAL::micros64(),
        vel_mode,
        lin_vel_cms,
        yaw_data,
        accepted,
        reject_reason);
#endif

    _last_nspd_ms = now;
    _last_nspd_vel_mode = vel_mode;
    _last_nspd_lin_vel = lin_vel_cms;
    _last_nspd_yaw_data = yaw_data;
    _last_nspd_accepted = accepted;
    _last_nspd_reject_reason = reject_reason;
    _last_nspd_valid = true;
}

// Mode 转弯事件：Cmd / Phase / Done / Abort（见 NCULog::TURN_ACTION_*）
void AP_CompanionComputer::log_ntrn(uint8_t action, uint8_t turn_mode, uint8_t direction,
                                   uint16_t target_angle_cd, uint16_t ang_vel_cds,
                                   uint8_t phase, uint8_t accepted, uint8_t reject_reason)
{
    if (log_level() == 0) {
        return;
    }

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming(
        "NTRN",
        "TimeUS,Act,TMode,Dir,TAng,AVel,Phase,Acc,RRej",
        "QBBBHHBBB",
        AP_HAL::micros64(),
        action,
        turn_mode,
        direction,
        target_angle_cd,
        ang_vel_cds,
        phase,
        accepted,
        reject_reason);
#else
    (void)action;
    (void)turn_mode;
    (void)direction;
    (void)target_angle_cd;
    (void)ang_vel_cds;
    (void)phase;
    (void)accepted;
    (void)reject_reason;
#endif
}

// Mode 通用事件：超时/进退 VGSL/解锁清指令/上锁（见 NCULog::EVT_*）
void AP_CompanionComputer::log_nevt(uint8_t event_id, int32_t param1, int32_t param2)
{
    if (log_level() == 0) {
        return;
    }

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming(
        "NEVT",
        "TimeUS,EventId,Param1,Param2",
        "QBii",
        AP_HAL::micros64(),
        event_id,
        param1,
        param2);
#else
    (void)event_id;
    (void)param1;
    (void)param2;
#endif
}
