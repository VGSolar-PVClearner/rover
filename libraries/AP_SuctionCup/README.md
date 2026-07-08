# VGSL 接入 AP_SuctionCup — 行为规格

本文档描述 `AP_SuctionCup` 与 `ModeVGSolar`（VGSL）集成时的行为约定，对齐 `protocol.md` 与 `FCU_开发指南.md`。

---

## 一、协议字段分工

| 字段 | 含义 | 转弯期间上报 |
|------|------|----------------|
| **control_mode** | 当前控制模式 | 整个 TURN 子模式期间 = **`0x03`** |
| **motion_state** | 实际运动状态 | **仅已吸附且未抬起、非冻结** = **`0x03`** |
| **fault_code** | 故障位 | bit4 吸盘 / bit7 NCU 超时 / bit9 倾角 等 |

**control_mode 与 motion_state 不同步是正常的**：前者表示「在执行转弯任务」，后者表示「处于协议定义的差速转弯段」。

---

## 二、motion_state 何时报 `0x03`

协议：吸盘已放下 → 差速转弯 → 吸盘未抬起。

```cpp
// publish_status_feedback()
const bool turning = (_vg_submode == VGSubMode::TURN)
    && AP::suction_cup().is_lowered()
    && !AP::suction_cup().is_raised()
    && !_turn_frozen
    && !AP::suction_cup().is_frozen();
```

| 转弯阶段 | control_mode | motion_state | 说明 |
|----------|--------------|--------------|------|
| STOPPING / WAIT_STOPPED | `0x03` | `0x00` | 停车等待 |
| LOWERING | `0x03` | `0x00` | 放下/密封/开泵 |
| LOWERED + TURNING | `0x03` | **`0x03`** | 差速转（气泵已停，阀密封） |
| RAISING | `0x03` | `0x00` | 关泵/放气/抬起 |
| 完成退出 TURN | 恢复转弯前模式 | `0x00` | — |

---

## 三、VGSL 与吸盘库接线

| 时机 | 调用 |
|------|------|
| `_enter()` | `clear_fault()` → `set_active(true)` |
| `_exit()` | `emergency_release()` → `set_active(false)` |
| `update()` | `suction_cup.update()` → `check_tilt_safety()` → … → `try_recover_safety_hold()` |
| 转弯 LOWER_SUCTION | `lower()`，等 `is_lowered()` |
| 转弯 RAISE_SUCTION | `raise()`，等 `is_raised()` |
| 急停指令 | `emergency_release()`（`update_estop()` 仅停车+关刷） |

转弯阶段由 `AP_SuctionCup` 内部时序 + `is_lowered()` / `is_raised()` 驱动，Mode 层无额外纯延时。

---

## 四、吸盘状态机（AP_SuctionCup）

### 4.1 吸附序列 lower()

```
放气+停泵 → 放下升降 → 等 LIFT_DLY → 密封阀 → 开泵 → 等 VAC_DLY → LOWERED
```

### 4.2 LOWERED 维持（`apply_lowered_hold`）

进入 LOWERED 后及转向全程：

- 气泵 **停**（1000µs）
- 气阀 **密封**（维持负压）
- 升降 **放下**

### 4.3 释放序列 raise()

```
关泵 → 开阀放气 → 等 VENT_DLY → 抬起 → 等 LIFT_DLY → RAISED
```

### 4.4 freeze() / unfreeze()

| 方法 | 行为 |
|------|------|
| `freeze()` | 停推进 lower/raise；已吸附则 `apply_lowered_hold()`；不主动抬起 |
| `unfreeze()` | 清 `_frozen`；已完成负压 → `LOWERED`；LOWERING 中途冻结 → `RAISED` |

---

## 五、安全保持与恢复（ModeVGSolar）

### 5.1 标志位 `_safety_hold_mask`

| 位 | 触发 | 解除条件 |
|----|------|----------|
| `SAFETY_HOLD_TILT` | 已吸附且 \|roll\|/\|pitch\|>30° | 倾角回限 |
| `SAFETY_HOLD_NCU_COMM` | NCU 200ms 无帧（bit7） | 收到 NCU 帧，bit7 清除 |

可叠加；**ESTOP 期间不自动恢复**。

### 5.2 触发动作

| 场景 | 动作 |
|------|------|
| 倾角过大（已吸附） | `stop_vehicle()` + `enter_safety_hold(TILT)` + `freeze()`；TURN 时 `_turn_frozen=true` |
| NCU 超时 | `stop_vehicle()` + 关刷 + bit7 + `enter_safety_hold(NCU)` + `freeze()`（已吸附或 TURN 冻结） |

**NCU 超时豁免**（协议单次长指令）：

- TURN 执行中且 **未** `_turn_frozen` → 不判超时
- NAV 执行中 → 不判超时

### 5.3 恢复 `try_recover_safety_hold()`

每周期在 `read_companion_commands()`、`check_ncu_timeout()` 之后调用。条件全部满足时：

1. `abort_motion_for_safety_recovery()` — NAV 取消 / TURN→STANDBY / YAW→STANDBY
2. `release_safety_hold_suction()` — `unfreeze()` + 若仍吸附则 `raise()`
3. 清除 `_safety_hold_mask`、`_turn_frozen`

| 恢复途径 | 说明 |
|----------|------|
| 自动 | 倾角回限和/或 NCU 通信恢复 |
| 速度帧 | TURN 且 `_turn_frozen` 时，速度帧刷新心跳并尝试恢复（倾角仍超限则继续等） |
| 解除急停 | `SYS_CMD_ESTOP_CLEAR` 后调用 `try_recover_safety_hold()` |

**安全保持期间**：速度帧仅刷新心跳；**NAV/YAW/YAWRATE 不再执行 motion update**（仅 `stop_vehicle` / STANDBY）；TURN 走 `_turn_frozen` 路径。

`unfreeze()`：若 freeze 时未完成负压（LOWERING 中途），回到 **RAISED** 而非 LOWERED。

`clear_safety_hold_mask()`：ESTOP / 吸盘 fault 收尾时清零 hold 标志。

---

## 六、其它安全场景

### 6.1 急停（SYS_CMD_ESTOP）

`stop_vehicle()` + 关刷 + `emergency_release()` + ESTOP。  
急停解除回 STANDBY，再尝试安全恢复。

### 6.2 吸盘故障 bit4

- 触发：`lower`/`raise` 超时（`SCUP_ACT_TOUT_MS`）等 → `State::FAULT`
- 收尾：`abort_turn_suction_fault()` — STANDBY + `emergency_release()` + bit4
- 新 turn：`has_fault()` 或 `_safety_hold_mask` 或 `is_busy()` → 拒绝
- 清除：`_enter()` VGSL 时 `clear_fault()`

### 6.3 角度转弯超时（VGS_TURN_TO）

**不置 bit4**。若已吸附则 `raise()`；未吸附则直接 `complete_turn()`。

---

## 七、转弯状态机（ModeVGSolar）

```
STOPPING → WAIT_STOPPED(500ms) → LOWER_SUCTION → TURNING → RAISE_SUCTION → complete_turn
```

| 阶段 | 说明 |
|------|------|
| `_turn_frozen` | 安全保持时冻结，仅 `stop_vehicle()`，等 `try_recover_safety_hold()` |
| LOWER_SUCTION | `lower()` 失败 → bit4 收尾 |
| TURNING | 按累计转角或 VGS_TURN_TO 结束 |
| RAISE_SUCTION | `raise()` 失败 → bit4 收尾 |

### GCS 调试日志

`send_turn_pwm_gcs()`：阶段切换立即上报，同阶段最多 1Hz。

```
VG_SOLAR TURN pwm: tph=%u scup_st=%u scup_ph=%u lift=%u valve=%u pump=%u
```

| tph | 含义 |
|-----|------|
| 1 | STOPPING |
| 2 | WAIT_STOPPED |
| 3 | LOWER_SUCTION |
| 4 | TURNING |
| 5 | RAISE_SUCTION |

---

## 八、motion_state 计算优先级

`compute_motion_state()`：故障 mask → 急停 → turning → 前进/后退/静止。  
bit7/bit9 在 mask 中时 motion_state 为 **0x05**，而非 0x03。

---

## 九、硬件与参数

| 项 | 说明 |
|----|------|
| SERVO5/10/11 → FUNCTION 159/160/161 | **需地面站手动配置** |
| 升降 1000=抬 / 2000=放 | `SCUP_LIFT_PWM_*` 可调 |
| 气阀 1000=放气 / 2000=密封 | `SCUP_VLV_PWM_*` 可调 |
| 吸附判定 | **仅延时**，无负压传感器（实机建议后续接入） |

### 地面站参数（SCUP_）

| 参数 | 默认 | 范围 | 含义 |
|------|------|------|------|
| SCUP_PUMP_PWR | 100 | 0~100 | 建负压时气泵功率%，100→2000µs |
| SCUP_LIFT_DLY_MS | **2000** | 100~5000 | 升降动作后等待（放下/抬起共用） |
| SCUP_VAC_DLY_MS | **3000** | 100~10000 | 开泵后建立负压等待 |
| SCUP_VENT_DLY_MS | **2000** | 100~5000 | 放气后、抬起前等待 |
| SCUP_ACT_TOUT_MS | **20000** | 1000~30000 | lower/raise 整段超时 → FAULT |

`ACT_TOUT_MS` 应大于 `LIFT_DLY+VAC_DLY` 与 `VENT_DLY+LIFT_DLY` 之和。

### SERVO Function

| Function | 典型通道 | 用途 |
|----------|----------|------|
| 159 VGSolarSuctionLift | SERVO5 | 升降 |
| 160 VGSolarAirValve | SERVO10 | 气阀 |
| 161 VGSolarAirPump | SERVO11 | 气泵 |

---

## 十、场景速查

| 场景 | 履带 | 滚刷 | 吸盘 | fault | motion_state |
|------|------|------|------|-------|--------------|
| 转弯 LOWERED 段 | 差速 | 按 NCU | 关泵+密封 | — | **0x03** |
| 倾角 hold | 停 | — | freeze | bit9 | 0x05 |
| NCU hold | 停 | 关 | freeze | bit7 | 0x05 |
| hold 恢复 | 停→STANDBY | — | raise | 条件清 | 0x00 |
| 急停 | 停 | 关 | release | — | 0x04 |
| 吸盘 FAULT | 停 | — | release | bit4 | 0x05 |

---

## 十一、库 API 速查

| 方法 | 用途 |
|------|------|
| `set_active(bool)` | VGSL 进入/退出 |
| `update()` | 推进 lower/raise |
| `lower()` / `raise()` | 吸附 / 释放序列 |
| `freeze()` / `unfreeze()` | 安全保持 / 解除（unfreeze 后 FROZEN→LOWERED） |
| `emergency_release()` | 急停/退出：完整释放 |
| `clear_fault()` | 清除 FAULT（`_enter()` 时调用） |
| `is_lowered()` / `is_raised()` / `is_busy()` / `is_frozen()` / `has_fault()` | 状态查询 |
| `get_last_*_pwm_us()` / `get_state_u8()` / `get_phase_u8()` | GCS 调试 |

NCU 无吸盘专用协议；由 FCU 在转弯序列内调用本库。

---

## 十二、已知限制

1. 无负压/到位传感器，仅靠 `SCUP_*_DLY_MS` 判定吸附完成  
2. 放下与抬起共用 `SCUP_LIFT_DLY_MS`  
3. `LOWER_SEAL` 后下一周期即开泵，无单独 seal 等待  
4. 运行中 FAULT 需退出再进 VGSL 或地面站 `clear_fault()`，无 NCU 专用清障指令
