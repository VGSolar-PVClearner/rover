# VGSL 接入 AP_SuctionCup — 行为规格

本文档描述 `AP_SuctionCup` 与 `ModeVGSolar`（VGSL）集成时的行为约定，对齐 `protocol.md` 与 `FCU_开发指南.md`。

### 源码拆分

| 文件 | 内容 |
|------|------|
| `AP_SuctionCup.h` | 公共 API、状态机声明、`SCUP_*` 成员 |
| `AP_SuctionCup.cpp` | 参数、升降缓速、Relay、lower/raise 状态机 |
| `AP_SuctionCup_IR.cpp` | 升降红外 GPIO / 消抖 / 到位等待 |

参数仍统一为 `SCUP_*` / `SCUP_IR_*`，不单独建顶层 `AP_` 库。

---

## 一、协议字段分工

| 字段 | 含义 | 转弯期间上报 |
|------|------|----------------|
| **control_mode** | 当前控制模式 | 整个 TURN 子模式期间 = **`0x03`** |
| **motion_state** | 实际运动状态 | **仅已吸附且未抬起、非冻结** = **`0x03`** |
| **fault_code** | 故障位 | bit4 吸盘 / bit9 倾角 等（bit7 通信超时不再置位） |

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
| LOWERED + TURNING | `0x03` | **`0x03`** | 差速转（气泵开、阀密封） |
| RAISING | `0x03` | `0x00` | 关泵/放气/抬起 |
| 完成退出 TURN | 回待机 STANDBY（不恢复转弯前 yaw/yawrate） | `0x00` | — |

---

## 三、VGSL 与吸盘库接线

| 时机 | 调用 |
|------|------|
| `_enter()` | `clear_fault()` → `set_active(true)` |
| `_exit()` | `emergency_release()` → `set_active(false)` |
| `update()` | 已解锁：`suction_cup.update()` → 倾角/NCU 安全…；**未解锁**：外设安全位并中止 TURN/NAV |
| 转弯 LOWER_SUCTION | `lower()`，等 `is_lowered()`（需已解锁） |
| 转弯 RAISE_SUCTION | 若冻结则先 `unfreeze()`，再 `raise()`，等 `is_raised()` |
| 急停指令 | `emergency_release()`（`update_estop()` 仅停车+关刷） |
| 未解锁 / 中途 disarm | 滚刷停、吸盘释放（抬起+放气+停泵）；`lower()` 拒绝 |

转弯阶段由 `AP_SuctionCup` 内部时序 + `is_lowered()` / `is_raised()` 驱动，Mode 层无额外纯延时。  
进 VGSL **不要求**先解锁；未解锁时可进模式，但外设保持安全位，解锁后才允许滚刷/气泵/气阀/吸附动作。

---

## 四、吸盘状态机（AP_SuctionCup）

### 4.1 吸附序列 lower()

```
放气+停泵 → 缓速放下 → PWM 到位后等到位（红外有铁片→无铁片，或 LIFT_DLY）→ 密封阀 → 开泵 → 等负压（压力达标或 `VAC_DLY`）→ LOWERED（开泵维持）
```

有红外时：缓速过程中即可记下「有铁片」；PWM 到位后再等到「完全放下」。`LIFT_TO_MS`（自 PWM 到位起算）内未完成 → FAULT，**不密封、不开泵**。
（避免断线/下拉一直读「无铁片」时立刻密封开泵；也避免到位时铁片已离开而从未记过「有」。）

### 4.2 LOWERED 维持（`apply_lowered_hold`）

进入 LOWERED 后及转向全程：

- 气泵 **开**（Relay on），直至 `raise()` / 急停 / `freeze()`
- 气阀 **密封**（Relay on）
- 升降 **放下**

### 4.3 释放序列 raise()

```
关泵 → 开阀放气 → 等 VENT_DLY → 缓速抬起 → 等到位（红外见铁片+LIFT_DLY 且 PWM 到位，或仅 LIFT_DLY）→ RAISED
```

有红外时抬起到位：缓速过程中即可等到「有铁片」→ 再等 `LIFT_DLY_MS` 补行程（见过后抖动不重计时）；结束前还须 PWM 到位。  
`LIFT_TO_MS` 内一直看不到铁片 → FAULT。无红外时：PWM 到位后再等 `LIFT_DLY_MS`。

### 4.4 freeze() / unfreeze()

| 方法 | 行为 |
|------|------|
| `freeze()` | 停推进 lower/raise；已吸附则停泵+密封；不主动抬起 |
| `unfreeze()` | 清 `_frozen`；已完成负压 → `LOWERED`（恢复开泵维持）；LOWERING 中途冻结 → `RAISED` |

---

## 五、安全保持与恢复（ModeVGSolar）

### 5.1 标志位 `_safety_hold_mask`

| 位 | 触发 | 解除条件 |
|----|------|----------|
| `SAFETY_HOLD_TILT` | 已吸附且 \|roll\|/\|pitch\|>30° | 倾角回限 |
| `SAFETY_HOLD_NCU_COMM` | 运动丢控超时且已吸附（不置 bit7） | 再收到 NCU 指令 |

可叠加；**ESTOP 期间不自动恢复**。

### 5.2 触发动作

| 场景 | 动作 |
|------|------|
| 倾角过大（已吸附） | `stop_vehicle()` + `enter_safety_hold(TILT)` + `freeze()`；TURN 时 `_turn_frozen=true` |
| 运动丢控 | `stop_vehicle()` + 关刷 +（已吸附时）`enter_safety_hold(NCU)` + `freeze()`；**不置 bit7** |

**运动看门狗**：仅非零速度帧启动；零速/系统控制/TURN/NAV 不启动。静默不报通信故障。

### 5.3 恢复 `try_recover_safety_hold()`

每周期在 `read_companion_commands()`、`check_ncu_timeout()` 之后调用。条件全部满足时：

1. `abort_motion_for_safety_recovery()` — NAV 取消 / TURN→STANDBY / YAW→STANDBY
2. `release_safety_hold_suction()` — `unfreeze()` + 若仍吸附则 `raise()`
3. 清除 `_safety_hold_mask`、`_turn_frozen`

| 恢复途径 | 说明 |
|----------|------|
| 自动 | 倾角回限；运动丢控 hold 需再收到 NCU 指令（清 `_await_ncu_after_lost_motion`）后，同周期末 `try_recover_safety_hold()` |
| 速度帧 | **TURN 全程拒速度**（`REJECT_TURN_ACTIVE`）；非 TURN 时速度可清 `_await` 并参与恢复 |
| 解除急停 | `SYS_CMD_ESTOP_CLEAR` 后调用 `try_recover_safety_hold()` |

**安全保持期间**：**NAV/YAW/YAWRATE 不再执行 motion update**（仅 `stop_vehicle` / STANDBY）；TURN 走 `_turn_frozen` 路径且拒速度。

`unfreeze()`：若 freeze 时未完成负压（LOWERING 中途），回到 **RAISED** 而非 LOWERED。

`clear_safety_hold_mask()`：ESTOP / 吸盘 fault 收尾时清零 hold 标志。

---

## 六、其它安全场景

### 6.1 急停（SYS_CMD_ESTOP）

`stop_vehicle()` + 关刷 + `emergency_release()` + ESTOP。  
急停解除回 STANDBY，再尝试安全恢复。

### 6.2 吸盘故障 bit4

- 触发：`lower`/`raise` 超时（`SCUP_ACT_TOUT_MS`）、升降红外到位超时（`SCUP_LIFT_TO_MS`）等 → `State::FAULT`
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

### GCS 里程碑（阶段边沿，各一条）

| 时机 | 文案 |
|------|------|
| 进入转弯 | `VG_SOLAR: TURN start dir=… mode=… angle=…` |
| 开始放吸盘 | `VG_SOLAR: TURN lowering suction` |
| 负压到位 | `Suction vacuum established …`（`AP_SuctionCup`） |
| 开始差速转弯 | `VG_SOLAR: TURN rotating` |
| 开始抬盘 | `VG_SOLAR: TURN raising suction` |
| 抬盘完成回待机 | `VG_SOLAR: TURN complete, standby` |

拒绝/超时等 WARNING 仍按原逻辑上报。

---

## 八、motion_state 计算优先级

`compute_motion_state()`：故障 mask → 急停 → turning → 前进/后退/静止。  
bit9 等在 mask 中时 motion_state 为 **0x05**，而非 0x03（bit7 已移出 mask）。

---

## 九、硬件与参数

| 项 | 说明 |
|----|------|
| 升降 SERVO → FUNCTION **159** | **需地面站手动配置** |
| 升降默认 1900=抬 / 1100=放 | `SCUP_LIFT_PWM_*` 可调；按 `SCUP_LIFT_RATE` 缓变 |
| 气阀 / 气泵 | **Relay**（`SCUP_VLV_RLY` / `SCUP_PUMP_RLY`）；需配 `RELAYx_PIN` 等 |
| 升降到位 | 槽型光电（`SCUP_IR_PIN`，默认 98）或禁用后仅 `LIFT_DLY` |
| 负压判定 | 启用 `AP_SuctionPressure` 时按 `VAC_P_KPA`/`VAC_DEB_MS`；禁用时固定等 `VAC_DLY_MS` |

### 地面站参数（SCUP_）

| 参数 | 默认 | 范围 | 含义 |
|------|------|------|------|
| SCUP_LIFT_DLY_MS | **2000** | 0~5000 | 无红外：PWM **到位后**等待；有红外抬起：见到铁片后的补行程 |
| SCUP_VAC_DLY_MS | **3000** | 100~10000 | 启用压力时：建负压最大等待；禁用时：固定等待 |
| SCUP_VENT_DLY_MS | **2000** | 100~5000 | 放气后、抬起前等待 |
| SCUP_ACT_TOUT_MS | **30000** | 1000~60000 | lower/raise 整段超时 → FAULT |
| SCUP_LIFT_PWM_R | **1900** | 1000~2000 | 抬起位置 PWM µs |
| SCUP_LIFT_PWM_L | **1100** | 1000~2000 | 放下位置 PWM µs |
| SCUP_LIFT_RATE | **400** | 50~5000 | 升降 PWM 缓变速率 µs/s（默认约 2s 走完 800µs） |
| SCUP_VLV_RLY | **0** | 0~5 | 气阀 Relay 实例（0=RELAY1）；on=密封 / off=放气 |
| SCUP_PUMP_RLY | **1** | 0~5 | 气泵 Relay 实例（1=RELAY2）；on=开泵 / off=关泵 |
| SCUP_IR_PIN | **98** | ≥0 或 -1 | 槽型光电 GPIO（VGSolar BP_IR）；**-1=禁用** |
| SCUP_IR_POL | **0** | 0/1 | 0：高=有铁片、低=完全放下；1：反相 |
| SCUP_IR_DEB_MS | **30** | 0~500 | 红外电平消抖时间 |
| SCUP_LIFT_TO_MS | **5000** | 500~15000 | 有红外时等放下/见到铁片超时 → FAULT |
| SCUP_VAC_P_KPA | **-50** | -100~0 | 吸附建立阈值 kPa（需压力传感器） |
| SCUP_VAC_DEB_MS | **300** | 0~2000 | 压力连续达标消抖 ms |
| SCUP_VAC_HYST | **5** | 0~50 | 掉压回差 kPa |

`ACT_TOUT_MS` 应覆盖：缓速时间 + 红外/`LIFT_DLY` + `VAC_DLY`（或 `VENT_DLY` + 缓速 + 红外/`LIFT_DLY`）。  
有红外时另受 `LIFT_TO_MS` 约束（下降超时不会进入密封/开泵）。

红外输入：`INPUT` + **PULLDOWN**（对齐 hwdef）；脚无效时回退 `LIFT_DLY_MS` 并打 GCS 警告。

### 升降 SERVO

| Function | 典型通道 | 用途 |
|----------|----------|------|
| 159 VGSolarSuctionLift | SERVO5 | 升降 |

### Relay 配置示例

```text
RELAY1_PIN = <气阀 GPIO>
RELAY1_FUNCTION = 1          # Relay
RELAY2_PIN = <气泵 GPIO>
RELAY2_FUNCTION = 1
# 若硬件低有效：RELAYx_INVERTED = 1
```

旧 SERVO FUNCTION 160/161（气阀/气泵 PWM）已不再由本库使用。

---

## 十、场景速查

| 场景 | 履带 | 滚刷 | 吸盘 | fault | motion_state |
|------|------|------|------|-------|--------------|
| 转弯 LOWERED 段 | 差速 | 按 NCU | 开泵+密封 | — | **0x03** |
| 倾角 hold | 停 | — | freeze | bit9 | 0x05 |
| 运动丢控 hold | 停 | 关 | freeze | — | 0x00（不置 bit7） |
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
| `get_last_lift_pwm_us()` / `get_last_valve_on()` / `get_last_pump_on()` | GCS 调试 |
| `get_state_u8()` / `get_phase_u8()` | GCS 调试 |

NCU 无吸盘专用协议；由 FCU 在转弯序列内调用本库。

---

## 十二、已知限制

1. 负压：默认走 `AP_SuctionPressure`（`SPRESS_*` + `SCUP_VAC_P_KPA` 等）；`SPRESS_PIN=-1` 时回退固定 `VAC_DLY_MS`  
2. 红外表示「是否完全放下」（槽型+铁片），不是双端点；抬起=见到铁片后再 `LIFT_DLY`，且须 PWM 到位  
3. `LOWER_SEAL` 后下一周期即开泵，无单独 seal 等待  
4. 运行中 FAULT 需退出再进 VGSL 或地面站 `clear_fault()`，无 NCU 专用清障指令  
5. 未解锁时外设强制安全位；解锁预检含 `servo_checks`，VGSL 侧不再单独复检  
6. 红外脚无效时回退 `LIFT_DLY_MS` 并打 GCS 警告；`IR_POL` 需实机标定  
7. 已保存旧参数的飞控不会自动更新升降默认值，需手动设 `LIFT_PWM_R/L` 或重置 SCUP 相关参数
