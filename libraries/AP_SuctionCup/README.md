# VGSL 接入 AP_SuctionCup — 完整行为规格

本文档描述 `AP_SuctionCup` 与 `ModeVGSolar`（VGSL）集成时的行为约定，对齐 `protocol.md` 与 `FCU_开发指南.md`。

---

## 一、协议字段分工（对齐 protocol / FCU 开发指南）

| 字段 | 含义 | 转弯期间上报 |
|------|------|----------------|
| **control_mode** | 当前控制模式 | 整个 TURN 子模式期间 = **`0x03` 转弯模式** |
| **motion_state** | 实际运动状态 | **仅吸盘吸附后、抬起前** = **`0x03` 转弯中** |
| **fault_code** | 故障位 | 按场景置 bit4/7/9 等 |

要点：**control_mode 和 motion_state 不同步是正常的**——前者表示「在执行转弯任务」，后者表示「车处于协议定义的转弯运动段」。

---

## 二、motion_state 何时报 `0x03`（转弯中）

### 协议 / 指南原文

- protocol：`0x03` = 转弯中
- FCU 开发指南 4.1：**转弯指令执行期间（吸盘已放下 → 转弯 → 吸盘未抬起）**

### 实现判定（与协议对齐）

```
motion_state = 0x03 (TURNING) 当且仅当：

  _vg_submode == TURN
  AND suction_cup.is_lowered() == true    // LOWERED 或 FROZEN（已吸附）
  AND suction_cup.is_raised() == false    // 尚未完成抬起
  AND 非急停
  AND 无 motion 级故障（见 compute_motion_state 现有 FAULT_MOTION_MASK）
```

### 各转弯阶段 motion_state 对照

| 转弯阶段 | control_mode | motion_state | 说明 |
|----------|--------------|--------------|------|
| STOPPING / WAIT_STOPPED | `0x03` | **`0x00` 静止** | 停车等待，吸盘未放下 |
| LOWERING（吸附中） | `0x03` | **`0x00` 静止** | 放下/密封/开泵中 |
| LOWERED + TURNING（差速转） | `0x03` | **`0x03` 转弯中** | 协议定义的「真转弯」 |
| RAISING（释放中） | `0x03` | **`0x00` 静止** | 关泵/放气/抬起中 |
| 抬起完成 → 退出 TURN | 恢复转弯前模式 | **`0x00` 静止** | 转弯任务结束 |

### 代码落点

在 `publish_status_feedback()` 中，将原来的：

```cpp
const bool turning = (_vg_submode == VGSubMode::TURN);
```

改为基于吸盘状态的精细判断：

```cpp
const bool turning = (_vg_submode == VGSubMode::TURN)
    && AP::suction_cup().is_lowered()
    && !AP::suction_cup().is_raised();
```

`control_mode` 在 `_vg_submode == TURN` 时仍报 `0x03`，不变。

---

## 三、VGSL 与吸盘库的基本接线

| 时机 | 调用 |
|------|------|
| `_enter()` | `AP::suction_cup().set_active(true)` |
| `_exit()` | `emergency_release()` → `set_active(false)` |
| `update()` 每周期 | `AP::suction_cup().update()` |
| 转弯 `LOWER_SUCTION` | 首次进入时 `lower()`，等 `is_lowered()` |
| 转弯 `RAISE_SUCTION` | 首次进入时 `raise()`，等 `is_raised()` |
| 急停 `update_estop()` | `stop_vehicle()` + 关刷 + `emergency_release()` |

转弯状态机里 **`WAIT_SUCTION_DN` / `WAIT_SUCTION_UP` 纯延时可去掉**，由 `AP_SuctionCup` 内部时序 + `is_lowered()` / `is_raised()` 替代。

---

## 四、安全场景行为

### 4.1 NCU 200ms 超时（bit7）

**指南**：停履带和滚刷，保持吸盘安全状态，报 bit7。

| 子模式 | FCU 行为 |
|--------|----------|
| YAW / YAWRATE / STANDBY | `stop_vehicle()` + 关刷 + 回 STANDBY |
| **TURN** | **`stop_vehicle()` + 关刷 + `suction_cup.freeze()` + 冻结转弯状态机（`turn_phase` 不再推进）+ 置 bit7** |
| NAV / ESTOP | 保持现有逻辑（不退子模式），同样 **`freeze()` 吸盘**（若已激活） |

`freeze()` 语义：

- 已吸附：关泵，阀保持密封，升降保持放下
- 动作中：停止推进 lower/raise
- **不**主动放气抬起

> 注：需修改当前 `check_ncu_timeout()` 在 TURN 时 `return` 不停车的问题。

---

### 4.2 倾角过大（bit9）

**指南**：停止履带，必要时保持吸盘吸附，报 bit9。

| 检测 | 现有：`collect_sensor_faults()`，\|roll\| 或 \|pitch\| > 30° |
|------|----------------------------------------------------------------|
| Mode 动作（新增） | **`stop_vehicle()` + `suction_cup.freeze()`** |
| 吸盘 | 已吸附则关泵保密封；未吸附则保持抬起空闲 |
| 故障上报 | bit9 经 `fault_code` 上报；`motion_state` 可能为 `0x05` 故障 |
| 转弯中 | **冻结 `turn_phase`**，不再进入 TURNING / RAISE |

与 NCU 超时处理一致：**停车 + freeze，不 release**。

---

### 4.3 急停（0xAA 0x05 command=0x01）

**指南**：立即停止运动，关闭滚刷和吸盘。

| 动作 |
|------|
| `stop_vehicle()` |
| `stop_brushes()` |
| **`suction_cup.emergency_release()`**（关泵 → 放气 → 抬起） |
| `_vg_submode = ESTOP` |
| `motion_state = 0x04` |

急停解除（0x02）：回 STANDBY；吸盘保持 **RAISED**，不自动再吸。

---

### 4.4 退出 VGSL 模式

| 动作 |
|------|
| `emergency_release()` |
| `set_active(false)` |
| 关刷 |

---

## 五、吸盘故障（bit4）后转弯收尾

**指南**：拒绝转弯或终止转弯，报 bit4。

### 触发条件

- `lower()` / `raise()` 返回失败
- `suction_cup.has_fault()`（如 `SCUP_ACT_TIMEOUT_MS` 超时）
- `AP_SuctionCup` 进入 `State::FAULT`

### 收尾流程

```
1. _fault_flags |= FAULT_SUCTION_CUP   // 置 bit4，保持至 clear_fault() 或退出 VGSL
2. stop_vehicle()
3. 终止转弯：_vg_submode = STANDBY（或 _submode_before_turn 策略二选一，建议 STANDBY）
4. _turn_phase = IDLE（或冻结在失败相位，建议 IDLE）
5. suction_cup.emergency_release()   // 尝试放气抬起
6. GCS 日志告警
```

### bit4 清除

- **不**在转弯正常结束时自动清
- 在 `_enter()` VGSL 时清零 `_fault_flags`（现有逻辑）
- 或显式 `suction_cup.clear_fault()` + 清 bit4（故障恢复后人工/NCU 确认）

### 角度转弯超时（VGS_TURN_TO）与吸盘故障区分

| 场景 | bit4 | 行为 |
|------|------|------|
| 吸盘 lower/raise 超时 | **置 bit4** | 按上表终止 + emergency_release |
| 仅角度转不够 / VGS_TURN_TO | **不置 bit4** | 若 `is_lowered()` 则 `raise()`；若未吸附则直接结束 TURN |

---

## 六、转弯状态机接入后的阶段逻辑（概要）

```
STOPPING        → stop_vehicle，停稳后进 WAIT_STOPPED
WAIT_STOPPED    → 延时后进入 LOWER_SUCTION
LOWER_SUCTION   → lower()；失败 → bit4 收尾；成功 is_lowered() → TURNING
TURNING         → 差速转；完成或 VGS_TURN_TO → RAISE_SUCTION
RAISE_SUCTION   → raise()；失败 → bit4 收尾；成功 is_raised() → 恢复 _submode_before_turn，phase=IDLE
```

**冻结条件**（NCU 超时 / bit9）：`turn_phase` 停在当前步，不再 `switch` 推进，直至退出 TURN 或新 TURN 指令（新 TURN 前需评估是否 `clear_fault` / unfreeze）。

---

## 七、优先级（motion_state 计算顺序）

`compute_motion_state()` 顺序不变：

1. 故障 mask 命中 → `0x05`
2. 急停 → `0x04`
3. **turning（新语义）** → `0x03`
4. 前进 / 后退 / 静止

因此：**bit9 / bit7 等进入 FAULT mask 时，motion_state 为 0x05，而不是 0x03**——与指南「故障优先」一致。

---

## 八、硬件与参数

| 项 | 默认假设 |
|----|----------|
| SERVO5/10/11 → FUNCTION 159/160/161 | 需地面站配置 |
| 升降 1000=抬 / 2000=放 | 可用 `SCUP_LIFT_PWM_*` 反转 |
| 气阀 1000=放气 / 2000=密封 | 可用 `SCUP_VALVE_PWM_*` 调 |
| 吸附完成 | 仅时间判定，无负压传感器 |

### 地面站参数（SCUP_）

| 参数 | 默认 | 含义 |
|------|------|------|
| SCUP_PUMP_PWR | 100 | 气泵转速 0~100%，100→2000µs |
| SCUP_LIFT_DLY_MS | 500 | 升降动作等待 |
| SCUP_VAC_DLY_MS | 800 | 开泵后建立负压等待 |
| SCUP_VENT_DLY_MS | 400 | 放气后抬起前等待 |
| SCUP_ACT_TOUT_MS | 5000 | lower/raise 总超时 |

### SERVO Function

| Function | 硬件 | 用途 |
|----------|------|------|
| 159 VGSolarSuctionLift | SERVO5 | 升降舵机 |
| 160 VGSolarAirValve | SERVO10 | 气阀 |
| 161 VGSolarAirPump | SERVO11 | 气泵 |

---

## 九、实现检查清单

- [ ] `_enter/_exit`：`set_active` / `emergency_release`
- [ ] `update()`：`suction_cup.update()`
- [ ] 转弯 TODO → `lower()` / `raise()` + 状态等待
- [ ] `motion_state`：`is_lowered() && !is_raised()` 才报 0x03
- [ ] `control_mode`：TURN 全程 0x03
- [ ] `check_ncu_timeout()`：TURN 也停车 + freeze + 冻 phase
- [ ] 倾角 bit9：Mode 侧 `stop_vehicle()` + `freeze()`
- [ ] 急停 / exit：`emergency_release()`
- [ ] 吸盘 fault：bit4 + 终止转弯 + `emergency_release()`
- [ ] `has_fault()` 合并进 `_fault_flags` 上报

---

## 十、一句话汇总

| 场景 | 履带 | 滚刷 | 吸盘 | fault | motion_state |
|------|------|------|------|-------|----------------|
| 转弯吸附段~抬起前 | 转/停 | 按 NCU | lower→turn→raise | — | **0x03 仅 is_lowered 段** |
| NCU 超时 | **停** | 关 | **freeze** | bit7 | 0x05 或 0x00 |
| 倾角过大 | **停** | — | **freeze** | bit9 | 0x05 |
| 急停 | 停 | 关 | **release** | — | 0x04 |
| 吸盘故障 | 停 | — | **release 尝试** | **bit4** | 0x05 |

---

## 十一、库对外 API 速查

| 方法 | 用途 |
|------|------|
| `set_active(bool)` | VGSL 进入/退出 |
| `update()` | 周期推进 lower/raise 状态机 |
| `lower()` | 开始吸附序列 |
| `raise()` | 开始释放序列 |
| `freeze()` | NCU 超时 / 倾角过大：关泵保密封 |
| `emergency_release()` / `stop()` | 急停 / 退出：完整释放 |
| `clear_fault()` | 清除 FAULT 状态 |
| `is_lowered()` / `is_raised()` / `is_busy()` / `has_fault()` | 状态查询 |

NCU 无吸盘专用协议；由 FCU 在转弯序列内部调用本库。