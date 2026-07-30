# NCU 通信日志设计（DataFlash）

本文约定飞控 **DataFlash BIN** 中与 NCU（上位机 / Companion）串口通信相关的日志消息，用于事后排查：是否收到指令、是否被执行、是否超时，并与 `RCOU` 等曲线对齐时间轴。

> **状态**：已实现（`CC_LOG`，消息 `NCLK`/`NSPD`/`NTRN`/`NEVT`）。以本文字段为准；实现细节见第十节。

相关文档：[FCU_NCU通信协议.md](./FCU_NCU通信协议.md)、[ModeVGSolar模式总览.md](./ModeVGSolar模式总览.md)。

---

## 一、目的与原则

| 要回答的问题 | 对应日志 |
|--------------|----------|
| 链路通不通、是否接近 200ms 超时 | `NCLK` |
| 速度指令发了什么、飞控吃没吃 | `NSPD` |
| 转弯指令与阶段（开始/相位/结束/拒绝） | `NTRN` |
| 超时停车 / 进退模式等通用跳变 | `NEVT` |

**原则：**

- 写入 BIN，不用 GCS 文本刷屏。
- 速度默认降频或变化才记，避免卡写满；可用参数加详。
- 范围以本文四张消息为准；导航等专用消息需要时再扩展。

---

## 二、消息总览

| 消息名 | 频率 | 作用 |
|--------|------|------|
| **NCLK** | 1 Hz | 链路健康 |
| **NSPD** | 变化才记，或最多约 5 Hz（详档可 10 Hz） | 速度指令 |
| **NTRN** | 事件触发（收到转弯 / 阶段变化 / 结束 / 拒绝） | 转弯专用 |
| **NEVT** | 事件触发 | 通用事件（不含转弯细节） |

### 开关参数

| 参数 | 取值 | 含义 |
|------|------|------|
| `CC_LOG` | 0 | 关闭上述 NCU 通信日志 |
| | **1（默认）** | 本表：`NCLK` + `NSPD`（降频）+ `NTRN` + `NEVT` |
| | 2 | 在 1 基础上，速度按约 10 Hz 记录（每条 Mode 消费的速度指令都记） |

---

## 三、`NCLK` — 链路健康

**触发：** 约 1 Hz（建议仅 `CC_ENABLE` 且相关串口有效时）。

| 字段 | 类型 | 含义 |
|------|------|------|
| TimeUS | uint64 | 时间（µs） |
| SinceRxMs | uint32 | 距上一帧**合法** NCU 数据的毫秒数；从未收到合法帧时为 `UINT32_MAX` |
| RxPerSec | uint16 | 上一秒成功解析的帧数 |
| BadChecksum | uint16 | 上一秒校验失败次数 |
| BadLength | uint16 | 上一秒长度不符次数 |
| DropPeriodic | uint16 | 周期发送（如 10Hz 状态）因 TX 满丢弃的次数（**累计**） |
| DropEvent | uint16 | 事件发送（ACK 等）因 TX 满丢弃的次数（**累计**） |
| CtrlMode | uint8 | 当前上报 `control_mode` |
| FaultBits | uint16 | 当前 `fault_code` |

**读法：** `SinceRxMs` 经常 >200 → 易触发 NCU 超时停车；`BadChecksum`/`BadLength` 升高 → 串口噪声或帧错乱。

---

## 四、`NSPD` — 速度指令

**触发：**

- `CC_LOG=1`：线速度/角速度/模式/**Accepted·RejectReason** 变化时记；未变化时最短间隔 200 ms（约 ≤5 Hz）
- `CC_LOG=2`：Mode 每消费一条速度指令记一条（跟 NCU 约 10 Hz）

在 Companion **解析成功**且 Mode **决定接受或拒绝**之后写一条（含拒因）。

| 字段 | 类型 | 含义 |
|------|------|------|
| TimeUS | uint64 | 时间（µs） |
| VelMode | uint8 | 1=航向角 yaw，2=角速度 yawrate |
| LinVel | int16 | 线速度，**cm/s**（与协议一致） |
| YawData | int16 | yaw：航向 **0.01°**；yawrate：角速率 **0.01°/s** |
| Accepted | uint8 | 1=Mode 已采用，0=拒绝 |
| RejectReason | uint8 | 拒因，见下表；接受时为 0 |

### RejectReason（`NSPD` / `NTRN` 共用）

| 值 | 名称 | 含义 |
|----|------|------|
| 0 | None | 已接受，或无关 |
| 1 | Disarmed | 未解锁 |
| 2 | EStop | 急停中 |
| 3 | SuctionBusy | 吸盘动作中 |
| 4 | SafetyHold | 安全保持中 |
| 5 | NavActive | 导航中拒速度（主要用于 `NSPD`） |
| 6 | SuctionFault | 吸盘故障（主要用于 `NTRN`） |

**读法：** `LinVel` 连续且 `Accepted=1`，但车仍一卡一卡 → 多半不是丢指令，应对齐 `RCOU` / 死区 / 编码器。

---

## 五、`NTRN` — 转弯（专用）

与协议 `0x02` 及 `ModeVGSolar` 转弯状态机对齐，**不**再塞进 `NEVT` 的模糊 Param。

**触发（各写一条）：**

- 收到转弯指令并判定接受或拒绝  
- 转弯**阶段变化**（进入新 `TurnPhase`）  
- 转弯正常结束 / 中止（故障、超时、安全恢复等）  

| 字段 | 类型 | 含义 |
|------|------|------|
| TimeUS | uint64 | 时间（µs） |
| Action | uint8 | 本条记录在干什么，见下表 |
| TurnMode | uint8 | 1=原地转弯，2=行进间转弯（协议 `turn_mode`） |
| Direction | uint8 | 1=左转，2=右转（协议 `direction`） |
| TargetAngle | uint16 | 目标相对转角，**0.01°**（协议单位） |
| AngVel | uint16 | 角速度上限，**0.01°/s**（协议单位） |
| Phase | uint8 | 当前/进入的转弯阶段，见下表；拒绝时可为 0 |
| Accepted | uint8 | 1=接受或进行中事件，0=本条为拒绝指令 |
| RejectReason | uint8 | 拒因（同上表）；非拒绝为 0 |

### Action

| 值 | 名称 | 含义 |
|----|------|------|
| 1 | Cmd | 收到 NCU 转弯指令（看 `Accepted` / `RejectReason`） |
| 2 | Phase | 转弯阶段变化（看 `Phase`） |
| 3 | Done | 转弯正常结束（抬盘完成回待机） |
| 4 | Abort | 转弯中止（吸盘故障、整段超时、安全恢复打断等） |

### Phase（与 `ModeVGSolar::TurnPhase` 一致）

| 值 | 名称 | 含义 |
|----|------|------|
| 0 | Idle | 空闲 / 未进入 |
| 1 | Stopping | 停车中 |
| 2 | WaitStopped | 等待停稳 |
| 3 | LowerSuction | 放吸盘 / 吸附 |
| 4 | Turning | 差速转弯中 |
| 5 | RaiseSuction | 抬吸盘 |

**读法示例：**

- 有 `Action=1, Accepted=0, RejectReason=3` → 指令到了但吸盘忙被拒  
- `Action=2, Phase=4` → 已进入真正差速转  
- 只有 `Phase=3` 很久没有 `4` → 卡在放吸盘（与实车 GCS 的 tph 日志可对照）  

---

## 六、`NEVT` — 通用事件

**触发：** 下列跳变各写一条（**不含**转弯起止，转弯一律看 `NTRN`）。

| 字段 | 类型 | 含义 |
|------|------|------|
| TimeUS | uint64 | 时间（µs） |
| EventId | uint8 | 事件类型，见下表 |
| Param1 | int32 | 附带参数 1 |
| Param2 | int32 | 附带参数 2 |

### EventId

| 值 | 含义 | Param1 | Param2 |
|----|------|--------|--------|
| 1 | NCU 通信超时停车 | `SinceRxMs`（与 `NCLK.SinceRxMs` 同源；过大钳为 `INT32_MAX`） | 超时前 VG 子模式 |
| 2 | 进入 VGSL | 0 | 0 |
| 3 | 退出 VGSL | 0 | 0 |
| 4 | 解锁边沿并清除旧运动指令 | 0 | 0 |
| 5 | 上锁 | 0 | 0 |

---

## 七、后续可扩展（非本设计必含）

| 项 | 说明 |
|----|------|
| 导航专用消息（如 NNAV） | 需要时再加 |
| 状态反馈摘要加细 | 现有 `NCLK.CtrlMode` / `FaultBits` 已可对照；若要更细再单独定字段 |

---

## 八、实现落点

| 位置 | 写什么 |
|------|--------|
| `AP_CompanionComputer` 校验失败 / 长度拒绝 | 计入 `BadChecksum` / `BadLength`（供 `NCLK`） |
| Mode `read_companion_commands` 速度接受/拒绝 | `NSPD` |
| `start_turn` / `set_turn_phase` / `complete_turn` / `abort_turn_*` | `NTRN` |
| `send_frame` 丢帧计数 | `DropPeriodic` / `DropEvent`（累计写入 `NCLK`） |
| `check_ncu_timeout`（首次置位）、进退 VGSL、解锁清指令、上锁边沿 | `NEVT` |
| `update()` 约 1 Hz | `NCLK`（实现：`AP_CompanionComputer_Logging.cpp`） |

与现有日志配合：`NSPD`/`NTRN`/`NCLK`/`NEVT` 看「上位机与模式侧」；`RCOU` / 速度相关 PID 看「执行侧」。

---

## 九、排查备忘

1. 直行卡顿：看 **NSPD** 是否连续且 `Accepted=1`，再对齐 **RCOU**。  
2. 链路：看 **NCLK** `SinceRxMs`、坏包计数；**NEVT** `EventId=1` 是否超时。  
3. 转弯异常：只看 **NTRN**——指令是否接受、`Phase` 卡在哪、是 `Done` 还是 `Abort`。  

---
