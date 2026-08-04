# ModeVGSolar（VGSL）模式总览

本文概括飞控 **模式 17 / ModeVGSolar** 的职责、架构、子状态机与安全行为，作为 `docs/VGSolar/` 文档入口。细则见文末链接。

---

## 一、定位

| 项 | 说明 |
|----|------|
| 模式号 | **17**（`Mode::Number::VGSOLAR`） |
| 类 | `ModeVGSolar`，继承 `ModeGuided` |
| 使能 | 地面站 `VGS_ENABLE=1`，且飞行模式切到 VGSolar |
| 职责 | 消费 NCU 指令，驱动履带运动、转弯吸盘序列、滚刷、状态上报 |

**不负责**：MAVLink 主控链路（调试仍可用）；吸盘/滚刷底层时序分别在 `AP_SuctionCup` / `AP_Brush`；2BLD6010 RS485 遥测由独立的 `AP_ESC_Telem_2BLD6010` 后端运行，不属于模式状态机。

---

## 二、架构与调度

```
NCU (UART, PROTOCOL=50)
        │
        ▼
AP_CompanionComputer          ← 50Hz update：收帧、校验、ACK/参数反馈
        │ 缓存最新指令
        ▼
ModeVGSolar::update()         ← 主循环
  ├─ 未解锁 → 外设安全位，中止 TURN/NAV
  ├─ 已解锁 → suction_cup.update()
  ├─ read_companion_commands()  指令优先级见下
  ├─ 倾角 / 运动丢控看门狗 / 安全恢复
  └─ 按 VGSubMode 执行 YAW / YAWRATE / TURN / NAV / ESTOP
        │
        ├─► ModeGuided（航向/角速度/航点）
        ├─► AP_SuctionCup（转弯内吸附）
        └─► AP_Brush（NCU 运行参数）

10Hz：publish_status_feedback → companion send_data（0xBB 0x01）
      导航中另发 0xBB 0x04

独立 100Hz 调度：AP_ESC_Telem_2BLD6010::update()
      └─ SERIAL8 / protocol 51 轮询 1~4 台电调并发布标准 ESC 遥测
```

**NCU 指令优先级**（同周期内）：

```
系统控制 > 转弯 > 位置导航 > 速度控制
```

---

## 三、子模式 `VGSubMode`

| 子模式 | 含义 | 典型触发 |
|--------|------|----------|
| `STANDBY` | 待机停车 | 进模式、取消导航、任务结束 |
| `YAW` | 航向角 + 线速度 | NCU 速度 `0x01` / yaw |
| `YAWRATE` | 角速度 + 线速度 | NCU 速度 `0x01` / yawrate |
| `TURN` | 吸盘转弯序列 | NCU 转弯 `0x02` |
| `NAV` | 位置导航 | NCU 位置 `0x06` |
| `ESTOP` | 急停 | NCU 系统 `0x05` 急停 |

状态反馈中的 `control_mode` 与上述对应（导航再分为 GPS/NED=`0x04`、Body=`0x05`）。

---

## 四、转弯子状态机 `TurnPhase`

```
STOPPING → WAIT_STOPPED(约 500ms) → LOWER_SUCTION → TURNING → RAISE_SUCTION → 完成
```

| 阶段 | 行为 |
|------|------|
| STOPPING / WAIT_STOPPED | 停车、等停稳 |
| LOWER_SUCTION | `suction_cup.lower()`，等 `is_lowered()` |
| TURNING | 差速或行进间转，累计航向至目标角 |
| RAISE_SUCTION | `suction_cup.raise()`，等 `is_raised()` |
| `_turn_frozen` | 安全保持时冻结，仅停车，等恢复后再抬盘收尾 |

- 整段超时：`VGS_TURN_TO`（秒），**不置**吸盘 bit4；已吸附则尝试抬起。  
- 吸盘 FAULT：中止转弯，上报 bit4。  
- `control_mode` 全程转弯=`0x03`；`motion_state=0x03` 仅在已吸附差速段（见协议文档）。

---

## 五、位置导航（简要）

| 模式 | 坐标 |
|------|------|
| GPS | 经纬度×1e7 |
| NED | 相对全局原点的北/东 cm |
| Body | 相对发令时刻车体前/右 cm |
| 取消 | 立即停导航回待机 |

到达半径内可先对航向（`NavPhase`：巡航 → 对航向），再报到达。导航中周期性上报距目标与航向偏差。

---

## 六、进出模式与解锁

| 时机 | 行为 |
|------|------|
| `_enter()` | 要求 `VGS_ENABLE`；`clear_fault()`；激活吸盘/滚刷库；子模式待机；**丢弃未消费的 NCU 速度/转弯/导航**（避免 Manual 切回后突然跟旧指令） |
| `_exit()` | 停车；吸盘紧急释放并去激活；滚刷停并去激活；丢弃未消费运动指令 |
| 未解锁 | 可进入 VGSL，但滚刷/阀/泵/吸盘强制安全位；`lower()` 拒绝；运动类指令受限 |
| 中途 disarm | 同安全位，并中止 TURN/NAV 类动作 |

---

## 七、安全保持

| 触发 | 动作 |
|------|------|
| 已吸附且 \|roll\|/\|pitch\| 过大（约 >30°） | 停车 + `freeze` 吸盘 + bit9；转弯则 `_turn_frozen` |
| 非零速度后 ≈200 ms 无新速度帧 | 停车、关刷、清零速度目标；已吸附则 freeze；**不置 bit7**（零速/待机静默正常；TURN/NAV 不启看门狗） |
| 解锁边沿 | 丢弃上锁期间堆积的 NCU 运动指令并清零速度目标，避免一解锁就跟旧速度 |
| 急停 | 停车、关刷、吸盘完整释放；`motion_state=0x04` |
| 超声波安全带 | LEFT_OUT/RIGHT_OUT 不在 `VGS_RF_MIN~MAX` 或无效，消抖后进 ESTOP + bit11；**不自动解除** |

**恢复**：倾角回限和/或 NCU 恢复后 → 中止当前运动任务 → `unfreeze` → 若仍吸附则 `raise` → 回待机。急停期间不自动恢复。

---

## 八、电调控制与遥测边界

- 左右履带和前后滚刷的控制输出仍由 `SERVOx_FUNCTION` 对应的 PWM 通道完成。
- `AP_ESC_Telem_2BLD6010` 仅发送 Modbus `0x03` 读请求，读取故障码、电流、转速、温度、电压、方向和霍尔计数，不写电调寄存器。
- 电调遥测是否健康目前不参与 `ModeVGSolar` 的停车、转弯、导航或急停决策。
- 设备原始故障码目前只进入 `BESC` 日志和诊断接口，不会自动映射为 NCU 状态帧 bit0~bit3。

参数和遥测查看方法见 [地面站参数配置.md](./地面站参数配置.md)。

---

## 九、地面站参数

数值与推荐配置统一见 [地面站参数配置.md](./地面站参数配置.md)（`VGS_*`、`SCUP_*`、`CC_*`、`BESC_*`、SERVO/Relay/SERIAL）。

---

## 十、相关源码

| 路径 | 内容 |
|------|------|
| `Rover/mode_vgsolar.cpp` | 模式主逻辑 |
| `Rover/mode.h`（ModeVGSolar） | 子模式 / 转弯相位声明 |
| `libraries/AP_CompanionComputer/` | NCU 协议栈 |
| `libraries/AP_SuctionCup/` | 吸盘 |
| `libraries/AP_Brush/` | 滚刷 |
| `libraries/AP_ESC_Telem/AP_ESC_Telem_2BLD6010.cpp` | 2BLD6010 轮询、健康状态和日志 |
| `libraries/AP_ESC_Telem/AP_ESC_Telem_2BLD6010_Protocol.cpp` | Modbus 请求、流式解析和参数校验 |

---

## 十一、文档索引

| 文档 | 内容 |
|------|------|
| [FCU_NCU通信协议.md](./FCU_NCU通信协议.md) | 帧格式、指令、状态与故障位 |
| [NCU通信日志设计.md](./NCU通信日志设计.md) | DataFlash：`NCLK`/`NSPD`/`NTRN`/`NEVT` 字段与排查用法 |
| [吸盘行为说明.md](./吸盘行为说明.md) | 吸附/释放、freeze、故障 |
| [地面站参数配置.md](./地面站参数配置.md) | Mission Planner 参数清单 |
| [STM32H743VIT6的IO资源分配.md](./STM32H743VIT6的IO资源分配.md) | 硬件引脚与通道 |
| [AP_ESC_2BLD6010_开发过程.md](./AP_ESC_2BLD6010_开发过程.md) | 电调遥测后端设计与实现记录 |
| `libraries/AP_SuctionCup/README.md` | 吸盘库实现级规格 |
