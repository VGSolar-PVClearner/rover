# AP_SuctionPressure 开发过程

## 1. 开发目标

- 传感器：MCP-H10-B100KPN-P（MCP-H10-P）3.3V 负压表压模块。
- 使用场景：VGSolar 清洁机器人吸盘负压建立判定和吸附后掉压告警。
- 板卡与车辆：VGSolar / ArduRover。
- 接口：STM32H743VIT6 PC0，ADC1，ArduPilot 模拟引脚编号 `10`。
- 架构：独立 `AP_SuctionPressure` 库，不接入 `AP_Baro`，不参与 EKF 或高度估算；仅在 VGSolar 的 Rover MAVLink 输出层复用 `SCALED_PRESSURE2` 显示吸盘压力。

## 2. 硬件与换算

- 供电：3.3V。
- 标称输出：0.1V～3.1V，数据手册最大输出约 3.13V。
- 标称量程：0kPa～-100kPa，负压使用负数表示。
- 三点关系：0.1V=0kPa，1.6V=-50kPa，3.1V=-100kPa。
- 模块引脚顺序为 GND、VDD、VOUT；原理图连接为 JP2 pin 1=GND、pin 2=3.3V、pin 3=VOUT，VOUT 接 PC0。
- 换算公式：`P_MIN + (voltage - V_MIN) * (P_MAX - P_MIN) / (V_MAX - V_MIN)`。
- R163 100Ω 和 C5 100nF 保留；R95 3.3k 上拉不安装。因此软件不能可靠区分传感器开路与某些合法或漂移电压，开路检测必须作为台架限制处理。

## 3. 板级配置

- `PC0 BP_PRESSURE_SENS ADC1 SCALE(1)`。
- `HAL_SUCTION_PRESSURE_PIN=10`。
- 移除 PC0 的 `BATT2_CURRENT_SENS` 和 `HAL_BATT2_CURR_PIN=10` 定义。
- 将 VGSolar 默认参数改为 `BATT2_CURR_PIN=-1`；启动时若检测到历史保存值仍为 10，则自动保存为 -1，避免升级后继续把 PC0 当作第二路电流输入。其他非 10 的用户配置不改动。
- PA4 主电流采样保持不变。

## 4. 参数

### 压力模块 `SPRESS_`

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `SPRESS_PIN` | VGSolar 为 10，其他板为 -1 | ADC 逻辑引脚；-1 禁用 |
| `SPRESS_V_MIN` | 0.1V | 低端标定电压，对应 0kPa |
| `SPRESS_V_MAX` | 3.1V | 高端标定电压，对应 -100kPa |
| `SPRESS_P_MIN` | 0kPa | `SPRESS_V_MIN` 对应压力 |
| `SPRESS_P_MAX` | -100kPa | `SPRESS_V_MAX` 对应压力 |
| `SPRESS_FILT_HZ` | 5Hz | 一阶低通截止频率 |

升级启动时，仅当压力模块启用且四项已保存参数完全等于旧默认值 `0.2/2.7/-100/0` 时，固件自动保存为 `0.1/3.1/0/-100`，并发送一次 `MCP-H10 pressure calibration migrated`。任一端点已由用户修改时均不覆盖；迁移后的值不再匹配旧条件，因此后续启动不会重复保存。

### 吸盘模块 `SCUP_`（驱动代码默认值）

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `SCUP_VAC_P_KPA` | -50kPa | 吸附建立阈值 |
| `SCUP_VAC_DEB_MS` | 300ms | 建立和掉压消抖时间 |
| `SCUP_VAC_HYST` | 5kPa | 掉压告警回差 |
| `SCUP_VAC_DLY_MS` | 3000ms | 禁用压力传感器时的旧固定延时；启用时为建立负压最大等待时间 |
| `SCUP_ACT_TOUT_MS` | 30000ms | 完整动作总超时 |

VGSolar 板级默认参数以实机配置为准，覆盖其中两项：`SCUP_VAC_P_KPA=-70kPa`、`SCUP_VAC_DLY_MS=10000ms`。其余参数仍使用上表的驱动代码默认值，具体值见 [地面站参数配置.md](./地面站参数配置.md)。

## 5. 数据与健康状态

`AP_SuctionPressure` 以 50Hz 调用 `voltage_average()`，不使用比率式电压。压力经 5Hz 一阶低通后供状态机、遥测和日志使用。

状态枚举：

- `DISABLED`：`SPRESS_PIN=-1`。
- `NO_SOURCE`：已配置引脚，但 HAL 未提供模拟源。
- `WARMING_UP`：已有有效样本，但连续有效时间不足 100ms。
- `HEALTHY`：连续有效约 100ms，且更新未过期。
- `INVALID_VOLTAGE`：非有限值、标定无效或明显超出电气范围连续约 200ms。
- `STALE`：超过 250ms 未更新。

公开读取接口只在 `HEALTHY` 且未过期时返回数据，业务层不会使用旧压力伪装当前值。

## 6. 吸盘状态机接入

- `SPRESS_PIN=-1` 时，`LOWER_WAIT_VACUUM` 保持原固定延时逻辑。
- 启用压力传感器后，压力必须连续 `SCUP_VAC_DEB_MS` 小于等于 `SCUP_VAC_P_KPA` 才进入 `LOWERED`。
- 传感器不健康时不回退固定延时；超过 `SCUP_VAC_DLY_MS` 后停泵、放气、进入 `FAULT` 并发送警告。
- `LOWERED` 或具有已建立负压标记的 `FROZEN` 状态下，压力连续高于 `SCUP_VAC_P_KPA + SCUP_VAC_HYST` 时仅锁存一次掉压告警和状态日志。
- 掉压告警不调用 `set_fault()`，不改变泵、阀、舵机状态，也不主动中止转弯；压力恢复到建立阈值后解除锁存。
- `ModeVGSolar` 继续通过 `AP_SuctionCup::is_lowered()` 决定是否允许开始转弯。

## 7. Mission Planner 与日志

Mission Planner `Flight Data -> Status` 中以 2Hz 发送：

- `SUCT_PKPA`：健康时为滤波压力，单位 kPa；无有效压力时为 NaN。
- `SUCT_HLT`：健康为 1，其余状态为 0。

VGSolar 还通过现有 `STREAM_RAW_SENSORS` 的 `MSG_SCALED_PRESSURE2` 发送同一滤波压力：

- Mission Planner 标准字段名为 `press_abs2`，实际单位为 hPa。
- 换算为 `press_abs_hpa = pressure_kpa * 10.0f`，因此 0kPa、-50kPa、-100kPa 分别显示为 0hPa、-500hPa、-1000hPa。
- `press_diff`、`temperature` 和 `temperature_press_diff` 固定为 0。
- 禁用、无源、预热、无效或过期时，`press_abs2` 发送 NaN，不发送上一帧有效压力。
- 发送频率继续由 `RAW_SENS` 或各链路 `SRx_RAW_SENS` 控制，测试时应设置为至少 1Hz；没有增加独立高频发送任务。

VGSolar 板载两个 BMP388 仍由 `AP_Baro` 探测和读取。第一个 BMP388 的 `SCALED_PRESSURE` 保持不变；`SCALED_PRESSURE2` 被 Rover 输出层用于吸盘压力，因此第二个 BMP388 不再通过该消息对外发送。非 VGSolar 板仍使用通用的第二气压计发送逻辑。

状态文本按事件边沿发送：传感器由健康变为不健康时提示 `Suction pressure sensor unhealthy`；负压建立、建立超时、吸附后掉压和掉压恢复分别提示 `Suction vacuum established`、`Suction vacuum timeout`、`Suction pressure loss` 和 `Suction pressure recovered`，不会在周期任务中持续刷屏。

DataFlash 以 10Hz 写入 `SPRS`：`TimeUS,Volt,Press,Health,Status`。`Volt` 为原始平均电压（V），`Press` 为滤波压力（kPa），`Health` 为健康标志，`Status` 为状态枚举值。

## 8. 验证记录

- 已完成：参数索引和命名静态检查、PC0 复用检查、AP_Baro/EKF 隔离检查、VGSolar `SCALED_PRESSURE2` 覆盖及非 VGSolar 通用回退检查、NaN 失效输出检查、MAVLink 字段长度检查、`git diff --check`。
- 编译验证：`./waf configure --board VGSolar` 和 `./waf rover` 已成功，固件输出位于 `build/VGSolar/bin/ardurover.apj`。
- 未完成：真实 MCP-H10 三点标定、约 3.13V 上限和 ADC 异常输入、开路/短路表现、压力动态响应、Mission Planner `press_abs2`/MAVLink Inspector 实际接收、泵阀吸附台架、整机转弯与掉压试验。

## 9. 已知限制

- R95 不安装后不能声明具备可靠开路检测能力。
- “明显超出范围”只能发现非有限值及标称范围外较大的电气异常，不能覆盖所有断线和偏置故障。
- 滤波、管路容积、泵性能会影响达到项目阈值 -70kPa 的时间，`SCUP_VAC_DLY_MS` 和 `SCUP_VAC_DEB_MS` 必须结合实物调整。
