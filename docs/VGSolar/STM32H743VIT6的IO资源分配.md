# STM32H743VIT6 / VGSolar IO 资源分配

> **软件接入说明（2026-07-24 更新）**
> 下表记录 MCU 物理引脚、定时器、PWM/GPIO 和网络标签，属于硬件资源基线。`PWM(n)`/`GPIO(n)` 是 hwdef 资源编号，`SERVOx_FUNCTION` 是 ArduPilot 逻辑输出配置，三者不能互相直接推断。串口名称均优先表示 MCU 外设，例如“串口6”表示 `USART6`，不等于 ArduPilot 参数组 `SERIAL6`。当前映射已与 `libraries/AP_HAL_ChibiOS/hwdef/VGSolar/hwdef.dat` 对照；实机线序、电平和 RS485 收发器方向方式仍须台架确认。

## 当前软件映射摘要

| 功能 | 当前推荐逻辑参数 | 软件状态 | 硬件备注 |
| --- | --- | --- | --- |
| 左履带 | `SERVO1_FUNCTION=73` | 已接入 VGSL/ModeGuided | 物理输出表仍记录为 PE13/TIM1_CH3/PWM(3)，需源码和实板确认逻辑通道映射 |
| 右履带 | `SERVO3_FUNCTION=74` | 已接入 VGSL/ModeGuided | 物理输出表仍记录为 PE14/TIM1_CH4/PWM(4) |
| 前滚刷 | `SERVO2_FUNCTION=157` | `AP_Brush` 已实现 | 物理输出 PC8/TIM8_CH3 |
| 后滚刷 | `SERVO4_FUNCTION=158` | `AP_Brush` 已实现 | 物理输出 PD15/TIM4_CH4 |
| 吸盘升降 | `SERVO5_FUNCTION=159` | `AP_SuctionCup` 已实现 | PD14/TIM4_CH3 |
| 气阀 | `RELAY1_PIN=59` 或自定义 GPIO 控制 | IO 资源已改为 GPIO | PE5/GPIO59，默认低电平关闭；`AP_SuctionCup` 需同步改为 GPIO/Relay 控制 |
| 气泵 | `RELAY2_PIN=60` 或自定义 GPIO 控制 | IO 资源已改为 GPIO | PE6/GPIO60，只支持启停，不再支持 PWM 调速；`AP_SuctionCup` 需同步改为 GPIO/Relay 控制 |
| 数传模块 | MCU `UART5`，对应 `SERIAL2_PROTOCOL=2` | 目标分配已确认 | `PC12/PD2`；波特率按数传模块实际配置 |
| 遥控器 | MCU `UART7`，对应 `SERIAL1_PROTOCOL=23` | 从 UART5/UART7 中选择 UART7 | `PE8/PE7`；UART5 已分配给数传模块，避免复用冲突 |
| 超声波 1 | MCU `UART8`，对应 `SERIAL4_PROTOCOL=9` | 使用原预留 UART8 | `PE1/PE0`；DYP-A02 时使用 `SERIAL4_BAUD=9` |
| 超声波 2 | MCU `USART2`，对应 `SERIAL5_PROTOCOL=9` | USART2 从遥控器改为超声波 | `PA2/PA3`；DYP-A02 时使用 `SERIAL5_BAUD=9` |
| RK3588 通信 | MCU `USART3`，对应 `SERIAL6_PROTOCOL=50`、`SERIAL6_BAUD=115` | `AP_CompanionComputer` 已实现 | `PB10/PB11`；`CC_PORT` 选择第 N 个协议 50 串口 |
| GPS | MCU `USART6`，对应 `SERIAL7_PROTOCOL=5` | 目标分配已确认 | `PC6/PC7`；`SERIAL7_BAUD` 按 GPS 型号确认 |
| RS485 电调遥测 | MCU `UART4`，对应 `SERIAL8_PROTOCOL=51`、`SERIAL8_BAUD=115` | `AP_ESC_Telem_2BLD6010` 已实现 | `PC10/PC11`；只读 Modbus RTU，自动方向收发器使用 `SERIAL8_OPTIONS=0` |
| CAN1 | `PD0/PD1 FDCAN1` | 恢复为预留 CAN 总线 | 本次两路超声波使用 UART8 和 USART2，不占用 CAN1 |
| 关机保持 | `PE2 MCU_K/GPIO99` | 软件关机 `0x04` 未实现 | 必须先确认高/低有效、ACK 后延时和断电保持逻辑 |
| 物理急停 | `PE3 KILL/GPIO100` | 尚未接入 VGSL | NCU 急停已实现，但不能替代物理急停验收 |
| 红外 | `PB5/GPIO98` | 尚未接入 VGSL | 用于识别舵机吸盘是否到达目标位置 |

## 引脚资源表

| STM32H743VIT6引脚分配 | base on FC | 接口数 | 外设 | 备注 | 网络标签 | Mission Planner 参数配置 | 是否改变 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **PWM** |  |  | 标注(FC)为FC上面的接口 |  |  |  |  |
| PE13**√** | TIM1_CH3 | **1** | 左电机控制信号 | 左边无刷电机 | BP_PWM_CH1 | `SERVO1_FUNCTION=73` | 否 |
| PE14**√** | TIM1_CH4 | **1** | 右电机控制信号 | 右边无刷电机 | BP_PWM_CH2 | `SERVO3_FUNCTION=74` | 否 |
| PC8**√** | TIM8_CH3 | **1** | 前滚刷 | 前无刷电机 | BP_PWM_CH3 | `SERVO2_FUNCTION=157` | 新增 |
| PD15**√** | TIM4_CH4 | **1** | 后滚刷 | 后无刷电机 | BP_PWM_CH4 | `SERVO4_FUNCTION=158` | 否 |
| PA8√ | TIM1_CH1 | **1** | 右编码器 A 相 | 右轮 | BP_PWM_CH5 | `WENC2_PINA=50` | 否 |
| PE11√ | TIM1_CH2 | **1** | 右编码器 B 相 | 右轮 | BP_PWM_CH6 | `WENC2_PINB=51` | 否 |
| PA0√ | TIM5_CH1 | **1** | 左编码器 A 相 | 左轮 | BP_PWM_CH7 | `WENC_PINA=56` | 否 |
| PA1√ | TIM5_CH2 | **1** | 左编码器 B 相 | 左轮 | BP_PWM_CH8 | `WENC_PINB=57` | 否 |
| PD14**√** | TIM4_CH3 | **1** | 舵机 | 升降吸盘 | BP_PWM_CH9 | `SERVO5_FUNCTION=159` | 否 |
| PB1 | TIM3_CH4 | **1** | 蜂鸣器 | 板级 ALARM 输出 | BP_PWM_Buzzer | `NTF_BUZZ_TYPES=<按提示类型配置>` | 否 |
|  | 小计 | **10** | **引出10个PWM** | 气阀和气泵已移出 PWM，改为 GPIO |  |  |  |
| **串口** |  |  |  |  |  |  |  |
| PC12.PD2 | UART5_TX.UART5_RX | **2** | 数传模块 | MCU `UART5` 对应 ArduPilot `SERIAL2` | BP_UART5_TX_485.BP_UART5_RX_485 | `SERIAL2_PROTOCOL=2`<br/>`SERIAL2_BAUD=<按数传配置>`<br/>`SERIAL2_OPTIONS=0` | 修改用途：取消 RS485 复用描述 |
| PB10.PB11 | USART3_TX.USART3_RX | **2** | RK3588 通信 | MCU `USART3` 对应 `SERIAL6` | BP_USART3_TX.BP_USART3_RX | `SERIAL6_PROTOCOL=50`<br/>`SERIAL6_BAUD=115`<br/>`SERIAL6_OPTIONS=0` | 用途确认 |
| PE8.PE7 | UART7_TX.UART7_RX | **2** | 遥控器 | MCU `UART7` 对应 `SERIAL1` |  | `SERIAL1_PROTOCOL=23`<br/>`SERIAL1_BAUD=115`<br/>`SERIAL1_OPTIONS=0` | 修改用途：从 UART5/UART7 中选择 UART7 |
| PB6.PB7 | USART1_TX.USART1_RX | **2** | 预留串口 | MCU `USART1` 对应 `SERIAL3` |  | `SERIAL3_PROTOCOL=-1` | 取消 GPS 用途 |
| PA2.PA3 | USART2_TX.USART2_RX | **2** | 超声波 2 | MCU `USART2` 对应 `SERIAL5` | FMU_USART2_TX_TEL3.FMU_USART2_RX_TEL3 | `SERIAL5_PROTOCOL=9`<br/>`SERIAL5_BAUD=9`<br/>`SERIAL5_OPTIONS=0`<br/>`RNGFND2_TYPE=45` | 修改用途：遥控器迁移至 UART7 |
| PC6.PC7 | USART6_TX.USART6_RX | **2** | GPS | MCU `USART6` 对应 `SERIAL7` | FMU_USART6_TX_TO_IO.FMU_USART6_RX_FROM_IO | `SERIAL7_PROTOCOL=5`<br/>`SERIAL7_BAUD=<按GPS型号>`<br/>`SERIAL7_OPTIONS=0` | 修改用途 |
| PC10.PC11 | UART4_TX.UART4_RX | **2** | RS485 电调数据 | MCU `UART4` 对应 `SERIAL8` | FMU_UART4_TX.FMU_UART4_RX | `SERIAL8_PROTOCOL=51`<br/>`SERIAL8_BAUD=115`<br/>`SERIAL8_OPTIONS=0` | 修改用途 |
| PE0.PE1 | UART8_RX.UART8_TX | **2** | 超声波 1 | MCU `UART8` 对应 `SERIAL4` | FMU_UART8_TX.FMU_UART8_RX | `SERIAL4_PROTOCOL=9`<br/>`SERIAL4_BAUD=9`<br/>`SERIAL4_OPTIONS=0`<br/>`RNGFND1_TYPE=45` | 修改用途：启用原预留 UART8 |
|  | 小计 | **16** | **共 8 个 MCU 串口、16 个 TX/RX 引脚** |  |  |  |  |
| **ADC** |  |  |  |  |  |  |  |
| PC0√ | ADC123_INP10 | **1** | 气压 | MCP-H10-P 3.3V 吸附负压模拟量，0.1V=0kPa、1.6V=-50kPa、3.1V=-100kPa | BP_PRESSURE | `SPRESS_PIN=10`；Status 查看 `press_abs2`（hPa）、`SUCT_PKPA`/`SUCT_HLT` | 否 |
| PC1√ | ADC123_INP11 | **1** | 温度 | 模拟温度输入 | BP_TEMPERATURE | 自定义温度模块使用 ADC pin `11`；暂无标准 Mission Planner 参数 | 否 |
| PA4 | ADC12_INP18 | **1** | 电流采集 | 主电池电流 | FMU_BAT1_I | `BATT_CURR_PIN=18`<br/>`BATT_AMP_PERVLT=<实测标定>`<br/>`BATT_AMP_OFFSET=<实测标定>` | 否 |
| PC4 | ADC12_INP4 | **1** | 电压采集 | 主电池电压 | FMU_BAT1_V | `BATT_VOLT_PIN=4`<br/>`BATT_VOLT_MULT=<实测标定>` | 否 |
|  | 小计 | **4** | **引出4个ADC** |  |  |  |  |
| **IO** |  |  |  |  |  |  |  |
| PA10√ | 普通IO | **1** | 雨刮 | GPIO93，默认低电平 | BP_WIPER | 暂无标准参数；自定义模块控制 `GPIO93` | 新增 |
| PB12.PB13√ | 普通IO | **2** | LED | GPIO94/GPIO95，默认低电平 | BP_LED_0.BP_LED_1 | 板级/自定义通知灯；暂无独立引脚参数 | 新增 |
| PC13√ | 普通IO | **1** | 按键 | GPIO96，下拉输入 | BP_BUTTON | `BTN_ENABLE=1`<br/>`BTN_PIN1=96`<br/>`BTN_OPTIONS1=<按有效电平>` | 新增 |
| PE15√ | 普通IO | **1** | 补光 | GPIO97，默认低电平 | BP_LIGHT | 暂无标准参数；自定义模块控制 `GPIO97` | 新增 |
| PB5√ | 普通IO | **1** | 红外 | GPIO98，下拉输入 | BP_IR_SENSOR | 暂无标准参数；自定义安全模块读取 `GPIO98` | 新增 |
| PE2√ | 普通IO | 1 | 电源开关控制引脚 | GPIO99，默认低电平 | MCU_K | 暂无标准参数；关机模块控制 `GPIO99` | 新增 |
| PE3√ | 普通IO | 1 | 急停 | GPIO100，下拉输入 | KILL | 暂无标准参数；物理急停模块读取 `GPIO100` | 新增 |
| PE5√ | 普通IO | 1 | 气阀 | 两位两通常闭型真空泄压电磁阀；GPIO59 默认低电平关闭 | BP_PWM_CH10 | 若使用 AP_Relay：`RELAY1_FUNCTION=1`<br/>`RELAY1_PIN=59`<br/>`RELAY1_DEFAULT=0`<br/>否则由自定义吸盘模块控制 `GPIO59` | 新增 |
| PE6√ | 普通IO | 1 | 气泵 | 有刷直流真空气泵；GPIO60 仅用于启停，默认低电平关闭，不再支持 PWM 调速 | BP_PWM_CH11 | 若使用 AP_Relay：`RELAY2_FUNCTION=1`<br/>`RELAY2_PIN=60`<br/>`RELAY2_DEFAULT=0`<br/>否则由自定义吸盘模块控制 `GPIO60` | 修改用途：从 PWM 改为 GPIO |
|  | 小计 | **10** | **引出10个IO** | 气阀和气泵均为 GPIO 输出 |  |  |  |
| **SPI** |  |  |  |  |  |  |  |
| PD3.PD4.PC2_C.PC3_C | SPI2_SCK.SPI2_CS1.SPI2_MISO.SPI2_MOSI | 4 | FM25V02A-GTR(FC) | 板载 FRAM |  | 板级固定，无 Mission Planner 引脚参数 | 否 |
| PA5.PA6.PA7.PE12.PB2 | SPI1_SCK.SPI1_MISO.SPI1_MOSI.ICM42688_CS.ICM42688_DRDY | 5 | ICM-42688-P(FC) | 板载 IMU |  | 板级固定；IMU 参数由 `INS_*` 参数组管理 | 否 |
|  | 小计 | 9 | **引出0** |  |  |  |  |
| **其他** |  |  |  |  |  |  |  |
| PA13.PA14 | JTMS-SWDIO.JTCK-SWCLK | **2** | SWD调试接口 | 固件下载与调试 | FMU_SWDIO.FMU_SWCLK | 板级固定，无 Mission Planner 参数 | 否 |
| NRST,BOOT0 |  | **2** | OTA/启动控制 | 复位和启动模式 | NRST,BOOT0 | 板级固定，无 Mission Planner 参数 |  |
| PD1.PD0 | FDCAN1_TX.FDCAN1_RX | **2** | CAN1 预留 | 本次两路超声波使用 UART8 和 USART2 | FUM_CAN1_TX.FUM_CAN1_RX | 未使用时保持关闭；启用设备时配置 `CAN_P1_DRIVER`、`CAN_D1_PROTOCOL` | 恢复预留 |
| PB8.PB9 | FMU_I2C1_SCL.FMU_I2C1_SDA | 2 | GPS1(FC)(磁罗盘) | 外部 I2C 罗盘 |  | `COMPASS_ENABLE=1`；具体探测/方向按罗盘型号配置 | 否 |
| PD12.PD13 | I2C4_SCL.I2C4_SDA | 2 | BMP388i/QMC5883L(FC) | 内部 I2C 气压计/罗盘 |  | 板级自动探测；校准和方向使用 `BARO*`、`COMPASS*` 参数 | 否 |
| PB14.PB15.PD6.PD7,PB3.PB4 | SDMMC2_D0.SDMMC2_D1.SDMMC2_CK.SDMMC2_CMD.SDMMC2_D2.SDMMC2_D3 | 6 | TF卡(FC) | 日志存储 |  | `LOG_BACKEND_TYPE=1`（文件日志，按固件参数支持情况确认） | 否 |
| PA11.PA12.PA9 | OTG_FS_DM.OTG_FS_DP.VBUS | 2 | USB_TYPE_C(FC) | USB/Mission Planner |  | `SERIAL0_PROTOCOL=2` | 否 |
| PD10.PD11 | 普通IO.普通IO | 2 | LED(FC) | 飞控红/蓝状态灯 |  | 板级通知灯；提示类型由 `NTF_LED_TYPES` 等参数管理 | 否 |
|  | 小计 | **18** | **引出1个SWD.1个CAN.共4个引脚** |  |  |  |  |
| 总计 | 按本表底板相关资源统计 |  | 11 个 PWM 引脚 + 16 个串口引脚 + 4 个 ADC 引脚 + 9 个普通 IO + 2 个 SWD 引脚 + 2 个 CAN 引脚 = 44 个 MCU 信号引脚；SPI/I2C/SD/USB 等 FC 板内资源另计 |  |  |  |  |

**飞控板删除了**:GPS2接口.SPI2接口.MOTOR1_I.MOTOR2_I.电流电压检查接口.debug接口,删除串口5和7的流控

与本 IO 分配对应的 `libraries/AP_HAL_ChibiOS/hwdef/VGSolar/hwdef.dat` 配置要点如下。本次只更新本文档，实际板级文件和参数尚待后续同步：

1. 删除/注释掉 UART 流控
```text
PE9  UART7_RTS UART7
PE10 UART7_CTS UART7
PC8  UART5_RTS UART5
PC9  UART5_CTS UART5
```

2. USART3 改到 `PB10/PB11`
```text
# PD8 USART3_TX USART3
# PD9 USART3_RX USART3

PB10 USART3_TX USART3
PB11 USART3_RX USART3
```

3. 注释掉冲突的 I2C2
```text
# PB10 I2C2_SCL I2C2
# PB11 I2C2_SDA I2C2
```

4. `I2C_ORDER` 去掉 `I2C2`
```text
I2C_ORDER I2C4 I2C1
```

5. 固定串口业务分配；保持 `SERIAL_ORDER` 不变
```text
SERIAL_ORDER OTG1 UART7 UART5 USART1 UART8 USART2 USART3 USART6 UART4 OTG2

# telemetry radio: MCU UART5 -> ArduPilot SERIAL2
PC12 UART5_TX UART5
PD2  UART5_RX UART5

# RC receiver: MCU UART7 -> ArduPilot SERIAL1
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# ultrasonic sensor 1: MCU UART8 -> ArduPilot SERIAL4
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# ultrasonic sensor 2: MCU USART2 -> ArduPilot SERIAL5
PA2 USART2_TX USART2
PA3 USART2_RX USART2

# RK3588 companion computer: MCU USART3 -> ArduPilot SERIAL6
PB10 USART3_TX USART3
PB11 USART3_RX USART3

# GPS: MCU USART6 -> ArduPilot SERIAL7
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# 2BLD6010 RS485 telemetry: MCU UART4 -> ArduPilot SERIAL8
PC10 UART4_TX UART4
PC11 UART4_RX UART4
```

对应的目标参数关系：

```text
SERIAL1_PROTOCOL 23   # UART7 遥控器
SERIAL2_PROTOCOL 2    # UART5 数传模块
SERIAL4_PROTOCOL 9    # UART8 超声波 1 / Rangefinder
SERIAL4_BAUD     9    # DYP-A02 为 9600 bit/s
SERIAL5_PROTOCOL 9    # USART2 超声波 2 / Rangefinder
SERIAL5_BAUD     9    # DYP-A02 为 9600 bit/s
SERIAL6_PROTOCOL 50   # USART3 RK3588
SERIAL7_PROTOCOL 5    # USART6 GPS；SERIAL7_BAUD 按 GPS 型号确认
SERIAL8_PROTOCOL 51   # UART4 2BLD6010 RS485
SERIAL8_BAUD     115
SERIAL8_OPTIONS  0
```

> 编号说明：MCU `UART7` 对应 ArduPilot `SERIAL1`，MCU `UART8` 对应 `SERIAL4`，MCU `USART2` 对应 `SERIAL5`，MCU `USART6` 对应 `SERIAL7`，MCU `UART4` 对应 `SERIAL8`。不得仅凭硬件串口名称直接填写同号的 `SERIALx_*` 参数。

> 超声波说明：两路串口超声波分别使用 `PE1/PE0 UART8`（`SERIAL4`）和 `PA2/PA3 USART2`（`SERIAL5`）。DYP-A02：`PROTOCOL=9`、`BAUD=9`、`RNGFNDx_TYPE=45`，ORIENT 见 [地面站参数配置.md](./地面站参数配置.md)。`PD0/PD1 FDCAN1` 恢复为预留 CAN 总线。

6. ADC 按资源分配改为 4 路
```text
PC4 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PA4 BATT_CURRENT_SENS ADC1 SCALE(1)

PC0 BP_PRESSURE_SENS ADC1 SCALE(1)
PC1 BP_TEMPERATURE_SENS ADC1 SCALE(1)

# PC5 BATT4_CURRENT_SENS ADC1 SCALE(1)
# define HAL_BATT4_CURR_PIN 8

# PB0 BATT5_CURRENT_SENS ADC1 SCALE(1)
# define HAL_BATT5_CURR_PIN 9
```

7. 气阀和气泵改为 GPIO 输出
```text
PC8 TIM8_CH3 PWM(9)   # front roller brush
PE5 BP_VALVE OUTPUT LOW GPIO(59) # normally-closed vacuum relief solenoid valve
PE6 BP_PUMP OUTPUT LOW GPIO(60)  # vacuum pump on/off only
```

> 气泵说明：PE6 已取消 `TIM15_CH2/PWM(11)`，改为 GPIO60。当前只允许 GPIO 高低电平启停，不再提供硬件 PWM 调速。若后续需要调速，必须重新确认硬件和软件控制路径后恢复 PWM 配置。

8. 新增 9 个普通 IO
```text
PA10 BP_WIPER
PB12 BP_LED_0
PB13 BP_LED_1
PC13 BP_BUTTON
PE15 BP_LIGHT
PB5  BP_IR_SENSOR
PE2  MCU_K
PE3  KILL
PE5  BP_VALVE OUTPUT GPIO(59) LOW
PE6  BP_PUMP OUTPUT GPIO(60) LOW
```

> 气阀和气泵说明：PE5/GPIO59 与 PE6/GPIO60 均配置为普通 GPIO，默认低电平。气阀按两位两通常闭型真空泄压电磁阀使用；气泵仅做启停控制。当前 `AP_SuctionCup` 仍需同步改写 GPIO/Relay 输出路径，不能继续依赖 `SERVO10_FUNCTION=160` 或 `SERVO11_FUNCTION=161`。

9. 给所有 PWM 行加了用途注释
包括：
```text
right encoder A/B
left motor control
right motor control
servo lift suction cup
rear roller brush
left encoder A/B
front roller brush
brushed DC vacuum pump
buzzer
```

普通 IO 用途注释中增加：
```text
normally-closed vacuum relief solenoid valve
```

10. 更新当前 ArduPilot 可识别的罗盘探测宏
```text
define AP_COMPASS_PROBING_ENABLED 1
define AP_COMPASS_IST8310_INTERNAL_BUS_PROBING_ENABLED 0
```
