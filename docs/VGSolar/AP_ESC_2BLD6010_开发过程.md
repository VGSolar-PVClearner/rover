# AP_ESC_Telem_2BLD6010 开发过程

## 1. 开发目标

| 项目 | 内容 |
| --- | --- |
| 设备 | 2BLD6010 无刷电机控制器 |
| 类型 | 多实例 ESC 遥测后端 |
| 使用场景 | VGSolar 左右履带及滚刷电调运行状态监测 |
| 接入车辆/板卡 | Rover / VGSolar（STM32H743） |
| 接口 | UART4 + RS485，映射为 ArduPilot `SERIAL8` |
| 协议 | Modbus RTU，只读保持寄存器 |
| 上层输出 | 标准 `AP_ESC_Telem`、MAVLink ESC 遥测、DataFlash `ESC`/`BESC` 日志、诊断 API |

本驱动只读取遥测，不通过 RS485 控制电机。履带和滚刷的启停、方向与功率仍由 `SERVOx_FUNCTION` 对应的 PWM 输出完成。

## 2. 需求和协议确认

### 2.1 串口与轮询

- 串口协议号：`SerialProtocol_ESC_2BLD6010 = 51`。
- VGSolar 端口：MCU `UART4`，PC10/TX、PC11/RX，对应 `SERIAL8`。
- 默认波特率：115200；实际使用 `SERIAL8_BAUD`，查不到有效配置时回退到 115200。
- 支持数量：1~4 台，按 `BESC_ADDR1~4` 的参数槽位顺序轮询。
- Rover 调度频率：100Hz；每台目标频率由 `BESC_RATE` 设置，默认 10Hz。
- 单次 Modbus 响应超时：固定 40ms；遥测健康超时由 `BESC_TIMEOUT` 设置，默认 500ms。

当前代码没有控制独立 DE/RE GPIO，硬件应使用自动方向 RS485 收发器；此时建议 `SERIAL8_OPTIONS=0`。

### 2.2 Modbus 请求

驱动固定发送 8 字节读请求：

| 字段 | 值 |
| --- | --- |
| 从站地址 | `BESC_ADDRn` |
| 功能码 | `0x03`（读保持寄存器） |
| 起始寄存器 | `0x0020` |
| 寄存器数量 | `8`（`0x0020~0x0027`） |
| 校验 | Modbus CRC16，低字节先发送 |

驱动不会构造 `0x06`、`0x10` 等写寄存器请求。

### 2.3 响应寄存器

正常响应为 21 字节，数据区包含 8 个大端 16 位寄存器：

| 寄存器 | 数据 | 换算后单位 |
| --- | --- | --- |
| `0x0020` | 故障码 | 原始 `uint16` |
| `0x0021` | 电流 | 原始值 × 0.01 A |
| `0x0022` | 转速 | RPM |
| `0x0023` | 温度 | 有符号 °C，发布时转换为 0.01°C |
| `0x0024` | 电压 | 原始值 × 0.1 V |
| `0x0025` | 方向 | 0/1 |
| `0x0026` | 霍尔计数高 16 位 | 与下一寄存器组成 `uint32` |
| `0x0027` | 霍尔计数低 16 位 | 与上一寄存器组成 `uint32` |

解析时校验地址、功能码、字节数、CRC、温度范围和方向值。Modbus 异常响应 `0x83` 按 5 字节帧独立识别和计数。

## 3. 参考实现与子系统选择

- 采用 `AP_ESC_Telem_Backend`，不新建独立于 ArduPilot ESC 遥测框架的全局数据通道。
- 温度、电压、电流通过 `update_telem_data()` 发布，RPM 通过 `update_rpm()` 发布。
- 标准 MAVLink 输出继续由 `AP_ESC_Telem::send_esc_telemetry_mavlink()` 负责。
- 标准 DataFlash `ESC` 日志由 `AP_ESC_Telem` 负责；本后端另写 `BESC` 通信诊断日志。

## 4. 接入设计

| 文件 | 作用 |
| --- | --- |
| `libraries/AP_ESC_Telem/AP_ESC_Telem_2BLD6010.h` | 参数、状态、诊断结构和公开 API |
| `libraries/AP_ESC_Telem/AP_ESC_Telem_2BLD6010.cpp` | 初始化、轮询状态机、健康判断、发布和日志 |
| `libraries/AP_ESC_Telem/AP_ESC_Telem_2BLD6010_Protocol.cpp` | 请求构造、帧解析、流重同步和参数校验 |
| `libraries/AP_ESC_Telem/AP_ESC_Telem_config.h` | `AP_ESC_TELEM_2BLD6010_ENABLED` 编译开关 |
| `libraries/AP_SerialManager/AP_SerialManager.h` | 串口协议号 51 |
| `Rover/Parameters.h` / `Rover/Parameters.cpp` | `BESC_` 参数对象和参数子组 61 |
| `Rover/system.cpp` | 启动阶段调用 `init()` |
| `Rover/Rover.cpp` | 100Hz 调用 `update()` |
| `libraries/AP_HAL_ChibiOS/hwdef/VGSolar/hwdef.dat` | UART4/PC10/PC11 板级映射 |

编译开关只在存在 ESC 遥测框架、非 AP_Periph 且 Flash 大于 1MiB 时默认启用。运行时仍由 `BESC_ENABLE` 决定是否打开串口和开始轮询。

## 5. 参数

| 参数 | 默认 | 范围 | 说明 |
| --- | ---: | ---: | --- |
| `BESC_ENABLE` | 0 | 0/1 | 启用后须重启 |
| `BESC_NUM` | 1 | 1~4 | 参与轮询的设备数量 |
| `BESC_ADDR1` | 2 | 1~247 | 第 1 个槽位地址 |
| `BESC_ADDR2` | 1 | 1~247 | 第 2 个槽位地址 |
| `BESC_ADDR3` | 3 | 1~247 | 第 3 个槽位地址 |
| `BESC_ADDR4` | 4 | 1~247 | 第 4 个槽位地址 |
| `BESC_OFS` | 0 | 0~31 | 标准 ESC 实例起始索引 |
| `BESC_RATE` | 10Hz | 1~20Hz | 每台目标轮询频率 |
| `BESC_TIMEOUT` | 500ms | 100~5000ms | 最近合法遥测的最大允许年龄 |

全部启用参数先校验后一次性应用。地址重复、地址越界、实例范围越界或频率/超时非法时不会部分启动，并发送一次 `2BLD6010: invalid config (N)`。

## 6. 运行流程

1. `init()` 清空旧状态并读取 `BESC_*`。
2. 全部参数验证通过后，固定 `ADDRn -> BESC_OFS+n-1` 的 ESC 实例映射。
3. 查找第 0 个 protocol 51 串口，打开 UART 并丢弃启动前残留数据。
4. `update()` 在轮询期限到达时向当前地址发送请求。
5. 非阻塞读取串口并追加到 64 字节固定缓存。
6. 流解析器处理半帧、粘包、迟到帧、噪声和错误地址，并尝试重新同步。
7. 只有完整、地址匹配、CRC 正确且字段范围有效的帧才更新遥测时间戳。
8. 成功或失败后切换到下一个参数槽位，保持固定 round-robin 顺序。

某一实例离线只增加自己的超时和不健康状态，不会覆盖其他实例的数据或改变其他实例的地址映射。

## 7. 健康状态、日志和接口

- `healthy(instance)`：至少收到一帧合法数据，且最后更新时间未超过 `BESC_TIMEOUT`。
- `has_fault(instance)`：实例健康且设备故障码非零。
- `get_diagnostics(instance, out)`：复制最近一次合法数据和通信计数；该接口本身不要求数据当前仍健康，调用方应同时检查 `healthy()`。
- `configured_count()`：返回验证并应用后的实例数量。
- `config_error()`：返回最近一次初始化配置错误。

`BESC` 日志字段：

```text
TimeUS,Inst,Addr,Healthy,Dir,Fault,Hall,Success,CRCErr,Timeout,BadResp,Except
```

正常通信日志限频为 10Hz；首次不健康和健康状态变化会立即补写一条。标准 `ESC` 日志继续记录 RPM、电压、电流和温度。

## 8. 验证记录

- 已完成 VGSolar Rover 编译：`./waf configure --board VGSolar && ./waf rover`。
- 编译启用了 `-Werror`，未出现电调后端相关错误或警告。
- 已通过最终 ELF 符号检查确认 `init()`、`update()` 和 `parse_response()` 链接进固件。
- 已静态确认 UART4、protocol 51、`BESC_` 参数注册、Rover 初始化和 100Hz 调度接入一致。
- 尚未完成真实 2BLD6010 的多地址总线、CRC 干扰、断线恢复和长时间运行测试。

## 9. 已知限制和后续工作

- 当前依赖自动方向 RS485 收发器，不支持独立 DE/RE GPIO 控制。
- 单次响应超时固定为 40ms，不能通过参数调整。
- 驱动为只读遥测，不负责电机控制。
- 电调故障和遥测掉线不会自动触发 VGSolar 停车、滚刷关闭或 NCU fault bit。
- `BESC_ADDR1~4` 默认顺序不代表实机地址已配置正确，上板前必须逐台核对。
