# R9DS

RadioLink R9DS SBUS 接收机驱动模块 / Driver module for the RadioLink R9DS SBUS receiver

## 1. 模块作用 / Purpose

构造时，R9DS 把 UART 设为 100000 波特、8 数据位、偶校验、2 停止位（失败时 `ASSERT`），并创建线程 `r9ds_thread`（`HIGH` 优先级，栈深 `task_stack_depth`）。线程逐字节读取 UART：

- 相邻字节间隔超过 2.5 ms 时重新开始一帧。25 字节的帧以 `0x0F` 开头，并以 `0x00` 或 `0x04, 0x14, 0x24, 0x34` 序列中下一个期望值结尾时有效；否则缓冲区移位一个字节。
- 前 10 个通道换算为微秒：`0.644 * (raw - 1024) + 1500`。标志字节的 `0x08` 位为 failsafe。
- 无 failsafe 的帧计为有效帧，`signal_frequency_hz` 为最近 1 s 窗口内的有效帧数。连续 `signal_timeout_ms` 没有有效帧时置位 `no_signal`；UART 在该时间内无数据时，模块清零通道、置位 failsafe 并发布一次。
- 每个有效帧同时发布两个 Topic。

`OnMonitor()` 在 `no_signal` 置位期间输出告警。

Upon construction, R9DS sets the UART to 100000 baud, 8 data bits, even parity, 2 stop bits (`ASSERT` on failure) and creates the thread `r9ds_thread` (`HIGH` priority, stack depth `task_stack_depth`). The thread reads the UART byte by byte:

- A gap of more than 2.5 ms between bytes restarts the frame. A 25-byte frame is valid when it starts with `0x0F` and ends with `0x00` or the next expected value of the `0x04, 0x14, 0x24, 0x34` sequence; otherwise the buffer shifts by one byte.
- The first 10 channels are converted to microseconds: `0.644 * (raw - 1024) + 1500`. Bit `0x08` of the flag byte is failsafe.
- Frames without failsafe count as good frames, and `signal_frequency_hz` is the number of good frames in the last 1 s window. `no_signal` is set when no good frame has arrived for `signal_timeout_ms`; if the UART stays silent for that long, the module clears the channels, sets failsafe and publishes once.
- Every valid frame publishes both Topics.

`OnMonitor()` logs a warning while `no_signal` is set.

## 2. RC 状态换算 / RC State Conversion

`R9DS::State` 由 `R9DS::Data` 换算得到：

| 字段 | 含义 |
| --- | --- |
| `channels[8]` | 通道微秒值减 1500，限制在 -500 到 500；failsafe 或 `no_signal` 时通道 0 到 3 置 0 |
| `raw_us[8]` | 通道微秒值，负值按 0 处理 |
| `fail_safe`、`no_signal` | 取自 `Data` |
| `roll`、`pitch`、`yaw` | 通道 0、1、3 除以 500，范围 -1 到 1 |
| `throttle` | 通道 2 映射到 0 到 1 |
| `throttle_low` | 通道 2 小于 -350 |
| `flight_mode` | 取自通道 4：小于 -300 为 `ATT_STAB`，小于 200 为 `LOC_HOLD`，否则为 `RETURN_HOME` |
| `flight_mode2` | 取自通道 5：小于 -300 为 `MODE0`，小于 200 为 `MODE1`，否则为 `MODE2`；`no_signal` 时为 `MODE0` |

`R9DS::State` is derived from `R9DS::Data`:

| Field | Meaning |
| --- | --- |
| `channels[8]` | channel in µs minus 1500, clamped to -500 to 500; channels 0 to 3 are set to 0 on failsafe or `no_signal` |
| `raw_us[8]` | channel in µs, negative values treated as 0 |
| `fail_safe`, `no_signal` | copied from `Data` |
| `roll`, `pitch`, `yaw` | channels 0, 1, 3 divided by 500, in the range -1 to 1 |
| `throttle` | channel 2 mapped to 0 to 1 |
| `throttle_low` | channel 2 below -350 |
| `flight_mode` | from channel 4: below -300 `ATT_STAB`, below 200 `LOC_HOLD`, otherwise `RETURN_HOME` |
| `flight_mode2` | from channel 5: below -300 `MODE0`, below 200 `MODE1`, otherwise `MODE2`; `MODE0` on `no_signal` |

## 3. Shell 命令 / Shell Command

模块向 `ramfs` 添加命令 `r9ds`，`interval_ms` 限制在 10 到 1000：

```sh
r9ds show <time_ms> <interval_ms>     # 10 个通道微秒值、频率、标志、failsafe、no_signal / 10 channels in µs, frequency, flags, failsafe, no_signal
r9ds show_rc <time_ms> <interval_ms>  # 8 个归一化通道、飞行模式、failsafe、no_signal / 8 normalized channels, flight modes, failsafe, no_signal
```

The module adds the command `r9ds` to `ramfs`; `interval_ms` is clamped to 10 to 1000.

## 4. 构造接口 / Constructor

```cpp
R9DS(LibXR::UART& uart, LibXR::RamFS& ramfs,
     const char* data_topic_name = "r9ds_data",
     const char* rc_state_topic_name = "rc_state",
     uint32_t signal_timeout_ms = 50,
     size_t task_stack_depth = 1024);
```

依赖：

- `uart`：连接接收机 SBUS 输出的 UART。
- `ramfs`：接收 `r9ds` 命令的 RamFS。

配置参数：

- `data_topic_name`：`R9DS::Data` Topic 的名称，默认 `r9ds_data`。
- `rc_state_topic_name`：`R9DS::State` Topic 的名称，默认 `rc_state`。
- `signal_timeout_ms`：没有有效帧多久后置位 `no_signal`，单位 ms，默认 50。
- `task_stack_depth`：接收线程栈深，默认 1024。

Dependencies:

- `uart`: the UART connected to the SBUS output of the receiver.
- `ramfs`: the RamFS that receives the `r9ds` command.

Configuration parameters:

- `data_topic_name`: name of the `R9DS::Data` Topic, default `r9ds_data`.
- `rc_state_topic_name`: name of the `R9DS::State` Topic, default `rc_state`.
- `signal_timeout_ms`: time without a good frame before `no_signal` is set, in ms, default 50.
- `task_stack_depth`: stack depth of the receive thread, default 1024.

## 5. Topic

| Topic（默认名称） | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `data_topic_name`（默认 `r9ds_data`） | 发布 | `R9DS::Data` | `channels_us`（10 个通道，µs）、`signal_frequency_hz`、`flags`、`fail_safe`、`no_signal` |
| `rc_state_topic_name`（默认 `rc_state`） | 发布 | `R9DS::State` | 由 `Data` 换算的 RC 状态，字段见第 2 节 |

| Topic (default name) | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `data_topic_name` (default `r9ds_data`) | Publish | `R9DS::Data` | `channels_us` (10 channels, µs), `signal_frequency_hz`, `flags`, `fail_safe`, `no_signal` |
| `rc_state_topic_name` (default `rc_state`) | Publish | `R9DS::State` | RC state derived from `Data`, fields in section 2 |

## 6. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/R9DS` 写入的实例，`uart` 与 `ramfs` 填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/R9DS`, with `uart` and `ramfs` set to names registered by the BSP's `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/R9DS
    id: r9ds
    args:
      - uart: usart1
      - ramfs: ramfs
      - data_topic_name: "r9ds_data"
      - rc_state_topic_name: "rc_state"
      - signal_timeout_ms: 50
      - task_stack_depth: 1024
```

## 7. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一台 RadioLink R9DS 接收机，SBUS 输出经硬件或 UART 驱动反相后接入 UART。

Dependencies: LibXR.

Hardware: one RadioLink R9DS receiver whose SBUS output reaches the UART with the signal inverted by the hardware or the UART driver.
