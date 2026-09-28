# R9DS

XRobot Module for the RadioLink R9DS SBUS receiver.

The constructor sets the UART to 100000 baud, 8 data bits, even parity, 2 stop
bits (`ASSERT` on failure). The module does not invert the signal; SBUS
inversion must be handled by the hardware or the UART driver. The
`r9ds_thread` thread (`HIGH` priority) reads the UART byte by byte:

- A gap of more than 2.5 ms between bytes restarts the frame. A 25-byte frame
  is valid when it starts with `0x0F` and ends with `0x00` or the next expected
  value of the `0x04, 0x14, 0x24, 0x34` sequence; otherwise the buffer shifts
  by one byte.
- The first 10 channels are converted to microseconds:
  `0.644 * (raw - 1024) + 1500`. Bit `0x08` of the flag byte is failsafe.
- Frames without failsafe count as good frames (`signal_frequency_hz` is the
  number of good frames in the last 1 s window). `no_signal` is set when no good
  frame has arrived for `signal_timeout_ms`; if the UART stays silent for that
  long, the module clears the channels, sets failsafe and publishes once.
- Every valid frame publishes both topics.

`OnMonitor()` logs a warning while `no_signal` is set.

## Published topics

`data_topic_name` (default `r9ds_data`), type `R9DS::Data`:
`channels_us` (10 channels, µs), `signal_frequency_hz`, `flags`, `fail_safe`,
`no_signal`.

`rc_state_topic_name` (default `rc_state`), type `R9DS::State`:

| Field | Meaning |
| --- | --- |
| `channels[8]` | channel µs − 1500, clamped to −500..500; channels 0..3 are forced to 0 on failsafe / no signal |
| `raw_us[8]` | channel µs (negative values clamped to 0) |
| `fail_safe`, `no_signal` | copied from `Data` |
| `roll`, `pitch`, `yaw` | channels 0, 1, 3 divided by 500, −1..1 |
| `throttle` | channel 2 mapped to 0..1 |
| `throttle_low` | channel 2 below −350 |
| `flight_mode` | from channel 4: < −300 `ATT_STAB`, < 200 `LOC_HOLD`, else `RETURN_HOME` |
| `flight_mode2` | from channel 5: < −300 `MODE0`, < 200 `MODE1`, else `MODE2`; `MODE0` when there is no signal |

## Shell command

The module adds the command `r9ds` to `ramfs` (`interval_ms` is clamped to
10..1000):

```sh
r9ds show <time_ms> <interval_ms>     # 10 channels in µs, frequency, flags, failsafe, no_signal
r9ds show_rc <time_ms> <interval_ms>  # 8 normalized channels, flight modes, failsafe, no_signal
```

## Dependencies

No other Modules; LibXR only.

## Constructor

```cpp
R9DS(LibXR::UART& uart, LibXR::RamFS& ramfs,
     const char* data_topic_name = "r9ds_data",
     const char* rc_state_topic_name = "rc_state",
     uint32_t signal_timeout_ms = 50,
     size_t task_stack_depth = 1024);
```

Dependencies:

- `uart`: the UART connected to the receiver's SBUS output.
- `ramfs`: RamFS that receives the `r9ds` command.

Configuration:

- `data_topic_name`: name of the `R9DS::Data` topic, default `r9ds_data`.
- `rc_state_topic_name`: name of the `R9DS::State` topic, default `rc_state`.
- `signal_timeout_ms`: time without a good frame before `no_signal` is set, ms,
  default 50.
- `task_stack_depth`: stack size of the receive thread, default 1024.

## Use

```sh
xrobot module add xrobot-org/R9DS
xrobot setup
xrobot instance add xrobot-org/R9DS
```

`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of
objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/R9DS
    id: r9ds_0
    args:
      - uart: usart3
      - ramfs: ramfs
      - data_topic_name: '"r9ds_data"'
      - rc_state_topic_name: '"rc_state"'
      - signal_timeout_ms: '50'
      - task_stack_depth: '1024'
```

BSP side:

```cpp
XR_REGISTER(usart3, LibXR::UART);
XR_REGISTER(ramfs, LibXR::RamFS);
```

Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/R9DS` in a BSP, prints the current
constructor.
