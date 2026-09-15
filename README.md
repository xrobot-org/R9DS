# R9DS

## Static assembly source line

This source line uses explicit C++ constructor dependencies and ordered instance
arguments. Inspect the current primary header with `xrobot_mod_parser --path .`;
its declarations, not old manifest/config examples, define the interface.
Historical HardwareContainer/ApplicationManager examples below apply only to the
older dynamic source tags. Device/protocol descriptions remain relevant.
See the XRobot [migration guide](https://github.com/xrobot-org/XRobot/blob/dev/MIGRATION.md).
Compilation is not hardware validation; retain version-specific board evidence.


RadioLink R9DS UART SBUS receiver module for XRobot.

This module configures the SBUS UART, decodes R9DS receiver frames in a
background thread, publishes raw channel timing and normalized RC state topics,
tracks failsafe / no-signal state, and exposes a RamFS shell command for status
output.

## Required Hardware

- `sbus_uart`
- `ramfs`

## Constructor Arguments

- `data_topic_name`: default `"r9ds_data"`
- `rc_state_topic_name`: default `"rc_state"`
- `signal_timeout_ms`: default `50`
- `task_stack_depth`: default `1024`

## Published Topics

- `data_topic_name`: `R9DS::Data`, raw SBUS channel timing, signal frequency, flags, failsafe, and no-signal state
- `rc_state_topic_name`: `R9DS::State`, normalized RC channels, stick values, throttle state, and flight-mode switches

## Shell Commands

The module registers `r9ds` in `RamFS`.

- `r9ds` or `r9ds status`: print signal status, channel values, and decoded RC state

## XRobot Configuration Example

```yaml
- id: rc_receiver
  name: R9DS
  constructor_args:
    data_topic_name: "r9ds_data"
    rc_state_topic_name: "rc_state"
    signal_timeout_ms: 50
    task_stack_depth: 1024
```
