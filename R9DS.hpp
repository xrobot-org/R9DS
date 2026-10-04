#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: RadioLink R9DS SBUS 接收机驱动模块 / Driver Module for the RadioLink R9DS SBUS receiver
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "libxr_def.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

/**
 * @brief RadioLink R9DS SBUS 接收机驱动，解析 SBUS 帧并发布通道数据与 RC 状态。
 *        Driver for the RadioLink R9DS SBUS receiver; parses SBUS frames and publishes
 *        the channel data and the RC state.
 */
class R9DS
{
 public:
  static constexpr size_t CHANNEL_COUNT_DEF = 10;    ///< 解析的通道数 Parsed channels
  static constexpr size_t RC_CHANNEL_COUNT_DEF = 8;  ///< RC 状态通道数 RC state channels

  /**
   * @brief 由 `channels[4]` 决定的飞行模式。
   *        Flight mode selected by `channels[4]`.
   */
  enum class FlightMode : uint8_t
  {
    ATT_STAB = 0,     ///< `channels[4]` < -300
    LOC_HOLD = 1,     ///< -300 <= `channels[4]` < 200
    RETURN_HOME = 2,  ///< `channels[4]` >= 200
  };

  /**
   * @brief 由 `channels[5]` 决定的辅助模式，无信号时为 MODE0。
   *        Auxiliary mode selected by `channels[5]`; MODE0 when there is no signal.
   */
  enum class FlightMode2 : uint8_t
  {
    MODE0 = 0,  ///< `channels[5]` < -300
    MODE1 = 1,  ///< -300 <= `channels[5]` < 200
    MODE2 = 2,  ///< `channels[5]` >= 200
  };

  /**
   * @brief 解析出的通道数据，由数据 Topic 发布。
   *        Parsed channel data published on the data Topic.
   */
  struct Data
  {
    std::array<int16_t, CHANNEL_COUNT_DEF> channels_us = {};  ///< 通道值，µs
    ///< Channels in µs
    uint16_t signal_frequency_hz = 0;  ///< 最近 1 s 的有效帧数
    ///< Good frames in the last 1 s
    uint8_t flags = 0;      ///< SBUS 标志字节 SBUS flag byte
    bool fail_safe = true;  ///< failsafe 标志 Failsafe flag
    bool no_signal = true;  ///< 超时无有效帧 No good frame within the timeout
  };

  /**
   * @brief 由通道数据换算的 RC 状态，由 RC 状态 Topic 发布。
   *        RC state derived from the channel data, published on the RC state Topic.
   */
  struct State
  {
    /// 通道值减 1500，-500 到 500
    /// Channels minus 1500, -500 to 500
    std::array<int16_t, RC_CHANNEL_COUNT_DEF> channels = {};
    std::array<uint16_t, RC_CHANNEL_COUNT_DEF> raw_us = {};  ///< 通道值，µs
    ///< Channels in µs
    bool fail_safe = true;     ///< failsafe 标志 Failsafe flag
    bool no_signal = true;     ///< 超时无有效帧 No good frame within the timeout
    bool throttle_low = true;  ///< 油门通道低于 -350 Throttle channel below -350
    FlightMode flight_mode = FlightMode::ATT_STAB;  ///< 飞行模式 Flight mode
    FlightMode2 flight_mode2 = FlightMode2::MODE0;  ///< 辅助模式 Auxiliary mode
    float roll = 0.0f;                              ///< 横滚，-1 到 1 Roll, -1 to 1
    float pitch = 0.0f;                             ///< 俯仰，-1 到 1 Pitch, -1 to 1
    float yaw = 0.0f;                               ///< 偏航，-1 到 1 Yaw, -1 to 1
    float throttle = 0.0f;                          ///< 油门，0 到 1 Throttle, 0 to 1
  };

  /**
   * @brief 构造 R9DS：把 UART 设为 SBUS 参数，注册 `r9ds` 命令并创建接收线程。
   *        Construct R9DS: set the UART to the SBUS parameters, register the `r9ds`
   *        command and create the receive thread.
   *
   * @param uart 连接接收机 SBUS 输出的 UART。
   *             UART connected to the SBUS output of the receiver.
   * @param ramfs 接收 `r9ds` 命令的 RamFS。
   *              RamFS that receives the `r9ds` command.
   * @param data_topic_name 通道数据 Topic 名称。
   *                        Name of the channel data Topic.
   * @param rc_state_topic_name RC 状态 Topic 名称。
   *                            Name of the RC state Topic.
   * @param signal_timeout_ms 没有有效帧多久后置位 no_signal，单位 ms。
   *                          Time without a good frame before no_signal is set, in ms.
   * @param task_stack_depth 接收线程栈深。
   *                         Stack depth of the receive thread.
   */
  R9DS(
      LibXR::UART& uart,
      LibXR::RamFS& ramfs,
      const char* data_topic_name = "r9ds_data",
      const char* rc_state_topic_name = "rc_state",
      uint32_t signal_timeout_ms = 50,
      size_t task_stack_depth = 1024)
      : signal_timeout_ms_(signal_timeout_ms),
        data_topic_(LibXR::Topic::CreateTopic<Data>(data_topic_name)),
        rc_topic_(LibXR::Topic::CreateTopic<State>(rc_state_topic_name, nullptr, true)),
        uart_(std::addressof(uart)),
        cmd_file_(LibXR::RamFS::CreateFile("r9ds", CommandFunc, this))
  {
    ramfs.Add(cmd_file_);

    auto ans = uart_->SetConfig({.baudrate = 100000,
                                 .parity = LibXR::UART::Parity::EVEN,
                                 .data_bits = 8,
                                 .stop_bits = 2});
    ASSERT(ans == LibXR::ErrorCode::OK);

    thread_.Create(this, ThreadFunc, "r9ds_thread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  /**
   * @brief 监控回调：no_signal 置位期间输出告警。
   *        Monitor callback: log a warning while no_signal is set.
   */
  void OnMonitor()
  {
    if (data_.no_signal)
    {
      XR_LOG_WARN("R9DS: No valid frame.");
    }
  }

 private:
  static constexpr std::array<uint8_t, 4> FRAME_END = {0x04, 0x14, 0x24, 0x34};
  static constexpr size_t CH_ROL = 0;
  static constexpr size_t CH_PIT = 1;
  static constexpr size_t CH_THR = 2;
  static constexpr size_t CH_YAW = 3;
  static constexpr size_t AUX1 = 4;
  static constexpr size_t AUX2 = 5;
  static constexpr int16_t STICK_TRIGGER_DEF = 300;
  static constexpr int16_t CALI_STICK_TRIGGER_DEF = 350;

  template <typename T>
  static T Clamp(const T value, const T min_value, const T max_value)
  {
    return std::clamp(value, min_value, max_value);
  }

  static int16_t NormalizeRcChannel(const int16_t channel_us)
  {
    return static_cast<int16_t>(
        Clamp<int32_t>(static_cast<int32_t>(channel_us) - 1500, -500, 500));
  }

  static float NormalizeStick(const int16_t channel)
  {
    return Clamp(static_cast<float>(channel) / 500.0f, -1.0f, 1.0f);
  }

  bool ParseByte(uint8_t byte)
  {
    const uint32_t now_us = static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
    if ((now_us - last_byte_time_us_) > 2500U)
    {
      frame_count_ = 0;
    }
    last_byte_time_us_ = now_us;

    frame_buffer_[frame_count_++] = byte;

    if (frame_count_ < frame_buffer_.size())
    {
      return false;
    }

    frame_count_ = frame_buffer_.size() - 1;

    const bool valid_frame =
        frame_buffer_[0] == 0x0F &&
        (frame_buffer_[24] == 0x00 || frame_buffer_[24] == FRAME_END[frame_end_index_]);

    if (!valid_frame)
    {
      for (size_t i = 0; i < frame_buffer_.size() - 1; ++i)
      {
        frame_buffer_[i] = frame_buffer_[i + 1];
      }
      return false;
    }

    DecodeFrame();
    frame_count_ = 0;
    frame_end_index_ = (frame_end_index_ + 1U) % FRAME_END.size();
    return true;
  }

  void DecodeFrame()
  {
    uint16_t raw[16] = {};
    raw[0] = (static_cast<uint16_t>(frame_buffer_[2] & 0x07) << 8) | frame_buffer_[1];
    raw[1] =
        (static_cast<uint16_t>(frame_buffer_[3] & 0x3F) << 5) | (frame_buffer_[2] >> 3);
    raw[2] = (static_cast<uint16_t>(frame_buffer_[5] & 0x01) << 10) |
             (static_cast<uint16_t>(frame_buffer_[4]) << 2) | (frame_buffer_[3] >> 6);
    raw[3] =
        (static_cast<uint16_t>(frame_buffer_[6] & 0x0F) << 7) | (frame_buffer_[5] >> 1);
    raw[4] =
        (static_cast<uint16_t>(frame_buffer_[7] & 0x7F) << 4) | (frame_buffer_[6] >> 4);
    raw[5] = (static_cast<uint16_t>(frame_buffer_[9] & 0x03) << 9) |
             (static_cast<uint16_t>(frame_buffer_[8]) << 1) | (frame_buffer_[7] >> 7);
    raw[6] =
        (static_cast<uint16_t>(frame_buffer_[10] & 0x1F) << 6) | (frame_buffer_[9] >> 2);
    raw[7] = (static_cast<uint16_t>(frame_buffer_[11]) << 3) | (frame_buffer_[10] >> 5);
    raw[8] = (static_cast<uint16_t>(frame_buffer_[13] & 0x07) << 8) | frame_buffer_[12];
    raw[9] =
        (static_cast<uint16_t>(frame_buffer_[14] & 0x3F) << 5) | (frame_buffer_[13] >> 3);
    raw[10] = (static_cast<uint16_t>(frame_buffer_[16] & 0x01) << 10) |
              (static_cast<uint16_t>(frame_buffer_[15]) << 2) | (frame_buffer_[14] >> 6);
    raw[11] =
        (static_cast<uint16_t>(frame_buffer_[17] & 0x0F) << 7) | (frame_buffer_[16] >> 1);
    raw[12] =
        (static_cast<uint16_t>(frame_buffer_[18] & 0x7F) << 4) | (frame_buffer_[17] >> 4);
    raw[13] = (static_cast<uint16_t>(frame_buffer_[20] & 0x03) << 9) |
              (static_cast<uint16_t>(frame_buffer_[19]) << 1) | (frame_buffer_[18] >> 7);
    raw[14] =
        (static_cast<uint16_t>(frame_buffer_[21] & 0x1F) << 6) | (frame_buffer_[20] >> 2);
    raw[15] = (static_cast<uint16_t>(frame_buffer_[22]) << 3) | (frame_buffer_[21] >> 5);

    data_.flags = frame_buffer_[23];
    data_.fail_safe = (data_.flags & 0x08U) != 0U;
    for (size_t i = 0; i < data_.channels_us.size(); ++i)
    {
      data_.channels_us[i] =
          static_cast<int16_t>(0.644f * (static_cast<float>(raw[i]) - 1024.0f) + 1500.0f);
    }

    const uint32_t now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if (!data_.fail_safe)
    {
      last_good_frame_ms_ = now_ms;
      data_.no_signal = false;
      frame_counter_window_++;
    }

    if (now_ms - freq_window_start_ms_ >= 1000U)
    {
      data_.signal_frequency_hz = frame_counter_window_;
      frame_counter_window_ = 0;
      freq_window_start_ms_ = now_ms;
    }

    if (data_.fail_safe && (now_ms - last_good_frame_ms_ >= signal_timeout_ms_))
    {
      data_.no_signal = true;
    }

    UpdateRcState();
    data_topic_.Publish(data_);
    rc_topic_.Publish(rc_state_);
  }

  void HandleSignalTimeout()
  {
    const uint32_t now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if ((now_ms - last_good_frame_ms_) < signal_timeout_ms_)
    {
      return;
    }

    if (!data_.no_signal)
    {
      data_.no_signal = true;
      data_.fail_safe = true;
      data_.signal_frequency_hz = 0;
      data_.flags = 0;
      data_.channels_us.fill(0);
      UpdateRcState();
      data_topic_.Publish(data_);
      rc_topic_.Publish(rc_state_);
    }
  }

  void UpdateModes()
  {
    if (rc_state_.channels[AUX1] < -STICK_TRIGGER_DEF)
    {
      rc_state_.flight_mode = FlightMode::ATT_STAB;
    }
    else if (rc_state_.channels[AUX1] < 200)
    {
      rc_state_.flight_mode = FlightMode::LOC_HOLD;
    }
    else
    {
      rc_state_.flight_mode = FlightMode::RETURN_HOME;
    }

    if (rc_state_.no_signal)
    {
      rc_state_.flight_mode2 = FlightMode2::MODE0;
      return;
    }

    if (rc_state_.channels[AUX2] < -STICK_TRIGGER_DEF)
    {
      rc_state_.flight_mode2 = FlightMode2::MODE0;
    }
    else if (rc_state_.channels[AUX2] < 200)
    {
      rc_state_.flight_mode2 = FlightMode2::MODE1;
    }
    else
    {
      rc_state_.flight_mode2 = FlightMode2::MODE2;
    }
  }

  void UpdateRcState()
  {
    rc_state_.fail_safe = data_.fail_safe;
    rc_state_.no_signal = data_.no_signal;

    for (size_t i = 0; i < RC_CHANNEL_COUNT_DEF; ++i)
    {
      const uint16_t raw_us =
          static_cast<uint16_t>(std::max<int16_t>(0, data_.channels_us[i]));
      rc_state_.raw_us[i] = raw_us;
      rc_state_.channels[i] = NormalizeRcChannel(data_.channels_us[i]);
    }

    if (rc_state_.fail_safe || rc_state_.no_signal)
    {
      rc_state_.channels[CH_ROL] = 0;
      rc_state_.channels[CH_PIT] = 0;
      rc_state_.channels[CH_THR] = 0;
      rc_state_.channels[CH_YAW] = 0;
    }

    rc_state_.throttle_low = rc_state_.channels[CH_THR] < -CALI_STICK_TRIGGER_DEF;
    rc_state_.roll = NormalizeStick(rc_state_.channels[CH_ROL]);
    rc_state_.pitch = NormalizeStick(rc_state_.channels[CH_PIT]);
    rc_state_.yaw = NormalizeStick(rc_state_.channels[CH_YAW]);
    rc_state_.throttle =
        Clamp(static_cast<float>(rc_state_.channels[CH_THR] + 500) / 1000.0f, 0.0f, 1.0f);

    UpdateModes();
  }

  static void ThreadFunc(R9DS* r9ds)
  {
    LibXR::Semaphore read_sem;
    uint8_t byte = 0;
    r9ds->freq_window_start_ms_ =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    r9ds->last_good_frame_ms_ = r9ds->freq_window_start_ms_;

    while (true)
    {
      LibXR::ReadOperation op(read_sem, 10);
      auto ans = r9ds->uart_->Read({&byte, 1}, op);
      if (ans == LibXR::ErrorCode::OK)
      {
        (void)r9ds->ParseByte(byte);
      }
      else
      {
        r9ds->HandleSignalTimeout();
      }
    }
  }

  static int CommandFunc(R9DS* r9ds, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show [time_ms] [interval_ms] - Print the first 10 R9DS channels.\r\n">();
      LibXR::STDIO::Printf<
          "  show_rc [time_ms] [interval_ms] - Print processed RC state.\r\n">();
      return 0;
    }

    if (argc == 4 && std::strcmp(argv[1], "show") == 0)
    {
      int time_ms = std::atoi(argv[2]);
      int interval_ms = std::atoi(argv[3]);
      interval_ms = std::clamp(interval_ms, 10, 1000);

      while (time_ms > 0)
      {
        LibXR::STDIO::Printf<
            "R9DS: %d %d %d %d %d %d %d %d %d %d | freq=%u | flags=0x%02X | fs=%d | "
            "no_signal=%d\r\n">(r9ds->data_.channels_us[0], r9ds->data_.channels_us[1],
                                r9ds->data_.channels_us[2], r9ds->data_.channels_us[3],
                                r9ds->data_.channels_us[4], r9ds->data_.channels_us[5],
                                r9ds->data_.channels_us[6], r9ds->data_.channels_us[7],
                                r9ds->data_.channels_us[8], r9ds->data_.channels_us[9],
                                r9ds->data_.signal_frequency_hz, r9ds->data_.flags,
                                static_cast<int>(r9ds->data_.fail_safe),
                                static_cast<int>(r9ds->data_.no_signal));
        LibXR::Thread::Sleep(interval_ms);
        time_ms -= interval_ms;
      }
      return 0;
    }

    if (argc == 4 && std::strcmp(argv[1], "show_rc") == 0)
    {
      int time_ms = std::atoi(argv[2]);
      int interval_ms = std::atoi(argv[3]);
      interval_ms = std::clamp(interval_ms, 10, 1000);

      while (time_ms > 0)
      {
        LibXR::STDIO::Printf<
            "RC: ch=%d %d %d %d %d %d %d %d | mode=%d/%d | fs=%d | ns=%d\r\n">(
            r9ds->rc_state_.channels[0], r9ds->rc_state_.channels[1],
            r9ds->rc_state_.channels[2], r9ds->rc_state_.channels[3],
            r9ds->rc_state_.channels[4], r9ds->rc_state_.channels[5],
            r9ds->rc_state_.channels[6], r9ds->rc_state_.channels[7],
            static_cast<int>(r9ds->rc_state_.flight_mode),
            static_cast<int>(r9ds->rc_state_.flight_mode2),
            static_cast<int>(r9ds->rc_state_.fail_safe),
            static_cast<int>(r9ds->rc_state_.no_signal));
        LibXR::Thread::Sleep(interval_ms);
        time_ms -= interval_ms;
      }
      return 0;
    }

    LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
    return -1;
  }

  uint32_t signal_timeout_ms_ = 50;
  uint32_t last_byte_time_us_ = 0;
  uint32_t last_good_frame_ms_ = 0;
  uint32_t freq_window_start_ms_ = 0;
  uint16_t frame_counter_window_ = 0;
  size_t frame_count_ = 0;
  size_t frame_end_index_ = 0;
  std::array<uint8_t, 25> frame_buffer_ = {};
  Data data_;
  State rc_state_;
  LibXR::Topic data_topic_;
  LibXR::Topic rc_topic_;
  LibXR::UART* uart_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
