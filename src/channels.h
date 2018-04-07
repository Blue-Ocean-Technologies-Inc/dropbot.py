#ifndef ___DROPBOT__CHANNELS__H___
#define ___DROPBOT__CHANNELS__H___

#include <array>
#include <numeric>
#include <vector>
#include <Arduino.h>
#include <TimerOne.h>
#include <Wire.h>

#include <CArrayDefs.h>

#include "analog.h"


namespace dropbot {

constexpr uint16_t MAX_NUMBER_OF_CHANNELS = 120;


struct ChannelNeighbours {
  uint8_t up;
  uint8_t down;
  uint8_t left;
  uint8_t right;
};


struct Switch {
  uint8_t board;
  uint8_t port;
  uint8_t bit;
};


Switch channel_to_switch(uint8_t channel);
uint8_t switch_to_channel(Switch const &_switch);
void pack_channels(UInt8Array const &channels, UInt8Array &packed_channels);


// Accept iterators to support vectors, arrays, etc.
// See: https://stackoverflow.com/a/26684784/345236
template <typename Iterator>
std::vector<uint8_t> unpack_channels(Iterator begin, const Iterator end) {
  const auto ports_per_board = 5;
  const auto channels_per_port = 8;
  const auto channels_per_board = ports_per_board * channels_per_port;

  std::vector<uint8_t> channels;
  channels.reserve(MAX_NUMBER_OF_CHANNELS);

  auto i = 0;
  for (auto it = begin; it != end; i++, it++) {
    uint8_t board_i = i / ports_per_board;
    uint8_t port_i = i % ports_per_board;

    for (uint8_t j = 0; j < 8; j++) {
        if (*it & (1 << j)) {
            const Switch _switch = {board_i, port_i, j};
            uint8_t channel_ij = switch_to_channel(_switch);
            channels.push_back(channel_ij);
        }
    }
  }
  channels.shrink_to_fit();
  return channels;
}

//
// Accept iterators to support vectors, arrays, etc.
// See: https://stackoverflow.com/a/26684784/345236
template <typename Container>
std::vector<uint8_t> unpack_channels(Container packed_channels) {
  return unpack_channels(std::begin(packed_channels),
                         std::end(packed_channels));
}


// Accept iterators to support vectors, arrays, etc.
// See: https://stackoverflow.com/a/26684784/345236
template <typename Iterator>
std::vector<Switch> unpack_switches(Iterator begin,
                                           const Iterator end) {
  const auto ports_per_board = 5;
  const auto channels_per_port = 8;
  const auto channels_per_board = ports_per_board * channels_per_port;

  std::vector<Switch> switches;
  switches.reserve(MAX_NUMBER_OF_CHANNELS);

  auto i = 0;

  for (auto it = begin; it != end; it++, i++) {
    uint8_t board_i = i / ports_per_board;
    uint8_t port_i = i % ports_per_board;

    for (uint8_t j = 0; j < 8; j++) {
        if (*it & (1 << j)) {
            const Switch _switch = {board_i, port_i, j};
            switches.push_back(_switch);
        }
    }
  }
  switches.shrink_to_fit();
  return switches;
}


template <typename Container>
std::vector<Switch> unpack_switches(const Container &packed_switches) {
  return unpack_switches(std::begin(packed_switches),
                         std::end(packed_switches));
}


class Channels {
public:
  // PCA9505 (gpio) chip/register addresses
  static constexpr uint8_t PCA9505_CONFIG_IO_REGISTER = 0x18;
  static constexpr uint8_t PCA9505_OUTPUT_PORT_REGISTER = 0x08;

  uint16_t channel_count_;
  uint8_t switching_board_i2c_address_;

  typedef std::array<uint8_t, MAX_NUMBER_OF_CHANNELS / 8> packed_channels_t;
  packed_channels_t state_of_channels_;
  packed_channels_t disabled_channels_mask_;

  Channels(uint16_t channel_count, uint8_t switching_board_i2c_address)
    : channel_count_(channel_count),
      switching_board_i2c_address_(switching_board_i2c_address) {}

  const auto &state_of_channels() {
    // XXX Stop the timer (which toggles the HV square-wave driver) during i2c
    // communication.
    //
    // See https://gitlab.com/sci-bots/dropbot.py/issues/26
    Timer1.stop(); // stop the timer during i2c transmission
    for (uint8_t chip = 0; chip < channel_count_ / 40; chip++) {
      for (uint8_t port = 0; port < 5; port++) {
        Wire.beginTransmission(switching_board_i2c_address_ + chip);
        Wire.write(PCA9505_OUTPUT_PORT_REGISTER + port);
        Wire.endTransmission();

        delayMicroseconds(100); // needed when using Teensy

        Wire.requestFrom(switching_board_i2c_address_ + chip, 1);
        if (Wire.available()) {
          state_of_channels_[chip * 5 + port] = ~Wire.read();
        } else {
          Timer1.restart();
          // No response from switching board.
          std::fill(state_of_channels_.begin(), state_of_channels_.end(), 0);
          return state_of_channels_;
        }
      }
    }
    Timer1.restart();
    return state_of_channels_;
  }

  void set_state_of_channels(const packed_channels_t &channel_states) {
    state_of_channels_ = channel_states;
    _update_channels();
  }

  const auto &disabled_channels_mask() { return disabled_channels_mask_; }

  template <typename T>
  void set_disabled_channels_mask(const T &disabled_channels_mask) {
    std::copy(disabled_channels_mask.begin(), disabled_channels_mask.end(),
              disabled_channels_mask_.begin());
    _update_channels();
  }

  void _update_channels() {
    // XXX Stop the timer (which toggles the HV square-wave driver) during i2c
    // communication.
    //
    // See https://gitlab.com/sci-bots/dropbot.py/issues/26
    Timer1.stop(); // stop the timer during i2c transmission
    uint8_t data[2];
    // Each PCA9505 chip has 5 8-bit output registers for a total of 40 outputs
    // per chip. We can have up to 8 of these chips on an I2C bus, which means
    // we can control up to 320 channels.
    //   Each register represent 8 channels (i.e. the first register on the
    // first PCA9505 chip stores the state of channels 0-7, the second register
    // represents channels 8-15, etc.).
    for (uint8_t chip = 0; chip < channel_count_ / 40; chip++) {
      for (uint8_t port = 0; port < 5; port++) {
        data[0] = PCA9505_OUTPUT_PORT_REGISTER + port;
        data[1] = ~(state_of_channels_[chip * 5 + port] &
                    ~disabled_channels_mask_[chip * 5 + port]);
        Wire.beginTransmission(switching_board_i2c_address_ + chip);
        Wire.write(data, sizeof(data));
        Wire.endTransmission();
        // XXX Need the following delay if we are operating with a 400kbps
        // i2c clock.
        delayMicroseconds(200);
      }
    }
    Timer1.restart();
  }

  std::vector<Switch> actuated_switches() {
    const uint8_t port_count = channel_count_ / 8;
    std::vector<uint8_t> enabled_states(port_count);
    for (uint8_t i = 0; i < port_count; i++) {
      enabled_states[i] = (state_of_channels_[i] &
                           ~disabled_channels_mask_[i]);
    }
    return unpack_switches(enabled_states);
  }

  std::vector<uint8_t> actuated_channels() {
    auto switches = actuated_switches();
    std::vector<uint8_t> selected_channels(switches.size());
    std::transform(switches.begin(), switches.end(), selected_channels.begin(),
                   +[] (Switch _switch) { return switch_to_channel(_switch); });
    return selected_channels;
  }

  std::vector<uint8_t> detect_shorts(uint8_t delay_ms) {
    // Eight channels per port
    auto original_state_of_channels = state_of_channels_;

    std::vector<uint8_t> shorts;
    shorts.reserve(MAX_NUMBER_OF_CHANNELS);

    for (uint8_t i = 0; i < channel_count_; i++) {
      // Initialize all channels in off state
      std::fill(state_of_channels_.begin(), state_of_channels_.end(), 0);

      // Set bit to actuate channel i
      state_of_channels_[i / 8] = 1 << (i % 8);

      // Apply channel states
      _update_channels();

      // A delay (e.g., 1-5 ms) may be necessary to detect some shorts
      delay(delay_ms);

      // If we read less than half of Vcc, append this channel to the
      // list of shorts and disable the channel
      if (analogRead(0) < 65535 / 2) {
        shorts.push_back(i);
        disabled_channels_mask_[i / 8] |= 1 << (i % 8);
      } else { // enable the channel
        disabled_channels_mask_[i / 8] &= ~(1 << (i % 8));
      }
    }

    // Restore the previous channel state
    state_of_channels_ = original_state_of_channels;

    // Apply channel states
    _update_channels();

    shorts.shrink_to_fit();
    return shorts;
  }

  float capacitance(uint16_t n_samples) {
    /*
     * .. versionadded:: 1.41
     *
     * Measure device load capacitance based on the specified number of analog
     * samples.
     *
     * Amplitude of measured square wave is calculated by computing the `inter-quartile range (IQR) <https://en.wikipedia.org/wiki/Interquartile_range>`_,
     * i.e., the difference between the 75th percentile and the 25th
     * percentile.
     *
     * See: https://gitlab.com/sci-bots/dropbot.py/issues/25
     *
     * Notes
     * -----
     *
     * According to the figure below, the transfer function describes the
     * following relationship::
     *
     *     V₂   Z₂
     *     ── = ──
     *     V₁   Z₁
     *
     * where $V_{1}$ denotes the high-voltage actuation signal and $V_{2}$
     * denotes the signal sufficiently attenuated to fall within the measurable
     * input range of the analog-to-digital converter *(approx. 3.3 V)*.  The
     * feedback circuits for the control board is shown below.
     *
     * .. code-block:: none
     *
     *       V_1 @ frequency
     *           ┯
     *         ┌─┴─┐    ┌───┐
     *         │Z_1│  ┌─┤Z_2├─┐
     *         └─┬─┘  │ └───┘ │
     *           │    │  │╲   ├───⊸ V_2
     *           └────┴──│-╲__│
     *                ┌──│+╱
     *                │  │╱
     *                │
     *               ═╧═
     *
     * See `HVAC`_ in DropBot HV square wave driver and `A11` and `C16` in
     * `feedback filter`_:
     *
     *  - `C16`: 0.15 uF
     *
     * Where ``V1`` and ``V2`` are root-mean-squared voltages, and Z1 == jwC1``
     * and ``Z2 == jwC2``, ``C2 = V2 / V1 * C1``.
     *
     * Parameters
     * ----------
     * n_samples : uint16_t
     *     Number of analog samples to measure.
     *
     *     If 0, use default from :attr:`config_._`.
     *
     * Returns
     * -------
     * float
     *     Capacitance of device load in farads (F).
     *
     *
     * .. versionchanged:: 1.41
     *     If 0, use default from :attr:`config_._`.
     *
     * .. versionchanged:: 1.43
     *     Fix equation to divide by actuation voltage.
     *
     * .. _`HVAC`: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/boost-converter-boost-converter.pdf
     * .. _`feedback filter`: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/feedback-feedback.pdf
     */

    // Compute capacitance from measured square-wave RMS voltage amplitude.
    const uint16_t A11_raw = analog::u16_percentile_diff(11, n_samples, 25, 75);

    // Compute capacitance from measured square-wave RMS voltage amplitude.
    // V2 = 0.5 * (float(A11) / MAX_ANALOG) * AREF
    const float device_load_v = 0.5 * (A11_raw / float(1L << 16)) * 3.3;
    // C2 = V2 * C16 / HVAC
    const float C2 = device_load_v * 0.15e-6 / analog::high_voltage();
    return C2;
  }

  template <typename Iterator>
  std::vector<float> channel_capacitances(Iterator channels_begin,
                                          const Iterator channels_end,
                                          uint16_t n_samples) {
    auto original_state_of_channels = state_of_channels_;
    std::vector<float> capacitances;
    capacitances.reserve(MAX_NUMBER_OF_CHANNELS);

    auto i = 0;
    for (auto it = channels_begin; it != channels_end; i++, it++) {
      const uint8_t port_i = *it / 8;
      const uint8_t mask_i = 1 << (*it % 8);

      if (!(mask_i & ~disabled_channels_mask_[port_i])) {
        // Channel is disabled.
        capacitances.push_back(0);
        continue;
      }

      // Initialize all channels in off state
      std::fill(state_of_channels_.begin(), state_of_channels_.end(), 0);

      // Set bit to actuate channel i
      state_of_channels_[port_i] = mask_i;

      // Apply channel states
      _update_channels();

      capacitances.push_back(capacitance(n_samples));
    }

    // Restore the previous channel state
    state_of_channels_ = original_state_of_channels;

    // Apply channel states
    _update_channels();

    capacitances.shrink_to_fit();
    return capacitances;
  }

  template <typename T>
  std::vector<float> scatter_channels_capacitances(T channels,
                                                   uint16_t n_samples) {
    /*
    * Parameters
    * ----------
    * channels : STL container
    *     List of channels for which to measure capacitance - **MUST** be sorted.
    * n_samples : uint16_t
    *     Number of analog samples to measure.
    *
    *     If 0, use default from :attr:`config_._`.
    *
    * Returns
    * -------
    * std::vector<float>
    *     List of channel capacitances, indexed by id.  Value for all indices
    *     not in ``channels`` is zero.
    */

    // Only measure capacitance of specified channels.
    std::vector<float> capacitances(MAX_NUMBER_OF_CHANNELS, 0);
    channel_capacitances(channels.begin(), channels.end(), n_samples,
                         capacitances.begin());

    {
      /* At this point, capacitances are stored in the form:
      *
      *      [channel a, channel b, channel c, ..., channel N, 0, 0, 0, 0, 0, ..., 0]
      *
      *  where the **first N** entries are the measured capacitances for the
      *  specified list of channels and the remaining entries are 0.
      *
      *  Scatter measured capacitances to allow indexing by channel id, e.g.:
      *
      *      [<C0>, 0, 0, 0, <C4>, ..., <Cn>, ...]
      *
      *  Note that the reading for channel 0 is at index 0, the reading for
      *  channel 4 is at index 4, etc.
      */
      std::reverse_iterator<decltype(capacitances)::iterator>
        it_channel_c(capacitances.begin() + channels.size());
      for (auto it_channel = channels.rbegin();
           it_channel != channels.rend(); it_channel++, it_channel_c++) {
          auto &id = *it_channel;
          auto &channel_c = *it_channel_c;
          capacitances[id] = channel_c;
          if (&channel_c != &capacitances[id]) { channel_c = 0; }
      }
    }

    capacitances.shrink_to_fit();
    return capacitances;
  }

  template <typename Container>
  std::vector<float> channel_capacitances(Container channels,
                                          uint16_t n_samples) {
    return channel_capacitances(std::begin(channels), std::end(channels),
                                n_samples);
  }

  std::vector<uint8_t> enabled_channels() {
    decltype(disabled_channels_mask_) enabled_channels_mask;

    std::transform(disabled_channels_mask_.begin(),
                   disabled_channels_mask_.end(),
                   enabled_channels_mask.begin(),
                   +[] (uint8_t port) { return ~port; });
    return unpack_channels(enabled_channels_mask);
  }

  std::vector<float> all_channel_capacitances(uint16_t n_samples) {
    // Fill `channels` with range `(0, <channel_count_>)`.
    std::vector<uint8_t> channels(channel_count_);
    std::iota(channels.begin(), channels.end(), 0);
    return channel_capacitances(channels, n_samples);
  }

  float _benchmark_channel_update(uint32_t count) {
    /* Apply channel state to **all channels** across **all switching boards**
     * repeatedly the specified number of times.
     *
     * Parameters
     * ----------
     * count : uint32_t
     *     Number of times to repeat channels update.
     *
     * Returns
     * -------
     * float
     *     Average seconds per update.
     */
    // Back up current channel states.
    auto original_state_of_channels = state_of_channels_;

    // Initialize all channels in off state
    std::fill(state_of_channels_.begin(), state_of_channels_.end(), 0);

    uint32_t start = micros();
    for (uint32_t i = 0; i < count; i++) {
      // Apply channel states
      _update_channels();
    }
    uint32_t end = micros();

    // Restore the previous channel state
    state_of_channels_ = original_state_of_channels;

    // Apply channel states
    _update_channels();

    return (end - start) / float(count) * 1e-6;
  }
};

}  // namespace dropbot

#endif  // #ifndef ___DROPBOT__CHANNELS__H___
