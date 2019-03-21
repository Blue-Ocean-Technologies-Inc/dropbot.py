#ifndef ___DROPBOT__CHANNELS__H___
#define ___DROPBOT__CHANNELS__H___

#include <array>
#include <numeric>
#include <vector>

#include <CArrayDefs.h>

#include "analog.h"


namespace dropbot {

namespace channels {

    /**
     * @brief Capacitance of `C16` capacitor in chip load feedback circuit.
     *
     * This value is used in the calculation of chip load impedance.
     *
     * See [schematic diagram][1].
     *
     * \since 2.4.0
     *
     * [1]: https://trello-attachments.s3.amazonaws.com/5c4b58289f679337b5b09afb/5c67076e0426676533c989ba/551ff5aef10f5d16b0db6ab3eb0574c9/image.png
     */
    extern float C16;

}

constexpr uint16_t MAX_NUMBER_OF_CHANNELS = 120;


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

  packed_channels_t const &state_of_channels();

  void set_state_of_channels(const packed_channels_t &channel_states) {
    state_of_channels_ = channel_states;
    _update_channels();
  }

  /**
   * @brief Turn off all channels.
   */
  void turn_off_all_channels() {
    std::fill(state_of_channels_.begin(), state_of_channels_.end(), 0);
    _update_channels();
  }

  /**
   * @brief Disable all channels.
   */
  void disable_all_channels() {
    std::fill(disabled_channels_mask_.begin(), disabled_channels_mask_.end(),
              0xff);
    _update_channels();
  }

  const auto &disabled_channels_mask() { return disabled_channels_mask_; }

  template <typename T>
  void set_disabled_channels_mask(const T &disabled_channels_mask) {
    std::copy(disabled_channels_mask.begin(), disabled_channels_mask.end(),
              disabled_channels_mask_.begin());
    _update_channels();
  }

  /**
   * @brief Push configured state of channels to switching boards.
   *
   * @param force  If `true`, ignore the disabled channels mask, i.e., force
   *   channels selected for actuation to turn on _even if they have been marked
   *   as disabled in the `disabled_channels_mask_`_.
   *
   * \version 1.64  add \p force to optionally override disabled channels
   *   mask.
   */
  void _update_channels(bool force=false);

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

  /**
   * @brief Scan through all channels, actuating each channel one at a time.
   *
   * @param callback  Callback function to call for each channel.
   * @param delay_ms  Delay (in milliseconds) after each actuation before
   *     calling callback.
   * @param force  If `true`, ignore the disabled channels mask, i.e., force
   *     actuation of channels _even if they have been marked as disabled in
   *     the `disabled_channels_mask_`_.
   *
   * \since 1.64
   */
  template <typename T>
  void channel_scan(T callback, uint8_t delay_ms, bool force=false) {
    // Eight channels per port
    auto original_state_of_channels = state_of_channels_;

    for (uint8_t i = 0; i < channel_count_; i++) {
      // Initialize all channels in off state
      std::fill(state_of_channels_.begin(), state_of_channels_.end(), 0);

      // Set bit to actuate channel i
      state_of_channels_[i / 8] = 1 << (i % 8);

      // Apply channel states
      _update_channels(force);

      // A delay (e.g., 1-5 ms) may be necessary to detect some shorts
      delay(delay_ms);

      callback(i);
    }

    // Restore the previous channel state
    state_of_channels_ = original_state_of_channels;

    // Apply channel states
    _update_channels();
  }

  /**
  * @brief Measure short detection voltage for each channel.
  *
  * Apply actuation voltage V) to each channel in isolation.  Record measured
  * voltage on `A0`.  If voltage is ~3.3 V, the corresponding channel is not
  * shorted to ground.
  *
  * @param delay_ms  Amount of time to wait after sending updated channel
  *     states to switching boards before measuring applied voltage.
  *
  * @return Array of measured voltages, one per channel.
  *
  * \since 1.64
  */
  std::vector<uint16_t> short_detection_voltages(uint8_t delay_ms);

  /**
   * @brief For each channel in isolation, apply 3.3 V and test if channel is
   *    shorted to ground.
   *
   * @param delay_ms  Amount of time to wait after sending updated channel
   *     states to switching boards before measuring test voltage.
   *
   * @return  List of indexes of shorted channels.
   *
   * \version 1.64
   *  - Test all channels, even those marked as disabled.
   *  - Select 3.3 V output source for duration of test.
   */
  std::vector<uint8_t> detect_shorts(uint8_t delay_ms);

  /**
   * @brief Measure device load capacitance based on the specified number of
   * analog samples.
   *
   * \version added: 1.41
   *
   * \version changed: 1.43  Fix equation to divide by actuation voltage.
   *
   * \version changed: 1.69  Measure using differential mode and 1.2 V
   *   reference voltage.
   *
   *
   * Amplitude of measured square wave is calculated by computing the
   * [inter-quartile range (IQR)][IQR], i.e., the difference between the 75th
   * percentile and the 25th percentile.
   *
   * [IQR]: https://en.wikipedia.org/wiki/Interquartile_range
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
   * where \f$V_{1}\f$ denotes the high-voltage actuation signal and
   * \f$V_{2}\f$ denotes the signal sufficiently attenuated to fall within the
   * measurable input range of the analog-to-digital converter *(approx. 1.2
   * V)*.  The feedback circuits for the control board is shown below.
   *
   * ```
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
   * ```
   *
   * See [HVAC][HVAC] in DropBot HV square wave driver and `A11` and `C16` in
   * [feedback filter][feedback filter]:
   *
   *  - `C16`: 0.15 uF
   *
   * Where ``V1`` and ``V2`` are root-mean-squared voltages, and Z1 == jwC1``
   * and ``Z2 == jwC2``, ``C2 = V2 / V1 * C1``.
   *
   *
   * [HVAC]: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/boost-converter-boost-converter.pdf
   * [feedback filter]: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/feedback-feedback.pdf
   *
   * @param n_samples  Number of analog samples to measure. If 0, use default
   *   from :attr:`config_._`.
   *
   * @return  Capacitance of device load in farads (F).
   */
  float capacitance(uint16_t n_samples) {
    // Compute capacitance from measured square-wave RMS voltage amplitude.
    float device_load_v;

    analog::adc_context([&] (auto adc_config) {
      // Configure ADC for measurement.
      auto &adc = *analog::adc_.adc[0];  // Use ADC 0.
      auto const resolution = adc.getResolution();
      const int16_t max_analog =
        ((resolution == 16) ? (1L << 15) : (1L << resolution)) - 1;
      adc.setReference(ADC_REFERENCE::REF_1V2);
      adc.wait_for_cal();

      uint16_t A11_raw = analog::s16_percentile_diff(A10, A11, n_samples, 25,
                                                     75);
      // Compute capacitance from measured square-wave RMS voltage amplitude.
      // V2 = 0.5 * (float(A11) / ANALOG_RANGE) * AREF
      device_load_v = 0.5 * (A11_raw / (2 * float(max_analog))) * 1.2;
    });
    // C2 = V2 * C16 / HVAC
    const float C2 = device_load_v * channels::C16 / analog::high_voltage();
    return C2;
  }

  template <typename InIterator, typename OutIterator>
  void channel_capacitances(InIterator channels_begin,
                            const InIterator channels_end, uint16_t n_samples,
                            OutIterator capacitances_begin) {
    auto original_state_of_channels = state_of_channels_;

    auto it_channel = channels_begin;
    auto it_capacitance = capacitances_begin;

    for (; it_channel != channels_end; it_channel++, it_capacitance++) {
      const uint8_t port_i = *it_channel / 8;
      const uint8_t mask_i = 1 << (*it_channel % 8);
      auto &capacitance_i = *it_capacitance;

      if (!(mask_i & ~disabled_channels_mask_[port_i])) {
        // Channel is disabled.
        capacitance_i = 0;
        continue;
      }

      // Initialize all channels in off state
      std::fill(state_of_channels_.begin(), state_of_channels_.end(), 0);

      // Set bit to actuate channel i
      state_of_channels_[port_i] = mask_i;

      // Apply channel states
      _update_channels();

      capacitance_i = capacitance(n_samples);
    }

    // Restore the previous channel state
    state_of_channels_ = original_state_of_channels;

    // Apply channel states
    _update_channels();
  }

  template <typename Iterator>
  std::vector<float> channel_capacitances(Iterator channels_begin,
                                          const Iterator channels_end,
                                          uint16_t n_samples) {
    std::vector<float> capacitances(channels_end - channels_begin);
    channel_capacitances(channels_begin, channels_end, n_samples,
                         capacitances.begin());
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

  float _benchmark_channel_update(uint32_t count);

  float _benchmark_capacitance(uint16_t n_samples, uint32_t count);
};

}  // namespace dropbot

#endif  // #ifndef ___DROPBOT__CHANNELS__H___
