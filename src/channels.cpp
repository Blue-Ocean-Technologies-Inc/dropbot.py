#include <Arduino.h>
#include <TimerOne.h>
#include <Wire.h>

#include "channels.h"
#include "voltage_source.h"


namespace dropbot {

Switch channel_to_switch(uint8_t channel) {
    /*
     *  - 5 IO register ports per switching board.
     *  - 8 bits per IO register port.
     *  - Channels in LSB-first order within each IO register port.
     */
    const auto ports_per_board = 5;
    const auto channels_per_port = 8;
    const auto channels_per_board = ports_per_board * channels_per_port;

    Switch _switch;
    _switch.board = channel / channels_per_board;
    _switch.port = (channel % channels_per_board) / channels_per_port;
    _switch.bit = channel % channels_per_port;
    return _switch;
}


uint8_t switch_to_channel(Switch const &_switch) {
  /*
   *  - 5 IO register ports per switching board.
   *  - 8 bits per IO register port.
   *  - Channels in LSB-first order within each IO register port.
   */

  // Port number within all IO register ports concatenated.
  const auto ports_per_board = 5;
  const auto channels_per_port = 8;
  const auto channels_per_board = ports_per_board * channels_per_port;

  return (_switch.board * channels_per_board + _switch.port * channels_per_port
          + _switch.bit);
}


void pack_channels(UInt8Array const &channels, UInt8Array &packed_channels) {
  const auto ports_per_board = 5;
  packed_channels.length = 0;

  for (auto i = 0; i < channels.length; i++) {
    const Switch _switch = channel_to_switch(channels.data[i]);
    const uint8_t byte_i = _switch.board * ports_per_board + _switch.port;

    if (byte_i + 1 >= packed_channels.length) {
        for (auto board_i = packed_channels.length;
             board_i < _switch.board + 1; board_i++) {
            for (auto port_j = 0; port_j < ports_per_board + 1; port_j++) {
                packed_channels.data[_switch.board * ports_per_board
                                     + port_j] = 0;
            }
        }
        packed_channels.length = (_switch.board + 1) * ports_per_board;
    }

    packed_channels.data[byte_i] |= 1 << _switch.bit;
  }
}


Channels::packed_channels_t const &Channels::state_of_channels() {
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


void Channels::_update_channels(bool force) {
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
                  (force ? std::numeric_limits<uint8_t>::max() :
                   ~disabled_channels_mask_[chip * 5 + port]));
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


std::vector<uint16_t> Channels::short_detection_voltages(uint8_t delay_ms) {
  std::vector<uint16_t> voltages(MAX_NUMBER_OF_CHANNELS);

  const auto selected_output = voltage_source::selected_output();
  // Select 3.3 V output voltage.
  voltage_source::select_output(voltage_source::OUTPUT_3V3);

  // Execute channel scan to turn on each channel, one at a time.  Force
  // actuation of channels regardless of status in disabled channels mask.
  channel_scan([&] (uint8_t channel) { voltages[channel] = analogRead(0); },
               delay_ms, true);

  // Restore originally selected voltage.
  voltage_source::select_output(selected_output);

  return voltages;
}


std::vector<uint8_t> Channels::detect_shorts(uint8_t delay_ms) {
  auto short_voltages = short_detection_voltages(delay_ms);

  constexpr uint16_t max_analog = std::numeric_limits<uint16_t>::max();
  constexpr uint16_t short_threshold = max_analog / 2;

  // Count number of detected shorts to determine required size of output
  // vector.
  uint8_t short_count = 0;
  for (auto voltage : short_voltages) {
    if (voltage < short_threshold) {
      short_count += 1;
    }
  }

  // Store list of channels corresponding to detected shorts in output vector.
  // Add shorted channels to the disabled channels mask.
  std::vector<uint8_t> shorts(short_count);
  short_count = 0;
  for (auto i = 0; i < short_voltages.size(); i++) {
    if (short_voltages[i] < short_threshold) {
      // Voltage was less than half Vcc. Append this channel to the list of
      // shorts and disable the channel.
      shorts[short_count++] = i;
      disabled_channels_mask_[i / 8] |= 1 << (i % 8);
    } else {
      // Voltage was at least half Vcc.  Mark the channel as enabled.
      disabled_channels_mask_[i / 8] &= ~(1 << (i % 8));
    }
  }

  return shorts;
}


float Channels::_benchmark_channel_update(uint32_t count) {
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


}  // namespace dropbot
