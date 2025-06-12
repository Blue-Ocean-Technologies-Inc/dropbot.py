#ifndef ___DROPBOT__VOLTAGE_SOURCE__H___
#define ___DROPBOT__VOLTAGE_SOURCE__H___

#include <stdint.h>
#include <Arduino.h>
#include <SlowSoftWire.h>
#include <TimerOne.h>

namespace dropbot {
namespace voltage_source {

extern SlowSoftWire i2c;

// Configuration.
extern float pot_max;
extern float R7;
extern float max_voltage;
extern float min_frequency;
extern float max_frequency;

// State
extern float target_voltage;
extern float frequency;

constexpr float R6 = 2e6;

#if defined(__MK20DX256__) // Teensy 3.1/3.2
constexpr uint8_t OUTPUT_3V3 = 0;
constexpr uint8_t OUTPUT_HIGH_VOLTAGE = 1;

// High-voltage Output Enable pin
constexpr uint8_t OE_PIN = 22;

// pins connected to the boost converter
constexpr uint8_t SHDN_PIN = 10;
constexpr uint8_t SSDA_PIN = 4;
constexpr uint8_t SSCL_PIN = 5;
constexpr uint8_t DRIVER_HIGH_PIN = 6;
constexpr uint8_t DRIVER_LOW_PIN = 7;
constexpr uint8_t HV_OUTPUT_SELECT_PIN = 8;

#elif defined(__IMXRT1062__) // Teensy 4.0
// TODO: Define pins for Teensy 4.0
#else
#error "Unknown board"
#endif

void begin();
void timer_callback();

inline float min_waveform_voltage() {
  return 1.5 / 2.0 * (R6 / (pot_max + R7) + 1);
}

/**
 * @brief Set output high voltage.
 *
 * @param voltage  Target RMS voltage, absolute minimum of 1 V (regardless of
 *     `pot_max`).
 *
 * @return  `true` if voltage was set, otherwise `false`.
 *
 *
 * \version 1.73.3  Clamp out-of-range values for `pot_value` and `wiper_value`
 *     (fixes [issue 44][i44]).
 *
 * [i44]: https://gitlab.com/sci-bots/dropbot.py/issues/44
 */
inline bool _set_voltage(float voltage) {
  const float pot_value = max(0., R6 / ((2 * max(1, voltage) / 1.5) - 1) - R7);
  const uint8_t wiper_value = min(pot_value / pot_max * 255, 255.);
  if (voltage <= max_voltage) {
    // This method is triggered whenever a voltage is included in a state
    // update.
    i2c.beginTransmission(44);
    i2c.write(0);
    i2c.write(wiper_value);
    i2c.endTransmission();

    // verify that we wrote the correct value to the pot
    i2c.requestFrom(44, 1);
    if (i2c.read() == wiper_value) {
      target_voltage = voltage;
      return true;
    }
  }
  return false;
}

inline void enable_high_voltage_output() {
  // If we're turning the output on, we need to start with a low voltage,
  // enable the MAX1771, then increase the voltage. Otherwise, if the voltage
  // is > ~100 the MAX1771 will not turn on.
  const auto target_voltage_ = target_voltage;
  _set_voltage(min_waveform_voltage());
  target_voltage = target_voltage_;
  delay(100);
  digitalWrite(SHDN_PIN, false);
  delay(100);
  _set_voltage(target_voltage_);
  Timer1.setPeriod(500e3 / frequency); // set timer period in ms
  Timer1.restart();
}

inline void disable_high_voltage_output() {
  digitalWrite(SHDN_PIN, true);
  Timer1.stop(); // stop timer
}

inline bool high_voltage_output_enabled() {
  return !digitalRead(SHDN_PIN) && !digitalRead(OE_PIN);
}

inline uint8_t selected_output() {
  return digitalRead(HV_OUTPUT_SELECT_PIN) ? OUTPUT_3V3 : OUTPUT_HIGH_VOLTAGE;
}

inline bool select_output(uint8_t output) {
  switch (output) {
    case OUTPUT_3V3:
      digitalWrite(HV_OUTPUT_SELECT_PIN, true);
      return true;
    case OUTPUT_HIGH_VOLTAGE:
      digitalWrite(HV_OUTPUT_SELECT_PIN, false);
      if (high_voltage_output_enabled()) {
        // If high voltage output is selected (as opposed to low short
        // detection voltage), and high voltage output is enabled (i.e., the
        // hardware interlock `OE_PIN` is pulled LOW and the boost converter is
        // on), the high voltage should be connected to the switching boards.
        // However, this currently does not happen.
        //
        // XXX As a workaround, force the high voltage to be turned off and on,
        // to ensure high voltage is actually connected to the switching
        // boards.
        //
        // See [issue #23][i23] for more information.
        //
        // [i23]: https://gitlab.com/sci-bots/dropbot.py/issues/23
        disable_high_voltage_output();
        enable_high_voltage_output();
      }
      return true;
    default:
      return false;
  }
}

inline bool set_frequency(float frequency_) {
  /* This method is triggered whenever a frequency is included in a state
    * update. */
  if (frequency_ == 0) { // DC mode
    digitalWrite(DRIVER_HIGH_PIN, HIGH); // set voltage high
    digitalWrite(DRIVER_LOW_PIN, LOW);
    Timer1.stop(); // stop timer
  } else if ((min_frequency <= frequency_) && (frequency_ <= max_frequency)) {
    Timer1.setPeriod(500e3 / frequency_); // set timer period in ms
    Timer1.restart();
  } else {
    return false;
  }
  frequency = frequency_;
  return true;
}

}}  // namespace voltage_source {

#endif  // #ifndef ___DROPBOT__VOLTAGE_SOURCE__H___
