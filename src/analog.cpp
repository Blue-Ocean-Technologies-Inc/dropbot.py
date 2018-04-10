#include <Arduino.h>

#include "analog.h"


namespace dropbot {
namespace analog {

std::vector<uint16_t> analog_reads_simple(uint8_t pin, uint16_t n_samples) {
  std::vector<uint16_t> analog_values(n_samples);
  std::generate(analog_values.begin(), analog_values.end(),
                [&] () { return analogRead(pin); });
  return analog_values;
}


uint16_t u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                             float low_percentile, float high_percentile) {
  /*
    * ..versionadded:: 1.41
    *
    * Measure samples from specified analog pin and compute difference between
    * specified high and low percentiles.
    *
    * For example, :data:`low_percentile` as 25 and :data:`high_percentile` as
    * 75 is equivalent to computing the `inter-quartile range <https://en.wikipedia.org/wiki/Interquartile_range>`_.
    *
    * Parameters
    * ----------
    * pin : uint8_t
    *     Analog pin number.
    * n_samples : uint16_t
    *     Number of samples to measure.
    * low_percentile : float
    *     Low percentile of range.
    * high_percentile : float
    *     High percentile of range.
    *
    * Returns
    * -------
    * uint16_t
    *     Difference between high and low percentiles.
    */
  auto result = analog_reads_simple(pin, n_samples);
  kx::radix_sort(result.begin(), result.end());
  const uint16_t high_i = (int)round((high_percentile / 100.) * n_samples);
  const uint16_t low_i = (int)round((low_percentile / 100.) * n_samples);
  return result[high_i] - result[low_i];
}


float high_voltage() {
  /* See [`A1/HV_FB`][1] in DropBot boost converter schematic:
    *
    *  - `R8`: 2 Mohms
    *  - `R9`: 20 Kohms
    *  - `AREF`: 3.3 V
    *
    * [1]: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/boost-converter-boost-converter.pdf
    */
  // HV_FB = (float(A1) / MAX_ANALOG) * AREF
  const float hv_fb = (analogRead(A1) / float(1L << 16)) * 3.3;
  // HV peak-to-peak = HV_FB * R8 / R9
  const float R8 = 2e6;
  const float R9 = 20e3;
  const float hv_peak_to_peak = hv_fb * R8 / R9;
  // HV RMS = 0.5 * HV peak-to-peak
  return 0.5 * hv_peak_to_peak;
}


/*
* Measure the temperature (in degrees Celsius) of the MCU
* via it's internal sensor.
*
* More details available [here][1] and [here][2].
*
* [1]: https://forum.pjrc.com/threads/30480-Teensy-3-1-use-internal-Temp-Sensor
* [2]: https://github.com/LAtimes2/InternalTemperature
*/
float measure_temperature() {
  analogReference(INTERNAL);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 255; i++) {
      sum += analogRead(38);
  }
  analogReference(DEFAULT);
  float voltage = (float)sum / 255.0 / 65535.0 * 1.2;
  return 25.0 + 583.0904 * (0.719 - voltage);
}


/*
 * Measure the analog reference voltage by comparing it
 * to the internal reference (1.195 V) accessible via
 * analog pin 39.
 */
float measure_aref() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 255; i++) {
      sum += analogRead(39);
  }
  return 1.195 * 65535.0 / (float)sum * 255.0;
}


float benchmark_analog_read(uint8_t pin, uint32_t n_samples) {
  std::vector<uint16_t> values(1);
  const unsigned long start = micros();
  for (uint32_t i = 0; i < n_samples; i++) {
    values[0] = analogRead(pin);
  }
  const unsigned long end = micros();
  return (end - start) * 1e-6;
}


float benchmmark_u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                                      float low_percentile,
                                      float high_percentile,
                                      uint32_t n_repeats) {
  const unsigned long start = micros();
  for (uint32_t i = 0; i < n_repeats; i++) {
    u16_percentile_diff(pin, n_samples, low_percentile, high_percentile);
  }
  const unsigned long end = micros();
  return (end - start) * 1e-6;
}

}  // namespace analog {
}  // namespace dropbot {
