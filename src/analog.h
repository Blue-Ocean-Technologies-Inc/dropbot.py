#ifndef ___DROPBOT__ANALOG__H___
#define ___DROPBOT__ANALOG__H___

#include <algorithm>
#include <vector>
#include <stdint.h>

#include <ADC.h>

#include "kxsort.h"

namespace dropbot {
namespace analog {

extern ADC adc_;

constexpr uint8_t PIN_CHIP_LOAD_VOLTAGE = 11;

extern float high_voltage_;

/**
 * @brief Read the specified number of samples from the specified analog pin.
 *
 * @param pin  Analog pin.
 * @param n_samples  Number of analog samples.
 *
 * @return Analog samples read from the specified pin.
 */
std::vector<uint16_t> analog_reads_simple(uint8_t pin, uint16_t n_samples);

/**
 * @brief Measure samples from specified analog pin and compute difference
 * between specified high and low percentiles.
 *
 * For example, `low_percentile` as 25 and `high_percentile` as 75 is
 * equivalent to computing the [inter-quartile range][IQR].
 *
 * [IQR]: https://en.wikipedia.org/wiki/Interquartile_range
 *
 * @since **1.41**: Add function.
 *
 * @param pin  Analog pin number.
 * @param n_samples  Number of samples to measure.
 * @param low_percentile  Low percentile of range, in \f$[0, 100]\f$.
 * @param high_percentile  High percentile of range, in \f$[0, 100]\f$.
 *
 * @return Difference between high and low percentiles.
 */
uint16_t u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                             float low_percentile, float high_percentile);

/**
 * @brief Measure differential samples between specified analog pins and
 * compute difference between specified high and low percentiles.
 *
 * @param pinP  Positive pin number.
 * @param pinN  Negative pin number.
 * @param n_samples  Number of samples to measure.
 * @param low_percentile  Low percentile of range, in \f$[0, 100]\f$.
 * @param high_percentile  High percentile of range, in \f$[0, 100]\f$.
 *
 * @return Difference between high and low percentiles.
 */
uint16_t s16_percentile_diff(uint8_t pinP, uint8_t pinN, uint16_t n_samples,
                             float low_percentile, float high_percentile);

/**
 * @brief See [`A1/HV_FB`][1] in DropBot boost converter schematic:
 *
 *  - `R8`: 2 Mohms
 *  - `R9`: 20 Kohms
 *  - `AREF`: 3.3 V
 *
 * [1]: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/boost-converter-boost-converter.pdf
 *
 *
 * @return RMS voltage of generated *high voltage* square wave signal.
 */
float high_voltage();

/*
*/

/**
 * @brief Measure the temperature (in degrees Celsius) of the MCU via its
 * internal sensor.
 *
 * More details available [here][1] and [here][2].
 *
 * [1]: https://forum.pjrc.com/threads/30480-Teensy-3-1-use-internal-Temp-Sensor
 * [2]: https://github.com/LAtimes2/InternalTemperature
 *
 * @return
 */
float measure_temperature();

/**
 * @brief Measure the analog reference voltage by comparing it to the internal
 * reference (1.195 V) accessible via analog pin 39.
 *
 * @return  Analog reference voltage.
 */
float measure_aref();

/**
 * @brief Measure analog pin repeatedly and return maximum reading.
 *
 * @param pin  Analog pin to read.
 * @param n  Number of samples to read.
 *
 * @return  Maximum value read.
 */
uint16_t read_max(uint8_t pin, uint32_t n);

/**
 * @brief Measure high-side output current.
 *
 * @param n  Number of samples to read.
 *
 * @return  Maximum current reading.
 */
float measure_output_current(uint32_t n=1);
float measure_output_current_rms(uint32_t n=10);

/**
 * @brief Measure input current.
 *
 * @param n  Number of samples to read.
 *
 * @return  Maximum current reading.
 */
float measure_input_current(uint32_t n=1);
float measure_input_current_rms(uint32_t n=10);

/**
 * @brief Measure the time required to read the specified number of samples
 * from an analog pin.
 *
 * @param pin  Analog pin to read from.
 * @param n_samples  Number of samples to read.
 *
 * @return Time (in seconds) to measure specified samples.
 */
float benchmark_analog_read(uint8_t pin, uint32_t n_samples);

/**
 * @brief Measure the time required to compute the difference between two
 * percentiles in a collection of samples from the specified analog pin.
 *
 * @param pin  Analog pin to read from.
 * @param n_samples  Number of samples to read.
 * @param low_percentile  Low percentile (e.g., `25.`)
 * @param high_percentile  High percentile (e.g., `75.`)
 * @param n_repeats  Number of times to repeat process.
 *
 * @return Time (in seconds) to repeatedly measure analog samples on the
 * specified pin and compute the percentile difference.
 */
float benchmark_u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                                    float low_percentile,
                                    float high_percentile,
                                    uint32_t n_repeats);

/**
 * @brief Measure the time required to compute the difference between two
 * percentiles in a collection of samples of the voltage differential between
 * the specified analog pins.
 *
 * @param pinP  Positive pin number.
 * @param pinN  Negative pin number.
 * @param n_samples  Number of samples to read.
 * @param low_percentile  Low percentile (e.g., `25.`)
 * @param high_percentile  High percentile (e.g., `75.`)
 * @param n_repeats  Number of times to repeat process.
 *
 * @return Time (in seconds) to repeatedly measure analog samples between the
 * specified pins and compute the percentile difference.
 */
float benchmark_s16_percentile_diff(uint8_t pinP, uint8_t pinN,
                                    uint16_t n_samples, float low_percentile,
                                    float high_percentile, uint32_t n_repeats);

template <typename Config>
ADC_REFERENCE _analog_reference(Config const &adc_config) {
  switch (adc_config.savedSC2 & 0x03) {
    case 1: return ADC_REFERENCE::REF_1V2;
    default: return ADC_REFERENCE::REF_3V3;
  }
}

template <typename Config>
ADC_SAMPLING_SPEED _sampling_speed(Config const &adc_config) {
  if (!(adc_config.savedCFG1 & (1 << 7))) { // ADLSMP is bit 7
    return ADC_SAMPLING_SPEED::VERY_HIGH_SPEED;
  } else if ((adc_config.savedCFG2 & 0x3) == 0x3) { // ADLSTS bits 1:0 = 11
    return ADC_SAMPLING_SPEED::HIGH_SPEED;
  } else if ((adc_config.savedCFG2 & 0x3) == 0x2) { // ADLSTS bits 1:0 = 10
    return ADC_SAMPLING_SPEED::MED_SPEED;
  } else if ((adc_config.savedCFG2 & 0x3) == 0x1) { // ADLSTS bits 1:0 = 01
    return ADC_SAMPLING_SPEED::LOW_SPEED;
  } else if ((adc_config.savedCFG2 & 0x3) == 0x0) { // ADLSTS bits 1:0 = 00
    return ADC_SAMPLING_SPEED::VERY_LOW_SPEED;
  }
  return ADC_SAMPLING_SPEED::VERY_LOW_SPEED;
}


template <typename Config>
ADC_CONVERSION_SPEED _conversion_speed(Config const &adc_config) {
  auto speed = adc_config.savedCFG1 & 0b1100011;

  if (adc_config.savedCFG2 & (1 << 7)) { // ADC_CFG2_ADHSC is bit 7
    // Either HI_SPEED_16_BITS, HI_SPEED, or VERY_HIGH_SPEED
    if (speed == 0b1100001) { // VERY_HIGH_SPEED
      return ADC_CONVERSION_SPEED::VERY_HIGH_SPEED;
    } else if (speed == 0b1000001) { // HI_SPEED
      return ADC_CONVERSION_SPEED::HIGH_SPEED;
    } else { // HI_SPEED_16_BITS
      return ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS;
    }
  } else {
    // Either MED_SPEED, LOW_SPEED, or VERY_LOW_SPEED
    if (speed == 0b0100001) { // MED_SPEED
      return ADC_CONVERSION_SPEED::MED_SPEED;
    } else if (speed == 0b0000001) { // LOW_SPEED
      return ADC_CONVERSION_SPEED::LOW_SPEED;
    } else { // VERY_LOW_SPEED
      return ADC_CONVERSION_SPEED::VERY_LOW_SPEED;
    }
  }
}

template <typename Config>
uint8_t _averaging(Config const &adc_config) {
  if (!(adc_config.savedSC3 & ADC_SC3_AVGE)) {
    return 0;
  } else {
    uint8_t avgs = adc_config.savedSC3 & 0x03; // bits 1:0
    return 1 << (avgs + 2); // 2^2 = 4, 2^3 = 8, etc.
  }
}


/**
* @brief Serialize ADC configuration registers.
*
* \See load_config()
*
* @param adc_num  Zero-based ADC index.
*
* @return  ADC configuration containing the contents of the following 32-bit
*   registers (in order): `SC1A`, `SC2`, `SC3`, `CFG1`, `CFG2`.
*/
ADC_Module::ADC_Config save_config(uint8_t adc_num);


/**
* @brief Apply a serialized configuration to ADC registers.
*
* \See save_config()
*
* **Note: individual mutator functions (e.g., `setAveraging`, etc.) are
* called to make sure other internal state of `ADC_Module` is kept up to
* date.**  In contrast, if `ADC_Module::loadConfig()` was used directly, the
* `ADC_Module` internal state would become stale if the ADC register values
* changed.
*
* @param adc_num  Zero-based ADC index.
* @param config  Serialized ADC configuration containing the contents of the
*   following 32-bit registers (in order): `SC1A`, `SC2`, `SC3`, `CFG1`,
*   `CFG2`.
*/
void load_config(ADC_Module::ADC_Config const &config, int8_t adc_num);


/**
* @brief Wrapper function to save/restore ADC state.
*
* @param func  Function (e.g., lambda) to call while in safe I2C state.
*
* Example
* -------
*
* ```c++
* adc_context([&] () {
* });
* ```
*/
template <typename Func>
void adc_context(Func func) {
  auto const adc_config = save_config(0);
  func(adc_config);
  load_config(adc_config, 0);
}

}  // namespace analog {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__ANALOG__H___
