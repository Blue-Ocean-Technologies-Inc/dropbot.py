#ifndef ___DROPBOT__ANALOG__H___
#define ___DROPBOT__ANALOG__H___

#include <algorithm>
#include <vector>
#include <stdint.h>

#include "kxsort.h"

namespace dropbot {
namespace analog {

extern float high_voltage_;

std::vector<uint16_t> analog_reads_simple(uint8_t pin, uint16_t n_samples);

uint16_t u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                             float low_percentile, float high_percentile);

float high_voltage();

float measure_temperature();

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

float benchmark_analog_read(uint8_t pin, uint32_t n_samples);

float benchmmark_u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                                     float low_percentile,
                                     float high_percentile,
                                     uint32_t n_repeats);

}  // namespace analog {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__ANALOG__H___
