#ifndef ___DROPBOT__ANALOG__H___
#define ___DROPBOT__ANALOG__H___

#include <algorithm>
#include <vector>
#include <stdint.h>

#include "kxsort.h"

namespace dropbot {
namespace analog {

std::vector<uint16_t> analog_reads_simple(uint8_t pin, uint16_t n_samples);

uint16_t u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                             float low_percentile, float high_percentile);

float high_voltage();

float measure_temperature();

float measure_aref();

float benchmark_analog_read(uint8_t pin, uint32_t n_samples);

float benchmmark_u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                                     float low_percentile,
                                     float high_percentile,
                                     uint32_t n_repeats);

}  // namespace analog {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__ANALOG__H___
