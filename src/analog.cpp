#include <math.h>
#include <Arduino.h>

#include "analog.h"


namespace dropbot {
namespace analog {

ADC adc_;
float high_voltage_ = 0;


std::vector<uint16_t> analog_reads_simple(uint8_t pin, uint16_t n_samples) {
  std::vector<uint16_t> analog_values(n_samples);
  std::generate(analog_values.begin(), analog_values.end(),
                [&] () { return analogRead(pin); });
  return analog_values;
}

std::vector<int16_t> differential_reads_simple(uint8_t pinP, uint8_t pinN,
                                               uint16_t n_samples) {
  std::vector<int16_t> analog_values(n_samples);
  std::generate(analog_values.begin(), analog_values.end(), [&] () {
    return adc_.adc0->analogReadDifferential(pinP, pinN);
  });

  return analog_values;
}


uint16_t u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                             float low_percentile, float high_percentile) {
  auto result = analog_reads_simple(pin, n_samples);
  std::sort(result.begin(), result.end());
  const uint16_t high_i = (int)round((high_percentile / 100.) * n_samples);
  const uint16_t low_i = (int)round((low_percentile / 100.) * n_samples);
  return result[high_i] - result[low_i];
}


uint16_t s16_percentile_diff(uint8_t pinP, uint8_t pinN, uint16_t n_samples,
                             float low_percentile, float high_percentile) {
  auto result = differential_reads_simple(pinP, pinN, n_samples);
  std::sort(result.begin(), result.end());
  const int32_t high_i = (int)round((high_percentile / 100.) * n_samples);
  const int32_t low_i = (int)round((low_percentile / 100.) * n_samples);
  // By definition, 75th percentile is **_guaranteed_** to be _at least_ as
  // great as 25th percentile, so return value is _unsigned_.
  return static_cast<uint16_t>(result[high_i] - result[low_i]);
}


/**
* @brief Measure high-side *root mean-squared (RMS)* voltage.
*
* See [`A1/HV_FB`][1] in DropBot boost converter schematic:
* [1]: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/boost-converter-boost-converter.pdf
*
* The MAX1771 boost converter generates a DC high voltage from 12V input.
* An NCP5304 half-bridge driver (D6/DRIVER_LO, D7/DRIVER_HI) drives two
* IRF740B MOSFETs (Q5, Q6) to produce an AC square wave (HVAC) from the
* DC rail.
*
*  - `R8`: 2 Mohms
*  - `R9`: 20 Kohms
*  - `AREF`: 3.3 V
*
* The DC rail voltage is measured at A1/HV_FB via a resistive divider:
*
*     HV ---[R6 2M]---+---[R8 2M]--- A1/HV_FB
*                      |
*                  [R9 20k]
*                      |
*                     GND
*
* The divider ratio is R9 / (R6 + R8 + R9) but since R6 and R8 are in
* series to the HV rail: V_FB = V_HV * R9 / (R8 + R9) ≈ V_HV * 20k / 2.02M.
* Rearranging: V_HV_pp = V_FB * R8 / R9 (simplified, R8 >> R9).
*
* For a 50% duty-cycle square wave, V_RMS = V_peak = 0.5 * V_peak-to-peak.
*
* \version 1.53  Cache most recent RMS voltage as `high_voltage_`.
*
* @return High-side RMS voltage.
*/
float high_voltage() {
  // Save/restore ADC configuration.
  adc_context([&] (auto adc_config) {
    // Configure ADC for measurement.
    auto &adc = *adc_.adc0;  // Use ADC 0.
    auto const resolution = adc.getResolution();
    adc.setReference(ADC_REFERENCE::REF_3V3);
    adc.wait_for_cal();

    // Convert ADC reading to voltage: HV_FB = (ADC / MAX_ANALOG) * AREF
    //   -> HV_FB = (float(A1) / 2^resolution) * 3.3V
    const float hv_fb = (adc.analogRead(A1) /
                         float(1L << resolution)) * 3.3;
    // Reconstruct HV peak-to-peak from divider ratio.
    // HV peak-to-peak = HV_FB * R8 / R9
    const float R8 = 2e6;
    const float R9 = 20e3;
    const float hv_peak_to_peak = hv_fb * R8 / R9;
    // For a square wave: HV_RMS (Vrms) = 0.5 * HV peak-to-peak (V_pp)
    // Cache most recent high voltage measurement.
    high_voltage_ = 0.5 * hv_peak_to_peak;
  });
  return high_voltage_;
}


float measure_temperature() {
  float voltage = 0;
  constexpr uint8_t TEMPERATURE_PIN =
    static_cast<uint8_t>(ADC_INTERNAL_SOURCE::TEMP_SENSOR);

  adc_context([&] (auto adc_config) {
    auto &adc = *adc_.adc0;  // Use ADC 0.

    auto const resolution = adc.getResolution();
    // Use internal 1.2V bandgap reference for temperature measurement.
    adc.setReference(ADC_REFERENCE::REF_1V2);
    adc.wait_for_cal();

    // Average 255 readings to reduce noise.
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 255; i++) {
        sum += analogRead(TEMPERATURE_PIN);
    }
    // Convert averaged ADC count to voltage: V = (sum/255) / 2^res * V_ref
    voltage = (float)sum / 255.0 / float(1L << resolution) * 1.2;
  });
  // K20 MCU internal temp sensor (from ADC_Module.h):
  //   V_temp = 0.719V at 25C, slope = -1.715 mV/C
  //   T = 25 + (0.719 - V_measured) / 1.715e-3
  return 25.0 + (0.719 - voltage) / 1.715e-3;
}


float measure_aref() {
  // AREF_PIN (pin 39) is connected to the internal 1.195V bandgap reference.
  // By reading this known voltage against the external AREF (3.3V nominal),
  // we can compute the actual AREF:
  //   ADC = V_bandgap / V_AREF * 2^16
  //   V_AREF = V_bandgap * 2^16 / ADC
  // With 255 averaged samples: V_AREF = 1.195 * 65535 / (sum / 255)
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 255; i++) {
      sum += analogRead(AREF_PIN);
  }
  return 1.195 * 65535.0 / (float)sum * 255.0;
}


uint16_t read_max(uint8_t pin, uint32_t n) {
  uint16_t max_value = 0;

  for (uint32_t i = 0; i < n; i++) {
    const uint16_t value = analogRead(pin);
    if (value > max_value) { max_value = value; }
  }
  return max_value;
}


uint16_t read_rms(uint8_t pin, uint32_t n) {
  float sum2 = 0;

  for (uint32_t i = 0; i < n; i++) {
    const uint16_t value = analogRead(pin);
    sum2 += value * value;
  }
  return sqrt(sum2 / n);
}


float measure_output_current(uint32_t n) {
  float voltage;
  adc_context([&] (auto adc_config) {
    // Configure ADC for measurement.
    auto &adc = *adc_.adc0;  // Use ADC 0.
    auto const resolution = adc.getResolution();
    adc.setReference(ADC_REFERENCE::REF_3V3);
    adc.wait_for_cal();

    voltage = static_cast<float>(read_max(OUTPUT_CURRENT_PIN, n)) / (1L << resolution) * 3.3;
  });
  // A2/HV7802_CS: The HV7802 (U4) high-voltage current sense IC measures
  // current flowing through the HVAC output to the switching boards.
  // R15 (51k) and R13 (5.1k) set the sense amplifier gain = 51k / 5.1k = 10.
  // R14 (1 ohm) is the sense resistor in the HV path.
  // I_out = V_adc / (gain * R_sense) = V_adc / (10 * 1)
  return (voltage / (51e3 / 5.1e3 * 1));
}


float measure_output_current_rms(uint32_t n) {
  float voltage;
  adc_context([&] (auto adc_config) {
    // Configure ADC for measurement.
    auto &adc = *adc_.adc0;  // Use ADC 0.
    auto const resolution = adc.getResolution();
    adc.setReference(ADC_REFERENCE::REF_3V3);
    adc.wait_for_cal();

    voltage = static_cast<float>(read_rms(OUTPUT_CURRENT_PIN, n)) / (1L << resolution) * 3.3;
  });
  // Same as measure_output_current: HV7802 gain = R15/R13 = 51k/5.1k = 10,
  // R14 = 1 ohm sense resistor.
  return (voltage / (51e3 / 5.1e3 * 1));
}


float measure_input_current(uint32_t n) {
  float voltage;
  adc_context([&] (auto adc_config) {
    // Configure ADC for measurement.
    auto &adc = *adc_.adc0;  // Use ADC 0.
    auto const resolution = adc.getResolution();
    adc.setReference(ADC_REFERENCE::REF_3V3);
    adc.wait_for_cal();

    voltage = static_cast<float>(read_max(INPUT_CURRENT_PIN, n)) / (1L << resolution) * 3.3;
  });
  // A3/MAX1771_CS: R5 (30 milliohm) is the MAX1771 boost converter current
  // sense shunt resistor. The voltage across it is read directly by the ADC
  // (no amplifier gain).
  // I_in = V_shunt / R5 = voltage / 0.03
  return voltage / 0.03;
}


float measure_input_current_rms(uint32_t n) {
  float voltage;
  adc_context([&] (auto adc_config) {
    // Configure ADC for measurement.
    auto &adc = *adc_.adc0;  // Use ADC 0.
    auto const resolution = adc.getResolution();
    adc.setReference(ADC_REFERENCE::REF_3V3);
    adc.wait_for_cal();

    voltage = static_cast<float>(read_rms(INPUT_CURRENT_PIN, n)) / (1L << resolution) * 3.3;
  });
  // Same as measure_input_current: R5 = 30m (0.03 ohm), no amplifier gain.
  return voltage / 0.03;
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


float benchmark_u16_percentile_diff(uint8_t pin, uint16_t n_samples,
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


float benchmark_s16_percentile_diff(uint8_t pinP, uint8_t pinN,
                                    uint16_t n_samples, float low_percentile,
                                    float high_percentile, uint32_t n_repeats) {
  const unsigned long start = micros();
  for (uint32_t i = 0; i < n_repeats; i++) {
    s16_percentile_diff(pinP, pinN, n_samples, low_percentile, high_percentile);
  }
  const unsigned long end = micros();
  return (end - start) * 1e-6;
}


ADC_Module::ADC_Config save_config(uint8_t adc_num) {
  ADC_Module::ADC_Config config = {0};
  if (adc_num == 0) {
    adc_.adc0->saveConfig(&config);
  } else if (adc_num == 1) {
    adc_.adc1->saveConfig(&config);
  }
  return config;
}


void load_config(ADC_Module::ADC_Config const &config, int8_t adc_num) {
  if (adc_num == 0) {
    adc_.adc0->setAveraging(_averaging(config));
    adc_.adc0->setConversionSpeed(_conversion_speed(config));
    adc_.adc0->setSamplingSpeed(_sampling_speed(config));
    adc_.adc0->setReference(_analog_reference(config));
  } else if (adc_num == 1) {
    adc_.adc1->setAveraging(_averaging(config));
    adc_.adc1->setConversionSpeed(_conversion_speed(config));
    adc_.adc1->setSamplingSpeed(_sampling_speed(config));
    adc_.adc1->setReference(_analog_reference(config));
  }
}

}  // namespace analog {
}  // namespace dropbot {
