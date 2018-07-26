#include "voltage_source.h"
#include "Dropbot/config_pb.h"
#include "Dropbot/state_pb.h"

namespace dropbot {
namespace voltage_source {

SlowSoftWire i2c(SSDA_PIN, SSCL_PIN);

// Configuration.
float pot_max = dropbot_Config_pot_max_default;
float R7 = dropbot_Config_R7_default;
float max_voltage = dropbot_Config_max_voltage_default;
float min_frequency = dropbot_Config_min_frequency_default;
float max_frequency = dropbot_Config_max_frequency_default;

// State.
float target_voltage = dropbot_State_voltage_default;
float frequency = dropbot_State_frequency_default;

void begin() {
  // Configure pins as outputs.
  pinMode(DRIVER_HIGH_PIN, OUTPUT);
  pinMode(DRIVER_LOW_PIN, OUTPUT);
  pinMode(SHDN_PIN, OUTPUT);
  pinMode(HV_OUTPUT_SELECT_PIN, OUTPUT);
  pinMode(SSDA_PIN, OUTPUT);
  pinMode(SSCL_PIN, OUTPUT);

  // Initialize timer1, and set a 0.05 ms period, i.e., 10 kHz frequency.
  Timer1.initialize(50);
  Timer1.stop();

  // attach timer_callback() as a timer overflow interrupt
  Timer1.attachInterrupt(timer_callback);
}

void timer_callback() {
  uint8_t high_pin_state = digitalRead(DRIVER_HIGH_PIN);
  if (high_pin_state == HIGH) {
    digitalWrite(DRIVER_HIGH_PIN, LOW);
    digitalWrite(DRIVER_LOW_PIN, HIGH);
  } else {
    digitalWrite(DRIVER_LOW_PIN, LOW);
    digitalWrite(DRIVER_HIGH_PIN, HIGH);
  }
}

}  // namespace voltage_source {
}  // namespace dropbot {
