#include "voltage_source.h"
#include "Dropbot/config_pb.h"
#include "Dropbot/state_pb.h"

namespace dropbot {
namespace voltage_source {

FlexWire i2c = FlexWire(SSDA_PIN, SSCL_PIN);

// Configuration — use protobuf defaults (NOT tag numbers).
// These are overridden by EEPROM values in Node::begin().
float pot_max = 50000.0f;
float R7 = 10000.0f;
float max_voltage = 150.0f;
float min_frequency = 100.0f;
float max_frequency = 10000.0f;

// State — use protobuf defaults (NOT tag numbers).
float target_voltage = 100.0f;
float frequency = 10000.0f;



bool _set_voltage(float voltage) {
  // Calculate digipot wiper value from target voltage.
  //   pot_value = R6 / ((2 * V / 1.5) - 1) - R7
  //   wiper_value = pot_value / pot_max * 255
  // Clamp voltage to minimum 1V to avoid division by zero/negative.
  float clamped_voltage = (voltage < 1.0f) ? 1.0f : voltage;
  float pot_value = R6 / ((2.0f * clamped_voltage / 1.5f) - 1.0f) - R7;
  if (pot_value < 0.0f) pot_value = 0.0f;
  float wiper_f = pot_value / pot_max * 255.0f;
  uint8_t wiper_value = (wiper_f > 255.0f) ? 255 : (uint8_t)wiper_f;

  // Stop Timer1 to prevent ISR from interfering with FlexWire bit-bang I2C.
  Timer1.stop();
  i2c.begin();  // Reinitialize FlexWire pin states before each transaction.
  i2c.beginTransmission(DIGIPOT_ADDRESS);
  i2c.write(0);
  i2c.write(wiper_value);
  uint8_t error = i2c.endTransmission();
  Timer1.restart();

  if (error == 0) {
    target_voltage = voltage;
    return true;
  }
  return false;
}

void begin() {
  // Configure pins as outputs.
  pinMode(DRIVER_HIGH_PIN, OUTPUT);
  pinMode(DRIVER_LOW_PIN, OUTPUT);
  pinMode(SHDN_PIN, OUTPUT);
  pinMode(HV_OUTPUT_SELECT_PIN, OUTPUT);
//  pinMode(SSDA_PIN, OUTPUT);
//  pinMode(SSCL_PIN, OUTPUT);
  pinMode(OE_PIN, INPUT);

  i2c.begin();

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
