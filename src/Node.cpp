#include "Node.h"

namespace dropbot {

const float Node::R6 = 2e6;
SlowSoftWire Node::i2c = SlowSoftWire(Node::SSDA_PIN, Node::SSCL_PIN);

void Node::begin() {
  pinMode(DRIVER_HIGH_PIN, OUTPUT);
  pinMode(DRIVER_LOW_PIN, OUTPUT);
  pinMode(SHDN_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(HV_OUTPUT_SELECT_PIN, OUTPUT);
  pinMode(SSDA_PIN, OUTPUT);
  pinMode(SSCL_PIN, OUTPUT);

  // Set D0-D2 high (these are used to select test capacitors for
  // on-board calibration).
  for (uint8_t i = 0; i <= 3; i++) {
    pinMode(i, OUTPUT);
    digitalWriteFast(i, HIGH);
  }

  // set SPI pins high
  digitalWrite(SCK_PIN, HIGH);
  digitalWrite(MOSI_PIN, HIGH);

  config_.set_buffer(get_buffer());
  config_.validator_.set_node(*this);
  config_.reset();
  config_.load();

  state_.set_buffer(get_buffer());
  state_.validator_.set_node(*this);
  state_.reset();
  // Mark voltage, frequency, hv_output_enabled and hv_output_select
  // state for validation.
  state_._.has_voltage = true;
  state_._.has_frequency = true;
  state_._.has_hv_output_enabled = true;
  state_._.has_hv_output_selected = true;
  // Validate state to trigger on-changed handling for state fields that are
  // set (which initializes the state to the default values supplied in the
  // state protocol buffer definition).
  state_.validate();

  Serial.begin(115200);

  // only set the i2c clock if we have a valid i2c address (i.e., if
  // Wire.begin() was called
  if (config_._.i2c_address > 0) {
    Wire.setClock(400000);
  }

  Timer1.initialize(50); // initialize timer1, and set a 0.05 ms period
  Timer1.stop();

  // attach timer_callback() as a timer overflow interrupt
  Timer1.attachInterrupt(timer_callback);

  //_initialize_switching_boards();

  adc_ = new ADC();
}

void Node::_initialize_switching_boards() {
  // Check how many switching boards are connected.  Each additional board's
  // address must equal the previous boards address +1 to be valid.
  number_of_channels_ = 0;

  uint8_t I2C_DELAY_US = 200;

  for (uint8_t chip = 0; chip < 8; chip++) {
    // set IO ports as inputs
    buffer_[0] = PCA9505_CONFIG_IO_REGISTER;
    buffer_[1] = 0xFF;
    i2c_write((uint8_t)config_._.switching_board_i2c_address + chip,
              UInt8Array_init(2, (uint8_t *)&buffer_[0]));
    // XXX Delay required when operating with a 400kbps i2c clock.
    delayMicroseconds(I2C_DELAY_US);

    // read back the register value
    // if it matches what we previously set, this might be a PCA9505 chip
    if (i2c_read((uint8_t)config_._.switching_board_i2c_address + chip, 1).data[0] == 0xFF) {
      // try setting all ports in output mode and initialize to ground
      uint8_t port=0;
      for (; port<5; port++) {
        buffer_[0] = PCA9505_CONFIG_IO_REGISTER + port;
        buffer_[1] = 0x00;
        i2c_write((uint8_t)config_._.switching_board_i2c_address + chip,
                  UInt8Array_init(2, (uint8_t *)&buffer_[0]));
        // XXX Delay required when operating with a 400kbps i2c clock.
        delayMicroseconds(I2C_DELAY_US);

        // check that we successfully set the IO config register to 0x00
        if (i2c_read((uint8_t)config_._.switching_board_i2c_address + chip, 1).data[0] != 0x00) {
          return;
        }
        buffer_[0] = PCA9505_OUTPUT_PORT_REGISTER + port;
        buffer_[1] = 0xFF;
        i2c_write((uint8_t)config_._.switching_board_i2c_address + chip,
                  UInt8Array_init(2, (uint8_t *)&buffer_[0]));
        // XXX Delay required when operating with a 400kbps i2c clock.
        delayMicroseconds(I2C_DELAY_US);
      }

      // if port=5, it means that we successfully initialized all IO config
      // registers to 0x00, and this is probably a PCA9505 chip
      if (port==5) {
        if (number_of_channels_ == 40 * chip) {
          number_of_channels_ = 40 * (chip + 1);
        }
      }
    }
  }
}

void Node::timer_callback() {
  uint8_t high_pin_state = digitalRead(DRIVER_HIGH_PIN);
  if (high_pin_state == HIGH) {
    digitalWrite(DRIVER_HIGH_PIN, LOW);
    digitalWrite(DRIVER_LOW_PIN, HIGH);
  } else {
    digitalWrite(DRIVER_LOW_PIN, LOW);
    digitalWrite(DRIVER_HIGH_PIN, HIGH);
  }
}

}  // namespace dropbot
