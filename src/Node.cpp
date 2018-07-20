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
  state_._.has_event_mask = true;
  state_._.has_output_current_limit = true;
  state_._.has_chip_load_range_margin = true;
  // Validate state to trigger on-changed handling for state fields that are
  // set (which initializes the state to the default values supplied in the
  // state protocol buffer definition).
  state_.validate();

  Serial.begin(115200);

  // If we have a valid i2c address (i.e., if Wire.begin() was called)
  if (config_._.i2c_address > 0) {
    // set the i2c clock
    Wire.setClock(400000);

    /* Scan for connected switching boards and determine the number of actuation
     * channels available. */
    initialize_switching_boards();
  }

  Timer1.initialize(50); // initialize timer1, and set a 0.05 ms period
  Timer1.stop();

  // attach timer_callback() as a timer overflow interrupt
  Timer1.attachInterrupt(timer_callback);

  adc_ = new ADC();

  analogReadResolution(16);
}

uint16_t Node::initialize_switching_boards() {
  /*
   * Scan for connected switching boards and determine the number of actuation
   * channels available.
   *
   * Returns
   * -------
   * uint16_t
   *     Number of available actuation channels.
   */
  // Each additional board's address must equal the previous boards address +1
  // to be valid.
  uint16_t  number_of_channels = 0;
  const uint8_t base_address = config_._.switching_board_i2c_address;

  uint8_t I2C_DELAY_US = 200;

  for (uint8_t chip_i = 0; chip_i < 8; chip_i++) {
    const uint8_t address_i = base_address + chip_i;

    // Set IO ports as inputs.
    buffer_[0] = channels_.PCA9505_CONFIG_IO_REGISTER;
    buffer_[1] = 0xFF;
    i2c_write(address_i, UInt8Array_init(2, (uint8_t *)&buffer_[0]));
    // XXX Delay required when operating with a 400kbps i2c clock.
    delayMicroseconds(I2C_DELAY_US);

    // Read back the register value
    // if it matches what we previously set, this might be a PCA9505 chip
    if (i2c_read(address_i, 1).data[0] != 0xFF) {
        // No switching board found at I2C `address_i`.
        break;
    } else {
      // Assume the device at I2C `address_i` is a PCA9505 chip.
      // Try setting all ports in output mode and initialize to ground.
      for (uint8_t port_ij = 0; port_ij < 5; port_ij++) {
        buffer_[0] = channels_.PCA9505_CONFIG_IO_REGISTER + port_ij;
        buffer_[1] = 0x00;
        i2c_write(address_i, UInt8Array_init(2, (uint8_t *)&buffer_[0]));
        // XXX Delay required when operating with a 400kbps i2c clock.
        delayMicroseconds(I2C_DELAY_US);

        // Check that we successfully set the IO config register to 0x00.
        if (i2c_read(address_i, 1).data[0] != 0x00) {
            // Error setting IO port pins as output.
            break;
        } else {
            // Verified all IO port pins set as output.
            // Set output pins to ground.  **N.B.** `PCA9505` outputs are
            // **active low**.
            buffer_[0] = channels_.PCA9505_OUTPUT_PORT_REGISTER + port_ij;
            buffer_[1] = 0xFF;
            i2c_write(address_i, UInt8Array_init(2, (uint8_t *)&buffer_[0]));
            // XXX Delay required when operating with a 400kbps i2c clock.
            delayMicroseconds(I2C_DELAY_US);

            number_of_channels += 8;
        }
      }
    }
  }
  // Update channel count according to the number of discovered channels.
  state_._.channel_count = number_of_channels;
  state_._.has_channel_count = true;
  channels_.channel_count_ = state_._.channel_count;
  channels_.switching_board_i2c_address_ = base_address;
  return state_._.channel_count;
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
