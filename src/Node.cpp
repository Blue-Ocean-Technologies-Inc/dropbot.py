#include "Node.h"

namespace dropbot {

void Node::begin() {
  Serial.begin(115200);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);

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
  // Set default values.
  config_.reset();
  // Load values set explicitly in EEPROM.
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

 // Call Wire.begin() with the i2c address specified in the config.
  on_config_i2c_address_changed(config_._.i2c_address);

  // If we have a valid i2c address (i.e., if Wire.begin() was called)
  if (config_._.i2c_address > 0) {
    // set the i2c clock
    Wire.setClock(400000);

    /* Scan for connected switching boards and determine the number of actuation
     * channels available. */
    initialize_switching_boards();
  }

  voltage_source::begin();

  analog::adc_.adc0->setResolution(16);
}

uint8_t Node::query_shift_register_count(uint8_t address) {
  /*
   * Query shift register count from a switching board.
   * Returns 5 (default) if the command is not supported (v3.1 and below).
   */
  uint8_t I2C_DELAY_US = 200;
  
  // Try the new CMD_GET_SHIFT_REGISTER_COUNT command (firmware v4.1+)
  buffer_[0] = 0xA6;  // CMD_GET_SHIFT_REGISTER_COUNT
  i2c_write(address, UInt8Array_init(1, (uint8_t *)&buffer_[0]));
  delayMicroseconds(I2C_DELAY_US);
  
  // Read the first response byte which indicates how many bytes to read next
  uint8_t bytes_to_read = i2c_read(address, 1).data[0];
  
  if (bytes_to_read == 1) {
    // v3.1 firmware: Single byte response indicates RETURN_UNKNOWN_COMMAND
    return 5;  // Default to 5 shift registers for v3.1 and below
  } else if (bytes_to_read == 2) {
    // v4.1+ firmware: Two byte response with shift register count
    delayMicroseconds(I2C_DELAY_US);  // Small delay between I2C transactions
    return i2c_read(address, bytes_to_read).data[0];
  } else {
    // Unexpected response length - fallback to default
    return 5;
  }
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
      // Assume the device at I2C `address_i` is a switching board.
      // Query shift register count - this automatically handles v3/v4 differentiation
      uint8_t ports_to_check = query_shift_register_count(address_i);
      
      // Try setting all ports in output mode and initialize to ground.
      for (uint8_t port_ij = 0; port_ij < ports_to_check; port_ij++) {
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

}  // namespace dropbot
