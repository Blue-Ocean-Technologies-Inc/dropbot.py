#include "Node.h"

namespace dropbot {

void Node::begin() {
  Serial.begin(115200);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);

  // Set D0-D2 high (these are used to select test capacitors for
  // on-board calibration).
  for (uint8_t i = 0; i < 3; i++) {
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

  // Propagate config values to voltage_source namespace.
  // config_.reset()/load() populate the config struct but do NOT trigger
  // validator callbacks, so we must copy values explicitly.
  voltage_source::R7 = config_._.R7;
  voltage_source::pot_max = config_._.pot_max;
  voltage_source::max_voltage = config_._.max_voltage;
  voltage_source::min_frequency = config_._.min_frequency;
  voltage_source::max_frequency = config_._.max_frequency;

  // Initialize voltage source (FlexWire I2C, Timer1) BEFORE state validation,
  // since state validators call _set_voltage() which requires I2C to be ready.
  voltage_source::begin();

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

  analog::adc_.adc0->setResolution(16);
}

uint16_t Node::initialize_switching_boards() {
  /*
   * Scan for connected switching boards and determine the number of actuation
   * channels available.  Auto-detects the number of shift register ports per
   * board by probing up to `config_._.max_ports_per_board` ports on each board.
   *
   * Returns
   * -------
   * uint16_t
   *     Number of available actuation channels.
   */
  // Each additional board's address must equal the previous boards address +1
  // to be valid.
  uint16_t number_of_channels = 0;
  const uint8_t base_address = config_._.switching_board_i2c_address;
  const uint8_t max_ports = config_._.max_ports_per_board;
  uint8_t detected_ports_per_board = 0;

  uint8_t I2C_DELAY_US = 200;

  for (uint8_t chip_i = 0; chip_i < 8; chip_i++) {
    const uint8_t address_i = base_address + chip_i;

    // Probe: write 0xFF to config register port 0, read back.
    buffer_[0] = channels_.PCA9505_CONFIG_IO_REGISTER;
    buffer_[1] = 0xFF;
    i2c_write(address_i, UInt8Array_init(2, (uint8_t *)&buffer_[0]));
    // XXX Delay required when operating with a 400kbps i2c clock.
    delayMicroseconds(I2C_DELAY_US);

    // Read back the register value.
    // If it matches what we previously set, this might be a switching board.
    if (i2c_read(address_i, 1).data[0] != 0xFF) {
        // No switching board found at I2C `address_i`.
        break;
    }

    // Probe ports up to max_ports to detect actual register count.
    uint8_t ports_found = 0;
    for (uint8_t port_ij = 0; port_ij < max_ports; port_ij++) {
      buffer_[0] = channels_.PCA9505_CONFIG_IO_REGISTER + port_ij;
      buffer_[1] = 0x00;
      i2c_write(address_i, UInt8Array_init(2, (uint8_t *)&buffer_[0]));
      // XXX Delay required when operating with a 400kbps i2c clock.
      delayMicroseconds(I2C_DELAY_US);

      // Check that we successfully set the IO config register to 0x00.
      if (i2c_read(address_i, 1).data[0] != 0x00) {
          // Port doesn't exist on this board.
          break;
      }

      // Verified all IO port pins set as output.
      // Set output pins to ground.  **N.B.** outputs are **active low**.
      buffer_[0] = channels_.PCA9505_OUTPUT_PORT_REGISTER + port_ij;
      buffer_[1] = 0xFF;
      i2c_write(address_i, UInt8Array_init(2, (uint8_t *)&buffer_[0]));
      // XXX Delay required when operating with a 400kbps i2c clock.
      delayMicroseconds(I2C_DELAY_US);

      number_of_channels += 8;
      ports_found++;
    }

    // Use first board's port count as the reference for all boards.
    if (chip_i == 0 && ports_found > 0) {
      detected_ports_per_board = ports_found;
    }
  }

  // Store detected ports per board (fallback to 5 for safety).
  if (detected_ports_per_board == 0) {
    detected_ports_per_board = 5;
  }
  channels_.ports_per_board_ = detected_ports_per_board;

  // Clamp to MAX_NUMBER_OF_CHANNELS to prevent out-of-bounds access on
  // internal uint8_t-indexed arrays.  The full detected count is still
  // reported in state_._.channel_count so the host knows the true hardware
  // capability.
  state_._.channel_count = number_of_channels;
  state_._.has_channel_count = true;
  channels_.channel_count_ = (number_of_channels > MAX_NUMBER_OF_CHANNELS)
                              ? MAX_NUMBER_OF_CHANNELS : number_of_channels;
  channels_.switching_board_i2c_address_ = base_address;
  return state_._.channel_count;
}

}  // namespace dropbot
