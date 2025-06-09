#include "Node.h"

namespace dropbot
{

#define I2C_STATUS_PIN 13 // Use the built-in LED

  void Node::begin()
  {
    pinMode(SCK_PIN, OUTPUT);
    pinMode(MOSI_PIN, OUTPUT);

    // Initialize I2C early with other hardware
    Wire.begin();

    // Set D0-D2 high (these are used to select test capacitors for
    // on-board calibration).
    for (uint8_t i = 0; i <= 3; i++)
    {
      pinMode(i, OUTPUT);
      digitalWriteFast(i, HIGH);
    }

    // set SPI pins high
    digitalWrite(SCK_PIN, HIGH);
    digitalWrite(MOSI_PIN, HIGH);

    // Start serial early for debugging
    Serial.begin(115200);
    Serial.println("DropBot initialization starting...");
    Serial.println("I2C bus already initialized");

    config_.set_buffer(get_buffer());
    config_.validator_.set_node(*this);
    // Set default values.
    config_.reset();
    // Load values set explicitly in EEPROM.
    config_.load();

    Serial.print("I2C address from config: 0x");
    Serial.println(config_._.i2c_address, HEX);
    Serial.print("Switching board I2C address: 0x");
    Serial.println(config_._.switching_board_i2c_address, HEX);

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

    // If we have a valid i2c address (i.e., if Wire.begin() was called)
    if (config_._.i2c_address > 0)
    {
      Serial.print("Valid I2C address found: 0x");
      Serial.println(config_._.i2c_address, HEX);

      // set the i2c clock
      Wire.setClock(400000);
      Serial.println("I2C clock set to 400kHz");

      /* Scan for connected switching boards and determine the number of actuation
       * channels available. */
      Serial.println("Scanning for switching boards...");
      uint16_t channels = initialize_switching_boards();
      Serial.print("Found ");
      Serial.print(channels);
      Serial.println(" channels on switching boards");
    }
    else
    {
      Serial.println("ERROR: No valid I2C address found");
      Serial.println("I2C initialization failed");
    }

    voltage_source::begin();
    Serial.println("Voltage source initialized");

    analog::adc_.adc[0]->setResolution(16);
    Serial.println("ADC resolution set to 16-bit");

    Serial.println("DropBot initialization complete");
  }

  uint16_t Node::initialize_switching_boards()
  {
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
    uint16_t number_of_channels = 0;
    const uint8_t base_address = config_._.switching_board_i2c_address;

    uint8_t I2C_DELAY_US = 200;

    Serial.print("Scanning for switching boards starting at address 0x");
    Serial.println(base_address, HEX);

    for (uint8_t chip_i = 0; chip_i < 8; chip_i++)
    {
      const uint8_t address_i = base_address + chip_i;

      Serial.print("Checking for switching board at address 0x");
      Serial.print(address_i, HEX);
      Serial.print("...");

      // Set IO ports as inputs.
      buffer_[0] = channels_.PCA9505_CONFIG_IO_REGISTER;
      buffer_[1] = 0xFF;
      i2c_write(address_i, UInt8Array_init(2, (uint8_t *)&buffer_[0]));
      // XXX Delay required when operating with a 400kbps i2c clock.
      delayMicroseconds(I2C_DELAY_US);

      // Read back the register value
      // if it matches what we previously set, this might be a PCA9505 chip
      if (i2c_read(address_i, 1).data[0] != 0xFF)
      {
        // No switching board found at I2C `address_i`.
        Serial.println(" not found");
        break;
      }
      else
      {
        Serial.println(" found");
        // Assume the device at I2C `address_i` is a PCA9505 chip.
        // Try setting all ports in output mode and initialize to ground.
        for (uint8_t port_ij = 0; port_ij < 5; port_ij++)
        {
          Serial.print("  Configuring port ");
          Serial.print(port_ij);
          Serial.print("...");

          buffer_[0] = channels_.PCA9505_CONFIG_IO_REGISTER + port_ij;
          buffer_[1] = 0x00;
          i2c_write(address_i, UInt8Array_init(2, (uint8_t *)&buffer_[0]));
          // XXX Delay required when operating with a 400kbps i2c clock.
          delayMicroseconds(I2C_DELAY_US);

          // Check that we successfully set the IO config register to 0x00.
          if (i2c_read(address_i, 1).data[0] != 0x00)
          {
            // Error setting IO port pins as output.
            Serial.println(" error");
            break;
          }
          else
          {
            Serial.println(" success");
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

    Serial.print("Total channels discovered: ");
    Serial.println(state_._.channel_count);

    return state_._.channel_count;
  }

} // namespace dropbot
