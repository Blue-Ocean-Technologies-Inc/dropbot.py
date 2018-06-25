#ifndef ___NODE__H___
#define ___NODE__H___

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <Arduino.h>
#include <NadaMQ.h>
#include <CArrayDefs.h>
#include "RPCBuffer.h"  // Define packet sizes
#include "Dropbot/Properties.h"  // Define package name, URL, etc.
#include <BaseNodeRpc/BaseNode.h>
#include <BaseNodeRpc/BaseNodeEeprom.h>
#include <BaseNodeRpc/BaseNodeI2c.h>
#include <BaseNodeRpc/BaseNodeConfig.h>
#include <BaseNodeRpc/BaseNodeState.h>
#include <BaseNodeRpc/BaseNodeI2cHandler.h>
#include <BaseNodeRpc/BaseNodeSerialHandler.h>
#include <BaseNodeRpc/SerialHandler.h>
#include <ADC.h>
#include <RingBufferDMA.h>
#include <DMAChannel.h>
#include <InputDebounce.h>
#include <TeensyMinimalRpc/ADC.h>  // Analog to digital converter
#include <TeensyMinimalRpc/DMA.h>  // Direct Memory Access
#include <TeensyMinimalRpc/SIM.h>  // System integration module (clock gating)
#include <TeensyMinimalRpc/PIT.h>  // Programmable interrupt timer
#include <TeensyMinimalRpc/aligned_alloc.h>
#include <pb_cpp_api.h>
#include <pb_validate.h>
#include <pb_eeprom.h>
#include <LinkedList.h>
#include <TimerOne.h>
#include <SlowSoftWire.h>
#include "FastAnalogWrite.h"
#include "dropbot_config_validate.h"
#include "dropbot_state_validate.h"
#include "Dropbot/config_pb.h"
#include "Dropbot/state_pb.h"
#include "packet_stream.h"
#include "kxsort.h"

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

/* .. versionadded:: 1.42
 *
 * Add `printf`/`sprintf` float formatting support.
 *
 * See https://forum.pjrc.com/threads/41761-float-to-string-issue?p=131683&viewfull=1#post131683 */
asm(".global _printf_float");

extern uint8_t watchdog_status_;
extern bool watchdog_refresh_;
extern uint16_t STARTUP_WDOG_STCTRLH_VALUE;

const uint32_t ADC_BUFFER_SIZE = 4096;

extern void dma_ch0_isr(void);
extern void dma_ch1_isr(void);
extern void dma_ch2_isr(void);
extern void dma_ch3_isr(void);
extern void dma_ch4_isr(void);
extern void dma_ch5_isr(void);
extern void dma_ch6_isr(void);
extern void dma_ch7_isr(void);
extern void dma_ch8_isr(void);
extern void dma_ch9_isr(void);
extern void dma_ch10_isr(void);
extern void dma_ch11_isr(void);
extern void dma_ch12_isr(void);
extern void dma_ch13_isr(void);
extern void dma_ch14_isr(void);
extern void dma_ch15_isr(void);

namespace dropbot {

// Define the array that holds the conversions here.
// The buffer is stored with the correct alignment in the DMAMEM section
// the +0 in the aligned attribute is necessary b/c of a bug in gcc.
DMAMEM static volatile int16_t __attribute__((aligned(ADC_BUFFER_SIZE+0))) adc_buffer[ADC_BUFFER_SIZE];


const size_t FRAME_SIZE = (3 * sizeof(uint8_t)  // Frame boundary
                           - sizeof(uint16_t)  // UUID
                           - sizeof(uint16_t)  // Payload length
                           - sizeof(uint16_t));  // CRC


inline void __watchdog_refresh__() {
  while(WDOG_TMROUTL < 2) {}
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
  while(WDOG_TMROUTL >= 2) {}
}

inline void __watchdog_disable__() {
  bool enabled = WDOG_STCTRLH & WDOG_STCTRLH_WDOGEN;
  if (!enabled) { return; }
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  __asm__ volatile ("nop");
  __asm__ volatile ("nop");

  WDOG_TMROUTH = 0;
  WDOG_TMROUTL = 0;
  WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;
  WDOG_TMROUTH = 0;
  WDOG_TMROUTL = 0;
  interrupts();
}


class Node;
const char HARDWARE_VERSION_[] = "3.6";

typedef nanopb::EepromMessage<dropbot_Config,
                              config_validate::Validator<Node> > config_t;
typedef nanopb::Message<dropbot_State,
                        state_validate::Validator<Node> > state_t;


/* Class to detect chip insertion/removal and publish corresponding events to
 * serial stream. */
class OutputEnableDebounce : public InputDebounce {
public:
  OutputEnableDebounce(Node &parent, int8_t pinIn=-1, unsigned long
                       debDelay=DEFAULT_INPUT_DEBOUNCE_DELAY,
                       PinInMode pinInMode=PIM_INT_PULL_UP_RES,
                       unsigned long pressedDuration=0)
    : InputDebounce(pinIn, debDelay, pinInMode, pressedDuration),
      parent_(parent) {}
  virtual ~OutputEnableDebounce() {}
protected:
  virtual void pressed();
  virtual void released();
private:
  Node &parent_;
};


// XXX For control-board hardware version v3.5
class Node :
  public BaseNode,
  public BaseNodeEeprom,
  public BaseNodeI2c,
  public BaseNodeConfig<config_t>,
  public BaseNodeState<state_t>,
#ifndef DISABLE_SERIAL
  public BaseNodeSerialHandler,
#endif  // #ifndef DISABLE_SERIAL
  public BaseNodeI2cHandler<base_node_rpc::i2c_handler_t> {
private:
  float _high_voltage;  // Cached high-voltage reading.
public:
  typedef PacketParser<FixedPacket> parser_t;

  static void timer_callback();
  static SlowSoftWire i2c;

  static const uint32_t BUFFER_SIZE = 8192;  // >= longest property string

  static const uint16_t MAX_NUMBER_OF_CHANNELS = 120;

  static const uint8_t DRIVER_HIGH_PIN = 6;
  static const uint8_t DRIVER_LOW_PIN = 7;
  static const uint8_t HV_OUTPUT_SELECT_PIN = 8;

  // pins connected to the boost converter
  static const uint8_t SHDN_PIN = 10;
  static const uint8_t SSDA_PIN = 4;
  static const uint8_t SSCL_PIN = 5;

  // SPI pins
  static const uint8_t MOSI_PIN = 11;
  static const uint8_t SCK_PIN = 13;

  // On-board calibration pins
  // XXX The following pins are **active LOW**, i.e., each corresponding
  // capacitance is activated between **high-voltage** and **(virtual) ground**
  // when the respective pin is set **LOW**.
  static const uint8_t CAPACITANCE_1PF_PIN = 0;
  static const uint8_t CAPACITANCE_10PF_PIN = 1;
  static const uint8_t CAPACITANCE_100PF_PIN = 2;

  // High-voltage Output Enable pin
  static const uint8_t OE_PIN = 22;

  // PCA9505 (gpio) chip/register addresses
  static const uint8_t PCA9505_CONFIG_IO_REGISTER = 0x18;
  static const uint8_t PCA9505_OUTPUT_PORT_REGISTER = 0x08;

  static const float R6;

  uint8_t buffer_[BUFFER_SIZE];
  uint8_t state_of_channels_[MAX_NUMBER_OF_CHANNELS / 8];
  uint8_t disabled_channels_mask_[MAX_NUMBER_OF_CHANNELS / 8];

  ADC *adc_;
  uint32_t adc_period_us_;
  uint32_t adc_timestamp_us_;
  bool adc_tick_tock_;
  uint32_t adc_millis_;
  uint32_t adc_SYST_CVR_;
  uint32_t adc_millis_prev_;
  uint32_t adc_SYST_CVR_prev_;
  uint32_t adc_count_;
  bool dma_adc_active_;
  int8_t dma_channel_done_;
  int8_t last_dma_channel_done_;
  bool adc_read_active_;
  LinkedList<uint32_t> allocations_;
  LinkedList<uint32_t> aligned_allocations_;
  UInt8Array dma_data_;
  uint16_t dma_stream_id_;
  bool watchdog_disable_request_;
  base_node_rpc::FastAnalogWrite fast_analog_;

  // Detect chip insertion/removal.
  OutputEnableDebounce output_enable_input;

  //: .. versionadded:: 1.42
  //
  // Time of most recent capacitance measurement.
  uint32_t capacitance_timestamp_ms_;

  Node() : BaseNode(),
           BaseNodeConfig<config_t>(dropbot_Config_fields),
           BaseNodeState<state_t>(dropbot_State_fields),
           _high_voltage(0),
           adc_period_us_(0), adc_timestamp_us_(0), adc_tick_tock_(false),
           adc_count_(0), dma_adc_active_(false), dma_channel_done_(-1),
           last_dma_channel_done_(-1), adc_read_active_(false),
           dma_stream_id_(0), watchdog_disable_request_(false),
           output_enable_input(*this, -1, DEFAULT_INPUT_DEBOUNCE_DELAY,
                               InputDebounce::PinInMode::PIM_EXT_PULL_UP_RES,
                               0),
           capacitance_timestamp_ms_(0) {
    pinMode(LED_BUILTIN, OUTPUT);
    dma_data_ = UInt8Array_init_default();
    output_enable_input.setup(OE_PIN);
  }

  UInt8Array get_buffer() { return UInt8Array_init(sizeof(buffer_), buffer_); }
  /* This is a required method to provide a temporary buffer to the
   * `BaseNode...` classes. */

  void begin();
  /****************************************************************************
   * # User-defined methods #
   *
   * Add new methods below.  When Python package is generated using the
   * command, `paver sdist` from the project root directory, the signatures of
   * the methods below will be scanned and code will automatically be generated
   * to support calling the methods from Python over a serial connection.
   *
   * e.g.
   *
   *     bool less_than(float a, float b) { return a < b; }
   *
   * See [`arduino_rpc`][1] and [`base_node_rpc`][2] for more details.
   *
   * [1]: https://github.com/wheeler-microfluidics/arduino_rpc
   * [2]: https://github.com/wheeler-microfluidics/base_node_rpc
   */
  void soft_i2c_write(uint8_t address, UInt8Array data) {
    i2c.beginTransmission(address);
    i2c.write(data.data, data.length);
    i2c.endTransmission();
  }

 /*
  * Measure the temperature (in degrees Celsius) of the MCU
  * via it's internal sensor.
  *
  * More details available [here][1] and [here][2].
  *
  * [1]: https://forum.pjrc.com/threads/30480-Teensy-3-1-use-internal-Temp-Sensor
  * [2]: https://github.com/LAtimes2/InternalTemperature
  */
  float measure_temperature() {
    analogReference(INTERNAL);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 255; i++) {
        sum += analogRead(38);
    }
    analogReference(DEFAULT);
    float voltage = (float)sum / 255.0 / 65535.0 * 1.2;
    return 25.0 + 583.0904 * (0.719 - voltage);
  }

  /*
   * Measure the analog reference voltage by comparing it
   * to the internal reference (1.195 V) accessible via
   * analog pin 39.
   */
  float measure_aref() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 255; i++) {
        sum += analogRead(39);
    }
    return 1.195 * 65535.0 / (float)sum * 255.0;
  }

  UInt16Array kxsort_u16(UInt16Array data) {
    /*
     * .. versionadded:: 1.41
     *
     * Sort array (in-place) using (MIT-licensed)
     * `radix-sort <https://github.com/voutcn/kxsort>`_.
     *
     * Parameters
     * ----------
     * data : UInt16Array
     *     Input array.
     *
     * Returns
     * -------
     * UInt16Array
     *     Input array in sorted order.
     */
    kx::radix_sort(data.data, &data.data[data.length]);
    return data;
  }

  uint16_t u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                               float low_percentile, float high_percentile) {
    /*
     * ..versionadded:: 1.41
     *
     * Measure samples from specified analog pin and compute difference between
     * specified high and low percentiles.
     *
     * For example, :data:`low_percentile` as 25 and :data:`high_percentile` as
     * 75 is equivalent to computing the `inter-quartile range <https://en.wikipedia.org/wiki/Interquartile_range>`_.
     *
     * Parameters
     * ----------
     * pin : uint8_t
     *     Analog pin number.
     * n_samples : uint16_t
     *     Number of samples to measure.
     * low_percentile : float
     *     Low percentile of range.
     * high_percentile : float
     *     High percentile of range.
     *
     * Returns
     * -------
     * uint16_t
     *     Difference between high and low percentiles.
     */
    UInt16Array result = analog_reads_simple(pin, n_samples);
    kx::radix_sort(result.data, &result.data[result.length]);
    const uint16_t high_i = (int)round((high_percentile / 100.) * n_samples);
    const uint16_t low_i = (int)round((low_percentile / 100.) * n_samples);
    return result.data[high_i] - result.data[low_i];
  }

  float capacitance(uint16_t n_samples) {
    /*
     * .. versionadded:: 1.41
     *
     * Measure device load capacitance based on the specified number of analog
     * samples.
     *
     * Amplitude of measured square wave is calculated by computing the `inter-quartile range (IQR) <https://en.wikipedia.org/wiki/Interquartile_range>`_,
     * i.e., the difference between the 75th percentile and the 25th
     * percentile.
     *
     * See: https://gitlab.com/sci-bots/dropbot.py/issues/25
     *
     * Notes
     * -----
     *
     * According to the figure below, the transfer function describes the
     * following relationship::
     *
     *     V₂   Z₂
     *     ── = ──
     *     V₁   Z₁
     *
     * where $V_{1}$ denotes the high-voltage actuation signal and $V_{2}$
     * denotes the signal sufficiently attenuated to fall within the measurable
     * input range of the analog-to-digital converter *(approx. 3.3 V)*.  The
     * feedback circuits for the control board is shown below.
     *
     * .. code-block:: none
     *
     *       V_1 @ frequency
     *           ┯
     *         ┌─┴─┐    ┌───┐
     *         │Z_1│  ┌─┤Z_2├─┐
     *         └─┬─┘  │ └───┘ │
     *           │    │  │╲   ├───⊸ V_2
     *           └────┴──│-╲__│
     *                ┌──│+╱
     *                │  │╱
     *                │
     *               ═╧═
     *
     * See `HVAC`_ in DropBot HV square wave driver and `A11` and `C16` in
     * `feedback filter`_:
     *
     *  - `C16`: 0.15 uF
     *
     * Where ``V1`` and ``V2`` are root-mean-squared voltages, and Z1 == jwC1``
     * and ``Z2 == jwC2``, ``C2 = V2 / V1 * C1``.
     *
     * Parameters
     * ----------
     * n_samples : uint16_t
     *     Number of analog samples to measure.
     *
     *     If 0, use default from :attr:`config_._`.
     *
     * Returns
     * -------
     * float
     *     Capacitance of device load in farads (F).
     *
     *
     * .. versionchanged:: 1.41
     *     If 0, use default from :attr:`config_._`.
     *
     * .. versionchanged:: 1.43
     *     Fix equation to divide by actuation voltage.
     *
     * .. _`HVAC`: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/boost-converter-boost-converter.pdf
     * .. _`feedback filter`: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/feedback-feedback.pdf
     */

    // Compute capacitance from measured square-wave RMS voltage amplitude.
    n_samples = (n_samples) ? n_samples : config_._.capacitance_n_samples;
    const uint16_t A11_raw = u16_percentile_diff(11, n_samples, 25, 75);

    // Compute capacitance from measured square-wave RMS voltage amplitude.
    // V2 = 0.5 * (float(A11) / MAX_ANALOG) * AREF
    const float device_load_v = 0.5 * (A11_raw / float(1L << 16)) * 3.3;
    // C2 = V2 * C16 / HVAC
    const float C2 = device_load_v * 0.15e-6 / high_voltage();
    return C2;
  }

  /**
  * @brief Measure high-side *root mean-squared (RMS)* voltage.
  *
  * See [`A1/HV_FB`][1] in DropBot boost converter schematic:
  *
  *  - `R8`: 2 Mohms
  *  - `R9`: 20 Kohms
  *  - `AREF`: 3.3 V
  *
  * [1]: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/boost-converter-boost-converter.pdf
  *
  *
  * \version 1.51  Cache most recent RMS voltage as `_high_voltage`.
  *
  * @return High-side RMS voltage.
  */
  float high_voltage() {
    // HV_FB = (float(A1) / MAX_ANALOG) * AREF
    const float hv_fb = (analogRead(A1) / float(1L << 16)) * 3.3;
    // HV peak-to-peak = HV_FB * R8 / R9
    const float R8 = 2e6;
    const float R9 = 20e3;
    const float hv_peak_to_peak = hv_fb * R8 / R9;
    // HV RMS = 0.5 * HV peak-to-peak
    _high_voltage = 0.5 * hv_peak_to_peak;
    return _high_voltage;
  }

  UInt16Array analog_reads_simple(uint8_t pin, uint16_t n_samples) {
    UInt8Array byte_buffer = get_buffer();
    UInt16Array output;
    output.data = reinterpret_cast<uint16_t *>(byte_buffer.data);

    for (uint16_t i = 0; i < n_samples; i++) {
      output.data[i] = analogRead(pin);
    }
    output.length = n_samples;
    return output;
  }

  UInt8Array soft_i2c_read(uint8_t address, uint8_t n_bytes_to_read) {
    UInt8Array output = get_buffer();
    i2c.requestFrom(address, n_bytes_to_read);
    uint8_t n_bytes_read = 0;
    uint8_t value;
    while (n_bytes_read < n_bytes_to_read) {
      value = i2c.read();
      output.data[n_bytes_read++] = value;
    }
    output.length = n_bytes_read;
    return output;
  }
  UInt8Array soft_i2c_scan() {
    UInt8Array output = get_buffer();
    uint16_t count = 0;

    /* The I2C specification has reserved addresses in the ranges `0x1111XXX`
     * and `0x0000XXX`.  See [here][1] for more details.
     *
     * [1]: http://www.totalphase.com/support/articles/200349176-7-bit-8-bit-and-10-bit-I2C-Slave-Addressing */
    for (uint8_t i = 8; i < 120; i++) {
      if (count >= output.length) { break; }
      i2c.beginTransmission(i);
      if (i2c.endTransmission() == 0) {
        output.data[count++] = i;
        delay(1);  // maybe unneeded?
      }
    }
    output.length = count;
    return output;
  }
  uint16_t number_of_channels() const { return state_._.channel_count; }
  UInt8Array hardware_version() { return UInt8Array_init(strlen(HARDWARE_VERSION_),
                      (uint8_t *)&HARDWARE_VERSION_[0]); }
  UInt8Array _uuid() {
    /* Read unique chip identifier. */
    UInt8Array result = get_buffer();
    result.length = 4 * sizeof(uint32_t);
    memcpy(&result.data[0], &SIM_UIDH, result.length);
    return result;
  }

  bool set_id(UInt8Array id) {
    if (id.length > sizeof(config_._.id) - 1) {
      return false;
    }
    memcpy(config_._.id, &id.data[0], id.length);
    config_._.id[id.length] = 0;
    config_._.has_id = true;
    config_.save();
    return true;
  }

  /**
  * @brief Detect shorts between channels.
  *
  * Apply low voltage (3.3 V) to each channel in isolation.  If the measured
  * voltage is less than half of the applied voltage, mark channel as detected
  * short.
  *
  * @param delay_ms  Amount of time to wait after sending updated channel
  * states to switching boards before measuring applied voltage.
  *
  * @return Array of channel numbers where shorts were detected.
  *
  *
  * \version 1.51
  *     Send `shorts-detected` event stream packet containing:
  *      - `"values"`: list of identifiers of shorted channels.
  */
  UInt8Array detect_shorts(uint8_t delay_ms) {
    // Deselect the HV output
    on_state_hv_output_selected_changed(false);

    // Eight channels per port
    const uint8_t port_count = state_._.channel_count / 8;
    uint8_t old_state[port_count];
    memcpy(old_state, state_of_channels_, port_count);

    UInt8Array shorts = get_buffer();
    shorts.length = 0;
    for (uint8_t i = 0; i < state_._.channel_count; i++) {
      // Initialize all channels in off state
      memset(state_of_channels_, 0, port_count);

      // Set bit to actuate channel i
      state_of_channels_[i / 8] = 1 << i % 8;

      // Apply channel states
      _update_channels();

      // A delay (e.g., 1-5 ms) may be necessary to detect some shorts
      delay(delay_ms);

      // If we read less than half of Vcc, append this channel to the
      // list of shorts and disable the channel
      if (analog_read(0) < 65535 / 2) {
        shorts.data[shorts.length++] = i;
        disabled_channels_mask_[i / 8] |= 1 << i % 8;
      } else { // enable the channel
        disabled_channels_mask_[i / 8] &= ~(1 << i % 8);
      }
    }

    // Restore the previous channel state
    memcpy(state_of_channels_, old_state, port_count);

    // Apply channel states
    _update_channels();

    // Restore the HV output selection
    on_state_hv_output_selected_changed(state_._.hv_output_selected);

    {
      // Stream `shorts-detected` event packet.
      UInt8Array result;
      result.data = &shorts.data[shorts.length];

      sprintf((char *)result.data, "{\"event\": \"shorts-detected\", \"values\": [");
      result.length = strlen((char *)result.data);

      char *start = reinterpret_cast<char *>(&result.data[result.length]);
      for (int i = 0 ; i < shorts.length; i++) {
        if (i > 0) {
          sprintf(start, ", ");
          start += 2;
        }
        sprintf(start, "%d", shorts.data[i]);
        start += strlen(start);
      }

      sprintf(start, "]};");
      start += strlen(start);
      result.length = reinterpret_cast<uint8_t *>(start) - result.data;

      {
        PacketStream output;
        output.start(Serial, result.length);
        output.write(Serial, (char *)result.data, result.length);
        output.end(Serial);
      }
    }

    return shorts;
  }

  UInt8Array state_of_channels() {
    // XXX Stop the timer (which toggles the HV square-wave driver) during i2c
    // communication.
    //
    // See https://gitlab.com/sci-bots/dropbot.py/issues/26
    Timer1.stop(); // stop the timer during i2c transmission
    for (uint8_t chip = 0; chip < state_._.channel_count / 40; chip++) {
      for (uint8_t port = 0; port < 5; port++) {
        Wire.beginTransmission((uint8_t)config_._.switching_board_i2c_address + chip);
        Wire.write(PCA9505_OUTPUT_PORT_REGISTER + port);
        Wire.endTransmission();

        delayMicroseconds(100); // needed when using Teensy

        Wire.requestFrom(config_._.switching_board_i2c_address + chip, 1);
        if (Wire.available()) {
          state_of_channels_[chip * 5 + port] = ~Wire.read();
        } else {
          Timer1.restart();
          return UInt8Array_init_default();
        }
      }
    }
    Timer1.restart();
    return UInt8Array_init(state_._.channel_count / 8,
                           (uint8_t *)&state_of_channels_[0]);
  }

  UInt8Array disabled_channels_mask() {
    return UInt8Array_init(state_._.channel_count / 8,
                           (uint8_t *)&disabled_channels_mask_[0]);
  }

  bool set_disabled_channels_mask(UInt8Array disabled_channels_mask) {
    if (disabled_channels_mask.length == state_._.channel_count / 8) {
      for (uint16_t i = 0; i < disabled_channels_mask.length; i++) {
        disabled_channels_mask_[i] = disabled_channels_mask.data[i];
      }
      _update_channels();
      return true;
    }
    return false;
  }

  bool set_state_of_channels(UInt8Array channel_states) {
    if (channel_states.length == state_._.channel_count / 8) {
      for (uint16_t i = 0; i < channel_states.length; i++) {
        state_of_channels_[i] = channel_states.data[i];
      }
      _update_channels();
      return true;
    }
    return false;
  }

  /**
  * @brief Apply state of channels to switching boards.
  *
  * Turn off all channels that are marked in the disabled channels mask.
  *
  * \version 1.51
  *     Send `channels-updated` event stream packet containing:
  *      - `"n"`: number of actuated channel
  *      - `"actuated"`: list of actuated channel identifiers.
  *      - `"start"`: millisecond counter before setting shift registers
  *      - `"end"`: millisecond counter after setting shift registers
  *
  */
  void _update_channels() {
    // XXX Stop the timer (which toggles the HV square-wave driver) during i2c
    // communication.
    //
    // See https://gitlab.com/sci-bots/dropbot.py/issues/26
    Timer1.stop(); // stop the timer during i2c transmission
    const unsigned long start = microseconds();
    uint8_t data[2];
    // Each PCA9505 chip has 5 8-bit output registers for a total of 40 outputs
    // per chip. We can have up to 8 of these chips on an I2C bus, which means
    // we can control up to 320 channels.
    //   Each register represent 8 channels (i.e. the first register on the
    // first PCA9505 chip stores the state of channels 0-7, the second register
    // represents channels 8-15, etc.).
    for (uint8_t chip = 0; chip < state_._.channel_count / 40; chip++) {
      for (uint8_t port = 0; port < 5; port++) {
        data[0] = PCA9505_OUTPUT_PORT_REGISTER + port;
        data[1] = ~(state_of_channels_[chip * 5 + port] &
                    ~disabled_channels_mask_[chip * 5 + port]);
        i2c_write(config_._.switching_board_i2c_address + chip,
                  UInt8Array_init(2, (uint8_t *)&data[0]));
        // XXX Need the following delay if we are operating with a 400kbps
        // i2c clock.
        delayMicroseconds(200);
      }
    }
    const unsigned long end = microseconds();
    Timer1.restart();
    {
      // Stream `channels-updated` event packet.
      UInt8Array result = get_buffer();
      char * const data = reinterpret_cast<char *>(result.data);

      result.length = sprintf((char *)result.data, "{\"event\": "
                              "\"channels-updated\", \"actuated\": [");

      // XXX LSB of chip 0 and port 0 is channel 0.
      uint8_t actuated_count = 0;
      for (uint8_t chip = 0; chip < state_._.channel_count / 40; chip++) {
        for (uint8_t port = 0; port < 5; port++) {
          const uint8_t byte_j = chip * 5 + port;
          for (uint8_t i = 0; i < 8; i++) {
            if (bitRead(state_of_channels_[byte_j] &
                        ~disabled_channels_mask_[byte_j], i)) {
              // Channel is actuated.
              const uint8_t channel_id = 8 * byte_j + i;
              if (actuated_count > 0) {
                result.length += sprintf(&data[result.length], ", ");
              }
              actuated_count++;
              result.length += sprintf(&data[result.length], "%d", channel_id);
            }
          }
        }
      }
      result.length += sprintf(&data[result.length], "], \"start\": %lu, \"end\": %lu, \"n\": %d}", start, end, actuated_count);

      {
        PacketStream output;
        output.start(Serial, result.length);
        output.write(Serial, (char *)result.data, result.length);
        output.end(Serial);
      }
    }
  }

  float min_waveform_voltage() {
    return 1.5 / 2.0 * (R6 / (config_._.pot_max + config_._.R7) + 1);
  }

  bool _set_voltage(float voltage) {
    float pot_value = R6 / ( 2 * voltage / 1.5 - 1 ) - config_._.R7;
    float wiper_value = pot_value / config_._.pot_max * 255;
    if ( voltage <= config_._.max_voltage && \
         wiper_value < 256 && pot_value >= 0 ) {
      // This method is triggered whenever a voltage is included in a state
      // update.
      i2c.beginTransmission(44);
      i2c.write(0);
      i2c.write((uint8_t)wiper_value);
      i2c.endTransmission();

      // verify that we wrote the correct value to the pot
      i2c.requestFrom(44, 1);
      if (i2c.read() == (uint8_t)wiper_value) {
        return true;
      }
    }
    return false;
  }

  uint16_t initialize_switching_boards();

  // # Callback methods
  bool on_state_frequency_changed(float frequency) {
    /* This method is triggered whenever a frequency is included in a state
     * update. */
    if (frequency == 0) { // DC mode
      digitalWrite(DRIVER_HIGH_PIN, HIGH); // set voltage high
      digitalWrite(DRIVER_LOW_PIN, LOW);
      Timer1.stop(); // stop timer
    } else if ((config_._.min_frequency <= frequency) &&
                (frequency <= config_._.max_frequency)) {
      Timer1.setPeriod(500000.0 / frequency); // set timer period in ms
      Timer1.restart();
    } else {
      return false;
    }
    return true;
  }

  bool on_state_voltage_changed(float voltage) {
    return _set_voltage(voltage);
  }

  bool on_state_hv_output_enabled_changed(bool value) {
    if (value) {
      // If we're turning the output on, we need to start with a low voltage,
      // enable the MAX1771, then increase the voltage. Otherwise, if the voltage
      // is > ~100 the MAX1771 will not turn on.
      _set_voltage(min_waveform_voltage());
      delay(100);
      digitalWrite(SHDN_PIN, !value);
      delay(100);
      _set_voltage(state_._.voltage);
      Timer1.setPeriod(500000.0 / state_._.frequency); // set timer period in ms
      Timer1.restart();
    } else {
      digitalWrite(SHDN_PIN, !value);
      Timer1.stop(); // stop timer
    }
    return true;
  }

  bool on_state_hv_output_selected_changed(bool value) {
    /* .. versionchanged:: 1.37.1
     *     Toggle HV output to address issue #23.
     */

    digitalWrite(HV_OUTPUT_SELECT_PIN, !value);
    if (value && state_._.hv_output_enabled) {
        // If high voltage output is selected (as opposed to low short
        // detection voltage), and high voltage output is enabled (i.e., the
        // hardware interlock `OE_PIN` is pulled LOW and the boost converter is
        // on), the high voltage should be connected to the switching boards.
        // However, this currently does not happen.
        //
        // XXX As a workaround, force the high voltage to be turned off and on,
        // to ensure high voltage is actually connected to the switching
        // boards.
        //
        // See [issue #23][i23] for more information.
        //
        // [i23]: https://gitlab.com/sci-bots/dropbot.py/issues/23
        on_state_hv_output_enabled_changed(false);
        on_state_hv_output_enabled_changed(true);
    }
    return true;
  }

  bool on_state_channel_count_changed(int32_t value) {
      // XXX This value is ready-only.
      return false;
  }

  void on_tick() {
    if (adc_read_active_) return;
    uint8_t channel;
    switch(adc_count_ & 0x01) {
      case(0): channel = A0; break;
      case(1): channel = A1; break;
      //case(2): channel = A2;
      //case(3): channel = A3;
      default: channel = A0;
    }
    adc_->startSingleRead(channel, ADC_0);
  }

  void on_adc_done() {
    if (adc_read_active_) return;
    adc_count_++;
    //adc_tick_tock_ = !adc_tick_tock_;
    //digitalWriteFast(LED_BUILTIN, adc_tick_tock_);
    //adc_SYST_CVR_prev_ = adc_SYST_CVR_;
    //adc_millis_ = millis();
    //adc_SYST_CVR_ = SYST_CVR;
  }

  /**
  * @brief Called periodically from the main program loop.
  *
  *
  * \version 1.42
  *     Add periodic capacitance measurement.  Each new value is sent as an
  *     event stream packet to the serial interface.
  *
  * \version 1.43
  *     Report `start` and `end` times of capacitance update events in
  *     **microseconds** instead of **milliseconds**.
  *
  *     If target capacitance state field is non-zero, compare current device
  *     load capacitance to the target capacitance.  If target capacitance has
  *     been exceeded, stream `"capacitance-exceeded"` event packet to serial
  *     interface.
  *
  * \version 1.51
  *     Add actuation voltage, `V_a`, to `capacitance-updated` and
  *     `capacitance-exceeded` event stream packets.
  */
  void loop() {
    unsigned long now = millis();

    // poll button state
    output_enable_input.process(now);

    // Send event if target capacitance has been set and exceeded.
    if (state_._.target_capacitance > 0) {
      UInt8Array result = get_buffer();
      result.length = 0;

      const unsigned long n_samples = config_._.capacitance_n_samples;
      unsigned long start = microseconds();
      float value = capacitance(n_samples);
      now = microseconds();

      if (value > state_._.target_capacitance) {
        // Target capacitance has been met.

        // Stream "capacitance-updated" event to serial interface.
        sprintf((char *)result.data, "{\"event\": \"capacitance-exceeded\", "
                "\"new_value\": %g, "  // Capacitance value
                "\"target\": %g, "  // Target capacitance value
                " \"start\": %lu, \"end\": %lu, "  // start/end times in ms
                "\"n_samples\": %lu",  // # of analog samples
                "\"V_a\": %g}",  // Actuation voltage
                value, state_._.target_capacitance,
                start, now, n_samples, _high_voltage);
        result.length = strlen((char *)result.data);
        unsigned long now = microseconds();

        // Reset target capacitance.
        state_._.target_capacitance = 0;

        {
          PacketStream output;
          output.start(Serial, result.length);
          output.write(Serial, (char *)result.data, result.length);
          output.end(Serial);
        }

        // Reset periodic capacitance timestamp since there is no need to send
        // `capacitance-updated` event since `capacitance-exceeded` event
        // contains latest capacitance value.
        capacitance_timestamp_ms_ = now;
      }
    }

    // Send periodic capacitance value update events.
    if ((state_._.capacitance_update_interval_ms > 0) &&
        (state_._.capacitance_update_interval_ms < now -
         capacitance_timestamp_ms_)) {
      UInt8Array result = get_buffer();
      result.length = 0;

      unsigned long start = microseconds();
      const unsigned long n_samples = config_._.capacitance_n_samples;
      float value = capacitance(n_samples);
      now = microseconds();

      // Stream "capacitance-updated" event to serial interface.
      sprintf((char *)result.data,
              "{\"event\": \"capacitance-updated\", "
              "\"new_value\": %g, "  // Capacitance value
              "\"start\": %lu, \"end\": %lu, "  // start/end times in ms
              "\"n_samples\": %lu, "  // # of analog samples taken for reading
              "\"V_a\": %g}",  // Actuation voltage
              value, start, now, n_samples, _high_voltage);
      result.length = strlen((char *)result.data);

      {
        PacketStream output;
        output.start(Serial, result.length);
        output.write(Serial, (char *)result.data, result.length);
        output.end(Serial);
      }

      capacitance_timestamp_ms_ = millis();
    }

    fast_analog_.update();
    if (watchdog_disable_request_) {
      watchdog_disable_request_ = false;
      watchdog_auto_refresh(false);
      __watchdog_disable__();
    }
    if (dma_channel_done_ >= 0) {
      // DMA channel has completed.
      last_dma_channel_done_ = dma_channel_done_;
      dma_channel_done_ = -1;
      dma_adc_active_ = false;

      // Copy DMA ADC data to serial port as a `STREAM` packet.
      if (dma_data_.length > 0) {
        serial_handler_.receiver_.write_f_(dma_data_,
                                           Packet::packet_type::STREAM,
                                           dma_stream_id_);
      }
    }
  }
  /** Returns current contents of DMA result buffer. */
  UInt8Array dma_data() const { return dma_data_; }

  // ##########################################################################
  // # Accessor methods
  uint32_t D__F_CPU() { return F_CPU; }
  uint32_t D__F_BUS() { return F_BUS; }
  uint32_t V__SYST_CVR() { return SYST_CVR; }
  uint32_t V__SCB_ICSR() { return SCB_ICSR; }
  float adc_period_us() const {
    uint32_t _SYST_CVR = ((adc_SYST_CVR_ < adc_SYST_CVR_prev_)
                          ? adc_SYST_CVR_ + 1000
                          : adc_SYST_CVR_);
    return (compute_timestamp_us(_SYST_CVR, 0) -
            compute_timestamp_us(adc_SYST_CVR_prev_, 0));
  }
  float adc_timestamp_us() const {
    return compute_timestamp_us(adc_SYST_CVR_, adc_millis_);
  }
  uint16_t analog_input_to_digital_pin(uint16_t pin) { return analogInputToDigitalPin(pin); }
  uint32_t benchmark_flops_us(uint32_t N) {
    /*
     * Parameters
     * ----------
     * N : uint32_t
     *     Number of floating point operations to perform.
     *
     * Returns
     * -------
     * uint32_t
     *     Number of microseconds to perform ``N`` floating point operations.
     */
    float a = 1e6;
    float b = 1e7;
    uint32_t start = micros();
    for (uint32_t i = 0; i < N; i++) {
      a /= b;
    }
    return (micros() - start);
  }
  uint32_t benchmark_iops_us(uint32_t N) {
    /*
     * Parameters
     * ----------
     * N : uint32_t
     *     Number of integer operations to perform.
     *
     * Returns
     * -------
     * uint32_t
     *     Number of microseconds to perform ``N`` integer operations.
     */
    uint32_t a = 1e6;
    uint32_t b = 1e7;
    uint32_t start = micros();
    for (uint32_t i = 0; i < N; i++) {
      a /= b;
    }
    return (micros() - start);
  }
  float compute_timestamp_us(uint32_t _SYST_CVR, uint32_t _millis) const {
    uint32_t current = ((F_CPU / 1000) - 1) - _SYST_CVR;
  #if defined(KINETISL) && F_CPU == 48000000
    return _millis * 1000 + ((current * (uint32_t)87381) >> 22);
  #elif defined(KINETISL) && F_CPU == 24000000
    return _millis * 1000 + ((current * (uint32_t)174763) >> 22);
  #endif
    return 1000 * (_millis + current * (1000. / F_CPU));
  }
  float compute_uint16_mean(uint32_t address, uint32_t size) {
    const uint16_t *data = reinterpret_cast<uint16_t *>(address);

    float sum = 0;

    for (uint16_t i = 0; i < size; i++) {
      sum += data[i];
    }
    return sum / size;
  }
  float compute_uint16_rms(uint32_t address, uint32_t size,
                           float mean) {
    const uint16_t *data = reinterpret_cast<uint16_t *>(address);
    float sum_squared = 0;

    for (uint16_t i = 0; i < size; i++) {
      const float data_i = data[i] - mean;
      sum_squared += data_i * data_i;
    }

    return sqrt(sum_squared / float(size));
  }
  float compute_uint16_rms_auto_mean(uint32_t address, uint32_t size) {
    const float mean = compute_uint16_mean(address, size);
    return compute_uint16_rms(address, size, mean);
  }
  float compute_int16_mean(uint32_t address, uint32_t size) {
    const int16_t *data = reinterpret_cast<int16_t *>(address);

    float sum = 0;

    for (int16_t i = 0; i < size; i++) {
      sum += data[i];
    }
    return sum / size;
  }
  float compute_int16_rms(uint32_t address, uint32_t size,
                           float mean) {
    const int16_t *data = reinterpret_cast<int16_t *>(address);
    float sum_squared = 0;

    for (int16_t i = 0; i < size; i++) {
      const float data_i = data[i] - mean;
      sum_squared += data_i * data_i;
    }

    return sqrt(sum_squared / float(size));
  }
  float compute_int16_rms_auto_mean(uint32_t address, uint32_t size) {
    const float mean = compute_int16_mean(address, size);
    return compute_int16_rms(address, size, mean);
  }
  uint16_t digital_pin_has_pwm(uint16_t pin) { return digitalPinHasPWM(pin); }
  uint16_t digital_pin_to_interrupt(uint16_t pin) { return digitalPinToInterrupt(pin); }
  uint16_t dma_channel_count() { return DMA_NUM_CHANNELS; }
  int8_t last_dma_channel_done() const { return last_dma_channel_done_; }
  UInt8Array mem_cpy_device_to_host(uint32_t address, uint32_t size) {
    UInt8Array output;
    output.length = size;
    output.data = (uint8_t *)address;
    return output;
  }
  UInt8Array read_adc_registers(uint8_t adc_num) {
    return teensy::adc::serialize_registers(adc_num, get_buffer());
  }
  UInt8Array read_dma_TCD(uint8_t channel_num) {
    return teensy::dma::serialize_TCD(channel_num, get_buffer());
  }
  void reset_dma_TCD(uint8_t channel_num) {
    teensy::dma::reset_TCD(channel_num);
  }
  UInt8Array read_dma_mux_chcfg(uint8_t channel_num) {
    return teensy::dma::serialize_mux_chcfg(channel_num, get_buffer());
  }
  UInt8Array read_dma_priority(uint8_t channel_num) {
    return teensy::dma::serialize_dchpri(channel_num, get_buffer());
  }
  UInt8Array read_dma_registers() {
    return teensy::dma::serialize_registers(get_buffer());
  }
  UInt8Array read_pit_registers() {
    return teensy::pit::serialize_registers(get_buffer());
  }
  UInt8Array read_pit_timer_config(uint8_t timer_index) {
    return teensy::pit::serialize_timer_config(timer_index, get_buffer());
  }
  UInt8Array read_sim_SCGC6() { return teensy::sim::serialize_SCGC6(get_buffer()); }
  UInt8Array read_sim_SCGC7() { return teensy::sim::serialize_SCGC7(get_buffer()); }

  // ##########################################################################
  // # Mutator methods
  void attach_dma_interrupt(uint8_t dma_channel) {
    void (*isr)(void);
    switch(dma_channel) {
      case 0: isr = &dma_ch0_isr; break;
      case 1: isr = &dma_ch1_isr; break;
      case 2: isr = &dma_ch2_isr; break;
      case 3: isr = &dma_ch3_isr; break;
      case 4: isr = &dma_ch4_isr; break;
      case 5: isr = &dma_ch5_isr; break;
      case 6: isr = &dma_ch6_isr; break;
      case 7: isr = &dma_ch7_isr; break;
      case 8: isr = &dma_ch8_isr; break;
      case 9: isr = &dma_ch9_isr; break;
      case 10: isr = &dma_ch10_isr; break;
      case 11: isr = &dma_ch11_isr; break;
      case 12: isr = &dma_ch12_isr; break;
      case 13: isr = &dma_ch13_isr; break;
      case 14: isr = &dma_ch14_isr; break;
      case 15: isr = &dma_ch15_isr; break;
      default: return;
    }
    _VectorsRam[dma_channel + IRQ_DMA_CH0 + 16] = isr;
    NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + dma_channel);
  }
  void clear_dma_errors() {
    DMA_CERR = DMA_CERR_CAEI;  // Clear All Error Indicators
  }
  void detach_dma_interrupt(uint8_t dma_channel) {
      NVIC_DISABLE_IRQ(IRQ_DMA_CH0 + dma_channel);
  }
  void free_all() {
    while (allocations_.size() > 0) { free((void *)allocations_.shift()); }
    while (aligned_allocations_.size() > 0) {
      aligned_free((void *)aligned_allocations_.shift());
    }
  }
  uint32_t mem_alloc(uint32_t size) {
    uint32_t address = (uint32_t)malloc(size);
    // Save to list of allocations for memory management.
    allocations_.add(address);
    return address;
  }
  uint32_t mem_aligned_alloc(uint32_t alignment, uint32_t size) {
    uint32_t address = (uint32_t)aligned_malloc(alignment, size);
    // Save to list of allocations for memory management.
    aligned_allocations_.add(address);
    return address;
  }
  uint32_t mem_aligned_alloc_and_set(uint32_t alignment, UInt8Array data) {
    // Allocate aligned memory.
    const uint32_t address = mem_aligned_alloc(alignment, data.length);
    if (!address) { return 0; }
    // Copy data to allocated memory.
    mem_cpy_host_to_device(address, data);
    return address;
  }
  void mem_aligned_free(uint32_t address) {
    for (int i = 0; i < aligned_allocations_.size(); i++) {
      if (aligned_allocations_.get(i) == address) {
        aligned_allocations_.remove(i);
      }
    }
    aligned_free((void *)address);
  }
  void mem_cpy_host_to_device(uint32_t address, UInt8Array data) {
    memcpy((uint8_t *)address, data.data, data.length);
  }
  void mem_fill_uint8(uint32_t address, uint8_t value, uint32_t size) {
    mem_fill((uint8_t *)address, value, size);
  }
  void mem_fill_uint16(uint32_t address, uint16_t value, uint32_t size) {
    mem_fill((uint16_t *)address, value, size);
  }
  void mem_fill_uint32(uint32_t address, uint32_t value, uint32_t size) {
    mem_fill((uint32_t *)address, value, size);
  }
  void mem_fill_float(uint32_t address, float value, uint32_t size) {
    mem_fill((float *)address, value, size);
  }
  void mem_free(uint32_t address) {
    for (int i = 0; i < allocations_.size(); i++) {
      if (allocations_.get(i) == address) { allocations_.remove(i); }
    }
    free((void *)address);
  }
  void reset_last_dma_channel_done() { last_dma_channel_done_ = -1; }
  int8_t update_adc_registers(uint8_t adc_num, UInt8Array serialized_adc_msg) {
    return teensy::adc::update_registers(adc_num, serialized_adc_msg);
  }
  int8_t update_dma_mux_chcfg(uint8_t channel_num, UInt8Array serialized_mux) {
    return teensy::dma::update_mux_chcfg(channel_num, serialized_mux);
  }
  int8_t update_dma_registers(UInt8Array serialized_dma_msg) {
    return teensy::dma::update_registers(serialized_dma_msg);
  }
  int8_t update_dma_TCD(uint8_t channel_num, UInt8Array serialized_tcd) {
    return teensy::dma::update_TCD(channel_num, serialized_tcd);
  }
  int8_t update_pit_registers(UInt8Array serialized_pit_msg) {
    return teensy::pit::update_registers(serialized_pit_msg);
  }
  int8_t update_pit_timer_config(uint32_t index,
                                 UInt8Array serialized_config) {
    return teensy::pit::update_timer_config(index, serialized_config);
  }
  int8_t update_sim_SCGC6(UInt8Array serialized_scgc6) {
    return teensy::sim::update_SCGC6(serialized_scgc6);
  }
  int8_t update_sim_SCGC7(UInt8Array serialized_scgc7) {
    return teensy::sim::update_SCGC7(serialized_scgc7);
  }
  /** Start ADC DMA transfers and copy the result as a stream packet to the
   * serial port when transfer has completed.
   *
   * \param pdb_config Programmable delay block status and control register
   *                   configuration.
   * \param addr Address to copy from after DMA transfer operations are
   *             complete.
   * \param size Number of bytes to copy to stream.
   * \param stream_id Identifier for stream packet.
   *
   * \see #loop
   */
  bool start_dma_adc(uint32_t pdb_config, uint32_t addr, uint32_t size,
                     uint16_t stream_id) {
    if (dma_adc_active_) {
        // Another DMA ADC transfer is already in progress.
        return false;
    }
    dma_data_ = UInt8Array_init(size, reinterpret_cast<uint8_t*>(addr));
    dma_stream_id_ = stream_id;
    dma_adc_active_ = true;

    // XXX Load configuration to Programmable Delay Block **but DO NOT start**.
    PDB0_SC = pdb_config & ~PDB_SC_SWTRIG;
    // XXX Trigger start of periodic ADC reads.
    PDB0_SC = pdb_config | PDB_SC_SWTRIG;
    return true;
  }

  // ##########################################################################
  // # Teensy library mutator methods
  int _analogRead(uint8_t pin, int8_t adc_num) {
  //! Returns the analog value of the pin.
  /** It waits until the value is read and then returns the result.
  * If a comparison has been set up and fails, it will return ADC_ERROR_VALUE.
  * This function is interrupt safe, so it will restore the adc to the state it was before being called
  * If more than one ADC exists, it will select the module with less workload, you can force a selection using
  * adc_num. If you select ADC1 in Teensy 3.0 it will return ADC_ERROR_VALUE.
  */
    return adc_->analogRead(pin, adc_num);
  }
  int analogReadContinuous(int8_t adc_num) {
  //! Reads the analog value of a continuous conversion.
  /** Set the continuous conversion with with analogStartContinuous(pin) or startContinuousDifferential(pinP, pinN).
  *   \return the last converted value.
  *   If single-ended and 16 bits it's necessary to typecast it to an unsigned type (like uint16_t),
  *   otherwise values larger than 3.3/2 V are interpreted as negative!
  */
    return adc_->analogReadContinuous(adc_num);
  }
  int analogReadDifferential(uint8_t pinP, uint8_t pinN, int8_t adc_num) {
  //! Reads the differential analog value of two pins (pinP - pinN).
  /** It waits until the value is read and then returns the result.
  * If a comparison has been set up and fails, it will return ADC_ERROR_VALUE.
  * \param pinP must be A10 or A12.
  * \param pinN must be A11 (if pinP=A10) or A13 (if pinP=A12).
  * Other pins will return ADC_ERROR_VALUE.
  *
  * This function is interrupt safe, so it will restore the adc to the state it was before being called
  * If more than one ADC exists, it will select the module with less workload, you can force a selection using
  * adc_num
  */
    return adc_->analogReadDifferential(pinP, pinN, adc_num);
  }
  void setAveraging(uint8_t num, int8_t adc_num) {
    //! Set the number of averages
    /*!
     * \param num can be 0, 4, 8, 16 or 32.
     */
    adc_->setAveraging(num, adc_num);
  }
  void setConversionSpeed(uint8_t speed, int8_t adc_num) {
    //! Sets the conversion speed (changes the ADC clock, ADCK)
    /**
     * \param speed can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED.
     *
     * ADC_VERY_LOW_SPEED is guaranteed to be the lowest possible speed within specs for resolutions less than 16 bits (higher than 1 MHz),
     * it's different from ADC_LOW_SPEED only for 24, 4 or 2 MHz bus frequency.
     * ADC_LOW_SPEED is guaranteed to be the lowest possible speed within specs for all resolutions (higher than 2 MHz).
     * ADC_MED_SPEED is always >= ADC_LOW_SPEED and <= ADC_HIGH_SPEED.
     * ADC_HIGH_SPEED_16BITS is guaranteed to be the highest possible speed within specs for all resolutions (lower or eq than 12 MHz).
     * ADC_HIGH_SPEED is guaranteed to be the highest possible speed within specs for resolutions less than 16 bits (lower or eq than 18 MHz).
     * ADC_VERY_HIGH_SPEED may be out of specs, it's different from ADC_HIGH_SPEED only for 48, 40 or 24 MHz bus frequency.
     *
     * Additionally the conversion speed can also be ADC_ADACK_2_4, ADC_ADACK_4_0, ADC_ADACK_5_2 and ADC_ADACK_6_2,
     * where the numbers are the frequency of the ADC clock (ADCK) in MHz and are independent on the bus speed.
     * This is useful if you are using the Teensy at a very low clock frequency but want faster conversions,
     * but if F_BUS<F_ADCK, you can't use ADC_VERY_HIGH_SPEED for sampling speed.
     *
     */
    adc_->setConversionSpeed((ADC_CONVERSION_SPEED)speed, adc_num);
  }
  void setReference(uint8_t type, int8_t adc_num) {
    //! Set the voltage reference you prefer, default is 3.3 V (VCC)
    /*!
     * \param type can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
     *
     *  It recalibrates at the end.
     */
    adc_->setReference((ADC_REFERENCE)type, adc_num);
  }
  void setResolution(uint8_t bits, int8_t adc_num) {
    //! Change the resolution of the measurement.
    /*
     *  \param bits is the number of bits of resolution.
     *  For single-ended measurements: 8, 10, 12 or 16 bits.
     *  For differential measurements: 9, 11, 13 or 16 bits.
     *  If you want something in between (11 bits single-ended for example) select the inmediate higher
     *  and shift the result one to the right.
     *
     *  Whenever you change the resolution, change also the comparison values (if you use them).
     */
    adc_->setResolution(bits, adc_num);
  }
  void setSamplingSpeed(uint8_t speed, int8_t adc_num) {
    //! Sets the sampling speed
    /** Increase the sampling speed for low impedance sources, decrease it for higher impedance ones.
     * \param speed can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED.
     *
     * ADC_VERY_LOW_SPEED is the lowest possible sampling speed (+24 ADCK).
     * ADC_LOW_SPEED adds +16 ADCK.
     * ADC_MED_SPEED adds +10 ADCK.
     * ADC_HIGH_SPEED (or ADC_HIGH_SPEED_16BITS) adds +6 ADCK.
     * ADC_VERY_HIGH_SPEED is the highest possible sampling speed (0 ADCK added).
     */
    adc_->setSamplingSpeed((ADC_SAMPLING_SPEED)speed, adc_num);
  }
  void disableCompare(int8_t adc_num) {
  //! Disable the compare function
    adc_->disableCompare(adc_num);
  }
  void disableDMA(int8_t adc_num) {
  //! Disable ADC DMA request
    adc_->disableDMA(adc_num);
  }
  void disableInterrupts(int8_t adc_num) {
  //! Disable interrupts
    adc_->disableInterrupts(adc_num);
  }
  void disablePGA(int8_t adc_num) {
  //! Disable PGA
    adc_->disablePGA(adc_num);
  }
  void enableCompare(int16_t compValue, bool greaterThan, int8_t adc_num) {
  //! Enable the compare function to a single value
  /** A conversion will be completed only when the ADC value
  *  is >= compValue (greaterThan=1) or < compValue (greaterThan=0)
  *  Call it after changing the resolution
  *  Use with interrupts or poll conversion completion with isComplete()
  */
    adc_->enableCompare(compValue, greaterThan, adc_num);
  }
  void enableCompareRange(int16_t lowerLimit, int16_t upperLimit, bool insideRange, bool inclusive, int8_t adc_num) {
  //! Enable the compare function to a range
  /** A conversion will be completed only when the ADC value is inside (insideRange=1) or outside (=0)
  *  the range given by (lowerLimit, upperLimit),including (inclusive=1) the limits or not (inclusive=0).
  *  See Table 31-78, p. 617 of the freescale manual.
  *  Call it after changing the resolution
  *  Use with interrupts or poll conversion completion with isComplete()
  */
    adc_->enableCompareRange(lowerLimit, upperLimit, insideRange, inclusive, adc_num);
  }
  void enableDMA(int8_t adc_num) {
  //! Enable DMA request
  /** An ADC DMA request will be raised when the conversion is completed
  *  (including hardware averages and if the comparison (if any) is true).
  */
    adc_->enableDMA(adc_num);
  }
  void enableInterrupts(int8_t adc_num) {
  //! Enable interrupts
  /** An IRQ_ADC0 Interrupt will be raised when the conversion is completed
  *  (including hardware averages and if the comparison (if any) is true).
  */
    adc_->enableInterrupts(adc_num);
  }
  void enablePGA(uint8_t gain, int8_t adc_num) {
  //! Enable and set PGA
  /** Enables the PGA and sets the gain
  *   Use only for signals lower than 1.2 V
  *   \param gain can be 1, 2, 4, 8, 16, 32 or 64
  *
  */
    adc_->enablePGA(gain, adc_num);
  }
  int readSingle(int8_t adc_num) {
  //! Reads the analog value of a single conversion.
  /** Set the conversion with with startSingleRead(pin) or startSingleDifferential(pinP, pinN).
  *   \return the converted value.
  */
    return adc_->readSingle(adc_num);
  }
  bool startContinuous(uint8_t pin, int8_t adc_num) {
  //! Starts continuous conversion on the pin.
  /** It returns as soon as the ADC is set, use analogReadContinuous() to read the value.
  */
    return adc_->startContinuous(pin, adc_num);
  }
  bool startContinuousDifferential(uint8_t pinP, uint8_t pinN, int8_t adc_num) {
  //! Starts continuous conversion between the pins (pinP-pinN).
  /** It returns as soon as the ADC is set, use analogReadContinuous() to read the value.
  * \param pinP must be A10 or A12.
  * \param pinN must be A11 (if pinP=A10) or A13 (if pinP=A12).
  * Other pins will return ADC_ERROR_DIFF_VALUE.
  */
    return adc_->startContinuousDifferential(pinP, pinN, adc_num);
  }
  bool startSingleDifferential(uint8_t pinP, uint8_t pinN, int8_t adc_num) {
  //! Start a differential conversion between two pins (pinP - pinN) and enables interrupts.
  /** It returns inmediately, get value with readSingle().
  *   \param pinP must be A10 or A12.
  *   \param pinN must be A11 (if pinP=A10) or A13 (if pinP=A12).
  *
  *   Other pins will return ADC_ERROR_DIFF_VALUE.
  *   If this function interrupts a measurement, it stores the settings in adc_config
  */
    return adc_->startSingleDifferential(pinP, pinN, adc_num);
  }
  bool startSingleRead(uint8_t pin, int8_t adc_num) {
  //! Starts an analog measurement on the pin and enables interrupts.
  /** It returns inmediately, get value with readSingle().
  *   If the pin is incorrect it returns ADC_ERROR_VALUE
  *   If this function interrupts a measurement, it stores the settings in adc_config
  */
    return adc_->startSingleRead(pin, adc_num);
  }
  void stopContinuous(int8_t adc_num) {
  //! Stops continuous conversion
    adc_->stopContinuous(adc_num);
  }

  // ##########################################################################
  // # Teensy library accessor methods
  uint32_t getMaxValue(int8_t adc_num) {
  //! Returns the maximum value for a measurement: 2^res-1.
    return adc_->getMaxValue(adc_num);
  }
  uint8_t getPGA(int8_t adc_num) {
  //! Returns the PGA level
  /** PGA level = from 1 to 64
  */
    return adc_->getPGA(adc_num);
  }
  uint8_t getResolution(int8_t adc_num) {
  //! Returns the resolution of the ADC_Module.
    return adc_->getResolution(adc_num);
  }
  bool isComplete(int8_t adc_num) {
  //! Is an ADC conversion ready?
  /**
  *  \return 1 if yes, 0 if not.
  *  When a value is read this function returns 0 until a new value exists
  *  So it only makes sense to call it with continuous or non-blocking methods
  */
    return adc_->isComplete(adc_num);
  }
  bool isContinuous(int8_t adc_num) {
  //! Is the ADC in continuous mode?
    return adc_->isContinuous(adc_num);
  }
  bool isConverting(int8_t adc_num) {
  //! Is the ADC converting at the moment?
    return adc_->isConverting(adc_num);
  }
  bool isDifferential(int8_t adc_num) {
  //! Is the ADC in differential mode?
    return adc_->isDifferential(adc_num);
  }

  float select_on_board_test_capacitor(int8_t index) {
    /*
     * Parameters
     * ----------
     * index : int8_t
     *     Index of the on-board test capacitor to activate.
     *
     *     If -1, de-activate all on-board test capacitors.
     *
     * Returns
     * -------
     * float
     *     Currently activated on-board capacitance.
     */
    // XXX De-activate all on-board calibration capacitors first to avoid
    // accidentally activating multiple capacitors at the same time while
    // switching.
    for (uint i = 0; i < 3; i++) {
      digitalWriteFast(i, HIGH);
    }
    if (index >= 0 && index < 3) {
      digitalWriteFast(index, LOW);
    }
    return on_board_capacitance();
  }

  float on_board_capacitance() {
    /*
     * Returns
     * -------
     * float
     *     Currently activated on-board capacitance.
     */
    // Read state of on-capacitor switches.
    // XXX The following pins are **active LOW**.
    const float CAPACITOR_0 = digitalRead(0) ? 0 : 1e-12;
    const float CAPACITOR_1 = digitalRead(1) ? 0 : 10e-12;
    const float CAPACITOR_2 = digitalRead(2) ? 0 : 100e-12;

    return CAPACITOR_0 + CAPACITOR_1 + CAPACITOR_2;
  }

  uint32_t watchdog_time_out_value() const {
    uint32_t result = WDOG_TOVALH << 16;
    result |= WDOG_TOVALL;
    return result;
  }

  uint32_t watchdog_timer_output() const {
    uint32_t result = WDOG_TMROUTH << 16;
    result |= WDOG_TMROUTL;
    return result;
  }

  uint16_t watchdog_reset_count() const { return WDOG_RSTCNT; }
  void watchdog_reset_count_clear(uint16_t mask) { WDOG_RSTCNT = mask; }

  bool watchdog_auto_refresh(bool value) {
    if (value != watchdog_refresh_) {
        watchdog_refresh_ = value;
        return true;
    }
    return false;
  }

  uint16_t C_STARTUP_WDOG_STCTRLH_VALUE() const {
      return STARTUP_WDOG_STCTRLH_VALUE;
  }

  uint16_t R_WDOG_STCTRLH() const { return WDOG_STCTRLH; }

  int8_t watchdog_enable(uint8_t prescaler, uint32_t timeout) {
    noInterrupts();
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");

    if (prescaler > 15) { return -1; }
    WDOG_PRESC = prescaler;  // Set watchdog timer frequency to 1kHz
    WDOG_TOVALL = timeout & 0x0FFFF;  // Set watchdog timeout period to 1 second.
    WDOG_TOVALH = timeout >> 16;
    WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE |
                    WDOG_STCTRLH_WDOGEN);
    interrupts();
    // Enable automatic watchdog refresh by default.
    watchdog_auto_refresh(true);
    return 0;
  }

  void watchdog_disable() {
      watchdog_auto_refresh(false);
      __watchdog_disable__();
  }

  void watchdog_refresh() {
    if (WDOG_TMROUTL >= 2) {
      __watchdog_refresh__();
    }
  }

  void reboot() {
      CPU_RESTART
  }

  void fast_analog_write(uint8_t pin, uint8_t duty_cycle, uint32_t period_us) {
    /* Parameters
     * ----------
     * pin : uint8_t
     *     Pin to toggle.
     * duty_cycle : uint8_t
     *     Waveform duty cycle, 0-255, where 0 is 0% and 255 is 100%.
     * period_us : uint32_t
     *     Waveform period in microseconds.
     */
    fast_analog_.set_pin(pin);
    fast_analog_.configure(duty_cycle, period_us);
  }
};
}  // namespace dropbot


#endif  // #ifndef ___NODE__H___
