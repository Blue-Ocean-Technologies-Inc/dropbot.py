#ifndef ___NODE__H___
#define ___NODE__H___

#include <array>
#include <functional>
#include <math.h>
#include <numeric>
#include <stdint.h>
#include <string.h>
#include <set>
#include <vector>
#include <Arduino.h>
#include <Eigen/Dense>

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
#include "FastAnalogWrite.h"
#include "dropbot_config_validate.h"
#include "dropbot_state_validate.h"
#include "Dropbot/config_pb.h"
#include "Dropbot/state_pb.h"
#include "packet_stream.h"
#include "kxsort.h"
#include "analog.h"
#include "channels.h"
#include "drops.h"
#include "format.h"
#include "voltage_source.h"
#include "Time.h"
#include "Signal.h"
#include "SignalTimer.h"
#include <ADC_Module.h>
#include "SwitchingMatrix.h"

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

/* .. versionadded:: 1.42
 *
 * Add `printf`/`sprintf` float formatting support.
 *
 * See https://forum.pjrc.com/threads/41761-float-to-string-issue?p=131683&viewfull=1#post131683 */
asm(".global _printf_float");
asm(".global _write");

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

using switching_matrix::SwitchingMatrixRowContoller;

const uint32_t EVENT_ACTUATED_CHANNEL_CAPACITANCES = (1 << 31);
const uint32_t EVENT_CHANNELS_UPDATED              = (1 << 30);
const uint32_t EVENT_SHORTS_DETECTED               = (1 << 28);
const uint32_t EVENT_DROPS_DETECTED                = (1 << 27);
const uint32_t EVENT_ENABLE                        = (1 << 0);

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

/**
* @brief Public interface for DropBot control board.
*
* \version 1.63 refactor to use `voltage_source` namespace functions.
*/
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
public:
  typedef PacketParser<FixedPacket> parser_t;

  static const uint32_t BUFFER_SIZE = 8192;  // >= longest property string

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

  static const float R6;

  static constexpr uint8_t CMD_GET_SENSITIVE_CHANNELS = 0xAA;
  static constexpr uint8_t CMD_GET_SWITCHING_MATRIX_ROW = 0xA7;
  static constexpr uint8_t CMD_SET_SENSITIVE_CHANNELS = 0xA9;
  static constexpr uint8_t CMD_SET_SWITCHING_MATRIX_ROW = 0xA6;
  static constexpr uint8_t PCA9505_CONFIG_IO_REGISTER_ = 0x18;
  static constexpr uint8_t PCA9505_OUTPUT_PORT_REGISTER_ = 0x08;
  static constexpr uint8_t CMD_GET_SENSITIVE_OFFSET = 0xAB;

  static constexpr uint8_t CMD_GET_CHANNEL_DUTY_CYCLE = 0xAC;
  static constexpr uint8_t CMD_SET_CHANNEL_DUTY_CYCLE = 0xAD;

  uint8_t buffer_[BUFFER_SIZE];

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
  Channels channels_;
  std::array<drops::ChannelNeighbours, MAX_NUMBER_OF_CHANNELS>
      channel_neighbours_;
  base_node_rpc::FastAnalogWrite fast_analog_;
  std::vector<std::vector<uint8_t> > drops_;

  // Detect chip insertion/removal.
  OutputEnableDebounce output_enable_input;

  //: .. versionadded:: 1.42
  //
  // Time of most recent capacitance measurement.
  uint32_t capacitance_timestamp_ms_;

  uint8_t target_count_;
  //: .. versionadded:: 1.53
  //
  // Time of most recent drops detection.
  uint32_t drops_timestamp_ms_;

  /**
  * @brief Chip status changed event.
  *
  * Sent when chip is inserted or removed.
  *
  * \version added: 1.58
  */
  Signal<std::function<void(bool /* chip inserted */)> > chip_status_changed_;
  /**
  * @brief Capacitance measured signal.
  *
  * Sent when a capacitance measurement has been read.
  *
  * \version added: 1.59
  */
  Signal<std::function<void(float /* capacitance */,
                            float /* actuation voltage */)> >
      capacitance_measured_;
  /**
  * @brief Chip load feedback saturated signal.
  *
  * Sent when chip load feedback voltage is within the specified margin from
  * either the high end or the low end of the input range.
  *
  * \version added: 1.61
  * \version changed: 1.69
  *     Change arguments to _signed_ integers.
  */
  Signal<std::function<void(int16_t, int16_t)> > chip_load_saturated_;
  /**
  * @brief High-side current exceeded signal.
  *
  * Sent when output current measurement exceeds specified threshold.
  *
  * \version added: 1.60
  */
  Signal<std::function<void(float)> > high_side_current_exceeded_;
  /**
  * @brief Time-based signal callback handler.
  *
  * Current time is updated on every `loop()` iteration.
  *
  * \version added: 1.57
  */
  SignalTimer signal_timer_ms_;
  /**
  * @brief Switching matrix controller.
  *
  * Updated on every `loop()` iteration.
  *
  * \version added: 2.0
  */
  SwitchingMatrixRowContoller matrix_controller_;

  /**
  * @brief Construct node.
  *
  * \version 1.58
  *     Send `output_enabled`/`output_disabled` serial event when change in
  *     chip status occurs and remains stable for 1 second.
  *     Connect callback to `chip_status_changed_` event to send
  *     `output_enabled`/`output_disabled` serial events.
  *     Run shorts detection whenever a chip is inserted.
  *
  * \version 1.59
  *     Measure **capacitance** every 25 ms and send `capacitance_measured_`
  *     signal.  Connect to `capacitance_measured_` to evaluate capacitance
  *     update events.
  *
  * \version 1.60
  *   Periodically measure **output RMS current** and send
  *   `high_side_current_exceeded_` signal when current threshold is exceeded.
  *   Connect callbacks to `high_side_current_exceeded_` signal to halt (i.e.,
  *   disable all channels and turn off high-voltage) and send `halted` serial
  *   event.
  *
  * \version 1.61
  *   Periodically measure **chip load feedback voltage** and send
  *   `chip_load_saturated_` signal when measured voltage is within the
  *   specified margin from either the high end or the low end of the ADC input
  *   range. Connect callbacks to `chip_load_saturated_` signal to halt (i.e.,
  *   disable all channels and turn off high-voltage) and send `halted` serial
  *   event.
  *
  * \version 2.0
  *   Send `sensitive-capacitances` serial event when \f$\vec{y}\f$ is computed
  *   by the switching matrix controller (i.e., at the end of each loop through
  *   the switching matrix).
  *
  * \version 2.1.0
  *   Disable chip load saturation check if saturation margin (i.e.,
  *   `state_._.chip_load_range_margin`) is negative.
  */
  Node() : BaseNode(),
           BaseNodeConfig<config_t>(dropbot_Config_fields),
           BaseNodeState<state_t>(dropbot_State_fields),
           adc_period_us_(0), adc_timestamp_us_(0), adc_tick_tock_(false),
           adc_count_(0), dma_adc_active_(false), dma_channel_done_(-1),
           last_dma_channel_done_(-1), adc_read_active_(false),
           dma_stream_id_(0), watchdog_disable_request_(false),
           channels_(0, dropbot_Config_switching_board_i2c_address_default),
           output_enable_input(*this, voltage_source::OE_PIN, 1000,
                               InputDebounce::PinInMode::PIM_EXT_PULL_UP_RES,
                               0),
           capacitance_timestamp_ms_(0), target_count_(0),
           drops_timestamp_ms_(0), matrix_controller_(40, 5e3) {
    pinMode(LED_BUILTIN, OUTPUT);
    dma_data_ = UInt8Array_init_default();
    clear_neighbours();

    // Send `output_enabled`/`output_disabled` event when change in chip status
    // occurs.
    chip_status_changed_.connect([] (bool chip_inserted) {
      PacketStream output;
      stream_byte_type message[] = "{\"event\": \"%s\"}";

      // Compute length of buffer required to store message.
      // See https://stackoverflow.com/a/35036033/345236
      const auto length = snprintf(NULL, 0, message, (chip_inserted) ?
                                   "output_enabled" : "output_disabled");

      char data[length];
      sprintf(data, message, (chip_inserted) ? "output_enabled" :
              "output_disabled");
      output.start(Serial, length);
      output.write(Serial, reinterpret_cast<stream_byte_type *>(&data[0]),
                   length);
      output.end(Serial);
    });

    // Run shorts detection whenever a chip is inserted.
    chip_status_changed_.connect([&](bool chip_inserted) {
      if (chip_inserted) {
        detect_shorts(5);
      }
    });

    // Measure capacitance every 25 ms and send `capacitance_measured_` signal.
    signal_timer_ms_.connect([&] (auto now) {
      if (state_._.hv_output_enabled && state_._.hv_output_selected) {
        // High-voltage output is enabled and selected.
        float value = this->capacitance(config_._.capacitance_n_samples);
        capacitance_measured_.send(value, analog::high_voltage_);
      }
    }, 25);

    capacitance_measured_.connect([&] (float capacitance,
                                       float actuation_voltage) {
      // Send event if target capacitance has been set and exceeded.
      if (state_._.target_capacitance > 0) {
        if (capacitance >= state_._.target_capacitance) {
          target_count_ += 1;
        } else {
          target_count_ = 0;
          return;
        }

        UInt8Array result = get_buffer();
        result.length = 0;

        uint32_t time_us = microseconds();

        if (target_count_ >= state_._.target_count) {
          // Target capacitance has been met.

          // Stream "capacitance-updated" event to serial interface.
          result.length =
            sprintf((char *)result.data,
                    "{\"event\": \"capacitance-exceeded\", "
                    "\"new_value\": %g, "  // Capacitance value
                    "\"target\": %g, "  // Target capacitance value
                    "\"time_us\": %lu, "  // end times in us
                    "\"n_samples\": %lu, "  // # of analog samples
                    "\"count\": %d, "  // # of consecutive readings > target
                    "\"V_a\": %g}",  // Actuation voltage
                    capacitance, state_._.target_capacitance, time_us,
                    config_._.capacitance_n_samples, target_count_,
                    actuation_voltage);

          // Reset target capacitance.
          state_._.target_capacitance = 0;

          {
            PacketStream output;
            output.start(Serial, result.length);
            output.write(Serial, (char *)result.data, result.length);
            output.end(Serial);
          }
        }
      }
    });

    capacitance_measured_.connect([&] (float capacitance,
                                       float actuation_voltage) {
      unsigned long now = millis();
      // Send periodic capacitance value update events.
      if ((state_._.capacitance_update_interval_ms > 0) &&
          (state_._.capacitance_update_interval_ms < now -
           capacitance_timestamp_ms_)) {
        UInt8Array result = get_buffer();
        result.length = 0;

        uint32_t time_us = microseconds();

        // Stream "capacitance-updated" event to serial interface.
        sprintf((char *)result.data,
                "{\"event\": \"capacitance-updated\", "
                "\"new_value\": %g, "  // Capacitance value
                "\"time_us\": %lu, "  // end times in us
                "\"n_samples\": %lu, "  // # of analog samples taken
                "\"V_a\": %g}",  // Actuation voltage
                capacitance, time_us, config_._.capacitance_n_samples,
                actuation_voltage);
        result.length = strlen((char *)result.data);

        {
          PacketStream output;
          output.start(Serial, result.length);
          output.write(Serial, (char *)result.data, result.length);
          output.end(Serial);
        }

        capacitance_timestamp_ms_ = millis();
      }
    });

    // XXX Connect periodic callback to check output current.
    signal_timer_ms_.connect([&] (auto now) {
      const float output_current = analog::measure_output_current_rms(20);
      if (output_current > state_._.output_current_limit) {
        high_side_current_exceeded_.send(output_current);
      }
    }, 1000);  // Check current every 1 second

    // XXX Halt (i.e., disable all channels and turn off high-voltage) when
    // current threshold is exceeded.
    high_side_current_exceeded_.connect([&] (float current) { halt(); });

    // Publish `halted` event when current threshold is exceeded.
    high_side_current_exceeded_.connect([&] (float current) {
      UInt8Array buffer = this->get_buffer();
      buffer.length = 0;

      buffer.length += sprintf((char *)&buffer.data[buffer.length],
                              "{\"event\": \"halted\", \"wall_time\": %.6f, "
                               "\"error\": {\"name\": "
                               "\"output-current-exceeded\", "
                               "\"output_current\": %g}}", time::wall_time(),
                               current);

      {
        PacketStream output_packet;
        output_packet.start(Serial, buffer.length);
        output_packet.write(Serial, (char *)buffer.data,
                            buffer.length);
        output_packet.end(Serial);
      }
    });

    // Measure chip load voltage every 25 ms.  If measured voltage is within
    // specified margin at high end or low end of analog input range, send
    // `chip_load_saturated_` signal.  If margin is negative, **disable chip
    // load saturation check**.
    signal_timer_ms_.connect([&] (auto now) {
      if (state_._.hv_output_enabled && state_._.hv_output_selected &&
          (state_._.chip_load_range_margin >= 0)) {
        analog::adc_context([&] (auto adc_config) {
          // High-voltage output is enabled and selected.
          const uint8_t resolution = analog::adc_.adc[0]->getResolution();
          // Maximum analog value is (2^15 - 1) for differential 16-bit
          // resolution, since the 16th bit of the analog return type
          // represents the sign. For resolutions less than 16-bit, a 16-bit
          // data type is still used so (2^resolution - 1) is the maximum
          // analog value.
          const int16_t max_analog =
            ((resolution == 16) ? (1L << 15) : (1L << resolution)) - 1;
          analog::adc_.adc[0]->setReference(ADC_REFERENCE::REF_1V2);
          analog::adc_.adc[0]->disablePGA();

          const int16_t chip_load =
            analog::adc_.adc[0]->analogReadDifferential(A10, A11);
          const int16_t margin = (state_._.chip_load_range_margin *
                                  static_cast<float>(max_analog));
          if ((chip_load > (max_analog - margin))
              || (chip_load < (margin - max_analog))) {
            chip_load_saturated_.send(chip_load, margin);
          }
        });
      }
    }, 25);

    // XXX Halt (i.e., disable all channels and turn off high-voltage) when
    // chip load analog measurement has saturated (either upper or lower end of
    // analog scale).
    chip_load_saturated_.connect([&] (int16_t chip_load, int16_t margin) {
      halt();
    });

    // Publish `halted` event when chip load analog measurement is saturated.
    chip_load_saturated_.connect([&] (int16_t chip_load, int16_t margin) {
      UInt8Array buffer = this->get_buffer();
      buffer.length = 0;

      buffer.length += sprintf((char *)&buffer.data[buffer.length],
                              "{\"event\": \"halted\", \"wall_time\": %f, "
                               "\"error\": {\"name\": "
                               "\"chip-load-saturated\", "
                               "\"chip_load\": %d, \"margin\": %d}}",
                               time::wall_time(),
                               static_cast<int>(chip_load),
                               static_cast<int>(margin));

      {
        PacketStream output_packet;
        output_packet.start(Serial, buffer.length);
        output_packet.write(Serial, (char *)buffer.data,
                            buffer.length);
        output_packet.end(Serial);
      }
    });

    matrix_controller_.sensitive_capacitances_.connect(
      [&] (auto channels, auto y) {
        // Sensitive capacitances have been updated.  Send serial stream event.
      UInt8Array buffer = this->get_buffer();
      buffer.length = 0;

      buffer.length += sprintf((char *)&buffer.data[buffer.length],
                               "{\"event\": \"sensitive-capacitances\", "
                               "\"wall_time\": %lu, "
                               "\"C\": [", time::wall_time());

      for (auto i = 0; i < channels.size(); i++) {
        if (i > 0) {
          buffer.length += sprintf((char *)&buffer.data[buffer.length], ", ");
        }
        buffer.length += sprintf((char *)&buffer.data[buffer.length],
                                 "[%d, %g]", channels[i], y(i, 0));
      }
      buffer.length += sprintf((char *)&buffer.data[buffer.length], "]}");

      {
        PacketStream output_packet;
        output_packet.start(Serial, buffer.length);
        output_packet.write(Serial, (char *)buffer.data,
                            buffer.length);
        output_packet.end(Serial);
      }

    });
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

  /**
  * @brief Write an array of bytes to the specified device on the software I2C
  * bus.
  *
  * @param address  Address of device on software I2C bus.
  * @param data  Array of bytes to send to device.
  *
  *
  * \version 1.63 refactor to use voltage_source::i2c
  */
  void soft_i2c_write(uint8_t address, UInt8Array data) {
    using namespace voltage_source;

    i2c.beginTransmission(address);
    i2c.write(data.data, data.length);
    i2c.endTransmission();
  }

  float measure_temperature() { return analog::measure_temperature(); }

  float measure_aref() { return analog::measure_aref(); }

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
    return analog::u16_percentile_diff(pin, n_samples, low_percentile,
                                       high_percentile);
  }

  uint16_t s16_percentile_diff(uint8_t pinP, uint8_t pinN, uint16_t n_samples,
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
     * pinP : uint8_t
     *     Positive pin.
     * pinN : uint8_t
     *     Negative pin.
     * pinN  Negative pin.
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
    uint16_t result = 0;
    analog::adc_context([&] (auto adc_config) {
      auto &adc = *analog::adc_.adc[0];  // Use ADC 0.
      adc.setResolution(16);
      adc.setReference(ADC_REFERENCE::REF_1V2);
      adc.wait_for_cal();

      result = analog::s16_percentile_diff(pinP, pinN, n_samples,
                                           low_percentile, high_percentile);
    });
    return result;
  }

  /**
  * @brief Measure high-side *root mean-squared (RMS)* voltage.
  *
  * \version 1.51  Cache most recent RMS voltage as `_high_voltage`.
  *
  * \version 1.53  Deprecate cached `_high_voltage`.  Most recent value is now
  * cached as `analog::high_voltage_`.
  *
  * @return High-side RMS voltage.
  */
  float high_voltage() {
    return analog::high_voltage();
  }

  /**
  * @brief Measure high-side *root mean-squared (RMS)* output current.
  *
  * @return  High-side RMS current.
  */
  float output_current_rms() {
    return analog::measure_output_current_rms(20);
  }

  /**
  * @brief Measure high-side output current.
  *
  * @return  High-side maximum current.
  */
  float output_current() {
    return analog::measure_output_current(20);
  }

  /**
  * @brief Measure *root mean-squared (RMS)* input current.
  *
  * @return  RMS input current.
  */
  float input_current_rms() {
    return analog::measure_input_current_rms(20);
  }

  /**
  * @brief Measure input current.
  *
  * @return  Maximum input current.
  */
  float input_current() {
    return analog::measure_input_current(20);
  }

  UInt16Array analog_reads_simple(uint8_t pin, uint16_t n_samples) {
    UInt16Array output;
    output.data = reinterpret_cast<uint16_t *>(get_buffer().data);
    output.length = n_samples;
    std::generate(output.data, output.data + output.length,
                  [&] () { return analogRead(pin); });
    return output;
  }

  /**
  * @brief Read samples from differential pair of pins as fast as possible.
  *
  * @param pinP  Positive pin.
  * @param pinN  Negative pin.
  * @param n_samples  Number of samples to read.
  *
  * @return Array of signed integer differential reading.
  */
  Int16Array differential_reads_simple(uint8_t pinP, uint8_t pinN,
                                       uint16_t n_samples) {
    Int16Array output;
    output.data = reinterpret_cast<int16_t *>(get_buffer().data);
    output.length = n_samples;
    std::generate(output.data, output.data + output.length,
                  [&] () { return analogReadDifferential(pinP, pinN, 0); });
    return output;
  }

  /**
  * @brief Read an array of bytes from the specified device on the software I2C
  * bus.
  *
  * @param address  Address of device on software I2C bus.
  * @param n_bytes_to_read  Number of bytes to read from device.
  *
  * @return  Array of bytes received from device.
  *
  * \version 1.63 refactor to use voltage_source::i2c
  */
  UInt8Array soft_i2c_read(uint8_t address, uint8_t n_bytes_to_read) {
    using namespace voltage_source;

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


  /**
  * @brief Query addresses of available devices on the software I2C bus.
  *
  * @return  Array of addresses of discovered devices.
  *
  * \version 1.63 refactor to use voltage_source::i2c
  */
  UInt8Array soft_i2c_scan() {
    using namespace voltage_source;

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
  *
  * \version 1.64  Delegate selection of 3.3 V output (and restoration of
  *     original selection) to voltage_source::detect_shorts().
  */
  UInt8Array detect_shorts(uint8_t delay_ms) {
    /*
     * .. versionchanged:: 1.53
     *     Send ``shorts-detected`` event stream packet containing:
     *      - ``"channels"``: list of identifiers of shorted channels.
     */
    UInt8Array shorts = get_buffer();
    {
      // Restrict scope of `detected_shorts` to free up memory after use.
      auto detected_shorts = channels_.detect_shorts(delay_ms);
      std::copy(detected_shorts.begin(), detected_shorts.end(), shorts.data);
      shorts.length = detected_shorts.size();
    }

    if (event_enabled(EVENT_SHORTS_DETECTED)) {
      // Shorts-detected event is enabled.
      UInt8Array buffer =
        UInt8Array_init(0, &shorts.data[shorts.length]);

      // Stream `shorts-detected` event packet.
      buffer.length += sprintf((char *)&buffer.data[buffer.length],
                               "{\"event\": \"shorts-detected\", "
                               "\"values\": [");

      for (int i = 0 ; i < shorts.length; i++) {
        if (i > 0) {
          buffer.length += sprintf((char *)&buffer.data[buffer.length], ", ");
        }
        buffer.length += sprintf((char *)&buffer.data[buffer.length], "%d",
                                 shorts.data[i]);
      }

      buffer.length += sprintf((char *)&buffer.data[buffer.length], "]}");

      {
        PacketStream output_packet;
        output_packet.start(Serial, buffer.length);
        output_packet.write(Serial, (char *)buffer.data,
                            buffer.length);
        output_packet.end(Serial);
      }
    }

    return shorts;
  }

  /**
  * @brief Measure short detection voltage for each channel.
  *
  * Apply low voltage (3.3 V) to each channel in isolation.  Record measured
  * voltage on `A0`.  If voltage is ~3.3 V, the corresponding channel is not
  * shorted to ground.
  *
  * @param delay_ms  Amount of time to wait after sending updated channel
  *     states to switching boards before measuring applied voltage.
  *
  * @return Array of measured voltages, one per channel.
  *
  * \since 1.64
  */
  UInt16Array short_detection_voltages(uint8_t delay_ms) {
    UInt16Array voltages;
    voltages.data = reinterpret_cast<uint16_t *>(get_buffer().data);

    {
      // Restrict scope of `detected_shorts` to free up memory after use.
      auto short_voltages = channels_.short_detection_voltages(delay_ms);
      std::copy(short_voltages.begin(), short_voltages.end(), voltages.data);
      voltages.length = short_voltages.size();
    }

    return voltages;
  }

  /**
  * @return Minimum target high voltage output.
  *
  * \version 1.63  Refactor to use voltage_source::min_waveform_voltage()
  */
  float min_waveform_voltage() {
    return voltage_source::min_waveform_voltage();
  }

  /**
  * @brief Set target high voltage.
  *
  * @param voltage  Target high voltage.
  *
  * @return `true` if voltage set successfully.
  *
  * \version 1.63  Refactor to use voltage_source::_set_voltage()
  */
  bool _set_voltage(float voltage) {
    return voltage_source::_set_voltage(voltage);
  }

  uint16_t initialize_switching_boards();

  // # Callback methods

  /**
  * @brief Callback function called when `state_._.target_capacitance` has been
  * changed.
  *
  * Reset the number of times the target capacitance has been exceeded, since
  * the target capacitance has changed.
  *
  * \version 1.53  Add function.
  *
  * @param target_capacitance
  *
  * @return
  */
  bool on_state_target_capacitance_changed(float target_capacitance) {
    target_count_ = 0;
    return true;
  }

  /**
  * @param frequency  Target actuation frequency.
  *
  * @return  `true` if frequency successfully set.
  *
  * \version 1.63  Refactor to use voltage_source::set_frequency()
  */
  bool on_state_frequency_changed(float frequency) {
    return voltage_source::set_frequency(frequency);
  }

  /**
  * @param voltage  Target actuation voltage.
  *
  * @return  `true` if voltage successfully set.
  *
  * \version 1.63  Refactor to use voltage_source::_set_voltage()
  */
  bool on_state_voltage_changed(float voltage) {
    return voltage_source::_set_voltage(voltage);
  }

  /**
  * @param value  If `true`, turn on high voltage driver.
  *
  * \version 1.63  Refactor to use
  *   voltage_source::enable_high_voltage_output() and
  *   voltage_source::disable_high_voltage_output()
  */
  bool on_state_hv_output_enabled_changed(bool value) {
    if (value) {
      voltage_source::enable_high_voltage_output();
    } else {
      voltage_source::disable_high_voltage_output();
    }
    return true;
  }

  /**
  * @param value  If `true`, select high voltage output.  Otherwise, select
  *     3.3 V output.
  *
  * \version 1.63  Refactor to use voltage_source::select_output()
  */
  bool on_state_hv_output_selected_changed(bool value) {
    const uint8_t output = (value ? voltage_source::OUTPUT_HIGH_VOLTAGE
                            : voltage_source::OUTPUT_3V3);
    return voltage_source::select_output(output);
  }

  bool on_state_channel_count_changed(int32_t value) {
      // XXX This value is ready-only.
      return false;
  }

  /**
  * @brief Update voltage_source::R7 when respective config value is changed.
  *
  * @param value
  *
  * \since 1.63
  */
  bool on_config_R7_changed(float value) {
    voltage_source::R7 = value;
    return true;
  }

  /**
  * @brief Update voltage_source::pot_max when respective config value is
  *     changed.
  *
  * @param value
  *
  * \since 1.63
  */
  bool on_config_pot_max_changed(float value) {
    voltage_source::pot_max = value;
    return true;
  }

  /**
  * @brief Update voltage_source::max_voltage when respective config value is
  *     changed.
  *
  * @param value
  *
  * \since 1.63
  */
  bool on_config_max_voltage_changed(float value) {
    voltage_source::max_voltage = value;
    return true;
  }

  /**
  * @brief Update voltage_source::min_frequency when respective config value is
  *     changed.
  *
  * @param value
  *
  * \since 1.63
  */
  bool on_config_min_frequency_changed(float value) {
    voltage_source::min_frequency = value;
    return true;
  }

  /**
  * @brief Update voltage_source::max_frequency when respective config value is
  *     changed.
  *
  * @param value
  *
  * \since 1.63
  */
  bool on_config_max_frequency_changed(float value) {
    voltage_source::max_frequency = value;
    return true;
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
  *
  * \version 1.51.2
  *     Only emit capacitance events is high voltage is both **selected** and
  *     **enabled**.
  *
  * \version 1.51.3
  *     Fix the `capacitance-updated` event handling:
  *       - Fix truncated JSON message due to a misplaced "," typo in the
  *         arguments of a call to `sprintf` (bug introduced in version 1.51).
  *       - Set the capacitance update counter using the _millisecond_ counter
  *         instead of _microsecond_ counter.
  *
  * \version 1.53
  *     Require capacitance to exceed target capacitance for multiple
  *     consecutive readings before triggering a `capacitance-exceeded` event.
  *     The number of required consecutive readings is set by
  *     `state_._.target_count`.
  *
  * \version 1.57
  *     Update `signal_timer_ms_` with current time, triggering any waiting
  *     callbacks with expired intervals.
  */
  void loop() {
    unsigned long now = millis();

    // poll button state
    output_enable_input.process(now);

    // Update signal timer counter.
    signal_timer_ms_.update(now);

    fast_analog_.update();
    if (watchdog_disable_request_) {
      watchdog_disable_request_ = false;
      watchdog_auto_refresh(false);
      __watchdog_disable__();
    }
    if (state_._.drops_update_interval_ms > 0 &&
        (state_._.drops_update_interval_ms < now - drops_timestamp_ms_) &&
        event_enabled(EVENT_DROPS_DETECTED)) {
      refresh_drops(0);
      drops_timestamp_ms_ = millis();
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
    matrix_controller_.update(*this, SwitchingMatrixRowContoller::TICK,
                              micros());
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
  /**
  * @brief Serialize ADC configuration registers.
  *
  * \See analog_load_config()
  *
  * @param adc_num  Zero-based ADC index.
  *
  * @return  Serialized ADC configuration containing the contents of the
  *   following 32-bit registers (in order): `SC1A`, `SC2`, `SC3`, `CFG1`,
  *   `CFG2`.
  */
  UInt32Array analog_save_config(uint8_t adc_num) {
    auto result = get_type_buffer<UInt32Array>();

    if (adc_num >= ADC_NUM_ADCS) {
      result.length = 0;
    } else {
      auto &config = *reinterpret_cast<ADC_Module::ADC_Config *>(result.data);
      config = {0};
      result.length = sizeof(config) / sizeof(uint32_t);
      analog::adc_.adc[adc_num]->saveConfig(&config);
    }
    return result;
  }

  /**
  * @brief Apply a serialized configuration to ADC registers.
  *
  * **Note: individual mutator functions (e.g., `setAveraging`, etc.) are
  * called to make sure other internal state of `ADC_Module` is kept up to
  * date.**  In contrast, if `ADC_Module::loadConfig()` was used directly, the
  * `ADC_Module` internal state would become stale if the ADC register values
  * changed.
  *
  * @param adc_num  Zero-based ADC index.
  * @param config  Serialized ADC configuration containing the contents of the
  *   following 32-bit registers (in order): `SC1A`, `SC2`, `SC3`, `CFG1`,
  *   `CFG2`.
  */
  void analog_load_config(UInt32Array config, uint8_t adc_num) {
    if (adc_num < ADC_NUM_ADCS && (config.length ==
                                   sizeof(ADC_Module::ADC_Config) /
                                   sizeof(uint32_t))) {
      auto &_config = *reinterpret_cast<ADC_Module::ADC_Config *>(config.data);
      analog::load_config(_config, adc_num);
    }
  }

  /**
  * @brief Programmable gain amplifier (PGA) state.
  *
  * @param adc_num  Zero-based ADC index.
  *
  * @return  `true` if PGA is enabled.
  */
  bool isPGAEnabled(uint8_t adc_num) {
    if (adc_num < ADC_NUM_ADCS) {
      return analog::adc_.adc[adc_num]->isPGAEnabled();
    }
    return false;
  }

  /**
  * @brief Wait for analog calibration to complete.
  *
  * @param adc_num  Zero-based ADC index.
  */
  void analog_wait_for_calibration(uint8_t adc_num) {
    if (adc_num < ADC_NUM_ADCS) {
      analog::adc_.adc[adc_num]->wait_for_cal();
    }
  }

  uint8_t _analog_reference(UInt32Array config) {
    if (config.length == sizeof(ADC_Module::ADC_Config) / sizeof(uint32_t)) {
      auto &adc_config = *reinterpret_cast<ADC_Module::ADC_Config *>(config.data);
      return static_cast<uint8_t>(analog::_analog_reference(adc_config));
    }
    return 255;
  }

  uint8_t _sampling_speed(UInt32Array config) {
    if (config.length == sizeof(ADC_Module::ADC_Config) / sizeof(uint32_t)) {
      auto &adc_config = *reinterpret_cast<ADC_Module::ADC_Config *>(config.data);
      return static_cast<uint8_t>(analog::_sampling_speed(adc_config));
    }
    return static_cast<uint8_t>(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
  }


  uint8_t _conversion_speed(UInt32Array config) {
    if (config.length == sizeof(ADC_Module::ADC_Config) / sizeof(uint32_t)) {
      auto &adc_config = *reinterpret_cast<ADC_Module::ADC_Config *>(config.data);
      return static_cast<uint8_t>(analog::_conversion_speed(adc_config));
    }
    return static_cast<uint8_t>(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
  }

  uint8_t _averaging(UInt32Array config) {
    if (config.length == sizeof(ADC_Module::ADC_Config) / sizeof(uint32_t)) {
      auto &adc_config = *reinterpret_cast<ADC_Module::ADC_Config *>(config.data);
      return static_cast<uint8_t>(analog::_averaging(adc_config));
    }
    return 0;
  }

  int _analogRead(uint8_t pin, int8_t adc_num) {
  //! Returns the analog value of the pin.
  /** It waits until the value is read and then returns the result.
  * If a comparison has been set up and fails, it will return ADC_ERROR_VALUE.
  * This function is interrupt safe, so it will restore the adc to the state it was before being called
  * If more than one ADC exists, it will select the module with less workload, you can force a selection using
  * adc_num. If you select ADC1 in Teensy 3.0 it will return ADC_ERROR_VALUE.
  */
    return analog::adc_.analogRead(pin, adc_num);
  }
  int analogReadContinuous(int8_t adc_num) {
  //! Reads the analog value of a continuous conversion.
  /** Set the continuous conversion with with analogStartContinuous(pin) or startContinuousDifferential(pinP, pinN).
  *   \return the last converted value.
  *   If single-ended and 16 bits it's necessary to typecast it to an unsigned type (like uint16_t),
  *   otherwise values larger than 3.3/2 V are interpreted as negative!
  */
    return analog::adc_.analogReadContinuous(adc_num);
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
    return analog::adc_.analogReadDifferential(pinP, pinN, adc_num);
  }
  void setAveraging(uint8_t num, int8_t adc_num) {
    //! Set the number of averages
    /*!
     * \param num can be 0, 4, 8, 16 or 32.
     */
    analog::adc_.setAveraging(num, adc_num);
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
    analog::adc_.setConversionSpeed((ADC_CONVERSION_SPEED)speed, adc_num);
  }
  void setReference(uint8_t type, int8_t adc_num) {
    //! Set the voltage reference you prefer, default is 3.3 V (VCC)
    /*!
     * \param type can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
     *
     *  It recalibrates at the end.
     */
    analog::adc_.setReference((ADC_REFERENCE)type, adc_num);
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
    analog::adc_.setResolution(bits, adc_num);
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
    analog::adc_.setSamplingSpeed((ADC_SAMPLING_SPEED)speed, adc_num);
  }
  void disableCompare(int8_t adc_num) {
  //! Disable the compare function
    analog::adc_.disableCompare(adc_num);
  }
  void disableDMA(int8_t adc_num) {
  //! Disable ADC DMA request
    analog::adc_.disableDMA(adc_num);
  }
  void disableInterrupts(int8_t adc_num) {
  //! Disable interrupts
    analog::adc_.disableInterrupts(adc_num);
  }
  void disablePGA(int8_t adc_num) {
  //! Disable PGA
    analog::adc_.disablePGA(adc_num);
  }
  void enableCompare(int16_t compValue, bool greaterThan, int8_t adc_num) {
  //! Enable the compare function to a single value
  /** A conversion will be completed only when the ADC value
  *  is >= compValue (greaterThan=1) or < compValue (greaterThan=0)
  *  Call it after changing the resolution
  *  Use with interrupts or poll conversion completion with isComplete()
  */
    analog::adc_.enableCompare(compValue, greaterThan, adc_num);
  }
  void enableCompareRange(int16_t lowerLimit, int16_t upperLimit, bool insideRange, bool inclusive, int8_t adc_num) {
  //! Enable the compare function to a range
  /** A conversion will be completed only when the ADC value is inside (insideRange=1) or outside (=0)
  *  the range given by (lowerLimit, upperLimit),including (inclusive=1) the limits or not (inclusive=0).
  *  See Table 31-78, p. 617 of the freescale manual.
  *  Call it after changing the resolution
  *  Use with interrupts or poll conversion completion with isComplete()
  */
    analog::adc_.enableCompareRange(lowerLimit, upperLimit, insideRange, inclusive, adc_num);
  }
  void enableDMA(int8_t adc_num) {
  //! Enable DMA request
  /** An ADC DMA request will be raised when the conversion is completed
  *  (including hardware averages and if the comparison (if any) is true).
  */
    analog::adc_.enableDMA(adc_num);
  }
  void enableInterrupts(int8_t adc_num) {
  //! Enable interrupts
  /** An IRQ_ADC0 Interrupt will be raised when the conversion is completed
  *  (including hardware averages and if the comparison (if any) is true).
  */
    analog::adc_.enableInterrupts(adc_num);
  }
  void enablePGA(uint8_t gain, int8_t adc_num) {
  //! Enable and set PGA
  /** Enables the PGA and sets the gain
  *   Use only for signals lower than 1.2 V
  *   \param gain can be 1, 2, 4, 8, 16, 32 or 64
  *
  */
    analog::adc_.enablePGA(gain, adc_num);
  }
  int readSingle(int8_t adc_num) {
  //! Reads the analog value of a single conversion.
  /** Set the conversion with with startSingleRead(pin) or startSingleDifferential(pinP, pinN).
  *   \return the converted value.
  */
    return analog::adc_.readSingle(adc_num);
  }
  bool startContinuous(uint8_t pin, int8_t adc_num) {
  //! Starts continuous conversion on the pin.
  /** It returns as soon as the ADC is set, use analogReadContinuous() to read the value.
  */
    return analog::adc_.startContinuous(pin, adc_num);
  }
  bool startContinuousDifferential(uint8_t pinP, uint8_t pinN, int8_t adc_num) {
  //! Starts continuous conversion between the pins (pinP-pinN).
  /** It returns as soon as the ADC is set, use analogReadContinuous() to read the value.
  * \param pinP must be A10 or A12.
  * \param pinN must be A11 (if pinP=A10) or A13 (if pinP=A12).
  * Other pins will return ADC_ERROR_DIFF_VALUE.
  */
    return analog::adc_.startContinuousDifferential(pinP, pinN, adc_num);
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
    return analog::adc_.startSingleDifferential(pinP, pinN, adc_num);
  }
  bool startSingleRead(uint8_t pin, int8_t adc_num) {
  //! Starts an analog measurement on the pin and enables interrupts.
  /** It returns inmediately, get value with readSingle().
  *   If the pin is incorrect it returns ADC_ERROR_VALUE
  *   If this function interrupts a measurement, it stores the settings in adc_config
  */
    return analog::adc_.startSingleRead(pin, adc_num);
  }
  void stopContinuous(int8_t adc_num) {
  //! Stops continuous conversion
    analog::adc_.stopContinuous(adc_num);
  }

  // ##########################################################################
  // # Teensy library accessor methods
  uint32_t getMaxValue(int8_t adc_num) {
  //! Returns the maximum value for a measurement: 2^res-1.
    return analog::adc_.getMaxValue(adc_num);
  }
  uint8_t getPGA(int8_t adc_num) {
  //! Returns the PGA level
  /** PGA level = from 1 to 64
  */
    return analog::adc_.getPGA(adc_num);
  }
  uint8_t getResolution(int8_t adc_num) {
  //! Returns the resolution of the ADC_Module.
    return analog::adc_.getResolution(adc_num);
  }
  bool isComplete(int8_t adc_num) {
  //! Is an ADC conversion ready?
  /**
  *  \return 1 if yes, 0 if not.
  *  When a value is read this function returns 0 until a new value exists
  *  So it only makes sense to call it with continuous or non-blocking methods
  */
    return analog::adc_.isComplete(adc_num);
  }
  bool isContinuous(int8_t adc_num) {
  //! Is the ADC in continuous mode?
    return analog::adc_.isContinuous(adc_num);
  }
  bool isConverting(int8_t adc_num) {
  //! Is the ADC converting at the moment?
    return analog::adc_.isConverting(adc_num);
  }
  bool isDifferential(int8_t adc_num) {
  //! Is the ADC in differential mode?
    return analog::adc_.isDifferential(adc_num);
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

  float capacitance(uint16_t n_samples) {
    n_samples = n_samples ? n_samples : config_._.capacitance_n_samples;
    return channels_.capacitance(n_samples);
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

  FloatArray scatter_channels_capacitances(UInt8Array channels) {
    auto capacitances =
      channels_.scatter_channels_capacitances(std::set<uint8_t>(channels.data,
                                                                channels.data +
                                                                channels
                                                                .length),
                                              config_._.capacitance_n_samples);

    FloatArray result;
    result.data = reinterpret_cast<float *>(get_buffer().data);
    result.length = capacitances.size();
    std::copy(capacitances.begin(), capacitances.end(), result.data);
    return result;
  }

  UInt8Array get_channels_drops(UInt8Array channels, float c_threshold) {
    /*
    * Parameters
    * ----------
    * channels : UInt8Array
    *     List of channels to measure for drop detection.
    * c_threshold : float
    *     Minimum capacitance (in farads) to consider as liquid present on a
    *     channel electrode.
    *
    *     If set to 0, a default of 3 pF is used.
    *
    * Returns
    * -------
    * UInt8Array
    *     List of channels where threshold capacitance was met, grouped by
    *     contiguous electrode regions (i.e., sets of electrodes that are
    *     connected by neighbours where capacitance threshold was also met).
    *
    *     Format:
    *
    *         [drop 0 channel count][drop 0: channel 0, channel 1, ...][drop 1 channel count][drop 1: channel 0, channel 1, ...]
    */
    std::set<uint8_t> channels_v(channels.data, channels.data +
                                 channels.length);
    auto drops = get_channels_drops(channels_v, c_threshold);
    UInt8Array result = UInt8Array_init(0, get_buffer().data);
    drops::pack_drops(drops, result);
    return result;
  }

  template <typename T>
  std::vector<std::vector<uint8_t> > get_channels_drops(T channels, float c_threshold) {
    /*
    * Parameters
    * ----------
    * channels : STL container
    *     Channels to measure for drop detection - **MUST** be sorted.
    * c_threshold : float
    *     Minimum capacitance (in farads) to consider as liquid present on a
    *     channel electrode.
    *
    *     If set to 0, a default of 3 pF is used.
    *
    * Returns
    * -------
    * std::vector<std::vector<uint8_t> >
    *     List of channels where threshold capacitance was met, grouped by
    *     contiguous electrode regions (i.e., sets of electrodes that are
    *     connected by neighbours where capacitance threshold was also met).
    */
    c_threshold = c_threshold ? c_threshold : drops::C_THRESHOLD;
    const unsigned long start = microseconds();

    // Only measure capacitance of specified channels.
    std::vector<float> capacitances =
        channels_.scatter_channels_capacitances(channels, config_._
                                                .capacitance_n_samples);
    auto drops = drops::get_drops(channel_neighbours_, capacitances, channels,
                                  c_threshold);
    const unsigned long end = microseconds();

    if (event_enabled(EVENT_DROPS_DETECTED)) {
      /*
      * Stream `drops-detected` event in the form:
      *
      *     {"event": "drops-detected",
      *      "drops": {"channels": [[<drop 0 ch 0>, <drop 0 ch 1>, ...],
      *                             [<drop 1 ch 0>, <drop 1 ch 1>, ...], ...],
      *                "capacitances": [[<drop 0 cap 0>, <drop 0 cap 1>, ...],
      *                                 [<drop 1 cap 0>, <drop 1 cap 1>, ...],
      *                                 ...]},
      *      "start": <start microseconds>, "end": <end microseconds>}
      */
      UInt8Array buffer = UInt8Array_init(0, get_buffer().data);
      sprintf_drops_detected(capacitances, drops, start, end, buffer);

      {
        PacketStream output;
        output.start(Serial, buffer.length);
        output.write(Serial, reinterpret_cast<char *>(buffer.data),
                     buffer.length);
        output.end(Serial);
      }
    }
    return drops;
  }

  UInt8Array get_all_drops(float c_threshold) {
    /*
    * Parameters
    * ----------
    * c_threshold : float
    *     Minimum capacitance (in farads) to consider as liquid present on a
    *     channel electrode.
    *
    *     If set to 0, a default of 3 pF is used.
    *
    * Returns
    * -------
    * UInt8Array
    *     List of channels where threshold capacitance was met, grouped by
    *     contiguous electrode regions (i.e., sets of electrodes that are
    *     connected by neighbours where capacitance threshold was also met).
    *
    *     Format:
    *
    *         [drop 0 channel count][drop 0: channel 0, channel 1, ...][drop 1 channel count][drop 1: channel 0, channel 1, ...]
    */
    // Fill `channels` with range `(0, <channel_count_>)`.
    std::vector<uint8_t> channels(state_._.channel_count);
    std::iota(channels.begin(), channels.end(), 0);
    drops_ = get_channels_drops(channels, c_threshold);

    UInt8Array result = UInt8Array_init(0, get_buffer().data);
    drops::pack_drops(drops_, result);
    return result;
  }

  void refresh_drops(float c_threshold) {
    if (drops_.size() > 0) {
      drops_ = get_channels_drops(drops::drop_channels(drops_,
                                                       channel_neighbours_),
                                  c_threshold);
    } else {
      get_all_drops(c_threshold);
    }
  }

  UInt8Array drops() {
    UInt8Array result = UInt8Array_init(0, get_buffer().data);
    drops::pack_drops(drops_, result);
    return result;
  }

  UInt8Array neighbours() {
    UInt8Array result = get_buffer();
    result.length = 4 * MAX_NUMBER_OF_CHANNELS;

    // Copy channel neighbour assignments from array to
    memcpy(result.data, reinterpret_cast<uint8_t *>(&channel_neighbours_[0]),
           result.length);
    return result;
  }

  void clear_neighbours() {
    mem_fill(reinterpret_cast<uint8_t *>(&channel_neighbours_[0]),
             static_cast<uint8_t>(255), 4 * MAX_NUMBER_OF_CHANNELS);
  }

  int8_t assign_neighbours(UInt8Array packed_channel_neighbours) {
    if (packed_channel_neighbours.length != 4 * MAX_NUMBER_OF_CHANNELS) {
      // Invalid payload size.
      return -1;
    }
    for (uint32_t i = 0; i < 4 * MAX_NUMBER_OF_CHANNELS; i++) {
      const uint8_t channel_i = packed_channel_neighbours.data[i];
      if ((channel_i > MAX_NUMBER_OF_CHANNELS - 1) && (channel_i != 255)) {
        // Invalid channel number.
        return -2;
      }
    }
    // Copy channel neighbour assignments from array to
    memcpy(reinterpret_cast<uint8_t *>(&channel_neighbours_[0]),
           packed_channel_neighbours.data,
           packed_channel_neighbours.length);
    return 0;
  }

  FloatArray all_channel_capacitances() {
    // Fill `channels` with range `(0, <channel_count_>)`.
    std::vector<uint8_t> channels(state_._.channel_count);
    std::iota(channels.begin(), channels.end(), 0);
    return channel_capacitances(UInt8Array_init(channels.size(),
                                                &channels[0]));
  }

  UInt8Array all_channels() {
    // Fill `channels` with range `(0, <channel_count_>)`.
    std::vector<uint8_t> channels(state_._.channel_count);
    std::iota(channels.begin(), channels.end(), 0);
    UInt8Array result = get_buffer();
    std::copy(channels.begin(), channels.end(), result.data);
    result.length = channels.size();
    return result;
  }

  /**
  * @brief Read chip load capacitance associated with each specified channel.
  *
  * @param channels  Array of channel numbers corresponding to channels to
  *     measure.
  *
  * @return Array of measured capacitances, one per specified channel.
  *
  * \version 1.63  Refactor to use voltage_source namespace functions.
  */
  FloatArray channel_capacitances(UInt8Array channels) {
    using namespace voltage_source;

    // High voltage (HV) output is required to perform capacitance
    // measurements.
    // XXX Since selecting and enabling the high voltage output take about
    // ~200 ms each, only select and enable it if necessary.
    bool restore_required = false;
    if (!high_voltage_output_enabled()) {
      enable_high_voltage_output();
      restore_required = true;
    }
    if (selected_output() != OUTPUT_HIGH_VOLTAGE) {
      select_output(OUTPUT_HIGH_VOLTAGE);
      restore_required = true;
    }

    // Disable events while measuring capacitances.  This prevents noise and
    // improves performance by not sending an event for each channel that is
    // set during the capacitance scan.
    auto events_disabled = disable_events();
    auto capacitances =
        channels_.channel_capacitances(channels.data, channels.data +
                                       channels.length,
                                       config_._.capacitance_n_samples);
    FloatArray output_capacitances;
    output_capacitances.length = capacitances.size();
    output_capacitances.data = reinterpret_cast<float *>(get_buffer().data);
    std::copy(capacitances.begin(), capacitances.end(),
              output_capacitances.data);
    if (events_disabled) { enable_events(); }

    // Restore the original high-votage (HV) output state.
    if (restore_required) {
      on_state_hv_output_selected_changed(state_._.hv_output_selected);
      on_state_hv_output_enabled_changed(state_._.hv_output_enabled);
    }

    return output_capacitances;
  }

  float _benchmark_channel_update(uint32_t count) {
    auto events_disabled = disable_events();
    float seconds_per_update = channels_._benchmark_channel_update(count);
    if (events_disabled) { enable_events(); }
    return seconds_per_update;
  }

  float _benchmark_capacitance(uint16_t n_samples, uint32_t count) {
    return channels_._benchmark_capacitance(n_samples, count);
  }

  UInt8Array state_of_channels() {
    auto &state_of_channels = channels_.state_of_channels();
    UInt8Array output = get_buffer();
    output.length = state_of_channels.size();
    std::copy(state_of_channels.begin(), state_of_channels.end(), output.data);
    return output;
  }

  void set_disabled_channels_mask(UInt8Array disabled_channels_mask) {
    // Copy from raw array to packed channels vector.
    std::vector<uint8_t> disabled_mask(disabled_channels_mask.data,
                                       disabled_channels_mask.data +
                                       disabled_channels_mask.length);
    channels_.set_disabled_channels_mask(disabled_mask);
  }

  UInt8Array disabled_channels_mask() {
    auto &mask = channels_.disabled_channels_mask();
    UInt8Array output = get_buffer();
    output.length = mask.size();
    std::copy(mask.begin(), mask.end(), output.data);
    return output;
  }

  bool set_state_of_channels(UInt8Array channel_states) {
    /*
     * .. versionchanged:: 1.53
     *     Send ``channels-updated`` event stream packet containing:
     *      - ``"n"``: number of actuated channel
     *      - ``"actuated"``: list of actuated channel identifiers.
     *      - ``"start"``: millisecond counter before setting shift registers
     *      - ``"end"``: millisecond counter after setting shift registers
     */
    const unsigned long start = microseconds();
    {
      Channels::packed_channels_t states;

      if (channel_states.length != states.size()) {
        return false;
      }
      std::copy(channel_states.data, channel_states.data +
                channel_states.length, states.begin());
      channels_.set_state_of_channels(states);
    }
    const unsigned long end = microseconds();

    if (event_enabled(EVENT_CHANNELS_UPDATED)) {
      // Stream `channels-updated` event packet.
      UInt8Array result = get_buffer();
      char * const data = reinterpret_cast<char *>(result.data);

      result.length = sprintf(data, "{\"event\": \"channels-updated\", "
                              "\"actuated\": [");

      auto actuated_channels = channels_.actuated_channels();

      // XXX LSB of chip 0 and port 0 is channel 0.
      for (auto i = 0; i < actuated_channels.size(); i++) {
        if (i > 0) {
          result.length += sprintf(&data[result.length], ", ");
        }
        result.length += sprintf(&data[result.length], "%d",
                                 actuated_channels[i]);
      }

      result.length += sprintf(&data[result.length], "], \"start\": %lu, "
                               "\"end\": %lu, \"n\": %d}", start, end,
                               actuated_channels.size());

      {
        PacketStream output;
        output.start(Serial, result.length);
        output.write(Serial, (char *)result.data, result.length);
        output.end(Serial);
      }
    }
    return true;
  }

  float benchmark_analog_read(uint8_t pin, uint32_t n_samples) {
    return analog::benchmark_analog_read(pin, n_samples);
  }

  float benchmark_u16_percentile_diff(uint8_t pin, uint16_t n_samples,
                                      float low_percentile,
                                      float high_percentile,
                                      uint32_t n_repeats) {
    return analog::benchmark_u16_percentile_diff(pin, n_samples,
                                                 low_percentile,
                                                 high_percentile, n_repeats);
  }

  float benchmark_s16_percentile_diff(uint8_t pinP, uint8_t pinN,
                                      uint16_t n_samples,
                                      float low_percentile,
                                      float high_percentile,
                                      uint32_t n_repeats) {
    return analog::benchmark_s16_percentile_diff(pinP, pinN, n_samples,
                                                 low_percentile,
                                                 high_percentile, n_repeats);
  }

  bool event_enabled(uint32_t event) {
    return (state_._.event_mask & (event | EVENT_ENABLE)) == (event |
                                                              EVENT_ENABLE);
  }
  void enable_event(uint32_t event) { state_._.event_mask |= event; }
  bool disable_event(uint32_t event) {
    if (state_._.event_mask & event) {
      state_._.event_mask &= ~event;
      return true;
    }
    return false;
  }
  bool disable_events() {
    if (state_._.event_mask & EVENT_ENABLE) {
      state_._.event_mask &= ~EVENT_ENABLE;
      return true;
    }
    return false;
  }
  void enable_events() { state_._.event_mask |= EVENT_ENABLE; }

  /**
  * @brief Synchronize specified timestamp to current milliseconds counter.
  *
  * @param wall_time  Unix timestamp, i.e., seconds since the epoch; January 1,
  * 1970 00:00:00.
  *
  * @return  Milliseconds count synced against.
  *
  * \See wall_time()
  *
  * \Version added: 1.55
  */
  uint32_t sync_time(double wall_time) { return time::sync_time(wall_time); }
  /**
  * @brief Return last synchronized time plus the number of seconds since time
  * was synced.
  *
  * @return  Number of seconds since time was last synchronized plus the
  * synchronized time.
  *
  * \See sync_time()
  *
  * \Version added: 1.55
  */
  double wall_time() { return time::wall_time(); }
  /**
   * @brief Turn off all channels.
   *
   * \version added: 1.56
   */
  void turn_off_all_channels() { channels_.turn_off_all_channels(); }
  /**
   * @brief Disable all channels.
   *
   * \version added: 1.56
   */
  void disable_all_channels() { channels_.disable_all_channels(); }
  /**
  * @brief Disable all channels and disable high voltage.
  *
  * Emergency shutdown of all high-voltage actuation.  Useful, e.g., when
  * current limit has been exceeded.
  *
  * \see high_side_current_exceeded_
  *
  * \version added: 1.56
  */
  void halt() {
    // Disable high voltage.
    state_._.hv_output_enabled = false;
    on_state_hv_output_enabled_changed(false);
    // Turn off all channels.
    channels_.disable_all_channels();
  }

  float _benchmark_eigen_inverse_double(uint32_t N, uint32_t repeats) {
    /*
    *  Benchmark Eigen library for computing $Y = (S_T S)^{-1} S_T M$
    *  using `double` matrix data types.
    */
    return _benchmark_eigen_inverse<double>(N, repeats);
  }

  float _benchmark_eigen_inverse_float(uint32_t N, uint32_t repeats) {
    /*
    *  Benchmark Eigen library for computing $Y = (S_T S)^{-1} S_T M$
    *  using `float` matrix data types.
    */
    return _benchmark_eigen_inverse<float>(N, repeats);
  }

  template <typename Float>
  float _benchmark_eigen_inverse(uint32_t N, uint32_t repeats) {
    /*
    * Benchmark Eigen library for computing $Y = (S_T S)^{-1} S_T M$
    *
    * where:
    *
    *  - $S$ is a $N \times N$ switching matrix encoding the actuation state of
    *    each channel during each measurement window in a measuring sequence,
    *    such that each row of $S$ corresponds to a window within a measurement
    *    period and each column corresponds to a _sensitive_ channel;
    *  - $Y$ is a $N \times 1$ matrix encoding the electrical admittance of
    *    each channel (where admittance is the inverse of the impedance) during
    *    each measurement period;
    *  - $M$ is a $N \times 1$ matrix M, containing a combined measurement for
    *    each row in $S$, corresponding to the _actuated_ channels in the row.
    *    Note that $SY = M$.
    *
    * Benchmarking steps:
    *
    *  1. Create a representative $N \times N$ switching matrix, $S$.
    *  2. Create a representative electrical admittance $N \times 1$ matrix, $Y$.
    *  3. Compute a mock capacitance measurement $N \times 1$ matrix, $M$,
    *     containing simulated a combined measurement for each row in $S$,
    *     corresponding to the actuated channels in the row.
    *  4. Compute $S_T S$.
    *  5. **Compute the inverse of $S S_T$** in 3 loops, repeating ``repeats``
    *     times in each loop and recording the duration of each loop.
    *  6. Return the minimum duration of a single inverse computation.
    */
    using namespace Eigen;

    typedef Matrix<Float, Dynamic, Dynamic> MatrixF;

    MatrixXi S = MatrixXi::Ones(N, N);

    for (auto i = 0; i < S.cols(); i++) {
        S(i, i) = 0;
    }

    MatrixF Y(S.cols(), 1);

    for (auto i = 0; i < Y.rows(); i++) {
        Y(i) = i + 1;
    }

    MatrixF M = S.cast<Float>() * Y;

    auto S_TS = (S.transpose() * S).cast<Float>();

    std::array<uint32_t, 3> durations;
    std::fill(durations.begin(), durations.end(),
              std::numeric_limits<uint32_t>::max());

    for (auto it_duration = durations.begin(); it_duration != durations.end();
         it_duration++) {
      auto &duration = *it_duration;

      auto start = micros();
      for (uint32_t i = 0; i < repeats; i++) {
        S_TS.inverse();
      }
      auto end = micros();
      duration = end - start;
    }
    return (float(*std::min_element(durations.begin(), durations.end())) /
            durations.size() * 1e-6);
  }

  void set_sensitive_channels(UInt8Array packed_channels) {
    matrix_controller_.reset();
    matrix_controller_.sensitive_channels(packed_channels.data,
                                          packed_channels.data +
                                          packed_channels.length);

    constexpr uint8_t CMD_SET_SENSITIVE_CHANNELS = 0xA9;
    i2c_safe([&] () {
      // Broadcast message to **all** switching boards, using _general call_
      // address, 0.
      Wire.beginTransmission(0);
      Wire.write(CMD_SET_SENSITIVE_CHANNELS);
      Wire.write(packed_channels.data, packed_channels.length);
      Wire.endTransmission();
      delayMicroseconds(100); // needed when using Teensy
    });
  }

  UInt8Array get_sensitive_channels() {
    auto packed_channels = get_buffer();
    packed_channels.length = 0;

    i2c_safe([&] () {
      // Broadcast sensitive channels request to all switching boards.
      Wire.beginTransmission(0);
      Wire.write(CMD_GET_SENSITIVE_CHANNELS);
      Wire.endTransmission();

      delayMicroseconds(100); // needed when using Teensy

      // Read response from each board individually.
      for (int board_id = 0; board_id < 3; board_id++) {
        const auto i2c_address = (channels_.switching_board_i2c_address_ +
                                  board_id);
        //  1. Request response length from switching board.
        Wire.requestFrom(i2c_address, 1);
        if (Wire.available()) {
          auto response_length = Wire.read();
          //  2. Request response payload (do not include return code) from
          //     switching board.
          //   - XXX Response in form:
          //
          //         [payload:<uint8_t[]>][return code:<uint8_t>]
          Wire.requestFrom(i2c_address, response_length - 1);
          while (Wire.available()) {
            auto port_i = Wire.read();
            packed_channels.data[packed_channels.length++] = port_i;
          }
        }
      }
    });
    return packed_channels;
  }

  UInt8Array get_sensitive_offsets() {
    auto sensitive_offsets = get_buffer();
    sensitive_offsets.length = 0;

    i2c_safe([&] () {
      // Broadcast sensitive offsets request to all switching boards.
      Wire.beginTransmission(0);
      Wire.write(CMD_GET_SENSITIVE_OFFSET);
      Wire.endTransmission();

      delayMicroseconds(100); // needed when using Teensy

      // Read response from each board individually.
      for (int board_id = 0; board_id < 3; board_id++) {
        const auto i2c_address = (channels_.switching_board_i2c_address_ +
                                  board_id);
        Wire.requestFrom(i2c_address, 1);
        if (Wire.available()) {
          auto payload_length = Wire.read();
          Wire.requestFrom(i2c_address, payload_length);
          payload_length = Wire.available();
          if (payload_length > 0) {
            auto sensitive_offset = Wire.read();
            sensitive_offsets.data[sensitive_offsets.length++] = sensitive_offset;
          }
        }
      }
    });
    return sensitive_offsets;
  }

  /**
  * @brief Apply duty cycle to **all** channels.
  *
  * @param duty_cycle  Duty cycle (in range $[0..1]$ inclusive).
  */
  void set_all_duty_cycles(float duty_cycle) {
    auto channels = get_buffer();
    channels.length = MAX_NUMBER_OF_CHANNELS;
    std::iota(channels.data, channels.data + channels.length, 0);
    set_duty_cycle(duty_cycle, channels);
  }

  /**
  * @brief Apply duty cycle to list of channels.
  *
  * @param duty_cycle  Duty cycle (in range $[0..1]$ inclusive).
  * @param channels  List of channel identifiers to which duty cycle should be
  *   applied.
  */
  void set_duty_cycle(float duty_cycle, UInt8Array channels) {
    matrix_controller_.sensitive_duty_cycles(duty_cycle, channels.data,
                                             channels.data + channels.length);
    // Process channels in chunks to avoid overflowing maximum I2C message
    // length.
    auto chunks_count = static_cast<uint8_t>(ceil(channels.length / 22.));
    auto chunk_size = static_cast<uint8_t>(ceil(channels.length /
                                                static_cast<float>
                                                (chunks_count)));
    i2c_safe([&] () {
      // Send each chunk of channels list in separate I2C message.
      for (auto chunk_i = 0; chunk_i < chunks_count; chunk_i++) {
        auto start_i = chunk_i * chunk_size;
        // Last chunk *may* be smaller than the rest if list channels does not
        // divide evenly by the chunk size.
        auto chunk_size_i = ((chunk_i + 1 < chunks_count) ? chunk_size
                            : chunk_size - (chunks_count * chunk_size -
                                            channels.length));

        // Broadcast sensitive offsets request to all switching boards.
        Wire.beginTransmission(0);
        Wire.write(CMD_SET_CHANNEL_DUTY_CYCLE);
        Wire.write(reinterpret_cast<uint8_t *>(&duty_cycle), sizeof(duty_cycle));
        Wire.write(&channels.data[start_i], chunk_size_i);
        Wire.endTransmission();
      }
    });
  }

  FloatArray sensitive_duty_cycles() {
    auto duty_cycles = matrix_controller_.sensitive_duty_cycles();
    return copy_to_buffer<FloatArray, decltype(duty_cycles.begin())>
        (duty_cycles.begin(), duty_cycles.end());
  }

  UInt8Array sensitive_channels() {
    auto channels = matrix_controller_.sensitive_channels();
    return copy_to_buffer<UInt8Array, decltype(channels.begin())>
        (channels.begin(), channels.end());
  }

  uint32_t switching_matrix_row_count() {
    return matrix_controller_.row_count();
  }

  void set_switching_matrix_row_count(uint32_t row_count) {
    matrix_controller_.reset(row_count);
  }

  UInt8Array S() {
    auto S_ = matrix_controller_.S();
    return copy_to_buffer<UInt8Array, decltype(S_.data())>
        (S_.data(), S_.data() + S_.size());
  }

  FloatArray OMEGA() {
    auto OMEGA_ = matrix_controller_.OMEGA();
    return copy_to_buffer<FloatArray, decltype(OMEGA_.data())>
        (OMEGA_.data(), OMEGA_.data() + OMEGA_.size());
  }

  FloatArray y() {
    auto y_ = matrix_controller_.y();
    return copy_to_buffer<FloatArray, decltype(y_.data())>
        (y_.data(), y_.data() + y_.size());
  }

  FloatArray m() {
    auto m_ = matrix_controller_.m();
    return copy_to_buffer<FloatArray, decltype(m_.data())>
        (m_.data(), m_.data() + m_.size());
  }

  /**
  * @brief Return buffer cast to specified `CArrayDefs` array type.
  *
  * @tparam T  `CArrayDefs` array type, e.g., `FloatArray`, `UInt16Array`.
  *
  * @return  `get_buffer()` buffer cast as specified array type.
  *
  * Example
  * -------
  *
  * ```c++
  * // Length is re-calculated and set according to atom (e.g., `float`) size.
    FloatArray result = get_type_buffer<FloatArray>();
  * ```
  */
  template <typename T>
  T get_type_buffer() {
    T result;
    auto buffer = get_buffer();
    result.data = reinterpret_cast<decltype(result.data)>(buffer.data);
    result.length = buffer.length / sizeof(result.data[0]);
    return result;
  }

  /**
  * @brief Copy from iterator range to typed CArrayDefs array.
  *
  * Data of returned CArrayDefs instance is held in shared buffer returned by
  * `Node::get_buffer()`.  This method is useful, e.g., to return an arbitrary
  * data range from an RPC method call.
  *
  * For example, see `Node::S()`.
  *
  * @tparam Array  CArrayDefs array type (e.g., `FloatArray`).
  * @param begin  Beginning of source.
  * @param end  End of source.
  *
  * @return  CArrayDefs array, with data stored in shared `Node` buffer.
  */
  template <typename Array, typename Iterator>
  Array copy_to_buffer(Iterator begin, Iterator end) {
    auto result = get_type_buffer<Array>();
    result.length = end - begin;
    std::copy(begin, end, result.data);
    return result;
  }

  /**
  * @brief Wrapper function to provide safe access to I2C bus.
  *
  * @param func  Function (e.g., lambda) to call while in safe I2C state.
  *
  * Example
  * -------
  *
  * ```c++
  * uint8_t matrix_row = 3;
  * uint8_t row_count = 10;
  *
  * i2c_safe([&] () {
  *   Wire.beginTransmission(0);
  *   Wire.write(CMD_SET_SWITCHING_MATRIX_ROW);
  *   Wire.write(matrix_row);
  *   Wire.write(row_count);
  *   Wire.endTransmission();
  * });
  * ```
  */
  template <typename Func>
  void i2c_safe(Func func) {
    // XXX Stop the timer (which toggles the HV square-wave driver) during i2c
    // communication.
    //
    // See https://gitlab.com/sci-bots/dropbot.py/issues/26
    if (state_._.hv_output_enabled) {
      Timer1.stop(); // stop the timer during i2c transmission
    }
    func();
    if (state_._.hv_output_enabled) {
      Timer1.start(); // stop the timer during i2c transmission
    }
  }

  /**
  * @brief Duty cycle for specified electrode.
  *
  * @param channel  Channel number.
  *
  * @return Duty cycle, in range $[0..1]$ inclusive.
  */
  float get_duty_cycle(uint8_t channel) {
    float duty_cycle = -1;
    i2c_safe([&] () {
      // Broadcast duty cycle request to all switching boards.
      Wire.beginTransmission(0);
      Wire.write(CMD_GET_CHANNEL_DUTY_CYCLE);
      Wire.write(channel);
      Wire.endTransmission();

      delayMicroseconds(200); // needed when using Teensy

      // Read response from each board individually.
      for (int board_id = 0; board_id < 3; board_id++) {
        const auto i2c_address = (channels_.switching_board_i2c_address_ +
                                  board_id);
        Wire.requestFrom(i2c_address, 1);
        if (Wire.available()) {
          auto payload_length = Wire.read();
          if (payload_length > 0) {
            Wire.requestFrom(i2c_address, payload_length);
            payload_length = Wire.available();
            if (payload_length >= sizeof(duty_cycle)) {
              auto duty_cycle_bytes = reinterpret_cast<uint8_t *>(&duty_cycle);
              for (auto i = 0; i < sizeof(duty_cycle); i++) {
                duty_cycle_bytes[i] = Wire.read();
              }
              break;
            }
          }
        }
      }
    });
    return duty_cycle;
  }

  /**
  * @brief Set active switching matrix row.
  *
  * @param matrix_row_i  Row index.
  * @param row_count  Number of rows in matrix (required to compute state based
  *   on duty cycle).
  */
  void set_switching_matrix_row(uint8_t matrix_row_i, uint8_t row_count) {
    i2c_safe([&] () {
      // Broadcast duty cycle request to all switching boards.
      Wire.beginTransmission(0);
      Wire.write(CMD_SET_SWITCHING_MATRIX_ROW);
      Wire.write(matrix_row_i);
      Wire.write(row_count);
      Wire.endTransmission();
    });
  }

  /**
  * @brief Automatically start cycling through switching matrix rows.
  *
  * @param row_count  Number of rows in matrix (required to compute state based
  *   on duty cycle).
  * @param t_settling_s  Minimum number of seconds between row activations.
  */
  void start_switching_matrix(uint32_t row_count, float t_settling_s) {
    matrix_controller_.reset(row_count, t_settling_s * 1e6);
    matrix_controller_.update(*this, SwitchingMatrixRowContoller::START,
                              micros());
  }

  /**
  * @brief Resume cycling through switching matrix rows (after stop).
  *
  * Uses the same row count and settling time the matrix was started with.
  */
  void resume_switching_matrix() {
    start_switching_matrix(matrix_controller_.row_count(),
                           1e-6 * matrix_controller_.t_settling());
  }

  /**
  * @brief Stop cycling through switching matrix rows.
  */
  void stop_switching_matrix() {
    matrix_controller_.stop(*this);
  }

  /**
  * @brief Compute time to cycle through switching matrix.
  *
  * @param row_count  Number of rows in matrix.
  * @param delay_s  Delay (in seconds) between row activations.
  * @param repeats  Number of times to cycle through switching matrix.
  *
  * @return Total time elapsed (in seconds).
  */
  float _benchmark_switching_matrix_row(uint8_t row_count, float delay_s,
                                        uint32_t repeats) {
    auto start = micros();
    for (auto i = 0; i < repeats; i++) {
      for (auto j = 0; j < row_count; j++) {
        set_switching_matrix_row(j, row_count);
        delayMicroseconds(delay_s * 1e6);
      }
    }
    auto end = micros();
    return (end - start) * 1e-6;
  }

  /**
  * @brief Compute time to cycle through switching matrix; measure capacitance
  * for each row; and compute individual capacitance for each channel.
  *
  * @param row_count  Number of rows in matrix.
  * @param delay_s  Delay (in seconds) between row activations.
  * @param repeats  Number of times to cycle through switching matrix.
  * @param measure_C  If `true`, measure capacitance for each row.
  * @param send_signal  If `true`, send signal each time final row has
  *   completed, including the computed capacitance for each channel.
  *
  * @return Total time elapsed (in seconds).
  */
  float _benchmark_switching_matrix_controller(uint8_t row_count,
                                               float delay_s,
                                               uint32_t repeats,
                                               bool measure_C,
                                               bool send_signal) {
    {
      UInt8Array packed_channels = get_buffer();
      packed_channels.length = 5;
      std::fill(packed_channels.data, packed_channels.data +
                packed_channels.length, 0);
      packed_channels.data[0] = 0b00001111;
      stop_switching_matrix();
      turn_off_all_channels();
      set_sensitive_channels(packed_channels);
    }

    UInt8Array channels = get_buffer();

    std::iota(channels.data, channels.data + 4, 0);
    set_duty_cycle(1, channels);
    start_switching_matrix(row_count, delay_s);
    const uint32_t delay_us_ = delay_s * 1e6;

    auto start = micros();
    uint8_t row_previous = 0b11111111;
    for (auto i = 0; i < repeats; i++) {
      for (auto j = 0; j < row_count; j++) {
        while (true) {
          const uint8_t row_ij =
              matrix_controller_.update(*this,
                                        SwitchingMatrixRowContoller::TICK,
                                        micros(), measure_C, send_signal);
          if (row_ij != row_previous) {
            row_previous = row_ij;
            break;
          }
        }
      }
    }
    auto end = micros();
    stop_switching_matrix();
    turn_off_all_channels();
    return (end - start) * 1e-6;
  }

};
}  // namespace dropbot


#endif  // #ifndef ___NODE__H___
