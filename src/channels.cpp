#include "channels.h"


namespace dropbot {

Switch channel_to_switch(uint8_t channel) {
    /*
     *  - 5 IO register ports per switching board.
     *  - 8 bits per IO register port.
     *  - Channels in LSB-first order within each IO register port.
     */
    const auto ports_per_board = 5;
    const auto channels_per_port = 8;
    const auto channels_per_board = ports_per_board * channels_per_port;

    Switch _switch;
    _switch.board = channel / channels_per_board;
    _switch.port = (channel % channels_per_board) / channels_per_port;
    _switch.bit = channel % channels_per_port;
    return _switch;
}


uint8_t switch_to_channel(Switch const &_switch) {
  /*
   *  - 5 IO register ports per switching board.
   *  - 8 bits per IO register port.
   *  - Channels in LSB-first order within each IO register port.
   */

  // Port number within all IO register ports concatenated.
  const auto ports_per_board = 5;
  const auto channels_per_port = 8;
  const auto channels_per_board = ports_per_board * channels_per_port;

  return (_switch.board * channels_per_board + _switch.port * channels_per_port
          + _switch.bit);
}


void pack_channels(UInt8Array const &channels, UInt8Array &packed_channels) {
  const auto ports_per_board = 5;
  packed_channels.length = 0;

  for (auto i = 0; i < channels.length; i++) {
    const Switch _switch = channel_to_switch(channels.data[i]);
    const uint8_t byte_i = _switch.board * ports_per_board + _switch.port;

    if (byte_i + 1 >= packed_channels.length) {
        for (auto board_i = packed_channels.length;
             board_i < _switch.board + 1; board_i++) {
            for (auto port_j = 0; port_j < ports_per_board + 1; port_j++) {
                packed_channels.data[_switch.board * ports_per_board
                                     + port_j] = 0;
            }
        }
        packed_channels.length = (_switch.board + 1) * ports_per_board;
    }

    packed_channels.data[byte_i] |= 1 << _switch.bit;
  }
}

}  // namespace dropbot
