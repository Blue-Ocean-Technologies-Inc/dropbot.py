// .. versionadded:: 1.37.1
#include "Node.h"

namespace dropbot {

void OutputEnableDebounce::pressed() {
    /* .. versionchanged:: 1.37.1
     *     Migrate to ``OutputEnableDebounce.cpp``.
     *
     *     Toggle HV output to address issue #23.
     */

    // The **output enable** pin has been pulled LOW, i.e., a DMF chip has been
    // **inserted**.
    PacketStream output;
    stream_byte_type data[] = "{\"event\": \"output_enabled\"}";
    output.start(Serial, sizeof(data) - 1);
    output.write(Serial, data, sizeof(data) - 1);
    output.end(Serial);

    // High voltage is disabled if the chip is not inserted regardless of the
    // *high voltage output enable* setting due to a hardware interlock (i.e.,
    // the active low output enable `OE_PIN`).
    if (parent_.state_._.hv_output_enabled) {
        // TODO XXX If a chip is inserted when the high voltage output enable
        // is set, the high voltage should be turned on accordingly.  However,
        // this currently does not happen.
        //
        // XXX As a workaround, force the high voltage to be turned off and on,
        // to ensure high voltage is actually turned on.
        //
        // See [issue #23][i23] for more information.
        //
        // [i23]: https://gitlab.com/sci-bots/dropbot.py/issues/23
        parent_.on_state_hv_output_enabled_changed(false);
        parent_.on_state_hv_output_enabled_changed(true);
    }
}

void OutputEnableDebounce::released() {
    /* .. versionchanged:: 1.37.1
     *     Migrate to ``OutputEnableDebounce.cpp``.
     */

    // The **output enable** pin has been pulled HIGH, i.e., a DMF chip has been
    // **removed**.
    PacketStream output;
    stream_byte_type data[] = "{\"event\": \"output_disabled\"}";
    output.start(Serial, sizeof(data) - 1);
    output.write(Serial, data, sizeof(data) - 1);
    output.end(Serial);
}

}
// namespace dropbot {
