// .. versionadded:: 1.37.1
#include "Node.h"

namespace dropbot {

/**
 * @brief Called when output is **enabled**.
 *
 * \version 1.37.1  Migrate to `OutputEnableDebounce.cpp` and toggle HV output
 * to address issue #23.
 *
 * \version 1.58  Send `chip_status_changed_` event, rather than calling
 *   callbacks directly.
 */
void OutputEnableDebounce::pressed() {
    // The **output enable** pin has been pulled LOW, i.e., a DMF chip has been
    // **inserted**.

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
    parent_.chip_status_changed_.send(true);
}

/**
 * @brief Called when output is **disabled**.
 *
 * \version 1.37.1  Migrate to `OutputEnableDebounce.cpp`.
 *
 * \version 1.58  Send `chip_status_changed_` event, rather than calling
 *   callbacks directly.
 */
void OutputEnableDebounce::released() {
    // The **output enable** pin has been pulled HIGH, i.e., a DMF chip has been
    // **removed**.
    parent_.chip_status_changed_.send(false);
}

}
// namespace dropbot {
