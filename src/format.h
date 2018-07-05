#ifndef ___DROPBOT__FORMAT__H___
#define ___DROPBOT__FORMAT__H___

#include <CArrayDefs.h>


template <typename Capacitances, typename Drops>
inline void sprintf_drops_detected(Capacitances const &capacitances,
                                   Drops const &drops, unsigned long start,
                                   unsigned long end, UInt8Array &buffer) {
    char* const text = reinterpret_cast<char *>(buffer.data);

    buffer.length += sprintf(&text[buffer.length], "{\"event\":"
                             "\"drops-detected\", " "\"drops\": "
                             "{\"channels\": [");

    auto i = 0;
    for (auto it_drop = drops.begin(); it_drop != drops.end(); it_drop++,
         i++) {
      const auto &drop = *it_drop;
      if (drop.size() < 1) { continue; }

      if (i > 0) {
        buffer.length += sprintf(&text[buffer.length], ", ");
      }
      buffer.length += sprintf(&text[buffer.length], "[");
      auto j = 0;
      for (auto it_channel = drop.begin(); it_channel != drop.end();
           it_channel++, j++) {
        if (j > 0) {
          buffer.length += sprintf(&text[buffer.length], ", ");
        }
        buffer.length += sprintf(&text[buffer.length], "%d",
                                 *it_channel);
      }
      buffer.length += sprintf(&text[buffer.length], "]");
    }

    buffer.length += sprintf(&text[buffer.length], "], \"capacitances\": [");

    i = 0;
    for (auto it_drop = drops.begin(); it_drop != drops.end(); it_drop++,
         i++) {
      const auto &drop = *it_drop;
      if (drop.size() < 1) { continue; }

      if (i > 0) {
        buffer.length += sprintf(&text[buffer.length], ", ");
      }
      buffer.length += sprintf(&text[buffer.length], "[");
      auto j = 0;
      for (auto it_channel = drop.begin(); it_channel != drop.end();
           it_channel++, j++) {
        if (j > 0) {
          buffer.length += sprintf(&text[buffer.length], ", ");
        }
        buffer.length += sprintf(&text[buffer.length], "%g",
                                 capacitances[*it_channel]);
      }
      buffer.length += sprintf(&text[buffer.length], "]");
    }

    buffer.length += sprintf(&text[buffer.length], "]}, \"start\": %lu, "
                             "\"end\": %lu}", start, end);
}


#endif  // #ifndef ___DROPBOT__FORMAT__H___
