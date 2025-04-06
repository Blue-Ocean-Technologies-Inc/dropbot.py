#ifndef ___PACKET_STREAM__H___
#define ___PACKET_STREAM__H___

#include "PacketWriter.h"


template <typename Stream, typename T>
uint16_t update_crc_any(uint16_t crc, T const &value) {
  stream_byte_type* proxy = reinterpret_cast<stream_byte_type *>(&value);
  /* Arduino and Intel x86 processors store [low byte at lower address][1]
   * _(i.e., little-endian)_, so we must traverse the bytes in reverse order
   * when serializing to [network-byte-order][2].
   *
   * >     test.c:
   * >     static int order __attribute__((unused)) = 0xAAFF;
   * >
   * >     avr-gcc test.c -c
   * >     avr-objdump -D test.o
   * >     test.o:     file format elf32-avr
   * >
   * >     Disassembly of section .data:
   * >
   * >     00000000 <order>:
   * >        0:   ff aa           std     Y+55, r15       ; 0x37
   *
   * [1]: https://diigo.com/01r4yj
   * [2]: http://en.wikipedia.org/wiki/Endianness#Endianness_in_networking */
  for (int i = sizeof(T) - 1; i >= 0; i--) {
    crc = update_crc(crc, proxy[i]);
  }
  return crc;
}


class PacketStream {
private:
    uint16_t crc_;
public:
    template <typename Stream>
    void start(Stream &output, uint16_t size, uint16_t iuid=0,
               uint8_t type=Packet::packet_type::STREAM) {
        /* This function **MUST** only be called once. */
        stream_byte_type startflag[] = "|||";
        output.write(startflag, 3);
        serialize_any(output, iuid);
        serialize_any(output, type);
        serialize_any(output, size);
        crc_ = crc_init();
    }

    template <typename Stream>
    inline void write(Stream &output, stream_byte_type *data,
                      uint16_t size) {
        /* This function **MAY** be called multiple times. */

        // Update CRC code with data.
        for (uint16_t i = 0; i < size; i++) {
            crc_ = update_crc(crc_, data[i]);
        }
        // Write data to output stream.
        output.write((stream_byte_type*)data, size);
    }

    template <typename Stream, typename T>
    inline void serialize(Stream &output, T value) {
        /* This function **MAY** be called multiple times. */

        // Update CRC code with bytes from value.
        crc_ = update_crc_any(crc_, value);
        // Serialize value to output.
        serialize_any(output, value);
    }

    template <typename Stream>
    void end(Stream &output) {
        /* This function **MUST** only be called once. */
        crc_ = finalize_crc(crc_);
        serialize_any(output, crc_);
    }
};

#endif  // #ifndef ___PACKET_STREAM__H___
