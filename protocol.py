"""This module contains functions for parsing the Decimator control protocol.

Decimator products contain an FTDI USB-serial converter chip that is
used in synchronous bit-banging mode. Each byte written to the
converter represents the state of the converter's 8 pins that drive
various parts in the Decimator. Writing multiple bytes just clocks
data out of those 8 pins at a fixed rate.

Pin information:
Bit 0 (0x01): Unknown
Bit 1 (0x02): Unknown (used in firmware erase) 
Bit 2 (0x04): Unknown (used in firmware erase)
Bit 3 (0x08): Data to/from FPGA (bidirectional)
Bit 4 (0x10): Unknown (used in firmware erase)
Bit 5 (0x20): Unknown (used in firmware erase)
Bit 6 (0x40): Clock to FPGA (always driven by FTDI)
Bit 7 (0x80): Unknown (used in firmware erase)

"""

from functools import reduce

def _bit_list_to_bytes(bits):
    return bytes(
        reduce(lambda a, b: (a << 1) | b, (int(bool(x)) for x in byte_bits))
        for byte_bits
        in zip(*[iter(bits)]*8)
    )

def raw_response_to_bytes(raw):
    status_bits = []
    for i in range(0, len(raw), 4):
        if raw[i+2] != raw[i+3]:
            print ("difference at bit", i)
        status_bits.append(bool(raw[i+2] & 0x8))
    #print (bitstr, b, chr(int(bitstr, 2)))
    return _bit_list_to_bytes(status_bits)

READ_PREAMBLE = b'\x00\x40\x00\x40\x48\x48\x40\x00'

COMMAND_PREAMBLE = b'\x00\x40\x00\x40\x48\x48\x40\x00'
COMMAND_POSTAMBLE = b'\x00\x40\x48'
def bytes_to_raw_command(command):
    # N.B. This is *NOT* the inverse of _raw_response_to_bytes.
    # Commands use three cycles per bit; responses use four cycles per bit.
    data = bytearray()
    for c in b:
        for i in range(7, -1, -1):
            data.extend(b'\x08\x48\x08' if (c & (1 << i)) else b'\x00\x40\x00')
    return bytes(data)

def raw_command_to_bytes(data):
    # Extract the data pin every time there is a rising edge on the clock pin.
    command_bits = []
    edge = False
    for byte in data:
        if edge:
            if not byte & 0x40:
                edge = False
        else:
            if byte & 0x40:
                edge = True
                command_bits.append(bool(byte & 0x08))
    return command_bits, _bit_list_to_bytes(command_bits)

