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
from ctypes import BigEndianStructure, c_ubyte, c_ushort, Array
import enum

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

READ_PREAMBLE  = b'\x00\x40\x00\x40\x48\x48\x40\x00'
WRITE_PREAMBLE = b'\x00\x40\x00\x40\x48\x48\x40\x00'
WRITE_POSTAMBLE = b'\x00\x40\x48'

def bytes_to_raw_command(command):
    # N.B. This is *NOT* the inverse of _raw_response_to_bytes.
    # Commands use three cycles per bit; responses use four cycles per bit.
    data = bytearray()
    for c in command:
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

class Registers(BigEndianStructure):
    _map = {}

    def __getattribute__(self, name):
        _map = BigEndianStructure.__getattribute__(self, '_map')
        value = BigEndianStructure.__getattribute__(self, name)
        if name in _map:
            EnumClass = _map[name]
            if isinstance(value, Array):
                return [EnumClass(x) for x in value]
            else:
                return EnumClass(value)
        else:
            return value

    def __str__(self):
        result = []
        result.append("struct {0} {{".format(self.__class__.__name__))
        for field in self._fields_:
            attr, attrType = field[:2]
            if attr in self._map:
                attrType = self._map[attr]
            value = getattr(self, attr)
            if isinstance(value, Array):
                value = list(value)
            result.append("    {0} [{1}] = {2!r};".format(attr, attrType.__name__, value))
        result.append("};")
        return '\n'.join(result)

    __repr__ = __str__

class HO_Type(enum.IntEnum):
    DVI_RGB_444 = 0
    HDMI_RGB_444_2ch = 1
    HDMI_YCbCr_444_2ch = 2
    HDMI_YCbCr_422_2ch = 3
    HDMI_RGB_444_8ch = 4
    HDMI_YCbCr_444_8ch = 5
    HDMI_YCbCr_422_8ch = 6

class SO_Source(enum.IntEnum):
    SDI_IN = 0
    HDMI_IN = 1
    DUC = 2

HO_Source = SO_Source

class DUC_Source(enum.IntEnum):
    SDI_IN = 0
    HDMI_IN = 1

class DUC_Ref(enum.IntEnum):
    Source = 0
    Free_Run = 1
    SDI_IN = 2
    HDMI_IN = 3

class LCDOffTime(enum.IntEnum):
    OFF_5s = 0
    OFF_15s = 1
    OFF_30s = 2
    OFF_1m = 3
    OFF_5m = 4
    OFF_10m = 5
    OFF_30m = 6
    NEVER = 7

class CPA_Registers(Registers):
    _pack_ = 1
    _fields_ = [
        # 0x00
        ("magic", c_ubyte * 2),
        ("version_major", c_ushort, 16),
        ("version_minor", c_ushort, 16),
        ("unknown06", c_ubyte * 10),
        # 0x10
        ("unknown10", c_ubyte * 16),
        # 0x20
        ("unknown20", c_ubyte * 2),
        ("DUCFormat", c_ubyte), # 0x22
        ("unknown23", c_ubyte * 8),
        ("Loop_Enable", c_ubyte), # 0x2b
        ("unknown2c", c_ubyte * 4),
        # 0x30
        ("HO_Type", c_ubyte),
        ("SO_Source", c_ubyte),
        ("HO_Source", c_ubyte),
        ("unknownDUC", c_ubyte, 4),
        ("DUC_Ref", c_ubyte, 3),
        ("DUC_Source", c_ubyte, 1),
        ("unknown34", c_ubyte),
        ("LCDOffTime", c_ubyte), # 5s, 15s, 30s, 1m, 5m, 10m, 30m, NEVER
    ]

    _map = {
        "HO_Type": HO_Type,
        "SO_Source": SO_Source,
        "HO_Source": HO_Source,
        "DUC_Ref": DUC_Ref,
        "DUC_Source": DUC_Source,
        "LCDOffTime": LCDOffTime,
    }

    # def __str__(self):
    #     out = []
    #     for field in self._fields_:
    #         value = getattr(self, field[0])
    #         value = getattr(value, "value", value)
    #         if isinstance(value, Array):
    #             value = list(value)
    #         out.append("%s: %s" % (field[0], value))
    #     return "\n".join(out)

