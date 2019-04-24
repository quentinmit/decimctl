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

    def __setattr__(self, name, value):
        field = getattr(type(self), name, None)
        if field and not hasattr(self, "_device"):
            raise ValueError("registers not linked to device")
        super(BigEndianStructure, self).__setattr__(name, value)
        if not field:
            return
        register_start = field.offset
        length = field.size
        if length > 0xFFFF:
            length = ((length >> 16) + 7) // 8
        self._device.fpga_write_bytes(register_start, bytes(self)[register_start:register_start+length])


class DUCFormat(enum.IntEnum):
    SD_720x480i59_94 = 0
    SD_720x576i50 = 1
    HD_1920x1080i60 = 2
    HD_1920x1080i59_94 = 3
    HD_1920x1080i50 = 4
    HD_1920x1080p30 = 5
    HD_1920x1080p29_97 = 6
    HD_1920x1080p25 = 7
    HD_1920x1080p24 = 8
    HD_1920x1080p23_98 = 9
    HD_1280x720p60 = 10
    HD_1280x720p59_94 = 11
    HD_1280x720p50 = 12
    HD_1280x720p30 = 13
    HD_1280x720p29_97 = 14
    HD_1280x720p25 = 15
    HD_1280x720p24 = 16
    HD_1280x720p23_98 = 17
    THREEG_1920x1080p60 = 18
    THREEG_1920x1080p59_94 = 19
    THREEG_1920x1080p50 = 20

class DUC_HF(enum.IntEnum):
    AUTO = 0
    NONE = 1
    LOW = 2
    MEDIUM = 3
    HIGH = 4

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
        ("PSFINEnable", c_ubyte), # & 0x1
        ("unknown21", c_ubyte),
        ("DUCFormat", c_ubyte), # 0x22 & 0x1f
        ("DUC_S2S_Aspect", c_ubyte), # & 0x1f
        ("DUC_S2H_Aspect", c_ubyte), # & 0x1f
        ("DUC_H2S_Aspect", c_ubyte), # & 0x1f
        ("DUC_H2H_Aspect", c_ubyte), # & 0x1f
        ("unknownDUC_TH1", c_ubyte, 5),
        ("DUC_TH1", c_ushort, 11), # & 0x3FF
        ("unknown29", c_ubyte),
        ("unknownDUC_HF", c_ubyte, 5),
        ("DUC_HF", c_ubyte, 3), # & 0x7
        ("Loop_Enable", c_ubyte), # 0x2b
        ("OUT_3G_B", c_ubyte), # & 0x1
        ("unknown2d", c_ubyte, 6),
        ("DUC_V_FLIP_EN", c_ubyte, 1), # & 0x2
        ("DUC_H_FLIP_EN", c_ubyte, 1), # & 0x1
        ("unknown2e", c_ubyte),
        ("AutoSave", c_ubyte),
        # 0x30
        ("HO_Type", c_ubyte), # & 0x7
        ("SO_Source", c_ubyte), # & 0x3
        ("HO_Source", c_ubyte), # & 0x3
        ("unknownDUC", c_ubyte, 4),
        ("DUC_Ref", c_ubyte, 3), # & 0x3
        ("DUC_Source", c_ubyte, 1), # & 0x1
        ("Overlay", c_ubyte), # & 0x3
        ("LCDOffTime", c_ubyte), # 5s, 15s, 30s, 1m, 5m, 10m, 30m, NEVER
        ("ReturnToStatusTime", c_ubyte), # & 0x7
        ("LXA_Out", c_ubyte), # & 0x3
        ("HDMI_Pair2", c_ubyte, 4),
        ("HDMI_Pair1", c_ubyte, 4),
        ("HDMI_Pair4", c_ubyte, 4),
        ("HDMI_Pair3", c_ubyte, 4),
        ("SDI_Pair2", c_ubyte, 4), # & 0xf
        ("SDI_Pair1", c_ubyte, 4), # & 0xf0
        ("SDI_Pair4", c_ubyte, 4),
        ("SDI_Pair3", c_ubyte, 4),
        ("SDI_Pair6", c_ubyte, 4),
        ("SDI_Pair5", c_ubyte, 4),
        ("SDI_Pair8", c_ubyte, 4),
        ("SDI_Pair7", c_ubyte, 4),
        ("unknown3e", c_ubyte * 2),
        # 0x40
        ("OSINFormatEnable", c_ubyte), # & 0x3
        ("OSFormatForeground", c_ubyte), # & 0x1f
        ("OSFormatBackground", c_ubyte), # & 0x1f
        ("NoSignalForeground", c_ubyte), # & 0x7
        ("NoSignalBackground", c_ubyte), # & 0x7
        ("UMDEnable", c_ubyte), # & 0x1
        ("UMDForeground", c_ubyte), # & 0x1f
        ("UMDBackground", c_ubyte), # & 0x1f
        ("unknown48", c_ubyte * 8),
        # 0x50
        ("unknown50", c_ubyte * 16),
        # 0x60
        ("unknown60", c_ubyte, 4),
        ("AM4_EN", c_ubyte, 1),
        ("AM3_EN", c_ubyte, 1),
        ("AM2_EN", c_ubyte, 1),
        ("AM1_EN", c_ubyte, 1),
        ("Scale_EN", c_ubyte, 1),
        ("AM_Transparency", c_ubyte, 2), # & 0x60
        ("AM_Combination", c_ubyte, 2), # & 0x18
        ("AM_Style", c_ubyte, 3), # & 0x7
        ("unknown62", c_ubyte * 14),
        # 0x70
        ("unknown70", c_ubyte * 16),
        # 0x80
        ("unknown80", c_ubyte, 7),
        ("TPG_EN", c_ubyte, 1), # & 0x1
        ("TPG_Pattern", c_ubyte), # & 0x3f
        ("SO_Test_Audio", c_ubyte), # & 0x3
        ("HO_Test_Audio", c_ubyte), # & 0x3
    ]

    _map = {
        "DUCFormat": DUCFormat,
        "DUC_HF": DUC_HF,
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

