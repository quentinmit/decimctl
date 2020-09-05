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

Copyright 2019 Quentin Smith <quentin@mit.edu>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

from functools import reduce
from ctypes import BigEndianStructure, c_char, c_ubyte, c_ushort, Array, Union
import enum

def _bit_list_to_bytes(bits):
    """Convert a sequence of truthy values into a byte string, MSB first."""
    return bytes(
        reduce(lambda a, b: (a << 1) | b, (int(bool(x)) for x in byte_bits))
        for byte_bits
        in zip(*[iter(bits)]*8)
    )

def raw_response_to_bytes(raw):
    """
    Convert data read from the FTDI chip to bytes.
    This assumes that each bit is represented by four bytes.
    """
    status_bits = []
    for i in range(0, len(raw), 4):
        if raw[i+2] != raw[i+3]:
            print("difference at bit", i)
            # TODO: Raise exception?
        status_bits.append(bool(raw[i+2] & 0x8))
    #print (bitstr, b, chr(int(bitstr, 2)))
    return _bit_list_to_bytes(status_bits)

READ_PREAMBLE =   b'\x00\x40\x00\x40\x48\x48\x40\x00'
WRITE_PREAMBLE =  b'\x00\x40\x00\x40\x48\x48\x40\x00'
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

MAGIC = b'GW'

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
        if isinstance(value, bytes):
            # ctypes does not set the remainder of the byte string to zero.
            # It also truncates the byte string to the first zero byte, so
            # write successively smaller strings to clear all the bytes.
            for i in range(getattr(type(self), name).size-1, 0, -1):
                super(Registers, self).__setattr__(name, b'x'*i)
        super(Registers, self).__setattr__(name, value)
        if not field:
            return
        register_start = field.offset
        length = field.size
        if length > 0xFFFF:
            length = ((length >> 16) + 7) // 8
        self._device.fpga_write_bytes(register_start, bytes(self)[register_start:register_start+length])

class IntEnum(enum.IntEnum):
    @classmethod
    def _missing_(cls, value):
        pseudo_member = int.__new__(cls)
        pseudo_member._name_ = "INVALID"
        pseudo_member._value_ = value
        return pseudo_member

class DUCFormat(IntEnum):
    SD_720x480i59_94 = 0
    SD_720x576i50 = 1
    ED_720x480p59_94 = 2
    ED_720x576p50 = 3
    HD_1920x1080i60 = 4
    HD_1920x1080i59_94 = 5
    HD_1920x1080i50 = 6
    HD_1920x1080psf30 = 7
    HD_1920x1080psf29_97 = 8
    HD_1920x1080psf25 = 9
    HD_1920x1080psf24 = 10
    HD_1920x1080psf23_98 = 11
    HD_1920x1080p30 = 12
    HD_1920x1080p29_97 = 13
    HD_1920x1080p25 = 14
    HD_1920x1080p24 = 15
    HD_1920x1080p23_98 = 16
    HD_1280x720p60 = 17
    HD_1280x720p59_94 = 18
    HD_1280x720p50 = 19
    HD_1280x720p30 = 20
    HD_1280x720p29_97 = 21
    HD_1280x720p25 = 22
    HD_1280x720p24 = 23
    HD_1280x720p23_98 = 24
    THREEG_1920x1080p60 = 25
    THREEG_1920x1080p59_94 = 26
    THREEG_1920x1080p50 = 27

class DUC_HF(IntEnum):
    AUTO = 0
    NONE = 1
    LOW = 2
    MEDIUM = 3
    HIGH = 4

class HO_Type(IntEnum):
    DVI_RGB_444 = 0
    HDMI_RGB_444_2ch = 1
    HDMI_YCbCr_444_2ch = 2
    HDMI_YCbCr_422_2ch = 3
    HDMI_RGB_444_8ch = 4
    HDMI_YCbCr_444_8ch = 5
    HDMI_YCbCr_422_8ch = 6

class SO_Source(IntEnum):
    SDI_IN = 0
    HDMI_IN = 1
    DUC = 2

HO_Source = SO_Source

class DUC_Source(IntEnum):
    SDI_IN = 0
    HDMI_IN = 1

class DUC_Ref(IntEnum):
    Source = 0
    Free_Run = 1
    SDI_IN = 2
    HDMI_IN = 3

class Time(IntEnum):
    T_5s = 0
    T_15s = 1
    T_30s = 2
    T_1m = 3
    T_5m = 4
    T_10m = 5
    T_30m = 6
    NEVER = 7

class AudioPair(IntEnum):
    GROUP_1_PAIR_1 = 0
    GROUP_1_PAIR_2 = 1
    GROUP_2_PAIR_1 = 2
    GROUP_2_PAIR_2 = 3
    GROUP_3_PAIR_1 = 4
    GROUP_3_PAIR_2 = 5
    GROUP_4_PAIR_1 = 6
    GROUP_4_PAIR_2 = 7
    OFF = 8

# Input*Status contains
# 0x8000 = Fractional (/1.001)
# 0x4000 = 3G-B
# 0x3b00 = Format
# 0x0003 = {SD, ED, HD, 3G}
# 0x0004 = Locked

class InputStandard(IntEnum):
    SD_SDI = 0
    ED_SDI = 1
    HD_SDI = 2
    THREEG_SDI = 3

def CPA_CalcFormatString(locked, standard, fractional, format, threeg_b):
    #flag1 = ((two << 1)) & 0x6 | ((one >> 7) & 0x1)
    #threegB = ((one >> 6) & 0x1)
    #format = one & 0x3f
    #locked = (two >> 2) & 0x1
    if locked == 0:
        return "Unlocked"

    if standard == InputStandard.SD_SDI:
        if fractional:
            return "Unknown"
        return {
            0: "SD 720x480i59.94",
            1: "SD 720x576i50",
            }.get(format, "Unknown")
    elif standard == InputStandard.ED_SDI:
        if fractional:
            return "Unknown"
        return {
            0: "ED 720x480p59.95",
            1: "ED 720x576p50",
        }.get(format, "Unknown")
    elif standard == InputStandard.HD_SDI and not fractional:
        return {
            0x36: "HD 1280x720p24",
            0x35: "HD 1280x720p25",
            0x33: "HD 1280x720p30",
            0x32: "HD 1280x720p50",
            0x30: "HD 1280x720p60",
            0x23: "HD 1920x1080p24",
            0x22: "HD 1920x1080p25",
            0x20: "HD 1920x1080p30",
            0x13: "HD 1920x1080psf24",
            0x12: "HD 1920x1080psf25",
            0x10: "HD 1920x1080psf30",
            0x06: "HD 1920x1080i50",
            0x04: "HD 1920x1080i60",
        }.get(format, "Unknown")
    elif standard == InputStandard.HD_SDI and fractional:
        return {
            0x37: "HD 1280x720p23.98",
            0x34: "HD 1280x720p29.97",
            0x31: "HD 1280x720p59.94",
            0x24: "HD 1920x1080p23.98",
            0x21: "HD 1920x1080p29.97",
            0x14: "HD 1920x1080psf23.98",
            0x11: "HD 1920x1080psf29.97",
            0x05: "HD 1920x1080i59.94",
        }.get(format, "Unknown")
    elif standard == InputStandard.THREEG_SDI:
        if threeg_b:
            threeg = "3G-B "
        else:
            threeg = "3G-A "
        return threeg + {
            0x22: "1920x1080p50",
            0x21: "1920x1080p59.94", # frac
            0x20: "1920x1080p60",
        }.get(format, "Unknown")


class CPA_Registers(Registers):
    """Register layout of CPA devices (MD-HX, MD-CROSS, MD-LX)."""

    _pack_ = 1
    _fields_ = [
        # 0x00
        ("magic", c_char * 2),
        ("version_major", c_ushort, 16),
        ("version_minor", c_ushort, 16),
        ("unknown06", c_ubyte * 10),
        # 0x10
        ("Input1HDFractional", c_ubyte, 1),
        ("Input13GB", c_ubyte, 1),
        ("Input1Format", c_ubyte, 6),
        ("unknown11", c_ubyte, 5),
        ("Input1Locked", c_ubyte, 1),
        ("Input1Standard", c_ubyte, 2),
        ("Input2HDFractional", c_ubyte, 1),
        ("Input23GB", c_ubyte, 1),
        ("Input2Format", c_ubyte, 6),
        ("unknown11", c_ubyte, 5),
        ("Input2Locked", c_ubyte, 1),
        ("Input2Standard", c_ubyte, 2),
        ("unknown14", c_ubyte * 12),
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
        "Input1Standard": InputStandard,
        "Input2Standard": InputStandard,
        "DUCFormat": DUCFormat,
        "DUC_HF": DUC_HF,
        "HO_Type": HO_Type,
        "SO_Source": SO_Source,
        "HO_Source": HO_Source,
        "DUC_Ref": DUC_Ref,
        "DUC_Source": DUC_Source,
        "LCDOffTime": Time,
        "ReturnToStatusTime": Time,
        "HDMI_Pair1": AudioPair,
        "HDMI_Pair2": AudioPair,
        "HDMI_Pair3": AudioPair,
        "HDMI_Pair4": AudioPair,
        "SDI_Pair1": AudioPair,
        "SDI_Pair2": AudioPair,
        "SDI_Pair3": AudioPair,
        "SDI_Pair4": AudioPair,
        "SDI_Pair5": AudioPair,
        "SDI_Pair6": AudioPair,
        "SDI_Pair7": AudioPair,
        "SDI_Pair8": AudioPair,
    }

    @property
    def Input1Status(self):
        return CPA_CalcFormatString(self.Input1Locked, self.Input1Standard, self.Input1HDFractional, self.Input1Format, self.Input13GB)

    @property
    def Input2Status(self):
        return CPA_CalcFormatString(self.Input2Locked, self.Input2Standard, self.Input2HDFractional, self.Input2Format, self.Input23GB)

class MVReference(IntEnum):
    FREE_RUN = 0
    WINDOW_1 = 1
    WINDOW_2 = 3
    WINDOW_3 = 5
    WINDOW_4 = 7

class MVAudioSource(IntEnum):
    WINDOW_1 = 0
    WINDOW_2 = 1
    WINDOW_3 = 2
    WINDOW_4 = 3
    WINDOW_5 = 4
    WINDOW_6 = 5

class VFA_HO_Source(IntEnum):
    INPUT_1 = 0
    INPUT_2 = 1
    INPUT_3 = 2
    INPUT_4 = 3
    MULTI_VIEW = 4

class OSEnable(IntEnum):
    OFF = 0
    SHOW_5s = 1
    SHOW_ALWAYS = 2

class UMDs_Justify(IntEnum):
    CENTRE = 0
    LEFT = 1
    RIGHT = 2

class VFA_Registers(Registers):
    """Register layout of VFA devices (DMON-QUAD, etc.)."""

    _pack_ = 1
    _fields_ = [
        # 0x00
        ("magic", c_char * 2),
        ("version_major", c_ushort, 16),
        ("version_minor", c_ushort, 16),
        ("unknown06", c_ubyte * 10),
        # 0x10
        ("unknown10", c_ubyte * 16),
        # 0x20
        # HO1-4 are used by MDA
        ("unknown20", c_ubyte, 2),
        ("HO1_Type", c_ubyte, 3),
        ("HO1_Source", c_ubyte, 3),
        ("unknown21", c_ubyte, 2),
        ("HO2_Type", c_ubyte, 3),
        ("HO2_Source", c_ubyte, 3),
        ("unknown22", c_ubyte, 2),
        ("HO3_Type", c_ubyte, 3),
        ("HO3_Source", c_ubyte, 3),
        ("unknown23", c_ubyte, 2),
        ("HO4_Type", c_ubyte, 3),
        ("HO4_Source", c_ubyte, 3),
        ("unknown24", c_ubyte, 6),
        ("LP4_EN", c_ubyte, 1), # & 0x2
        ("LP2_EN", c_ubyte, 1), # & 0x1
        ("unknown25", c_ubyte * 11),
        # 0x30
        # HO_Type on all types except OAA, OBA
        ("HO_Type", c_ubyte), # & 0x7
        ("OutputSelect", c_ubyte, 7), # & 0xf
        ("OutputMultiViewer", c_ubyte, 1), # & 0x1
        ("MVFormat", c_ubyte), # & 0x1f
        ("MVReference", c_ubyte), # & 0x7. 1 -> 1, 3 -> 2, 5 -> 3, 7 -> 4, else 0
        ("FSScaled", c_ubyte), # & 0x1
        ("NumWindows", c_ubyte), # & 0xf
        ("MVLayout", c_ubyte), # & 0x1f
        ("OSAudioSourceIDEnable", c_ubyte, 4), # & 0x3
        ("MVAudioSource", c_ubyte, 4), # & 0xf
        ("VS2", c_ubyte, 4), # & 0xf0
        ("VS1", c_ubyte, 4), # & 0xf
        ("VS4", c_ubyte, 4), # & 0xf0
        ("VS3", c_ubyte, 4), # & 0xf
        ("VS6", c_ubyte, 4), # & 0xf0
        ("VS5", c_ubyte, 4), # & 0xf
        ("VS8", c_ubyte, 4), # & 0xf0
        ("VS7", c_ubyte, 4), # & 0xf
        ("VS10", c_ubyte, 4), # & 0xf0
        ("VS9", c_ubyte, 4), # & 0xf
        ("VS12", c_ubyte, 4), # & 0xf0
        ("VS11", c_ubyte, 4), # & 0xf
        ("VS14", c_ubyte, 4), # & 0xf0
        ("VS13", c_ubyte, 4), # & 0xf
        ("VS16", c_ubyte, 4), # & 0xf0
        ("VS15", c_ubyte, 4), # & 0xf
        # 0x40
        # Addresses on all types except MDA, MPA (up to type window limit)
        ("UMDs_W1_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W2_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W3_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W4_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W5_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W6_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W7_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W8_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W9_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W10_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W11_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W12_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W13_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W14_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W15_TSL_Address", c_ubyte), # & 0x7f
        ("UMDs_W16_TSL_Address", c_ubyte), # & 0x7f
        # 0x50
        ("OSINFormatEnable", c_ubyte), # & 0x3
        ("OSFormatForeground", c_ubyte), # & 0x1f
        ("OSFormatBackground", c_ubyte), # & 0x1f
        ("unknown53", c_ubyte),
        ("Background", c_ubyte), # & 0x7
        ("BorderColour", c_ubyte), # & 0x7
        ("GPIConfig", c_ubyte), # & 0x3
        ("ApplyTallyTo", c_ubyte), # & 0x3
        ("OUT_3G_B", c_ubyte), # & 0x1
        ("unknown59", c_ubyte),
        ("LCDOffTime", c_ubyte), # & 0x7
        ("ReturnToStatusTime", c_ubyte), # & 0x7
        ("DemoCycleType", c_ubyte), # & 0x7
        ("DemoCycleTime", c_ubyte),
        ("AutoSave", c_ubyte), # & 0x1
        ("unknown5f", c_ubyte),
        # 0x60
        ("UMDForeground", c_ubyte), # & 0x1f
        ("UMDBackground", c_ubyte), # & 0x1f
        ("UMDs_Justify", c_ubyte), # & 0x3
        ("UMDs_W8_Enable", c_ubyte, 1),
        ("UMDs_W7_Enable", c_ubyte, 1),
        ("UMDs_W6_Enable", c_ubyte, 1),
        ("UMDs_W5_Enable", c_ubyte, 1),
        ("UMDs_W4_Enable", c_ubyte, 1),
        ("UMDs_W3_Enable", c_ubyte, 1),
        ("UMDs_W2_Enable", c_ubyte, 1),
        ("UMDs_W1_Enable", c_ubyte, 1),
        ("UMDs_W16_Enable", c_ubyte, 1),
        ("UMDs_W15_Enable", c_ubyte, 1),
        ("UMDs_W14_Enable", c_ubyte, 1),
        ("UMDs_W13_Enable", c_ubyte, 1),
        ("UMDs_W12_Enable", c_ubyte, 1),
        ("UMDs_W11_Enable", c_ubyte, 1),
        ("UMDs_W10_Enable", c_ubyte, 1),
        ("UMDs_W9_Enable", c_ubyte, 1),
        ("unknown65", c_ubyte * 10),
        ("TallyTransparency", c_ubyte), # & 0x3
        # 0x70
        ("unknown70", c_ubyte * 16),
        ("unknown80", c_ubyte * 16),
        ("unknown90", c_ubyte * 16),
        ("unknowna0", c_ubyte * 16),
        ("unknownb0", c_ubyte * 16),
        ("unknownc0", c_ubyte * 16),
        ("unknownd0", c_ubyte * 16),
        ("unknowne0", c_ubyte * 16),
        ("unknownf0", c_ubyte * 16),
        # 0x100
        ("UMD_W1", c_char * 0x10),
        ("UMD_W2", c_char * 0x10),
        ("UMD_W3", c_char * 0x10),
        ("UMD_W4", c_char * 0x10),
        ("UMD_W5", c_char * 0x10),
        ("UMD_W6", c_char * 0x10),
        ("UMD_W7", c_char * 0x10),
        ("UMD_W8", c_char * 0x10),
        ("UMD_W9", c_char * 0x10),
        ("UMD_W10", c_char * 0x10),
        ("UMD_W11", c_char * 0x10),
        ("UMD_W12", c_char * 0x10),
        ("UMD_W13", c_char * 0x10),
        ("UMD_W14", c_char * 0x10),
        ("UMD_W15", c_char * 0x10),
        ("UMD_W16", c_char * 0x10),
    ]

    _map = {
        "MVReference": MVReference,
        "MVAudioSource": MVAudioSource,
        "OSAudioSourceIDEnable": OSEnable,
        "OSINFormatEnable": OSEnable,
        "UMDs_Justify": UMDs_Justify,
        "HO1_Source": VFA_HO_Source,
        "HO2_Source": VFA_HO_Source,
        "HO3_Source": VFA_HO_Source,
        "HO4_Source": VFA_HO_Source,
        "LCDOffTime": Time,
        "ReturnToStatusTime": Time,
    }

class Genlock(IntEnum):
    NO = 0
    IF_PRESENT = 1
    ALWAYS = 2

class KPADUCFormat(IntEnum):
    SD_720x480i59_94 = 0
    SD_720x576i50 = 1
    ED_720x480p59_94 = 2
    ED_720x576p50 = 3
    HD_1280x720p60 = 4
    HD_1280x720p59_94 = 5
    HD_1280x720p50 = 6
    HD_1280x720p30 = 7
    HD_1280x720p29_97 = 8
    HD_1280x720p25 = 9
    HD_1280x720p24 = 10
    HD_1280x720p23_98 = 11
    HD_1920x1080i60 = 12
    HD_1920x1080i59_94 = 13
    HD_1920x1080i50 = 14
    HD_1920x1080psf30 = 15
    HD_1920x1080psf29_97 = 16
    HD_1920x1080psf25 = 17
    HD_1920x1080psf24 = 18
    HD_1920x1080psf23_98 = 19
    HD_1920x1080p30 = 20
    HD_1920x1080p29_97 = 21
    HD_1920x1080p25 = 22
    HD_1920x1080p24 = 23
    HD_1920x1080p23_98 = 24
    HD_2Kpsf30 = 25
    HD_2Kpsf29_97 = 26
    HD_2Kpsf25 = 27
    HD_2Kpsf24 = 28
    HD_2Kpsf23_98 = 29
    HD_2Kp30 = 30
    HD_2Kp29_97 = 31
    HD_2Kp25 = 32
    HD_2Kp24 = 33
    HD_2Kp23_98 = 34
    THREEG_1920x1080p60 = 35
    THREEG_1920x1080p59_94 = 36
    THREEG_1920x1080p50 = 37
    THREEG_2Kp60 = 38
    THREEG_2Kp59_94 = 39
    THREEG_2Kp50 = 40
    SIXG_2160p30 = 41
    SIXG_2160p29_97 = 42
    SIXG_2160p25 = 43
    SIXG_2160p24 = 44
    SIXG_2160p23_98 = 45
    SIXG_4Kp30 = 46
    SIXG_4Kp29_97 = 47
    SIXG_4Kp25 = 48
    SIXG_4Kp24 = 49
    SIXG_4Kp23_98 = 50
    TWELVEG_2160p60 = 51
    TWELVEG_2160p59_94 = 52
    TWELVEG_2160p50 = 53
    TWELVEG_4Kp60 = 54
    TWELVEG_4Kp59_94 = 55
    TWELVEG_4Kp50 = 56

class KPAAudioPair(IntEnum):
    GROUP_1_PAIR_1 = 0
    GROUP_1_PAIR_2 = 1
    GROUP_2_PAIR_1 = 2
    GROUP_2_PAIR_2 = 3
    GROUP_3_PAIR_1 = 4
    GROUP_3_PAIR_2 = 5
    GROUP_4_PAIR_1 = 6
    GROUP_4_PAIR_2 = 7
    OFF = 15

class KPADUC_HF(IntEnum):
    AUTO = 0
    NONE = 4
    LOW = 5
    MEDIUM = 6
    HIGH = 7

class SDAspect(IntEnum):
    ASPECT_4_3 = 0
    ASPECT_14_9 = 1
    ASPECT_16_9 = 2

class InputImageAspect(IntEnum):
    INPUT = 0
    ASPECT_4_3 = 1
    ASPECT_14_9 = 2
    ASPECT_16_9 = 3
    ASPECT_256_135 = 4
    ASPECT_64_87 = 5
    ASPECT_3_4 = 6
    ASPECT_1_1 = 7
    ASPECT_5_4 = 8
    ASPECT_32_25 = 9
    ASPECT_192_145 = 10
    ASPECT_296_221 = 11
    ASPECT_960_617 = 12
    ASPECT_467_300 = 13
    ASPECT_51_32 = 14
    ASPECT_307_192 = 15
    ASPECT_8_5 = 16
    ASPECT_5_3 = 17
    ASPECT_53_30 = 18
    ASPECT_85_48 = 19
    ASPECT_133_75 = 20
    ASPECT_71_40 = 21
    ASPECT_932_525 = 22
    ASPECT_341_192 = 23
    ASPECT_683_384 = 24
    ASPECT_9_5 = 25
    ASPECT_64_35 = 26
    ASPECT_72_35 = 27

class OutputImageAspect(IntEnum):
    FIT = 0
    FILL_CENTER = 1
    FILL_RIGHT_BOTTOM_CUT = 2
    FILL_LEFT_TOP_CUT = 3
    STRETCH = 4

class ColorSpace(IntEnum):
    REC_709 = 0
    REC_2020 = 1

class VESAColorSpace(IntEnum):
    REC_601 = 0
    REC_709 = 1
    REC_2020 = 2

class KPAInputStandard(IntEnum):
    SD_SDI = 0
    ED_SDI = 2
    VESA = 3
    HD_SDI = 4
    THREEG_SDI = 5
    SIXG_SDI = 6
    TWELVEG_SDI = 7

def KPA_CalcFormatString(locked, standard, fractional, threeg_b, format):
    out = "Unlocked "

    # 16-bit LE (first byte is low-order bits) status register contains:
    # 0x01FF = Format
    # 0x0200 = 3G-B
    # 0x0400 = Fractional (/1.001)
    # 0x3800 = {SD, x, ED, HD, 3G, 6G, 12G}
    # 0x4000 = Locked

    if threeg_b:
        threeg = "3G-B "
    else:
        threeg = "3G-A "
    aspect = 0
    if locked:
        if standard == KPAInputStandard.SD_SDI:
            if format == 0:
                out = "SD 720x480i59.94"
                aspect = 0
            else:
                out = "SD 720x576i50"
                aspect = 2
        elif standard == KPAInputStandard.ED_SDI:
            if format == 0:
                out = "ED 720x480p59.94"
            else:
                out = "ED 720x576p50"
        elif standard == KPAInputStandard.VESA:
            out, aspect = {
                0x0: ("640x350p85.08", 0xb),
                0x1: ("640x360p84.48", 0xc),
                0x2: ("640x400p85.08", 0xd),
                0x3: ("640x400p84.74", 0xd),
                0x4: ("640x400p59.94", 0xd),
                0x5: ("640x480p85.01", 0xe),
                0x6: ("640x480p84.60", 0xe),
                0x7: ("640x480p75.00", 0xe),
                0x8: ("640x480p75.00", 0xe),
                0x9: ("640x480p74.77", 0xe),
                0xa: ("640x480p72.81", 0xe),
                0xb: ("640x480p66.67", 0xe),
                0xc: ("640x480p66.62", 0xe),
                0xd: ("640x480p60.00", 0xe),
                0xe: ("640x480p59.94", 0xe),
                0xf: ("640x870p75.00", 0xf),
                0x10: ("720x350p70.09", 0x10),
                0x11: ("720x400p87.85", 0x11),
                0x12: ("720x400p85.04", 0x11),
                0x13: ("720x400p70.09", 0x11),
                0x14: ("720x400p59.94", 0x11),
                0x15: ("720x480p59.94", 0x0),
                0x16: ("768x480p84.50", 0x12),
                0x17: ("768x480p74.71", 0x12),
                0x18: ("768x480p59.90", 0x12),
                0x19: ("768x480p59.87", 0x12),
                0x1a: ("800x600p119.97", 0x13),
                0x1b: ("800x600p85.06", 0x13),
                0x1c: ("800x600p84.90", 0x13),
                0x1d: ("800x600p75.00", 0x13),
                0x1e: ("800x600p74.91", 0x13),
                0x1f: ("800x600p72.19", 0x13),
                0x20: ("800x600p60.32", 0x13),
                0x21: ("800x600p59.86", 0x13),
                0x22: ("800x600p59.84", 0x13),
                0x23: ("800x600p56.25", 0x13),
                0x24: ("800x600p49.92", 0x13),
                0x25: ("832x624p75.09", 0x14),
                0x26: ("848x480p84.75", 0x15),
                0x27: ("848x480p74.77", 0x15),
                0x28: ("848x480p60.00", 0x15),
                0x29: ("848x480p59.74", 0x15),
                0x2a: ("848x480p59.66", 0x15),
                0x2b: ("848x480p49.54", 0x15),
                0x2c: ("960x600p84.68", 0x16),
                0x2d: ("960x600p74.84", 0x16),
                0x2e: ("960x600p59.96", 0x16),
                0x2f: ("960x600p59.63", 0x16),
                0x30: ("960x600p49.65", 0x16),
                0x31: ("1024x576p84.75", 0x17),
                0x32: ("1024x576p74.80", 0x17),
                0x33: ("1024x576p59.90", 0x17),
                0x34: ("1024x576p59.82", 0x17),
                0x35: ("1024x576p49.81", 0x17),
                0x36: ("1024x640p84.69", 0x18),
                0x37: ("1024x640p74.85", 0x18),
                0x38: ("1024x640p59.92", 0x18),
                0x39: ("1024x640p59.89", 0x18),
                0x3a: ("1024x640p49.86", 0x18),
                0x3b: ("1024x768p119.99", 0x19),
                0x3c: ("1024x768p85.00", 0x19),
                0x3d: ("1024x768p84.89", 0x19),
                0x3e: ("1024x768p77.07", 0x19),
                0x3f: ("1024x768p75.78", 0x19),
                0x40: ("1024x768p75.03", 0x19),
                0x41: ("1024x768p75.02", 0x19),
                0x42: ("1024x768p74.93", 0x19),
                0x43: ("1024x768p74.92", 0x19),
                0x44: ("1024x768p74.90", 0x19),
                0x45: ("1024x768p71.80", 0x19),
                0x46: ("1024x768p70.07", 0x19),
                0x47: ("1024x768p70.01", 0x19),
                0x48: ("1024x768p60.00", 0x19),
                0x49: ("1024x768p60.00", 0x19),
                0x4a: ("1024x768p59.92", 0x19),
                0x4b: ("1024x768p59.87", 0x19),
                0x4c: ("1024x768p59.28", 0x19),
                0x4d: ("1024x768p49.98", 0x19),
                0x4e: ("1024x1024p61.40", 0x1a),
                0x4f: ("1024x1024p60.00", 0x1a),
                0x50: ("1064x600p84.91", 0x1b),
                0x51: ("1064x600p74.95", 0x1b),
                0x52: ("1064x600p59.86", 0x1b),
                0x53: ("1064x600p59.82", 0x1b),
                0x54: ("1064x600p49.71", 0x1b),
                0x55: ("1152x720p84.92", 0x1c),
                0x56: ("1152x720p74.72", 0x1c),
                0x57: ("1152x720p59.97", 0x1c),
                0x58: ("1152x720p59.92", 0x1c),
                0x59: ("1152x720p49.76", 0x1c),
                0x5a: ("1152x864p85.00", 0x1d),
                0x5b: ("1152x864p75.00", 0x1d),
                0x5c: ("1152x864p70.01", 0x1d),
                0x5d: ("1152x864p59.96", 0x1d),
                0x5e: ("1152x864p59.80", 0x1d),
                0x5f: ("1152x870p75.06", 0x1e),
                0x60: ("1152x900p76.15", 0x1f),
                0x61: ("1152x900p76.05", 0x1f),
                0x62: ("1152x900p66.00", 0x1f),
                0x63: ("1152x900p65.95", 0x1f),
                0x64: ("1184x884p76.05", 0x20),
                0x65: ("1184x884p60.00", 0x20),
                0x66: ("1200x1600p75.50", 0x21),
                0x67: ("1200x1600p66.32", 0x21),
                0x68: ("1224x768p84.79", 0x22),
                0x69: ("1224x768p74.77", 0x22),
                0x6a: ("1224x768p59.91", 0x22),
                0x6b: ("1224x768p59.82", 0x22),
                0x6c: ("1224x768p49.81", 0x22),
                0x6d: ("1280x720p84.85", 0x23),
                0x6e: ("1280x720p74.78", 0x23),
                0x6f: ("1280x720p59.98", 0x23),
                0x70: ("1280x720p59.86", 0x23),
                0x71: ("1280x720p49.83", 0x23),
                0x72: ("1280x768p119.80", 0x22),
                0x73: ("1280x768p84.84", 0x22),
                0x74: ("1280x768p74.89", 0x22),
                0x75: ("1280x768p59.99", 0x22),
                0x76: ("1280x768p59.87", 0x22),
                0x77: ("1280x768p49.93", 0x22),
                0x78: ("1280x800p119.91", 0x24),
                0x79: ("1280x800p84.88", 0x24),
                0x7a: ("1280x800p74.93", 0x24),
                0x7b: ("1280x800p59.91", 0x24),
                0x7c: ("1280x800p59.81", 0x24),
                0x7d: ("1280x800p49.95", 0x24),
                0x7e: ("1280x960p119.84", 0x25),
                0x7f: ("1280x960p85.00", 0x25),
                0x80: ("1280x960p84.86", 0x25),
                0x81: ("1280x960p75.00", 0x25),
                0x82: ("1280x960p74.86", 0x25),
                0x83: ("1280x960p60.00", 0x25),
                0x84: ("1280x960p59.94", 0x25),
                0x85: ("1280x960p59.92", 0x25),
                0x86: ("1280x960p49.85", 0x25),
                0x87: ("1280x1024p119.96", 0x26),
                0x88: ("1280x1024p85.02", 0x26),
                0x89: ("1280x1024p84.84", 0x26),
                0x8a: ("1280x1024p76.18", 0x26),
                0x8b: ("1280x1024p76.11", 0x26),
                0x8c: ("1280x1024p75.02", 0x26),
                0x8d: ("1280x1024p75.02", 0x26),
                0x8e: ("1280x1024p74.90", 0x26),
                0x8f: ("1280x1024p74.11", 0x26),
                0x90: ("1280x1024p72.37", 0x26),
                0x91: ("1280x1024p72.00", 0x26),
                0x92: ("1280x1024p67.00", 0x26),
                0x93: ("1280x1024p67.00", 0x26),
                0x94: ("1280x1024p66.72", 0x26),
                0x95: ("1280x1024p66.68", 0x26),
                0x96: ("1280x1024p60.02", 0x26),
                0x97: ("1280x1024p60.00", 0x26),
                0x98: ("1280x1024p60.00", 0x26),
                0x99: ("1280x1024p59.98", 0x26),
                0x9a: ("1280x1024p59.96", 0x26),
                0x9b: ("1280x1024p59.89", 0x26),
                0x9c: ("1280x1024p49.84", 0x26),
                0x9d: ("1360x768p119.97", 0x27),
                0x9e: ("1360x768p84.88", 0x27),
                0x9f: ("1360x768p74.89", 0x27),
                0xa0: ("1360x768p60.02", 0x27),
                0xa1: ("1360x768p59.96", 0x27),
                0xa2: ("1360x768p59.80", 0x27),
                0xa3: ("1360x768p49.89", 0x27),
                0xa4: ("1366x768p60.00", 0x28),
                0xa5: ("1366x768p59.79", 0x28),
                0xa6: ("1400x1050p119.90", 0x29),
                0xa7: ("1400x1050p84.96", 0x29),
                0xa8: ("1400x1050p74.87", 0x29),
                0xa9: ("1400x1050p59.98", 0x29),
                0xaa: ("1400x1050p59.95", 0x29),
                0xab: ("1400x1050p49.97", 0x29),
                0xac: ("1440x900p119.85", 0x2a),
                0xad: ("1440x900p84.84", 0x2a),
                0xae: ("1440x900p74.98", 0x2a),
                0xaf: ("1440x900p59.90", 0x2a),
                0xb0: ("1440x900p59.89", 0x2a),
                0xb1: ("1440x1080p60.00", 0x2b),
                0xb2: ("1536x960p84.88", 0x2c),
                0xb3: ("1536x960p74.84", 0x2c),
                0xb4: ("1536x960p59.98", 0x2c),
                0xb5: ("1536x960p59.91", 0x2c),
                0xb6: ("1536x960p49.93", 0x2c),
                0xb7: ("1600x900p60.00", 0x2d),
                0xb8: ("1600x1000p84.89", 0x2e),
                0xb9: ("1600x1000p74.84", 0x2e),
                0xba: ("1600x1000p59.91", 0x2e),
                0xbb: ("1600x1000p59.87", 0x2e),
                0xbc: ("1600x1000p49.93", 0x2e),
                0xbd: ("1600x1200p119.92", 0x2f),
                0xbe: ("1600x1200p85.00", 0x2f),
                0xbf: ("1600x1200p84.95", 0x2f),
                0xc0: ("1600x1200p80.00", 0x2f),
                0xc1: ("1600x1200p75.00", 0x2f),
                0xc2: ("1600x1200p74.98", 0x2f),
                0xc3: ("1600x1200p73.01", 0x2f),
                0xc4: ("1600x1200p70.00", 0x2f),
                0xc5: ("1600x1200p65.00", 0x2f),
                0xc6: ("1600x1200p60.00", 0x2f),
                0xc7: ("1600x1200p59.92", 0x2f),
                0xc8: ("1600x1200p59.87", 0x2f),
                0xc9: ("1600x1200p49.92", 0x2f),
                0xca: ("1600x1280p66.93", 0x30),
                0xcb: ("1664x1248p76.02", 0x31),
                0xcc: ("1664x1248p60.00", 0x31),
                0xcd: ("1680x1050p119.99", 0x32),
                0xce: ("1680x1050p84.94", 0x32),
                0xcf: ("1680x1050p74.89", 0x32),
                0xd0: ("1680x1050p59.95", 0x32),
                0xd1: ("1680x1050p59.88", 0x32),
                0xd2: ("1680x1050p49.97", 0x32),
                0xd3: ("1680x1080p60.00", 0x33),
                0xd4: ("1704x960p84.92", 0x34),
                0xd5: ("1704x960p74.87", 0x34),
                0xd6: ("1704x960p59.98", 0x34),
                0xd7: ("1704x960p59.87", 0x34),
                0xd8: ("1704x960p49.96", 0x34),
                0xd9: ("1728x1080p84.88", 0x35),
                0xda: ("1728x1080p74.91", 0x35),
                0xdb: ("1728x1080p59.95", 0x35),
                0xdc: ("1728x1080p59.94", 0x35),
                0xdd: ("1728x1080p49.92", 0x35),
                0xde: ("1792x1344p75.00", 0x36),
                0xdf: ("1792x1344p60.00", 0x36),
                0xe0: ("1800x1350p84.89", 0x37),
                0xe1: ("1800x1350p74.90", 0x37),
                0xe2: ("1800x1350p59.96", 0x37),
                0xe3: ("1800x1350p59.94", 0x37),
                0xe4: ("1800x1350p49.97", 0x37),
                0xe5: ("1856x1392p75.00", 0x38),
                0xe6: ("1856x1392p60.00", 0x38),
                0xe7: ("1864x1050p84.93", 0x39),
                0xe8: ("1864x1050p74.92", 0x39),
                0xe9: ("1864x1050p59.98", 0x39),
                0xea: ("1864x1050p59.93", 0x39),
                0xeb: ("1864x1050p49.91", 0x39),
                0xec: ("1868x1200p85.00", 0x3a),
                0xed: ("1868x1200p75.00", 0x3a),
                0xee: ("1920x1080p85.00", 0x7),
                0xef: ("1920x1080p84.88", 0x7),
                0xf0: ("1920x1080p75.00", 0x7),
                0xf1: ("1920x1080p74.91", 0x7),
                0xf2: ("1920x1080p60.00", 0x7),
                0xf3: ("1920x1080p59.96", 0x7),
                0xf4: ("1920x1080p59.93", 0x7),
                0xf5: ("1920x1080p49.93", 0x7),
                0xf6: ("1920x1200p85.00", 0x3b),
                0xf7: ("1920x1200p84.93", 0x3b),
                0xf8: ("1920x1200p75.00", 0x3b),
                0xf9: ("1920x1200p74.93", 0x3b),
                0xfa: ("1920x1200p59.95", 0x3b),
                0xfb: ("1920x1200p59.88", 0x3b),
                0xfc: ("1920x1200p49.97", 0x3b),
                0xfd: ("1920x1200p49.93", 0x3b),
                0xfe: ("1920x1234p85.00", 0x3c),
                0xff: ("1920x1234p75.00", 0x3c),
                0x100: ("1920x1440p75.00", 0x3d),
                0x101: ("1920x1440p74.95", 0x3d),
                0x102: ("1920x1440p60.00", 0x3d),
                0x103: ("1920x1440p59.97", 0x3d),
                0x104: ("1920x1440p59.97", 0x3d),
                0x105: ("1920x1440p49.98", 0x3d),
                0x106: ("2048x1152p84.94", 0x3e),
                0x107: ("2048x1152p75.01", 0x3e),
                0x108: ("2048x1152p60.00", 0x3e),
                0x109: ("2048x1152p59.91", 0x3e),
                0x10a: ("2048x1152p59.90", 0x3e),
                0x10b: ("2048x1152p49.98", 0x3e),
                0x10c: ("2048x1280p74.95", 0x3f),
                0x10d: ("2048x1280p59.96", 0x3f),
                0x10e: ("2048x1280p59.92", 0x3f),
                0x10f: ("2048x1280p49.91", 0x3f),
                0x110: ("2048x1536p60.00", 0x40),
                0x111: ("2048x1536p60.00", 0x40),
                0x112: ("2048x1536p59.98", 0x40),
                0x113: ("2048x1536p59.95", 0x40),
                0x114: ("2048x1536p49.98", 0x40),
                0x115: ("2128x1200p74.98", 0x41),
                0x116: ("2128x1200p59.99", 0x41),
                0x117: ("2128x1200p59.52", 0x41),
                0x118: ("2128x1200p49.91", 0x41),
                0x119: ("2304x1440p59.96", 0x42),
                0x11a: ("2304x1440p59.94", 0x42),
                0x11b: ("2304x1440p49.99", 0x42),
                0x11c: ("2456x1536p59.94", 0x43),
                0x11d: ("2456x1536p49.95", 0x43),
                0x11e: ("2560x1440p59.95", 0x44),
                0x11f: ("2560x1440p49.96", 0x44),
                0x120: ("2560x1536p59.98", 0x45),
                0x121: ("2560x1600p59.97", 0x46),
                0x122: ("2560x1600p49.95", 0x46),
                0x123: ("2728x1536p59.99", 0x47),
                0x124: ("2728x1536p49.97", 0x47),
                0x125: ("3840x2160p29.98", 0x9),
            }.get(format)
        elif standard == KPAInputStandard.HD_SDI and not fractional:
            out, aspect = {
                0x4: ("HD 1920x1080i60", 0x7),
                0x6: ("HD 1920x1080i50", 0x7),
                0x10: ("HD 1920x1080psf30", 0x7),
                0x12: ("HD 1920x1080psf25", 0x7),
                0x13: ("HD 1920x1080psf24", 0x7),
                0x18: ("HD 2048x1080psf30", 0x8),
                0x1a: ("HD 2048x1080psf25", 0x8),
                0x1b: ("HD 2048x1080psf24", 0x8),
                0x20: ("HD 1920x1080p30", 0x7),
                0x22: ("HD 1920x1080p25", 0x7),
                0x23: ("HD 1920x1080p24", 0x7),
                0x28: ("HD 2048x1080p30", 0x8),
                0x2a: ("HD 2048x1080p25", 0x8),
                0x2b: ("HD 2048x1080p24", 0x8),
                0x30: ("HD 1280x720p60", 0x6),
                0x32: ("HD 1280x720p50", 0x6),
                0x33: ("HD 1280x720p30", 0x6),
                0x35: ("HD 1280x720p25", 0x6),
                0x36: ("HD 1280x720p24", 0x6),
            }.get(format, ("Unknown", 0))
        elif standard == KPAInputStandard.HD_SDI and fractional:
            out, aspect = {
                0x5: ("HD 1920x1080i59.94", 0x7),
                0x11: ("HD 1920x1080psf29.97", 0x7),
                0x14: ("HD 1920x1080psf23.98", 0x7),
                0x19: ("HD 2048x1080psf29.97", 0x8),
                0x1c: ("HD 2048x1080psf23.98", 0x8),
                0x21: ("HD 1920x1080p29.97", 0x7),
                0x24: ("HD 1920x1080p23.98", 0x7),
                0x29: ("HD 2048x1080p29.97", 0x8),
                0x2c: ("HD 2048x1080p23.98", 0x8),
                0x31: ("HD 1280x720p59.94", 0x6),
                0x34: ("HD 1280x720p29.97", 0x6),
                0x37: ("HD 1280x720p23.98", 0x6),
            }.get(format, ("Unknown", 0))
        elif standard == KPAInputStandard.THREEG_SDI and not fractional:
            out, aspect = {
                0x20: (threeg + "1920x1080p60", 0x7),
                0x22: (threeg + "1920x1080p50", 0x7),
                0x28: (threeg + "2048x1080p60", 0x8),
                0x2a: (threeg + "2048x1080p50", 0x8),
            }.get(format, ("Unknown", 0))
        elif standard == KPAInputStandard.THREEG_SDI and fractional:
            out, aspect = {
                0x21: (threeg + "1920x1080p59.94", 0x7),
                0x29: (threeg + "2080x1080p59.94", 0x8),
            }.get(format, ("Unknown", 0))
        elif standard == KPAInputStandard.SIXG_SDI and not fractional:
            out, aspect = {
                0x20: ("6G 3840x2160p30", 0x9),
                0x22: ("6G 3840x2160p25", 0x9),
                0x23: ("6G 3840x2160p24", 0x9),
                0x28: ("6G 4096x2160p30", 0xa),
                0x2a: ("6G 4096x2160p25", 0xa),
                0x2b: ("6G 4096x2160p24", 0xa),
            }.get(format, ("Unknown", 0))
        elif standard == KPAInputStandard.SIXG_SDI and fractional:
            out, aspect = {
                0x21: ("6G 3840x2160p29.97", 0x9),
                0x24: ("6G 3840x2160p23.98", 0x9),
                0x29: ("6G 4096x2160p29.97", 0xa),
                0x2c: ("6G 4096x2160p23.98", 0xa),
            }.get(format, ("Unknown", 0))
        elif standard == KPAInputStandard.TWELVEG_SDI and not fractional:
            out, aspect = {
                0x20: ("12G 3840x2160p60", 0x9),
                0x22: ("12G 3840x2160p50", 0x9),
                0x28: ("12G 4096x2160p60", 0xa),
                0x2a: ("12G 4096x2160p50", 0xa),
            }.get(format, ("Unknown", 0))
        elif standard == KPAInputStandard.TWELVEG_SDI and fractional:
            out, aspect = {
                0x21: ("12G 3840x2160p59.94", 0x9),
                0x29: ("12G 4096x2160p59.94", 0xa),
            }.get(format, ("Unknown", 0))
    return out


class KPA_Registers(Registers):
    """Register layout of KPA devices (12G-CROSS, etc.)."""

    __pack__ = 1
    _fields_ = [
        # 0x00
        ("magic", c_char * 2),
        ("version_major", c_ushort, 16),
        ("version_minor", c_ushort, 16),
        ("unknown06", c_ubyte * 10),
        # 0x10
        ("Input1FormatLow", c_ubyte),
        ("unknown11", c_ubyte, 1),
        ("Input1Locked", c_ubyte, 1),
        ("Input1Standard", c_ubyte, 3),
        ("Input1HDFractional", c_ubyte, 1),
        ("Input13GB", c_ubyte, 1),
        ("Input1FormatHigh", c_ubyte, 1),
        ("Input2FormatLow", c_ubyte),
        ("unknown13", c_ubyte, 1),
        ("Input2Locked", c_ubyte, 1),
        ("Input2Standard", c_ubyte, 3),
        ("Input2HDFractional", c_ubyte, 1),
        ("Input23GB", c_ubyte, 1),
        ("Input2FormatHigh", c_ubyte, 1),
        ("unknown14", c_ubyte, 4),
        ("Input2ColorSpace", c_ubyte, 2),
        ("Input2ColorPacking", c_ubyte, 2),
        ("Input3FormatLow", c_ubyte),
        ("unknown16", c_ubyte, 1),
        ("Input3Locked", c_ubyte, 1),
        ("Input3Standard", c_ubyte, 3),
        ("Input3HDFractional", c_ubyte, 1),
        ("Input33GB", c_ubyte, 1),
        ("Input3FormatHigh", c_ubyte, 1),
        ("unknown15", c_ubyte * 9),
        # 0x20
        ("unknown20b", c_ubyte, 1), # unknown sample 2
        ("SI_CS_UD_Always", c_ubyte, 1),
        ("SI_CS_3G_Always", c_ubyte, 1),
        ("unknown20", c_ubyte, 2),
        ("SI_CS_UD", c_ubyte, 1),
        ("SI_CS_3G", c_ubyte, 1),
        ("PSFINEnable", c_ubyte, 1), # & 0x1
        ("unknown21", c_ubyte, 1),
        ("HI_CS_VESA_Always", c_ubyte, 1),
        ("HI_CS_UD_Always", c_ubyte, 1),
        ("HI_CS_3G_Always", c_ubyte, 1),
        ("HI_CS_VESA", c_ubyte, 2),
        ("HI_CS_UD", c_ubyte, 1),
        ("HI_CS_3G", c_ubyte, 1),
        ("DUCFormat", c_ubyte), # 0x22 & 0x1f
        ("Aspect_SD_Input", c_ubyte), # & 0x1f
        ("Aspect_SD_Output", c_ubyte), # & 0x1f
        ("Aspect_Image_Output", c_ubyte, 3),
        ("Aspect_Image_Input", c_ubyte, 5),
        ("unknown26", c_ubyte),
        ("unknown27", c_ubyte, 5),
        ("DUC_MOT_DET_LEVEL", c_ushort, 11), # & 0x3FF
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
        ("unknown30", c_ubyte, 2),
        ("HO_YCbCr_Full", c_ubyte, 1),
        ("HO_RGB_Full", c_ubyte, 1),
        ("HO_Type", c_ubyte, 4), # & 0x7 # unknown
        ("SO_Source", c_ubyte), # & 0x3
        ("HO_Source", c_ubyte), # & 0x3
        ("unknownDUC", c_ubyte, 3),
        ("Genlock", c_ubyte, 2), # & 0x3
        ("DUC_Ref", c_ubyte, 2), # & 0x3
        ("DUC_Source", c_ubyte, 1), # & 0x1
        ("unknown34", c_ubyte), # & 0x3
        ("LCDOffTime", c_ubyte), # 5s, 15s, 30s, 1m, 5m, 10m, 30m, NEVER
        ("ReturnToStatusTime", c_ubyte), # & 0x7
        ("unknown37", c_ubyte), # & 0x3
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
    ]

    _map = {
        "Input1Standard": KPAInputStandard, # unknown
        "Input2Standard": KPAInputStandard, # unknown
        "DUCFormat": KPADUCFormat,
        "DUC_HF": KPADUC_HF, # unknown
        "SI_CS_3G": ColorSpace,
        "SI_CS_UD": ColorSpace,
        "HI_CS_VESA": VESAColorSpace,
        "HI_CS_UD": ColorSpace,
        "HI_CS_3G": ColorSpace,
        "Aspect_SD_Input": SDAspect,
        "Aspect_SD_Output": SDAspect,
        "Aspect_Image_Output": OutputImageAspect,
        "Aspect_Image_Input": InputImageAspect,
        "HO_Type": HO_Type, # unknown
        "SO_Source": SO_Source, # works
        "HO_Source": HO_Source, # works
        "DUC_Ref": DUC_Ref, # works
        "DUC_Source": DUC_Source, # works
        "Genlock": Genlock, # works
        "LCDOffTime": Time, # works
        "ReturnToStatusTime": Time, # works
        "HDMI_Pair1": KPAAudioPair,
        "HDMI_Pair2": KPAAudioPair,
        "HDMI_Pair3": KPAAudioPair,
        "HDMI_Pair4": KPAAudioPair,
        "SDI_Pair1": KPAAudioPair,
        "SDI_Pair2": KPAAudioPair,
        "SDI_Pair3": KPAAudioPair,
        "SDI_Pair4": KPAAudioPair,
        "SDI_Pair5": KPAAudioPair,
        "SDI_Pair6": KPAAudioPair,
        "SDI_Pair7": KPAAudioPair,
        "SDI_Pair8": KPAAudioPair,
    }

    @property
    def Input1Status(self):
        return KPA_CalcFormatString(self.Input1Locked, self.Input1Standard, self.Input1HDFractional, self.Input13GB, self.Input1FormatLow | ((self.Input1FormatHigh & 1) << 8))

    @property
    def Input2Status(self):
        out = KPA_CalcFormatString(self.Input2Locked, self.Input2Standard, self.Input2HDFractional, self.Input23GB, self.Input2FormatLow | ((self.Input2FormatHigh & 1) << 8))
        if self.Input2Locked:
            out += " %s%s" % (
                {
                    0: "8-bit RGB 4:4:4",
                    1: "12-bit YCbCr 4:2:2",
                    2: "8-bit YCbCr 4:4:4",
                }.get(self.Input2ColorPacking),
                {
                    1: " / ITU-R BT.2020",
                    2: " / ITU-R BT.601",
                    3: " / ITU-R BT.709",
                }.get(self.Input2ColorSpace, ""),
            )
        return out

    @property
    def Input3Status(self):
        return KPA_CalcFormatString(self.Input3Locked, self.Input3Standard, self.Input3HDFractional, self.Input33GB, self.Input3FormatLow | ((self.Input3FormatHigh & 1) << 8))
