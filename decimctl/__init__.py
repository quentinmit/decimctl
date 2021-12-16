"""
Module for interfacing with Decimator Design products.

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

import os
import datetime
from struct import pack
from itertools import chain
from functools import reduce
from ctypes import byref, sizeof, cast, c_char_p, c_void_p, pointer, POINTER, Structure, create_string_buffer
import pylibftdi.driver
import pylibftdi.device
from enum import Flag

from . import protocol

VID = 0x215f
PID = 0x6000

# TODO: Push a patch upstream to allow pylibftdi to accept the VID/PID at runtime.
pylibftdi.driver.USB_VID_LIST = [VID]
pylibftdi.driver.USB_PID_LIST = [PID]
pylibftdi.device.USB_VID_LIST = pylibftdi.driver.USB_VID_LIST
pylibftdi.device.USB_PID_LIST = pylibftdi.driver.USB_PID_LIST
pylibftdi.device.ERR_HELP_NOT_FOUND_FAIL = """
No device could be found. Is the device connected?

Try running the following command to see if the device is listed:

    decimctl list
"""

def now():
    return datetime.datetime.now().isoformat()

def list_devices():
    return pylibftdi.driver.Driver().list_devices()

class ftdi_context(Structure):
    _fields_ = [
        ('usb_ctx', c_void_p),
        ('usb_dev', c_void_p),
    ]

# N.B. enum.auto does not allow combining with int
def auto(state=[1]):
    out = state[0]
    state[0] = out<<1
    return out

class DeviceType(Flag):
    DHA = auto()
    DUC = auto()
    CPA = auto()
    CPA_HX = CPA | auto()
    CPA_HX_V1 = CPA_HX | auto()
    CPA_HX_V2 = CPA_HX | auto()
    CPA_CROSS = CPA | auto()
    CPA_CROSS_V1 = CPA_CROSS | auto()
    CPA_CROSS_V2 = CPA_CROSS | auto()
    CPA_CROSS_V3 = CPA_CROSS | auto()
    CPA_LX = CPA | auto()
    MQS = auto()
    MQA = auto()
    VFA = auto()
    KPA = auto()
    SECTOR_4K = auto()
    SECTOR_64K = auto()
    SECTOR_256K = auto()
    # SECTOR_4K if ??B, SECTOR_64K if ??A
    VFA_VFA = VFA | auto() # 0x6 windows
    VFA_VLA = VFA | auto() # 0xc windows
    VFA_MPA = VFA | auto() # 0x10 windows
    VFA_VPA = VFA | auto() # 0x10 windows
    # SECTOR_4K if MDB, SECTOR_256K if MDA
    VFA_MDA = VFA | auto() # 0x4 windows

    VFA_MQC = VFA | SECTOR_64K | auto() # 0x4 windows
    VFA_MQD = VFA | SECTOR_4K | auto() # 0x4 windows, DMON-QUAD
    VFA_VHA = VFA | SECTOR_4K | auto() # 0x4 windows
    VFA_OAA = VFA | SECTOR_4K | auto() # 0x9 windows
    VFA_OBA = VFA | SECTOR_4K | auto() # 0x4 windows


_SERIAL_PREFIX_TO_TYPE = {
    b'DHA': DeviceType.DHA,
    b'DHB': DeviceType.DHA,
    b'DUC': DeviceType.DUC,
    b'CLA': DeviceType.CPA_HX_V1,
    b'LLA': DeviceType.CPA_HX_V1,
    b'CLB': DeviceType.CPA_HX_V1,
    b'LLB': DeviceType.CPA_HX_V1,
    b'CLC': DeviceType.CPA_HX_V1,
    b'LLC': DeviceType.CPA_HX_V1,
    b'CPA': DeviceType.CPA_CROSS_V1,
    b'LPA': DeviceType.CPA_CROSS_V1,
    b'CPB': DeviceType.CPA_CROSS_V2,
    b'LPB': DeviceType.CPA_CROSS_V2,
    b'CPC': DeviceType.CPA_CROSS_V2,
    b'LPC': DeviceType.CPA_CROSS_V2,
    # Nothing for CPA_CROSS_V3 yet?
    b'CXA': DeviceType.CPA_LX,
    b'LXA': DeviceType.CPA_LX,
    b'CXB': DeviceType.CPA_LX,
    b'LXB': DeviceType.CPA_LX,
    b'MQS': DeviceType.MQS,
    b'MQA': DeviceType.MQA,
    b'MQB': DeviceType.MQA,
    b'MQC': DeviceType.VFA_MQC,
    b'MQD': DeviceType.VFA_MQD, # DMON-QUAD
    b'MDA': DeviceType.VFA_MDA | DeviceType.SECTOR_256K,
    b'MDB': DeviceType.VFA_MDA | DeviceType.SECTOR_4K,
    b'MPA': DeviceType.VFA_MPA | DeviceType.SECTOR_64K,
    b'MPB': DeviceType.VFA_MPA | DeviceType.SECTOR_4K,
    b'VFA': DeviceType.VFA_VFA | DeviceType.SECTOR_64K,
    b'VFB': DeviceType.VFA_VFA | DeviceType.SECTOR_4K,
    b'VLA': DeviceType.VFA_VLA | DeviceType.SECTOR_64K,
    b'VLB': DeviceType.VFA_VLA | DeviceType.SECTOR_4K,
    b'VPA': DeviceType.VFA_VPA | DeviceType.SECTOR_64K,
    b'VPB': DeviceType.VFA_VPA | DeviceType.SECTOR_4K,
    b'OAA': DeviceType.VFA_OAA,
    b'OBA': DeviceType.VFA_OBA,
    b'KPA': DeviceType.KPA,
}

class Device(pylibftdi.device.Device):
    def __init__(self, log_raw_data=False, serial=None):
        self.log_file = None
        if log_raw_data:
            self.log_file = open("%s.raw" % now(), 'wb')
        self._requested_serial = serial
        self._serial = create_string_buffer(128)
        self._desc = create_string_buffer(128)
        self.custom_name = None

        super(Device, self).__init__(mode='b')

    def _open_device(self):
        serial = None
        if self._requested_serial:
            serial = c_char_p(self._requested_serial.encode('ascii'))
        return self.fdll.ftdi_usb_open_desc_index(byref(self.ctx), VID, PID, None, serial, 0)

    def _get_info(self):
        self.fdll.ftdi_usb_get_strings2.argtypes = self.fdll.ftdi_usb_get_strings.argtypes
        usb_dev_handle = cast(pointer(self.ctx), POINTER(ftdi_context)).contents.usb_dev
        self.fdll.libusb_get_device.argtypes = (c_void_p,)
        self.fdll.libusb_get_device.restype = c_void_p
        usb_dev = self.fdll.libusb_get_device(usb_dev_handle)
        self.ftdi_fn.ftdi_usb_get_strings2(usb_dev, None, 0, self._desc, 127, self._serial, 127)

        eeprom = create_string_buffer(128)
        self.ftdi_fn.ftdi_read_eeprom()
        self.ftdi_fn.ftdi_get_eeprom_buf(byref(eeprom), len(eeprom))
        self.custom_name = create_string_buffer(eeprom[92:92+16]).value

    def open(self):
        if self._opened:
            return
        super(Device, self).open()

        self._get_info()

        # FT_ResetDevice()
        self.ftdi_fn.ftdi_usb_reset()
        # FT_SetBaudRate(3000000)
        self.baudrate = 3000000
        # FT_SetUSBParameters(65535, 65535)
        self.ftdi_fn.ftdi_read_data_set_chunksize(65535)
        # FT_SetChars(0, 0, 0, 0)
        self.ftdi_fn.ftdi_set_error_char(0, 0)
        self.ftdi_fn.ftdi_set_event_char(0, 0)
        # FT_SetTimeouts(0, 5000) - readTimeout, writeTimeout
        # FT_SetLatencyTimer(1)
        self.ftdi_fn.ftdi_set_latency_timer(1)
        # FT_SetFlowControl(256, 0, 0)
        SIO_RTS_CTS_HS = 0x100
        self.ftdi_fn.ftdi_setflowctrl(SIO_RTS_CTS_HS)
        # FT_SetBitMode(0, 0)
        self.ftdi_fn.ftdi_set_bitmode(0, 0)
        # FT_SetBitMode(0, 4)
        self.ftdi_fn.ftdi_set_bitmode(0, 4)
        # FT_Write("H")
        # + Block until FT_GetStatus returns 1
        self.clock_raw_bytes(b'\x48')
        # FT_SetBitMode(72, 4)
        # This sets the FPGA clock and data pins to be outputs (in to the FPGA).
        self.ftdi_fn.ftdi_set_bitmode(0x48, 4)

    def close(self):
        # FT_SetBitMode(0, 0)
        self.ftdi_fn.ftdi_set_bitmode(0, 0)
        # FT_Close()
        super(Device, self).close()

    @property
    def serial(self):
        return self._serial.value

    @property
    def desc(self):
        return self._desc.value

    # Number of bytes to write/read at a time. If its buffer
    # overflows, the FTDI chip will block on writes.
    CHUNK_SIZE = 256
    
    def clock_raw_bytes(self, data_in):
        """Clock in raw bytes and return the corresponding output.

        Args:
          data_in: bytes
        Returns:
          data_out: bytes
        """
        out = bytes()
        for i in range(0, len(data_in), self.CHUNK_SIZE):
            to_send = data_in[i:i+self.CHUNK_SIZE]
            self.write(to_send)
            out += self.read(len(to_send))
        if self.log_file:
            self.log_file.write(bytes(bytearray(chain.from_iterable(zip(data_in, out)))))
        return out

    def fpga_write_bytes(self, register, value):
        # @TUSB@FPGA_WRITE_BYTES$q ui puc ui ui
        # CPA_SO_Source calls @TUSB@FPGA_WRITE_BYTES(0, &value, 0x31, 0x1) where value is 0->3

        buf = bytearray(protocol.WRITE_PREAMBLE)

        address = (register<<1) & 0xfffe

        buf.extend(protocol.bytes_to_raw_command(pack(">H", address)))

        buf.extend(protocol.bytes_to_raw_command(value))

        buf.extend(protocol.WRITE_POSTAMBLE)

        return self.clock_raw_bytes(bytes(buf))

    def fpga_read_bytes(self, start, length):
        # @TUSB@FPGA_READ_BYTES$q ui puc ui ui o
        # UpdateControls calls @TUSB@FPGA_READ_BYTES(device, &rx_reg, 0, 0x200, True)
        buf = bytearray(protocol.READ_PREAMBLE)

        address = (start<<1) | 1

        buf.extend(protocol.bytes_to_raw_command(pack(">H", address)))

        self.clock_raw_bytes(bytes(buf))

        # FT_Purge(3)
        self.flush()
        # FT_SetBitMode(64, 4)
        self.ftdi_fn.ftdi_set_bitmode(0x40, 4)


        # Toggle the clock line once for each bit.
        buf = b'\x00\x40\x40\x00'*(length<<3)
        data = protocol.raw_response_to_bytes(self.clock_raw_bytes(buf))

        # FT_Write([0])
        # + Block until FT_GetStatus() = 1
        self.clock_raw_bytes(b'\0')
        # FT_SetBitMode(72, 4)
        self.ftdi_fn.ftdi_set_bitmode(0x48, 4)
        # FT_Write(00 40 48)
        # + Block until FT_GetStatus() = 3
        self.clock_raw_bytes(b'\x00\x40\x48')
        return data

    @property
    def raw_registers(self):
        return self.fpga_read_bytes(0, 0x200)

    def _registers(self, type):
        ret = type.from_buffer_copy(self.raw_registers)
        ret._device = self
        return ret

    @property
    def CPA(self):
        return self._registers(protocol.CPA_Registers)

    @property
    def VFA(self):
        return self._registers(protocol.VFA_Registers)

    @property
    def KPA(self):
        return self._registers(protocol.KPA_Registers)

    @property
    def device_type(self):
        serial_3 = self.serial[:3]
        return _SERIAL_PREFIX_TO_TYPE.get(serial_3)

    @property
    def registers(self):
        if self.device_type & DeviceType.DHA:
            raise NotImplementedError('DHA not supported')
        elif self.device_type & DeviceType.DUC:
            raise NotImplementedError('DUC not supported')
        elif self.device_type & DeviceType.CPA:
            return self.CPA
        elif self.device_type & DeviceType.MQS:
            raise NotImplementedError('MQS not supported')
        elif self.device_type & DeviceType.MQA:
            raise NotImplementedError('MQA not supported')
        elif self.device_type & DeviceType.VFA:
            return self.VFA
        elif self.device_type & DeviceType.KPA:
            return self.KPA
        else:
            raise NotImplementedError('unrecognized serial prefix %s' % self.serial[:3])

    # To identify the type, look at the first 3 characters of the serial number
    # DHA, DHB => DHA
    # DUC => DUC
    # CLA, LLA => CPA(0) # _CPA_HX_V1
    # CLB, LLB => CPA(1) # _CPA_HX_V1
    # CLC, LLC => CPA(2) # _CPA_HX_V1
    # CLD, LLD => CPA(3) # _CPA_HX_V2
    # CPA, LPA => CPA(4) # _CPA_CROSS_V1
    # CPB, LPB => CPA(5) # _CPA_CROSS_V2
    # CPC, LPC => CPA(6) # _CPA_CROSS_V2
    # ??? => CPA(7) # _CPA_CROSS_V3
    # CXA, LXA => CPA(8) # _CPA_LX
    # CXB, LXB => CPA(9) # _CPA_LX
    # MQS => MQS
    # MQA, MQB => MQA
    # M, V =>
    # MQC => VFA(4,0,0)
    # MQD => VFA(4,2,0)
    # MDA => VFA(4,1,0)
    # MDB => VFA(4,1,1)
    # MPA => VFA(16,1,0)
    # MPB => VFA(16,1,1)
    # VFA => VFA(6,0,0)
    # VFB => VFA(6,0,1)
    # VLA => VFA(12,0,0)
    # VLB => VFA(12,0,1)
    # VPA => VFA(16,0,0)
    # VPB => VFA(16,0,1)
    # OAA => VFA(9,0,0)
    # OBA => VFA(4,4,0)
    
if __name__ == "__main__":
    print(list_devices())

    os.chdir("logs")

    with Device(log_raw_data=True) as dev:
        print(dev.serial)
        dev.CPA.SO_Source = 0

        status_bytes = dev.raw_registers
        open('%s-status.dat' % now(), 'wb').write(status_bytes)
        print(status_bytes)

        print(dev.CPA)

        if False:
            import time
            while True:
                time.sleep(1)
                dev.CPA.SO_Source = 1
                time.sleep(1)
                dev.CPA.SO_Source = 0
