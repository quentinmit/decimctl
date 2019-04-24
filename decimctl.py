#!/usr/bin/env python3

import os
import datetime
from struct import pack
from itertools import chain
from functools import reduce
import pylibftdi.driver
pylibftdi.driver.USB_VID_LIST = [8543]
pylibftdi.driver.USB_PID_LIST = [24576]
import pylibftdi.device
pylibftdi.device.USB_VID_LIST = pylibftdi.driver.USB_VID_LIST
pylibftdi.device.USB_PID_LIST = pylibftdi.device.USB_VID_LIST
from ctypes import byref, sizeof, cast, c_void_p, pointer, POINTER, Structure, create_string_buffer

import protocol

def now():
    return datetime.datetime.now().isoformat()

class ftdi_context(Structure):
    _fields_ = [
        ('usb_ctx', c_void_p),
        ('usb_dev', c_void_p),
    ]

class Decimator(pylibftdi.device.Device):
    def __init__(self, log_raw_data=False):
        self.log_file = None
        if log_raw_data:
            self.log_file = open("%s.raw" % now(), 'wb')
        self._serial = create_string_buffer(128)
        self._desc = create_string_buffer(128)

        super(Decimator, self).__init__(mode='b')

    def _open_device(self):
        return self.fdll.ftdi_usb_open_desc_index(byref(self.ctx), 8543, 24576, None, None, 0)

    def _get_info(self):
        self.fdll.ftdi_usb_get_strings2.argtypes = self.fdll.ftdi_usb_get_strings.argtypes
        usb_dev_handle = cast(pointer(self.ctx), POINTER(ftdi_context)).contents.usb_dev
        self.fdll.libusb_get_device.argtypes = (c_void_p,)
        self.fdll.libusb_get_device.restype = c_void_p
        usb_dev = self.fdll.libusb_get_device(usb_dev_handle)
        self.ftdi_fn.ftdi_usb_get_strings2(usb_dev, None, 0, self._desc, 127, self._serial, 127)

    def open(self):
        if self._opened:
            return
        super(Decimator, self).open()

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
        super(Decimator, self).close()

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

    @property
    def CPA(self):
        ret = protocol.CPA_Registers.from_buffer_copy(self.raw_registers)
        ret._device = self
        return ret

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
    print (pylibftdi.driver.Driver().list_devices())

    os.chdir("logs")

    with Decimator(log_raw_data=True) as dev:
        print(dev.serial)
        dev.CPA.SO_Source = 0

        status_bytes = dev.raw_registers
        open('%s-status.dat' % now(), 'wb').write(status_bytes)
        print (status_bytes)

        print (dev.CPA)

        if False:
            import time
            while True:
                time.sleep(1)
                dev.CPA.SO_Source = 1
                time.sleep(1)
                dev.CPA.SO_Source = 0
