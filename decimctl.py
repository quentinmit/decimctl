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
from ctypes import byref

import protocol

def now():
    return datetime.datetime.now().isoformat()

class Decimator(pylibftdi.device.Device):
    def __init__(self, log_raw_data=False):
        self.log_file = None
        if log_raw_data:
            self.log_file = open("%s.raw" % now(), 'wb')
        super(Decimator, self).__init__(mode='b')

    def _open_device(self):
        return self.fdll.ftdi_usb_open_desc_index(byref(self.ctx), 8543, 24576, None, None, 0)

    def open(self):
        if self._opened:
            return
        super(Decimator, self).open()

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
        

    def set_SO_Source(self, source):
        # CPA_SO_Source = register 0x31, & 0x3
        self.fpga_write_bytes(self.CPA_SO_Source, pack('>B', source & 0x3))

    CPA_HO_Type = 0x30 # & 0x7
    CPA_SO_Source = 0x31 # & 0x3
    CPA_HO_Source = 0x32 # & 0x3
    CPA_DUC_Source = 0x33 # bottom bit = SDI/HDMI as source, top 3 bits = Source, Free Run, SDI IN, HDMI IN
    CPA_LCDOffTime = 0x35 # & 0x7
    # 5s, 15s, 30s, 1m, 5m, 10m, 30m, NEVER
    CPA_Loop_Enable = 0x2b # & 0x1
    CPA_DUCFormat = 0x22 # & 0x1f
    
if __name__ == "__main__":
    print (pylibftdi.driver.Driver().list_devices())

    os.chdir("logs")

    with Decimator(log_raw_data=True) as dev:
        dev.set_SO_Source(0)

        status_bytes = dev.fpga_read_bytes(0, 0x200)
        open('%s-status.dat' % now(), 'wb').write(status_bytes)
        print (status_bytes)

        print (protocol.CPA_Registers.from_buffer_copy(status_bytes))

        if False:
            import time
            while True:
                time.sleep(1)
                dev.set_SO_Source(1)
                time.sleep(1)
                dev.set_SO_Source(0)
