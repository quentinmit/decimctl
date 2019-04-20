#!/bin/env python3

import datetime
import itertools
import pylibftdi.driver
pylibftdi.driver.USB_VID_LIST = [8543]
pylibftdi.driver.USB_PID_LIST = [24576]
import pylibftdi.device
pylibftdi.device.USB_VID_LIST = pylibftdi.driver.USB_VID_LIST
pylibftdi.device.USB_PID_LIST = pylibftdi.device.USB_VID_LIST
from ctypes import byref

print (pylibftdi.driver.Driver().list_devices())

class Decimator(pylibftdi.device.Device):
    def __init__(self, log_raw_data=False):
        super(Decimator, self).__init__(mode='b')
        self.log_file = None
        if log_raw_data:
            self.log_file = open("%s.raw" % (datetime.datetime.now().isoformat(),), 'wb')

    def _open_device(self):
        return self.fdll.ftdi_usb_open_desc_index(byref(self.ctx), 8543, 24576, None, None, 0)

    # Number of bytes to write/read at a time.
    CHUNK_SIZE = 256
    
    def clock_raw_bytes(self, data_in):
        """Clocks in raw bytes and returns the corresponding output.

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
            self.log_file.write(bytes(bytearray(itertools.chain.from_iterable(zip(data_in, out)))))
        return out

    @staticmethod
    def _raw_to_bytes(raw):
        status_bits = []
        for i in range(0, len(raw), 4):
            if raw[i+2] != raw[i+3]:
                print ("difference at bit", i)
            status_bits.append(bool(raw[i+2] & 0x8))
        status_bytes = bytearray()
        for i in range(0, len(status_bits), 8):
            bitstr = ''.join([str(int(x)) for x in status_bits[i:i+8]])
            b = int(bitstr, 2)
            status_bytes.append(b)
            print (bitstr, b, chr(int(bitstr, 2)))
        return status_bytes

    def read_bytes(self, n):
        data = b'\x00\x40\x40\x00'*4096
        raw = dev.clock_raw_bytes(data)
        return self._raw_to_bytes(raw)

with Decimator(log_raw_data=True) as dev:
    # FT_ResetDevice()
    dev.ftdi_fn.ftdi_usb_reset()
    # FT_SetBaudRate(3000000)
    dev.baudrate = 3000000
    # FT_SetUSBParameters(65535, 65535)
    dev.ftdi_fn.ftdi_read_data_set_chunksize(65535)
    # FT_SetChars(0, 0, 0, 0)
    dev.ftdi_fn.ftdi_set_error_char(0, 0)
    dev.ftdi_fn.ftdi_set_event_char(0, 0)
    # FT_SetTimeouts(0, 5000) - readTimeout, writeTimeout
    # FT_SetLatencyTimer(1)
    dev.ftdi_fn.ftdi_set_latency_timer(1)
    # FT_SetFlowControl(256, 0, 0)
    SIO_RTS_CTS_HS = 0x100
    dev.ftdi_fn.ftdi_setflowctrl(SIO_RTS_CTS_HS)
    # FT_SetBitMode(0, 0)
    dev.ftdi_fn.ftdi_set_bitmode(0, 0)
    # FT_SetBitMode(0, 4)
    dev.ftdi_fn.ftdi_set_bitmode(0, 4)
    # FT_Write("H")
    # + Block until FT_GetStatus returns 1
    print (dev.clock_raw_bytes(b'\x48'))
    # FT_SetBitMode(72, 4)
    dev.ftdi_fn.ftdi_set_bitmode(0x48, 4)
    # FT_Write(00 40 00 40 48 48 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 08 48 08)
    # + Block until FT_GetStatus returns len(data)=56
    print (dev.clock_raw_bytes(b'\x00\x40\x00\x40\x48\x48\x40' + b'\x00\x00\x40'*15 + b'\x00\x08\x48\x08'))
    # FT_Purge(3)
    dev.flush()
    # FT_SetBitMode(64, 4)
    dev.ftdi_fn.ftdi_set_bitmode(0x40, 4)
    # FT_Write(16384 chars)
    # + FT_Read(16384)
    status_bytes = dev.read_bytes(4096)
    print (status_bytes)
    # FT_Write([0])
    # + Block until FT_GetStatus() = 1
    dev.clock_raw_bytes(b'\0')
    # FT_SetBitMode(72, 4)
    dev.ftdi_fn.ftdi_set_bitmode(0x48, 4)
    # FT_Write(00 40 48)
    # + Block until FT_GetStatus() = 3
    dev.clock_raw_bytes(b'\x00\x40\x48')
    # FT_SetBitMode(0, 0)
    dev.ftdi_fn.ftdi_set_bitmode(0, 0)
    # FT_Close()
    
    
