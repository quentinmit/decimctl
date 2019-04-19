#!/bin/env python

import pylibftdi.driver
pylibftdi.driver.USB_VID_LIST = [8543]
pylibftdi.driver.USB_PID_LIST = [24576]
import pylibftdi.device

print pylibftdi.driver.Driver().list_device()

with pylibftdi.device.Device(device_id="CLA42882", mode='b') as dev:
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
    dev.ftdi_in.ftdi_setflowctrl(SIO_RTS_CTS_HS)
    # FT_SetBitMode(0, 0)
    dev.ftdi_in.ftdi_set_bitmode(0, 0)
    # FT_SetBitMode(0, 4)
    dev.ftdi_in.ftdi_set_bitmode(0, 4)
    # FT_Write("H")
    dev.write('H')
    # Block until FT_GetStatus returns >0
    dev.read(1)
    # FT_SetBitMode(72, 4)
    dev.ftdi_fn.ftdi_set_bitmode(0x48, 4)
    # FT_Write(00 40 00 40 48 48 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 00 40 00 08 48 08)
    data = [0x00, 0x40, 0x00, 0x40, 0x48, 0x48, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x08, 0x48, 0x08]
    dev.write(bytes(bytearray(data)))
    # Block until FT_GetStatus returns len(data)
    dev.read(len(data))
    # FT_SetBitMode(64, 4)
    dev.ftdi_in.ftdi_set_bitmode(0x40, 4)
    # FT_Write(16384 chars)
    data = [0x00, 0x40, 0x40, 0x00]*8192
    dev.write(bytes(bytearray(data)))
    # FT_Read(16384)
    status_raw = dev.read(16384)
    print status_raw
    # FT_Write([0])
    dev.write(b'\0')
    # Block until FT_GetStatus() = 1
    dev.read(1)
    # FT_SetBitMode(72, 4)
    dev.ftdi_in.ftdi_set_bitmode(0x48, 4)
    # FT_Write(00 40 48)
    data = [0x00, 0x40, 0x48]
    dev.write(bytes(bytearray(data)))
    # Block until FT_GetStatus() = 3
    dev.read(3)
    # FT_SetBitMode(0, 0)
    dev.ftdi_fn.ftdi_set_bitmode(0, 0)
    # FT_Close()
    
    
