"""This module contains a wrapper class for MicroPython's machine.UART class to
utilize python's context manager"""
# pylint: disable=import-error
from machine import UART

class SerialUART(UART):
    """A wrapper class for MicroPython's machine.UART class to utilize python's context
    manager. This wrapper may be incomplete as it is specialized for use with this library
    only as a drop-in replacement for CircuitPython's `busio.UART` or PySerial's
    `~serial.Serial` module API.

    :param ~microcontroller.Pin tx: The pin used for sending data.
    :param ~microcontroller.Pin rx: The pin used for receiving data.
    :param int baudrate: The baudrate of the Serial port. Defaults to ``9600``.
    :param int bits: The number of bits per byte. Options are limited to ``8`` or ``9``.
        Defaults to ``8``.
    :param int parity: This parameter is optional. The parity controls how the bytes are
        handled with respect the raising or falling edge of the clock signal. Options are
        limited to ``None``, ``0`` (even), or ``1`` (odd). Defaults to ``None``.
    :param int stop: The number of stop bits to be used to signify the end of the buffer
        payload (kinda like the null character in a C-style string). Options are limited to
        ``1`` or ``2``. Defaults to ``1``.
    """
    def __init__(self, tx=None, rx=None, baudrate=9600, bits=8, parity=None, stop=1):
        super(SerialUART, self).__init__(
            baudrate=baudrate, bits=bits, parity=parity, stop=stop, tx=tx, rx=rx
        )

    def __enter__(self):
        self.init(
            baudrate=self.baudrate,
            bits=self.bits,
            parity=self.parity,
            stop=self.stop,
            tx=self.tx,
            rx=self.rx
        )

    def __exit__(self, *exc):
        self.deinit()
        return False

    def in_waiting(self):
        """The number of bytes waiting to be read on the open Serial port."""
        return self.any()
