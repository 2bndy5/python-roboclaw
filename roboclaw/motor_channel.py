"""A helper module that allows manipulating the Roboclaw object like a
:py:class:`~drivetrain.motor.BiMotor` object"""

from drivetrain import Smooth

class RoboclawMotor(Smooth):
    """This class uses a serial connection and 2 functions from the
    :py:class:`~roboclaw.RoboClaw` object to control all/any connected Roboclaws
    on the same serial UART port.

    :param bool channel: The Roboclaw module's channel to be used for PWM output
        can be `True` for motor 1 or `False` for motor 2.
    :param roboclaw.Roboclaw rc_bus: the main UART serial bus to be used as a default means of
        communicating to the Roboclaw(s). You need only instantiate the object for
        this parameter once if all roboclaws are attached to the same port.
        If using multiple USB ports, this parameter holds the object instantiated on
        that port with the :py:attr:`~roboclaw.Roboclaw.address` attrubte configured accrdingly.
    :param int freq: The frewquency (in Hz) of the PWM output. Defaults to 500 Hz.
    :param int value: The initial duty cycle of the PWM output. Ranges [0, 65535]
    """
    def __init__(self, channel, rc_bus, value=0, ramp_time=2):
        if rc_bus is None:
            raise AttributeError('roboclaw has no serial bus designated by'
                                 ' parameter "rc_bus"')
        self._rc_bus = rc_bus
        self._value = value
        self._channel = channel
        super(RoboclawMotor, self).__init__(ramp_time)

    @property
    def value(self):
        """This attribute contains the current output value of the solenoid(s) in range
        [-65535, 65535]. An invalid input value will be clamped to an `int` in the proper range.
        A negative value represents the motor's speed in reverse rotation. A positive value
        reprsents the motor's speed in forward rotation."""
        return self._value

    @value.setter
    def value(self, val):
        self._value = min(65535, max(-65535, int(val)))
        if self._channel:
            self._rc_bus.duty_m1(int(self._value / 2))
        else:
            self._rc_bus.duty_m2(int(self._value / 2))

    def deinit(self):
        """de-initialize the serial object on its port for future use."""
        self._rc_bus.duty_m1_m2(0, 0)
        self._rc_bus.serial_obj.deinit()

    def __del__(self):
        self._rc_bus.serial_obj.deinit()
