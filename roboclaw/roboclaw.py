"""roboclaw driver module contains the roboclaw driver class that controls
the roboclaw via a UART serial"""
import time
import os
from struct import pack, unpack
from .serial_commands import Cmd
from .data_manip import crc16

# pylint: disable=line-too-long,invalid-name

class Roboclaw:
    """A driver class for the RoboClaw Motor Controller device.

    :param ~serial.Serial serial_obj: The serial obj associated with the serial port that is connected to the RoboClaw.
    :param int address: The unique address assigned to the particular RoboClaw. Valid addresses range [`0x80``, ``0x87``].
    :param int retries: The amount of attempts to read/write data over the serial port. Defaults to 3.
    """
    def __init__(self, serial_obj, address, retries=3):
        self._port = serial_obj
        if self._port.is_open:
            self._port.close()
        self._retries = retries
        self._address = bytes([address])

    def _send(self, buf, crc=16):
        buf = self._address + buf
        if crc == 16:
            buf += crc16(buf)
        with self._port as ser:
            ser.write(buf)

    def _recv(self, buf, length, crc=16):
        self._send(buf, crc)
        if crc == 16:
            buf += crc16(buf)
        with self._port as ser:
            ser.write(buf)
            buf = ser.read_until(length + (2 if crc == 16 else 0))
            # TODO perform crc validation here
        return buf[:length]

    # User accessible functions
    def send_random_data(self, cnt):
        """Send some randomly generated data of of a certain length.

        :param int cnt: the number of bytes to randomly generate."""
        self._send(os.urandom(cnt))

    def forward_m1(self, val):
        """Drive motor 1 forward.

        :param int val: Valid data range is 0 - 127. A value of 127 = full speed forward, 64 = about half speed forward and 0 = full stop.
        """
        # :Sends: [Address, 0, Value]
        return self._send(pack('>bb', Cmd.M1FORWARD, val))

    def backward_m1(self, val):
        """Drive motor 1 backwards.

        :param int val: Valid data range is 0 - 127. A value of 127 full speed backwards, 64 = about half speed backward and 0 = full stop.
        """
        # :Sends: [Address, 1, Value]
        return self._send(pack('>bb', Cmd.M1BACKWARD, val))

    def set_min_voltage_main_battery(self, val):
        """Sets main battery (B- / B+) minimum voltage level. If the battery voltages drops below the set voltage level, RoboClaw will stop driving the motors. The voltage is set in .2 volt increments. The minimum value allowed which is 6V.

        :param float val:The valid data range is [6, 34] Volts.
        """
        # :Sends: [Address, 2, Value]
        # translated byte value range = [0, 140]
        # The formula for calculating the voltage is: (Desired Volts - 6) x 5 = Value.
        # Examples of valid values are 6V = 0, 8V = 10 and 11V = 25.
        return self._send(pack('>bb', Cmd.SETMINMB, int(val / 5 + 6)))

    def set_max_voltage_main_battery(self, val):
        """Sets main battery (B- / B+) maximum voltage level. During regenerative breaking a back voltage is applied to charge the battery. When using a power supply, by setting the maximum voltage level, RoboClaw will, before exceeding it, go into hard braking mode until the voltage drops below the maximum value set. This will prevent overvoltage conditions when using power supplies.

        :param float val: The valid data range is [6, 34] Volts.
        """
        # :Sends: [Address, 3, Value]
        # translated byte value range = [30, 175]
        # The formula for calculating the voltage is: Desired Volts x 5.12 = Value.
        # Examples of valid values are 12V = 62, 16V = 82 and 24V = 123.
        return self._send(pack('>bb', Cmd.SETMAXMB, int(val / 5.12)))

    def forward_m2(self, val):
        """Drive motor 2 forward.

        :param int val: Valid data range is [0, 127]. A value of 127 full speed forward, 64 = about half speed forward and 0 = full stop.
        """
        # :Sends: [Address, 4, Value]
        return self._send(pack('>bb', Cmd.M2FORWARD, val))

    def backward_m2(self, val):
        """Drive motor 2 backwards.

        :param int val: Valid data range is [0, 127]. A value of 127 full speed backwards, 64 = about half speed backward and 0 = full stop.
        """
        # :Sends: [Address, 5, Value]
        return self._send(pack('>bb', Cmd.M2BACKWARD, val))

    def forward_backward_m1(self, val):
        """Drive motor 1 forward or reverse.

        :param int val: Valid data range is [0, 127]. A value of 0 = full speed reverse, 64 = stop and 127 = full speed forward.
        :Sends: [Address, 6, Value]
        """
        return self._send(pack('>bb', Cmd.M17BIT, val))

    def forward_backward_m2(self, val):
        """Drive motor 2 forward or reverse.

        :param int val: Valid data range is [0, 127]. A value of 0 = full speed reverse, 64 = stop and 127 = full speed forward.
        """
        # :Sends: [Address, 7, Value]
        return self._send(pack('>bb', Cmd.M27BIT, val))

    def forward_mixed(self, val):
        """Drive forward in mix mode.

        :param int val: Valid data range is [0, 127]. A value of 0 = full stop and 127 = full forward.
        """
        # :Sends: [Address, 8, Value]
        return self._send(pack('>bb', Cmd.MIXEDFORWARD, val))

    def backward_mixed(self, val):
        """Drive backwards in mix mode.

        :param int val: Valid data range is [0, 127]. A value of 0 = full stop and 127 = full reverse.
        :Sends: [Address, 9, Value]
        """
        return self._send(pack('>bb', Cmd.MIXEDBACKWARD, val))

    def turn_right_mixed(self, val):
        """Turn right in mix mode.

        :param int val: Valid data range is [0, 127]. A value of 0 = stop turn and 127 = full speed turn.
        """
        # :Sends: [Address, 10, Value]
        return self._send(pack('>bb', Cmd.MIXEDRIGHT, val))

    def turn_left_mixed(self, val):
        """Turn left in mix mode.

        :param int val: Valid data range is [0, 127]. A value of 0 = stop turn and 127 = full speed turn.
        """
        # :Sends: [Address, 11, Value]
        return self._send(pack('>bb', Cmd.MIXEDLEFT, val))

    def forward_backward_mixed(self, val):
        """Drive forward or backwards.

        :param int val: Valid data range is [0, 127]. A value of 0 = full backward, 64 = stop and 127 = full forward.
        """
        # :Sends: [Address, 12, Value]
        return self._send(pack('>bb', Cmd.MIXEDFB, val))

    def left_right_mixed(self, val):
        """Turn left or right.

        "param int val: Valid data range is [0, 127]. A value of 0 = full left, 64 = stop turn and 127 = full right.
        :Sends: [Address, 13, Value]
        """
        return self._send(pack('>bb', Cmd.MIXEDLR, val))

    def read_encoder_m1(self):
        """ Read M1 encoder count/position.

        :Returns: [Enc1(4 bytes), Status, crc16(2 bytes)]

        Quadrature encoders have a range of 0 to 4,294,967,295. Absolute encoder values are converted from an analog voltage into a value from 0 to 4095 for the full 5.1v range.

        The status byte tracks counter underflow, direction and overflow. The byte value represents:

        * Bit0 - Counter Underflow (1= Underflow Occurred, Clear After Reading)
        * Bit1 - Direction (0 = Forward, 1 = Backwards)
        * Bit2 - Counter Overflow (1= Underflow Occurred, Clear After Reading)
        * Bit3 through Bit7 - Reserved
        """
        return self._read4_1(self._address, Cmd.GETM1ENC)

    def read_encoder_m2(self):
        """ Read M2 encoder count/position.

        :Returns: [EncoderCount(4 bytes), Status]

        Quadrature encoders have a range of 0 to 4,294,967,295. Absolute encoder values are converted from an analog voltage into a value from 0 to 4095 for the full 5.1v range.

        The Status byte tracks counter underflow, direction and overflow. The byte value represents:

        * Bit0 - Counter Underflow (1= Underflow Occurred, Cleared After Reading)
        * Bit1 - Direction (0 = Forward, 1 = Backwards)
        * Bit2 - Counter Overflow (1= Underflow Occurred, Cleared After Reading)
        * Bit3 through Bit7 - Reserved

        """
        return unpack('>Ib', self._recv(pack('>b', Cmd.GETM2ENC), 5))

    def read_speed_m1(self):
        """Read M1 counter speed. Returned value is in pulses per second. MCP keeps track of how many pulses received per second for both encoder channels.

        :Returns: [Speed(4 bytes), Status]

        Status indicates the direction (0 – forward, 1 - backward).
        """
        return unpack('>Ib', self._recv(pack('>b', Cmd.GETM1SPEED), 5))

    def read_speed_m2(self):
        """Read M2 counter speed. Returned value is in pulses per second. MCP keeps track of how many pulses received per second for both encoder channels.

        :Returns: [Speed(4 bytes), Status]

        Status indicates the direction (0 – forward, 1 - backward).
        """
        return unpack('>Ib', self._recv(pack('>b', Cmd.GETM2SPEED), 5))

    def reset_encoders(self):
        """Will reset both quadrature decoder counters to zero. This command applies to quadrature encoders only."""
        return self._send(pack('>b', Cmd.RESETENC))

    def read_version(self):
        """Read RoboClaw firmware version. Returns up to 48 bytes(depending on the Roboclaw model) and is terminated by a line feed character and a null character.

        :Returns: ["MCP266 2x60A v1.0.0",10,0]

        The command will return up to 48 bytes. The return string includes the product name and firmware version. The return string is terminated with a line feed (10) and null (0) character.
        """
        trys = self._retries
        while trys:
            self._port.flushInput()
            self._sendcommand(self._address, Cmd.GETVERSION)
            string = ""
            passed = True
            for _ in range(48):
                data = self._port.read(1)
                if data:
                    val = ord(data)
                    self._crc_update(val)
                    if not val:
                        break
                    # string += data[0]
                    string += chr(data[0])
                else:
                    passed = False
                    break
            if passed:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc & 0xFFFF == crc[1] & 0xFFFF:
                        return (1, string)
                    else:
                        time.sleep(0.01)
            trys -= 1
        return (0, 0)

    def set_enc_m1(self, cnt):
        """Set the value of the Encoder 1 register. Useful when homing motor 1. This command applies to quadrature encoders only."""
        return self._send(pack('>bI', Cmd.SETM1ENCCOUNT, cnt))

    def set_enc_m2(self, cnt):
        """Set the value of the Encoder 2 register. Useful when homing motor 2. This command applies to quadrature encoders only."""
        return self._send(pack('>bI', Cmd.SETM2ENCCOUNT, cnt))

    def read_main_battery_voltage(self):
        """Read the main battery voltage level connected to B+ and B- terminals.

        :Returns: The voltage is returned in 10ths of a volt (eg 30.0).
        """
        # :Returns: [Value(2 bytes)]The voltage is returned in 10ths of a volt(eg 300 = 30v).
        return unpack('>h', self._recv(pack('>b', Cmd.GETMBATT), 2))[0] / 10

    def read_logic_battery_voltage(self,):
        """Read a logic battery voltage level connected to LB+ and LB- terminals. The voltage is returned in 10ths of a volt(eg 50 = 5v).

        :Returns: [Value.Byte1, Value.Byte0]
        """
        return self._read2(self._address, Cmd.GETLBATT)

    def set_min_voltage_logic_battery(self, val):
        """
        Sets logic input (LB- / LB+) minimum voltage level. RoboClaw will shut down with an error if the voltage is below this level. The voltage is set in .2 volt increments. The minimum value allowed which is 6V.

        :param float val: The valid data range is [6, 34].

        .. note:: This command is included for backwards compatibility. We recommend you use `set_logic_voltages()` instead.
        """
        # translated byte value range = [0, 140]
        # The formula for calculating the voltage is: (Desired Volts - 6) x 5 = Value.
        # Examples of valid values are 6V = 0, 8V = 10 and 11V = 25.
        return self._send(pack('>bb', Cmd.SETMINLB, int(val / 5 + 6)))

    def set_max_voltage_logic_battery(self, val):
        """Sets logic input (LB- / LB+) maximum voltage level. RoboClaw will shutdown with an error if the voltage is above this level.

        :param float val: The valid data range is [6, 34].

        .. note:: This command is included for backwards compatibility. We recommend you use `set_main_voltages()` instead.
        """
        # translated byte value ranges [30, 175]
        # The formula for calculating the voltage is: Desired Volts x 5.12 = Value.
        # Examples of valid values are 12V = 62, 16V = 82 and 24V = 123.
        return self._send(pack('>bb', Cmd.SETMAXLB, int(val / 5.12)))

    def set_m1_velocity_pid(self, p, i, d, qpps):
        """Several motor and quadrature combinations can be used with RoboClaw. In some cases the default PID values will need to be tuned for the systems being driven. This gives greater flexibility in what motor and encoder combinations can be used. The RoboClaw PID system consist of four constants starting with QPPS, P = Proportional, I= Integral and D= Derivative.

        :param int p: The default P is 0x00010000.
        :param int i: The default I is 0x00008000.
        :param int d: The default D is 0x00004000.
        :param int qpps: The default QPPS is 44000.

        QPPS is the speed of the encoder when the motor is at 100% power. P, I, D are the default values used after a reset.
        """
        # :Sends: [Address, 28, D(4 bytes), P(4 bytes), I(4 bytes), QPPS(4 byte)]
        return self._send(pack('>bIIII', Cmd.SETM1PID, d * 65536, p * 65536, i * 65536, qpps))

    def set_m2_velocity_pid(self, p, i, d, qpps):
        """Several motor and quadrature combinations can be used with RoboClaw. In some cases the default PID values will need to be tuned for the systems being driven. This gives greater flexibility in what motor and encoder combinations can be used. The RoboClaw PID system consist of four constants starting with QPPS, P = Proportional, I= Integral and D= Derivative.

        :param int p: The default P is 0x00010000.
        :param int i: The default I is 0x00008000.
        :param int d: The default D is 0x00004000.
        :param int qpps: The default QPPS is 44000.

        QPPS is the speed of the encoder when the motor is at 100% power. P, I, D are the default values used after a reset.
        """
        # :Sends: [Address, 29, D(4 bytes), P(4 bytes), I(4 bytes), QPPS(4 byte)]
        return self._send(pack('>bIIII', Cmd.SETM2PID, d * 65536, p * 65536, i * 65536, qpps))

    def read_raw_speed_m1(self):
        """Read the pulses counted in that last 300th of a second. This is an unfiltered version of `read_speed_m1()`. This function can be used to make a independent PID routine. Value returned is in encoder counts per second.

        :Returns: [Speed(4 bytes), Status]

        The Status byte is direction (0 – forward, 1 - backward).
        """
        return unpack('>Ib', self._recv(pack('>b', Cmd.GETM1ISPEED), 5))

    def read_raw_speed_m2(self):
        """Read the pulses counted in that last 300th of a second. This is an unfiltered version of `read_speed_m2()`. This function can be used to make a independent PID routine. Value returned is in encoder counts per second.

        :Returns: [Speed(4 bytes), Status]

        The Status byte is direction (0 – forward, 1 - backward).
        """
        return unpack('>Ib', self._recv(pack('>b', Cmd.GETM2ISPEED), 5))

    def duty_m1(self, val):
        """Drive M1 using a duty cycle value. The duty cycle is used to control the speed of the motor without a quadrature encoder.

        :param int val: The duty value is signed and the range [-32767, 32767] (eg. +-100% duty).
        """
        # :Sends: [Address, 32, Duty(2 Bytes)]
        return self._send(pack('>bh', Cmd.M1DUTY, val))

    def duty_m2(self, val):
        """Drive M2 using a duty cycle value. The duty cycle is used to control the speed of the motor without a quadrature encoder.

        :param int val: The duty value is signed and the range [-32767, 32767] (eg. +-100% duty).
        """
        # :Sends: [Address, 33, Duty(2 Bytes)]
        return self._send(pack('>bh', Cmd.M2DUTY, val))

    def duty_m1_m2(self, m1, m2):
        """Drive both M1 and M2 using a duty cycle value. The duty cycle is used to control the speed of the motor without a quadrature encoder.

        :param int m1: The duty value is signed and the range [-32767, 32767] (eg. +-100% duty).
        :param int m2: The duty value is signed and the range [-32767, 32767] (eg. +-100% duty).
        """
        # :Sends: [Address, 34, DutyM1(2 Bytes), DutyM2(2 Bytes)]
        return self._send(pack('>bhh', Cmd.MIXEDDUTY, m1, m2))

    def speed_m1(self, val):
        """Drive M1 using a speed value. The sign indicates which direction the motor will turn. This command is used to drive the motor by quad pulses per second. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate as fast as possible until the defined rate is reached.

        :param int val: Valid input ranges [-2147483647, 2147483647].
        """
        # :Sends: [Address, 35, Speed(4 Bytes)]
        return self._send(pack('>bi', Cmd.M1SPEED, val))

    def speed_m2(self, val):
        """Drive M2 with a speed value. The sign indicates which direction the motor will turn. This command is used to drive the motor by quad pulses per second. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent, the motor will begin to accelerate as fast as possible until the rate defined is reached.

        :param int val: Valid input ranges [-2147483647, 2147483647].
        """
        # :Sends: [Address, 36, Speed(4 Bytes)]
        return self._send(pack('>bi', Cmd.M2SPEED, val))

    def speed_m1_m2(self, m1, m2):
        """Drive M1 and M2 in the same command using a signed speed value. The sign indicates which direction the motor will turn. This command is used to drive the motor by quad pulses per second. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate as fast as possible until the rate defined is reached.

        :param int m1: Valid input ranges [-2147483647, 2147483647].
        :param int m2: Valid input ranges [-2147483647, 2147483647].
        """
        # :Sends: [Address, 37, SpeedM1(4 Bytes), SpeedM2(4 Bytes)]
        return self._send(pack('>bii', Cmd.MIXEDSPEED, m1, m2))

    def speed_accel_m1(self, accel, speed):
        """Drive M1 with a signed speed and acceleration value. The sign indicates which direction the motor will run. The acceleration values are not signed. This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate incrementally until the rate defined is reached.

        The acceleration is measured in speed increase per second. An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second. Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.
        """
        # :Sends: [Address, 38, Accel(4 Bytes), Speed(4 Bytes)]
        return self._send(pack('>bIi', Cmd.M1SPEEDACCEL, accel, speed))

    def speed_accel_m2(self, accel, speed):
        """Drive M2 with a signed speed and acceleration value. The sign indicates which direction the motor will run. The acceleration value is not signed. This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate incrementally until the rate defined is reached.

        The acceleration is measured in speed increase per second. An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second. Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.
        """
        # :Sends: [Address, 39, Accel(4 Bytes), Speed(4 Bytes)]
        return self._send(pack('>bIi', Cmd.M2SPEEDACCEL, accel, speed))

    def speed_accel_m1_m2(self, accel, speed1, speed2):
        """Drive M1 and M2 in the same command using one value for acceleration and two signed speed values for each motor. The sign indicates which direction the motor will run. The acceleration value is not signed. The motors are sync during acceleration. This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate incrementally until the rate defined is reached.

        The acceleration is measured in speed increase per second. An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second. Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.
        """
        # :Sends: [Address, 40, Accel(4 Bytes), SpeedM1(4 Bytes), SpeedM2(4 Bytes)]
        return self._send(pack('>bIii', Cmd.MIXEDSPEEDACCEL, accel, speed1, speed2))

    def speed_distance_m1(self, speed, distance, buffer):
        """Drive M1 with a signed speed and distance value. The sign indicates which direction the motor will run. The distance value is not signed. This command is buffered. This command is used to control the top speed and total distance traveled by the motor. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.

        The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
        """
        # :Sends: [Address, 41, Speed(4 Bytes), Distance(4 Bytes), Buffer]
        return self._send(pack('>biIb', Cmd.M1SPEEDDIST, speed, distance, buffer))

    def speed_distance_m2(self, speed, distance, buffer):
        """Drive M2 with a speed and distance value. The sign indicates which direction the motor will run. The distance value is not signed. This command is buffered. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.

        The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
        """
        # :Sends: [Address, 42, Speed(4 Bytes), Distance(4 Bytes), Buffer]
        return self._send(pack('>biIb', Cmd.M2SPEEDDIST, speed, distance, buffer))

    def speed_distance_m1_m2(self, speed1, distance1, speed2, distance2, buffer):
        """Drive M1 and M2 with a speed and distance value. The sign indicates which direction the motor will run. The distance value is not signed. This command is buffered. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.

        The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
        """
        # :Sends: [Address, 43, SpeedM1(4 Bytes), DistanceM1(4 Bytes), SpeedM2(4 Bytes), DistanceM2(4 Bytes), Buffer]
        return self._send(pack('>biIiIb', Cmd.MIXEDSPEEDDIST, speed1, distance1, speed2, distance2, buffer))

    def speed_accel_distance_m1(self, accel, speed, distance, buffer):
        """Drive M1 with a speed, acceleration and distance value. The sign indicates which direction the motor will run. The acceleration and distance values are not signed. This command is used to control the motors top speed, total distanced traveled and at what incremental acceleration value to use until the top speed is reached. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.

        The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
        """
        # :Sends: [Address, 44, Accel(4 bytes), Speed(4 Bytes), Distance(4 Bytes), Buffer]
        return self._send(pack('>bIiIb', Cmd.M1SPEEDACCELDIST, accel, speed, distance, buffer))

    def speed_accel_distance_m2(self, accel, speed, distance, buffer):
        """Drive M2 with a speed, acceleration and distance value. The sign indicates which direction the motor will run. The acceleration and distance values are not signed. This command is used to control the motors top speed, total distanced traveled and at what incremental acceleration value to use until the top speed is reached. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.

        The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
        """
        # :Sends: [Address, 45, Accel(4 bytes), Speed(4 Bytes), Distance(4 Bytes), Buffer]
        return self._send(pack('>bIiIb', Cmd.M2SPEEDACCELDIST, accel, speed, distance, buffer))

    def speed_accel_distance_m1_m2(self, accel, speed1, distance1, speed2, distance2, buffer):
        """Drive M1 and M2 with a speed, acceleration and distance value. The sign indicates which direction the motor will run. The acceleration and distance values are not signed. This command is used to control both motors top speed, total distanced traveled and at what incremental acceleration value to use until the top speed is reached. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.

        The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
        """
        # :Sends: [Address, 46, Accel(4 Bytes), SpeedM1(4 Bytes), DistanceM1(4 Bytes), SpeedM2(4 bytes), DistanceM2(4 Bytes), Buffer]
        return self._send(pack('>bIiIiIb', Cmd.MIXEDSPEEDACCELDIST, accel, speed1, distance1, speed2, distance2, buffer))

    def read_buffer_length(self):
        """Read both motor M1 and M2 buffer lengths. This command can be used to determine how many commands are waiting to execute.

        :Returns: [BufferM1, BufferM2]

        The return values represent how many commands per buffer are waiting to be executed. The maximum buffer size per motor is 64 commands(0x3F). A return value of 0x80(128) indicates the buffer is empty. A return value of 0 indiciates the last command sent is executing. A value of 0x80 indicates the last command buffered has finished.
        """
        val = self._read2(self._address, Cmd.GETBUFFERS)
        if val[0]:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    def read_pwms(self):
        val = self._read4(self._address, Cmd.GETPWMS)
        if val[0]:
            pwm1 = val[1] >> 16
            pwm2 = val[1] & 0xFFFF
            if pwm1 & 0x8000:
                pwm1 -= 0x10000
            if pwm2 & 0x8000:
                pwm2 -= 0x10000
            return (1, pwm1, pwm2)
        return (0, 0, 0)

    def read_currents(self):
        val = self._read4(self._address, Cmd.GETCURRENTS)
        if val[0]:
            cur1 = val[1] >> 16
            cur2 = val[1] & 0xFFFF
            if cur1 & 0x8000:
                cur1 -= 0x10000
            if cur2 & 0x8000:
                cur2 -= 0x10000
            return (1, cur1, cur2)
        return (0, 0, 0)

    def speed_accel_m1_m2_2(self, accel1, speed1, accel2, speed2):
        """Drive M1 and M2 in the same command using one value for acceleration and two signed speed values for each motor. The sign indicates which direction the motor will run. The acceleration value is not signed. The motors are sync during acceleration. This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate incrementally until the rate defined is reached.

        The acceleration is measured in speed increase per second. An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second. Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.
        """
        # :Sends: [Address, 50, AccelM1(4 Bytes), SpeedM1(4 Bytes), AccelM2(4 Bytes), SpeedM2(4 Bytes)]
        return self._send(pack('>bIiIi', Cmd.MIXEDSPEED2ACCEL, accel1, speed1, accel2, speed2))

    def speed_accel_distance_m1_m2_2(self, accel1, speed1, distance1, accel2, speed2, distance2, buffer):
        """Drive M1 and M2 in the same command using one value for acceleration and two signed speed values for each motor. The sign indicates which direction the motor will run. The acceleration value is not signed. The motors are sync during acceleration. This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate incrementally until the rate defined is reached.

        The acceleration is measured in speed increase per second. An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second. Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.
        """
        # :Sends: [Address, 50, AccelM1(4 Bytes), SpeedM1(4 Bytes), AccelM2(4 Bytes), SpeedM2(4 Bytes)]
        return self._send('>bIiIIiIb', Cmd.MIXEDSPEED2ACCELDIST, accel1, speed1, distance1, accel2, speed2, distance2, buffer)

    def duty_accel_m1(self, accel, duty):
        """Drive M1 with a signed duty and acceleration value. The sign indicates which direction the motor will run. The acceleration values are not signed. This command is used to drive the motor by PWM and using an acceleration value for ramping. Accel is the rate per second at which the duty changes from the current duty to the specified duty.

        The duty value is signed and the range is -32768 to +32767(eg. +-100% duty). The accel value range is 0 to 655359(eg maximum acceleration rate is -100% to 100% in 100ms).
        """
        # :Sends: [Address, 52, Duty(2 bytes), Accel(2 Bytes)]
        return self._send(pack('>bhI', Cmd.M1DUTYACCEL, duty, accel))

    def duty_accel_m2(self, accel, duty):
        """Drive M2 with a signed duty and acceleration value. The sign indicates which direction the motor will run. The acceleration values are not signed. This command is used to drive the motor by PWM and using an acceleration value for ramping. Accel is the rate at which the duty changes from the current duty to the specified dury.

        The duty value is signed and the range is -32768 to +32767 (eg. +-100% duty). The accel value range is 0 to 655359 (eg maximum acceleration rate is -100% to 100% in 100ms).
        """
        # :Sends: [Address, 53, Duty(2 bytes), Accel(2 Bytes)]
        return self._send(pack('>bhI', Cmd.M2DUTYACCEL, duty, accel))

    def duty_accel_m1_m2(self, accel1, duty1, accel2, duty2):
        """Drive M1 and M2 in the same command using acceleration and duty values for each motor. The sign indicates which direction the motor will run. The acceleration value is not signed. This command is used to drive the motor by PWM using an acceleration value for ramping.

        The duty value is signed and the range is -32768 to +32767 (eg. +-100% duty). The accel value range is 0 to 655359 (eg maximum acceleration rate is -100% to 100% in 100ms).
        """
        # :Sends: [Address, CMD, DutyM1(2 bytes), AccelM1(4 Bytes), DutyM2(2 bytes), AccelM1(4 bytes)]
        return self._send(pack('>bhIhI', Cmd.MIXEDDUTYACCEL, duty1, accel1, duty2, accel2))

    def read_m1_velocity_pid(self):
        """Read the PID and QPPS Settings.

        :Returns: [P(4 bytes), I(4 bytes), D(4 bytes), QPPS(4 byte)]
        """
        # :Sends: [Address, 55]
        data = self._recv(pack('>b', Cmd.READM1PID), 16)
        if data:
            return (data[0], data[1] / 65536.0, data[2] / 65536.0, data[3] / 65536.0)
        return (0, 0, 0, 0)

    def read_m2_velocity_pid(self):
        """Read the PID and QPPS Settings.

        :Returns: [P(4 bytes), I(4 bytes), D(4 bytes), QPPS(4 byte)]
        """
        # :Sends: [Address, 55]
        data = self._recv(pack('>b', Cmd.READM2PID), 16)
        if data:
            return (data[0], data[1] / 65536.0, data[2] / 65536.0, data[3] / 65536.0)
        return (0, 0, 0, 0)

    def set_main_voltages(self, minimum, maximum):
        """Set the Main Battery Voltage cutoffs, Min and Max. Min and Max voltages are in 10th of a volt increments. Multiply the voltage to set by 10."""
        # :Sends: [Address, 57, Min(2 bytes), Max(2bytes]
        return self._send(pack('>bHH', Cmd.SETMAINVOLTAGES, minimum, maximum))

    def set_logic_voltages(self, minimum, maximum):
        """Set the Logic Battery Voltages cutoffs, Min and Max. Min and Max voltages are in 10th of a volt increments. Multiply the voltage to set by 10."""
        # :Sends: [Address, 58, Min(2 bytes), Max(2bytes]
        return self._send(pack('>bHH', Cmd.SETLOGICVOLTAGES, minimum, maximum))

    def read_min_max_main_voltages(self):
        """Read the Main Battery Voltage Settings. The voltage is calculated by dividing the value by 10

        :Returns: [Min(2 bytes), Max(2 bytes)]
        """
        val = self._read4(self._address, Cmd.GETMINMAXMAINVOLTAGES)
        if val[0]:
            minimum = val[1] >> 16
            maximum = val[1] & 0xFFFF
            return (1, minimum, maximum)
        return (0, 0, 0)

    def read_min_max_logic_voltages(self):
        """Read the Logic Battery Voltage Settings. The voltage is calculated by dividing the value by 10

        :Returns: [Min(2 bytes), Max(2 bytes)]
        """
        val = self._read4(self._address, Cmd.GETMINMAXLOGICVOLTAGES)
        if val[0]:
            minimum = val[1] >> 16
            maximum = val[1] & 0xFFFF
            return (1, minimum, maximum)
        return (0, 0, 0)

    def set_m1_position_pid(self, kp, ki, kd, kimax, deadzone, minimum, maximum):
        """The RoboClaw Position PID system consist of seven constants starting with P = Proportional, I= Integral and D= Derivative, MaxI = Maximum Integral windup, Deadzone in encoder counts, MinPos = Minimum Position and MaxPos = Maximum Position. The defaults values are all zero.

        Position constants are used only with the Position commands, 65,66 and 67 or when encoders are enabled in RC/Analog modes.
        """
        # :Sends: [Address, 61, D(4 bytes), P(4 bytes), I(4 bytes), MaxI(4 bytes), Deadzone(4 bytes), MinPos(4 bytes), MaxPos(4 bytes)]
        return self._send(pack('>bIIIIIII', Cmd.SETM1POSPID, kd * 1024, kp * 1024, ki * 1024, kimax, deadzone, minimum, maximum))

    def set_m2_position_pid(self, kp, ki, kd, kimax, deadzone, minimum, maximum):
        """The RoboClaw Position PID system consist of seven constants starting with P = Proportional, I= Integral and D= Derivative, MaxI = Maximum Integral windup, Deadzone in encoder counts, MinPos = Minimum Position and MaxPos = Maximum Position. The defaults values are all zero.

        Position constants are used only with the Position commands, 65,66 and 67 or when encoders are enabled in RC/Analog modes.
        """
        # :Sends: [Address, 62, D(4 bytes), P(4 bytes), I(4 bytes), MaxI(4 bytes), Deadzone(4 bytes), MinPos(4 bytes), MaxPos(4 bytes)]
        return self._send(pack('>bIIIIIII', Cmd.SETM2POSPID, kd * 1024, kp * 1024, ki * 1024, kimax, deadzone, minimum, maximum))

    def read_m1_position_pid(self):
        """Read the Position PID Settings.

        :Returns: [P(4 bytes), I(4 bytes), D(4 bytes), MaxI(4 byte), Deadzone(4 byte), MinPos(4 byte), MaxPos(4 byte)]
        """
        data = unpack('>IIIIIII', self._recv(pack('>b', Cmd.READM1POSPID), 28))
        if data:
            return (data[0], data[1] / 1024.0, data[2] / 1024.0, data[3] / 1024.0)
        return (0, 0, 0, 0, 0, 0, 0)

    def read_m2_position_pid(self):
        """Read the Position PID Settings.

        :Returns: [P(4 bytes), I(4 bytes), D(4 bytes), MaxI(4 byte), Deadzone(4 byte), MinPos(4 byte), MaxPos(4 byte)]
        """
        data = unpack('>IIIIIII', self._recv(pack('>b', Cmd.READM2POSPID, 28)))
        if data:
            return (data[0], data[1] / 1024.0, data[2] / 1024.0, data[3] / 1024.0)
        return (0, 0, 0, 0, 0, 0, 0)

    def speed_accel_deccel_position_m1(self, accel, speed, deccel, position, buffer):
        """Move M1 position from the current position to the specified new position and hold the new position. Accel sets the acceleration value and deccel the decceleration value. QSpeed sets the speed in quadrature pulses the motor will run at after acceleration and before decceleration.
        """
        # :Sends: [Address, 65, Accel(4 bytes), Speed(4 Bytes), Deccel(4 bytes), Position(4 Bytes), Buffer]
        return self._send('>bIIIIb', Cmd.M1SPEEDACCELDECCELPOS, accel, speed, deccel, position, buffer)

    def speed_accel_deccel_position_m2(self, accel, speed, deccel, position, buffer):
        """Move M2 position from the current position to the specified new position and hold the new position. Accel sets the acceleration value and deccel the decceleration value. QSpeed sets the speed in quadrature pulses the motor will run at after acceleration and before decceleration.
        """
        # :Sends: [Address, 66, Accel(4 bytes), Speed(4 Bytes), Deccel(4 bytes), Position(4 Bytes), Buffer]
        return self._send(pack('>bIIIIb', Cmd.M2SPEEDACCELDECCELPOS, accel, speed, deccel, position, buffer))

    def speed_accel_deccel_position_m1_m2(self, accel1, speed1, deccel1, position1, accel2, speed2, deccel2, position2, buffer):
        """Move M1 & M2 positions from their current positions to the specified new positions and hold the new positions. Accel sets the acceleration value and deccel the decceleration value. QSpeed sets the speed in quadrature pulses the motor will run at after acceleration and before decceleration.
        """
        # :Sends: [Address, 67, AccelM1(4 bytes), SpeedM1(4 Bytes), DeccelM1(4 bytes), PositionM1(4 Bytes), AccelM2(4 bytes), SpeedM2(4 Bytes), DeccelM2(4 bytes), PositionM2(4 Bytes), Buffer]
        return self._send(pack('>bIIIIIIIIb', Cmd.MIXEDSPEEDACCELDECCELPOS, accel1, speed1, deccel1, position1, accel2, speed2, deccel2, position2, buffer))

    def set_m1_default_accel(self, accel):
        """Set the default acceleration for M1 when using duty cycle commands(Cmds 32,33 and 34) or when using Standard Serial, RC and Analog PWM modes.
        """
        # :Sends: [Address, 68, Accel(4 bytes)]
        return self._send(pack('>bI', Cmd.SETM1DEFAULTACCEL, accel))

    def set_m2_default_accel(self, accel):
        """Set the default acceleration for M2 when using duty cycle commands(Cmds 32,33 and 34) or when using Standard Serial, RC and Analog PWM modes.
        """
        # :Sends: [Address, 69, Accel(4 bytes)]
        return self._send(pack('>bI', Cmd.SETM2DEFAULTACCEL, accel))

    def set_pin_functions(self, s3mode, s4mode, s5mode):
        return self._send(pack('>bbbb', Cmd.SETPINFUNCTIONS, s3mode, s4mode, s5mode))

    def read_pin_functions(self):
        trys = self._retries
        while trys:
            self._sendcommand(self._address, Cmd.GETPINFUNCTIONS)
            val1 = self._readbyte()
            if val1[0]:
                val2 = self._readbyte()
                if val1[0]:
                    val3 = self._readbyte()
                    if val1[0]:
                        crc = self._readchecksumword()
                        if crc[0]:
                            if self._crc & 0xFFFF != crc[1] & 0xFFFF:
                                return (0, 0)
                            return (1, val1[1], val2[1], val3[1])
            trys -= 1
        return (0, 0)

    def set_deadband(self, minimum, maximum):
        return self._send(pack('>bbb', Cmd.SETDEADBAND, minimum, maximum))

    def get_deadband(self):
        val = unpack(self._recv(pack('>b', Cmd.GETDEADBAND), 2))
        if val:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    def restore_defaults(self):
        """Reset Settings to factory defaults.

        .. warning:: TTL Serial: Baudrate will change if not already set to 38400.  Communications will be lost.
        """
        # :Sends: [Address, 80]
        return self._send(pack('>b', Cmd.RESTOREDEFAULTS))

    def read_temp(self):
        """Read the board temperature. Value returned is in 10ths of degrees.

        :Returns: [Temperature(2 bytes)]
        """
        return unpack('>h', self._recv(pack('>b', Cmd.GETTEMP), 2))

    def read_temp2(self):
        """Read the second board temperature(only on supported units). Value returned is in 10ths of degrees.

        :Returns: [Temperature(2 bytes)]
        """
        return self._recv(pack('>b', Cmd.GETTEMP2), 2)

    def read_error(self):
        """Read the current unit status.

        :Returns: [Status]

        ========================= ===============
        Function                  Status Bit Mask
        ========================= ===============
        Normal                    0x0000
        M1 OverCurrent Warning    0x0001
        M2 OverCurrent Warning    0x0002
        E-Stop                    0x0004
        Temperature Error         0x0008
        Temperature2 Error        0x0010
        Main Battery High Error   0x0020
        Logic Battery High Error  0x0040
        Logic Battery Low Error   0x0080
        Main Battery High Warning 0x0400
        Main Battery Low Warning  0x0800
        Termperature Warning      0x1000
        Temperature2 Warning      0x2000
        ========================= ===============
        """
        return self._recv(pack('>b', Cmd.GETERROR), 4)

    def read_encoder_modes(self):
        """Read the encoder pins assigned for both motors.

        :Returns: [Enc1Mode, Enc2Mode]
        """
        val = self._recv(pack('>b', Cmd.GETENCODERMODE), 2)
        if val:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    def set_m1_encoder_mode(self, mode):
        """Set the Encoder Pin for motor 1. See `read_encoder_modes()`.
        """
        # :Sends: [Address, 92, Pin]
        return self._send(pack('>bb', Cmd.SETM1ENCODERMODE, mode))

    def set_m2_encoder_mode(self, mode):
        """Set the Encoder Pin for motor 2. See `read_encoder_modes()`.
        """
        # :Sends: [Address, 93, Pin]
        return self._send(pack('>bb', Cmd.SETM2ENCODERMODE, mode))

    def write_nvm(self):
        """Writes all settings to non-volatile memory. Values will be loaded after each power up.
        """
        # :Sends: [Address, 94]
        return self._send('>bI', Cmd.WRITENVM, 0xE22EAB7A)

    def read_nvm(self):
        """Read all settings from non-volatile memory.

        .. warning:: TTL Serial: If baudrate changes or the control mode changes communications will be lost.
        """
        # :Sends: [Address, 95] :Returns: [Enc1Mode, Enc2Mode]
        return self._send(pack('>b', Cmd.READNVM))

    def set_config(self, config):
        # Warning(TTL Serial): If control mode is changed from packet serial mode when setting config communications will be lost!
        # Warning(TTL Serial): If baudrate of packet serial mode is changed communications will be lost!
        return self._send('>bh', Cmd.SETCONFIG, config)

    def get_config(self):
        return self._recv(pack('>b', Cmd.GETCONFIG), 2)

    def set_m1_max_current(self, maximum):
        """Set Motor 1 Maximum Current Limit. Current value is in 10ma units. To calculate multiply current limit by 100.
        """
        # :Sends: [Address, 134, MaxCurrent(4 bytes), 0, 0, 0, 0]
        return self._send(pack('>bII', Cmd.SETM1MAXCURRENT, maximum, 0))

    def set_m2_max_current(self, maximum):
        """Set Motor 2 Maximum Current Limit. Current value is in 10ma units. To calculate multiply current limit by 100.
        """
        # :Sends: [Address, 134, MaxCurrent(4 bytes), 0, 0, 0, 0]
        return self._send(pack('>bII', Cmd.SETM2MAXCURRENT, maximum, 0))

    def read_m1_max_current(self):
        """Read Motor 1 Maximum Current Limit. Current value is in 10ma units. To calculate divide value by 100. MinCurrent is always 0.

        :Returns: [MaxCurrent(4 bytes), MinCurrent(4 bytes)]
        """
        data = unpack('>II', self._recv(pack('>b', Cmd.GETM1MAXCURRENT), 8))
        if data:
            return (data[0], data[1])
        return (0, 0)

    def read_m2_max_current(self):
        """Read Motor 2 Maximum Current Limit. Current value is in 10ma units. To calculate divide value by 100. MinCurrent is always 0.

        :Returns: [MaxCurrent(4 bytes), MinCurrent(4 bytes)]
        """
        data = unpack('>II', self._recv(pack('>b', Cmd.GETM2MAXCURRENT), 8))
        if data[0]:
            return (data[0], data[1])
        return (0, 0)

    def set_pwm_mode(self, mode):
        """Set PWM Drive mode. Locked Antiphase(0) or Sign Magnitude(1).
        """
        # :Sends: [Address, 148, Mode]
        return self._send(pack('>bb', Cmd.SETPWMMODE, mode))

    def read_pwm_mode(self):
        """Read PWM Drive mode. See `set_pwm_mode()`.

        :Returns: [PWMMode]
        """
        return unpack('>b', self._recv(pack('>b', Cmd.GETPWMMODE), 1))

    def read_eeprom(self, ee_address):
        """Read a value from the User EEProm memory(256 bytes).

        :Returns: [Value(2 bytes)]
        """
        # :Sends: [Address, 252, EEProm Address(byte)]
        trys = self._retries
        while trys:
            self._port.flushInput()
            self._sendcommand(self._address, Cmd.READEEPROM)
            self._crc_update(ee_address)
            self._port.write(bytes([ee_address]))
            val1 = self._readword()
            if val1[0]:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc & 0xFFFF != crc[1] & 0xFFFF:
                        return (0, 0)
                    return (1, val1[1])
            trys -= 1
        return (0, 0)

    def write_eeprom(self, ee_address, ee_word):
        """Get Priority Levels.
        """
        # :Sends: [Address, 253, Address(byte), Value(2 bytes)]
        retval = self._send(pack('>bbbb', Cmd.WRITEEEPROM, ee_address, ee_word >> 8, ee_word & 0xFF))
        if retval:
            trys = self._retries
            while trys:
                self._port.flushInput()
                val1 = self._readbyte()
                if val1[0]:
                    if val1[1] == 0xaa:
                        return True
                trys -= 1
        return False
