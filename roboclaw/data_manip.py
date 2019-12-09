"""A module for manipulating dat including generating CRC values and datatype constraints.
For more information on how CRC algorithms work: https://www.zlib.net/crc_v3.txt"""
import struct

def make_poly(bit_length, msb=False):
    """Make `int` "degree polynomial" in which each bit represents a degree who's coefficient is 1

    :params int bit_length: The amount of bits to play with
    :params bool msb: `True` make only the MSBit 1 and the rest a 0. `False` makes all bits 1.
    """
    if msb:
        return 1 << ((8 * int(bit_length / 8)) - 1)
    result = 0
    for x in range(int(bit_length / 8)):
        result += 0xff << int(x * 8)
    return result

def crc16(data, deg_poly=0x1021, init_value=0):
    """Calculates a checksum of 16-bit length"""
    return crc_bits(data, 16, deg_poly, init_value)


def crc32(data, deg_poly=0x5b06, init_value=0x555555):
    """Calculates a checksum of 32-bit length. Default ``deg_poly`` and ``init_value`` values
    are BLE compliant."""
    return crc_bits(data, 32, deg_poly, init_value)

def crc_bits(data, bit_length, deg_poly, init_value):
    """Calculates a checksum of various sized buffers

    :param bytearray data: This `bytearray` of data to be uncorrupted.
    :param int bit_length: The length of bits that will represent the checksum.
    :param int deg_poly: A preset "degree polynomial" in which each bit represents a degree who's
        coefficient is 1.
    :param int init_value: This will be the value that the checksum will use while shifting in the
        buffer data.
    """
    crc = init_value
    mask = make_poly(bit_length, msb=True)  # 0x8000
    for i, byte in enumerate(data):  # for each byte
        if i:  # shift out initial value 1 byte @ a time.
            crc ^= (byte << 8)
        for _ in range(8):  # for each bit
            if crc & mask:  # if divisible
                # 0x1021 is a standard polynomial used for crc16 algorithms
                # behaves like unsigned subtraction
                crc = (crc << 1) ^ deg_poly
            else:
                crc = crc << 1  # bring down next bit for binary long-division
    return crc & make_poly(bit_length)  # return only the remainder


def validate16(data, deg_poly=0x1021, init_value=0):
    """Validates a received data by comparing the calculated 16-bit checksum with the
    checksum included at the end of the data"""
    cal_d = crc_bits(data[:-2], 16, deg_poly, init_value)
    rcv_d = struct.unpack('>H', data[-2:])[0]
    print(validate(data, 16, deg_poly, init_value))
    return cal_d == rcv_d


def validate(data, bit_length, deg_poly, init_value):
    """Validates a received  checksum of various sized buffers

    :param bytearray data: This `bytearray` of data to be uncorrupted.
    :param int bit_length: The length of bits that will represent the checksum.
    :param int deg_poly: A preset "degree polynomial" (in which each bit represents a degree who's
        coefficient is 1) as a quotient.
    :param int init_value: This will be the value that the checksum will use while shifting in the
        buffer data.

    :Returns: `True` if data was uncorrupted. `False` if something went wrong.
        (either checksum didn't match or payload is altered).
    """
    cal_d = crc_bits(data[:-(bit_length / 8)], bit_length, deg_poly, init_value)
    rcv_d = 0
    for byte in data[-(bit_length / 8):]:
        rcv_d = (rcv_d << 8) | byte
    print(cal_d == rcv_d)
    return cal_d == rcv_d
