"""A module for manipulating dat including generating CRC values and datatype constraints."""
import struct

# For more information on how CRC algorithms work: https://www.zlib.net/crc_v3.txt
def crc16(data):
    """Calculates CRC16 of data in bytearray."""
    crc = 0x0000
    for byte in data: # for each byte
        crc ^= (byte << 8)
        for _ in range(8): # for each bit
            if crc & 0x8000: # if divisible
                # 0x1021 is a standard polynomial used for crc16 algorithms
                crc = (crc << 1) ^ 0x1021 # behaves like unsigned subtraction
            else:
                crc = crc << 1 # bring down next bit for binary long-division
    # print('checksum =', hex(crc & 0xffff))
    return crc & 0xFFFF # return only the remainder

def validate(message):
    """validates a received message by comparing the calculated checksum with the checksum
    included in the message"""
    print(hex(struct.unpack('>H', message[-2:])[0]), '==', crc16(message[:-2]))
    return struct.unpack('>H', message[-2:])[0] == crc16(message[:-2])
