"""A module for manipulating dat including generating CRC values and datatype constraints."""

def crc16(data):
    """Calculates CRC16 of data in bytearray."""
    crc = b'\x00'
    for byte in data: # for each byte
        crc = crc ^ (byte << 8)
        for _ in range(8): # for each bit
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
    return crc
