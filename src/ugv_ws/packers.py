# Conversion to CAN data, 4 bytes
def pack_int32(val):
    val = int(val)
    return bytes([
        (val >> 24) & 0xFF,
        (val >> 16) & 0xFF,
        (val >> 8) & 0xFF,
        val & 0xFF
    ])
