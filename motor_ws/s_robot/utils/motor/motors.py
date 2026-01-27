#!/usr/bin/env python
# Bitrate 1Mbit
BITRATE = 1_000_000

# Motor IDs on the CAN bus, must be equal to ID set in CubeMarsTool (ID:1 -> 0x01, ID:2 -> 0x02,..., ID:104 -> 0x68)
MOTOR_IDS = {
    "front_left":  0x01,
    "front_right": 0x02,
    "rear_left":   0x03,
    "rear_right":  0x04,
}

# CAN control-command IDs (from motor datasheet)
CMD_SET_CURRENT = 1   # torque/current mode
CMD_SET_RPM     = 3   # velocity mode
