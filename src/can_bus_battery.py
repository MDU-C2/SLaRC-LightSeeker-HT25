#!/usr/bin/env python
import time
import subprocess
import can

# The battery uses 1 Mbits
from utils.motor.motors import BITRATE

# CAN Bus communication set up
def sh(cmd):
    subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
# QOL Function that sets up can safely each time the program is ran
def bring_up_can():
    sh(["sudo", "ip", "link", "set", "can1", "down"])
    sh(["sudo", "ip", "link", "set", "can1", "type", "can", "bitrate", str(BITRATE)])
    sh(["sudo", "ip", "link", "set", "can1", "up"])
    time.sleep(0.15)

# Create bus and returns socket for channel/usb can0
def create_bus():
    bring_up_can()
    return can.interface.Bus(channel="can1", bustype="socketcan")
