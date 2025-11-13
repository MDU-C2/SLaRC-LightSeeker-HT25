#!/usr/bin/env python3
import can, time, subprocess

BITRATE = 1_000_000
MOTOR_ID = 0x02

ID_SET_CURRENT = (1 << 8) | MOTOR_ID    # Packet ID 1

def sh(cmd):
    subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def bring_up_can():
    sh(["sudo","ip","link","set","can0","down"])
    sh(["sudo","ip","link","set","can0","type","can","bitrate",str(BITRATE)])
    sh(["sudo","ip","link","set","can0","up"])
    time.sleep(0.1)

def pack_int32(v):
    return int(v).to_bytes(4, "big", signed=True)

# Change Torque
test_current = 1.5    # in Amps

bring_up_can()
bus = can.interface.Bus(channel="can0", bustype="socketcan")

print(f"\n Setting motor torque to {test_current} A (CURRENT MODE)\n")

raw = int(test_current * 1000) 
msg = can.Message(
    arbitration_id=ID_SET_CURRENT,
    is_extended_id=True,
    data=pack_int32(raw)
)

try:
    while True:
        bus.send(msg)
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\n Stopping (0A)")
    stop = can.Message(
        arbitration_id=ID_SET_CURRENT,
        is_extended_id=True,
        data=pack_int32(0)
    )
    for _ in range(5):
        bus.send(stop)
        time.sleep(0.02)
