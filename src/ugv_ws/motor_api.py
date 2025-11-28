#!/usr/bin/env python
import time
import can
from utils.motor.motor_telemetry import MotorTelemetry
from utils.motor.packers import pack_int32

# CubeMars CAN command numbers (datasheet)
CMD_SET_DUTY     = 0
CMD_SET_CURRENT  = 1
CMD_SET_BRAKE    = 2
CMD_SET_RPM      = 3
CMD_SET_POS      = 4
CMD_SET_ZERO     = 5
CMD_SET_POS_SPD  = 6

# CubeMars packet size for mode commands example (duty: AA 05 46 00 00 4E 20 D6 4C BB)
CMD_SET_MODE     = 10 

class Motor:
    # Constructor
    def __init__(self, bus, motor_id):
        # CAN interface & motor ID
        self.bus = bus
        self.id = motor_id

        # Telemetry struct
        self.telemetry = MotorTelemetry()

        # Telemetry frame ID from motor → host
        self.rx_id = 0x2900 | motor_id

        # Internal state for anti-spam and watchdog keepalive
        self._last_rpm = None
        self._last_rpm_ts = 0.0
        self._KEEPALIVE_S = 0.25   # MUST refresh motor command within this time

    # Build extended CAN ID (CubeMars style)
    def _eid(self, command):
        return (command << 8) | self.id


    # Motor Mode
    def set_mode(self, mode):
        """
        mode:
            0 = STOP / FREEWHEEL
            1 = TORQUE mode
            2 = VELOCITY mode
            3 = POSITION mode
        """
        msg = can.Message(
            arbitration_id=self._eid(CMD_SET_MODE),
            is_extended_id=True,
            data=pack_int32(int(mode))
        )
        self.bus.send(msg)


    # Current/Torque Control
    def set_current(self, amps):
        raw = int(amps * 1000)   # CubeMars uses mA
        msg = can.Message(
            arbitration_id=self._eid(CMD_SET_CURRENT),
            is_extended_id=True,
            data=pack_int32(raw)
        )
        self.bus.send(msg)


    # Velocity / RPM Control
    def set_rpm(self, rpm, force=False):
        rpm = int(rpm)
        now = time.time()
        need_send = force

        # Check if we must send a new command
        if self._last_rpm is None:
            need_send = True
        elif rpm != self._last_rpm:
            need_send = True
        elif (now - self._last_rpm_ts) >= self._KEEPALIVE_S:
            need_send = True

        # If not need return early
        if not need_send:
            return

        # Build and send command
        msg = can.Message(
            arbitration_id=self._eid(CMD_SET_RPM),
            is_extended_id=True,
            data=pack_int32(rpm)
        )

        try:
            self.bus.send(msg)
            # Only update cache when send succeeds
            self._last_rpm = rpm
            self._last_rpm_ts = now
        except Exception as e:
            print(f"[Motor {self.id}] Warning: failed to send RPM {rpm}: {e}")


    # Stop motor by setting RPM to 0
    def stop(self):
        self.set_rpm(0, force=True)


    # Telemetry handler
    def handle_rx(self, msg):
        # Accept & decode telemetry from motor; telemetry frame = extended ID
        if msg.arbitration_id == self.rx_id and msg.dlc == 8:
            self.telemetry.update_from_frame(msg.data)
            return True
        return False
