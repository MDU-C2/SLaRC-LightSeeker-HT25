#!/usr/bin/env python
import time
import can
from s_msgs.msg import BatteryInfo

# Battery uses CAN Extended Frame (29-bit), same as motors
# MESSAGE_TYPE = 0x1092
# LEFT_NODE_ID = 0x16    # default CAN node ID = 0x16 according to documentation (p.23)
# RIGHT_NODE_ID = 0x17

# Construct the full 29-bit CAN ID:
# LEFT_CAN_ID_DOC = (MESSAGE_TYPE << 8) | LEFT_NODE_ID  # = 0x01109216 adding them together
# RIGHT_CAN_ID_DOC = (MESSAGE_TYPE << 8) | RIGHT_NODE_ID  # = 0x01109217

# candump requires these ID:s, which means that the battery documentation from Tattu above is incorrect
LEFT_CAN_ID = 0x01109216
RIGHT_CAN_ID = 0x01109217

class Battery:
    def __init__(self, battery_id, name):
        # Initialization
        self.id = battery_id
        self.name = name

        # Runtime data
        self.manufacturerID = 0.0
        self.sku_code =  0.0
        self.cells_voltage = 0.0 # mV
        self.charge_discharge_current = 0.0 # mA
        self.temperature = 0.0  # Celsius
        self.remaining_capacity_percent = 0.0 # %
        self.cycle_life = 0.0 # times
        self.health_status = 0.0 #% According to the battery chemical characteristics curve analysis
        self.cell_1_voltage = 0.0 # mV
        self.cell_2_voltage = 0.0 # mV
        self.cell_3_voltage = 0.0 # mV
        self.cell_4_voltage = 0.0 # mV
        self.cell_5_voltage = 0.0 # mV
        self.cell_6_voltage = 0.0 # mV
        self.cell_7_voltage = 0.0 # mV
        self.cell_8_voltage = 0.0 # mV
        self.cell_9_voltage = 0.0 # mV
        self.cell_10_voltage = 0.0 # mV
        self.cell_11_voltage = 0.0 # mV
        self.cell_12_voltage = 0.0 # mV
        self.standard_capacity = 0.0 # mAh
        self.remaining_capacity_mAh = 0.0 # mAh
        self.error_information = 0.0

        # Error/alarm flags according to datasheet
        self.error_flags = [
            "LOW_TEMPERATURE", "OVER_TEMPERATURE", "OVER_CURRENT_WHILE_CHARGING",
            "OVER_CURRENT_WHILE_DISCHARGING", "TOTAL_VOLTAGE_IS_UNDERVOLTAGE", "TOTAL_VOLTAGE_IS_OVERVOLTAGE",
            "HUGE_VOLTAGE_IMBALANCE_OF_SINGLE_CELL", "VOLTAGE_OF_SINGLE_CELL_IS_OVERVOLTAGE", "SINGLE_CELL_UNDERVOLTAGE",
            "SHORT_CIRCUIT_WHILE_CHARGING", "SHORT_CIRCUIT_WHILE_DISCHARGING", "LOW_REMAINING_CAPACITY",
            "USE_NON_ORIGINAL_CHARGER"]

        # Create instance attributes for all error flags and set them to 0
        for name in self.error_flags:
            setattr(self, name, 0)

        # Last succesful data packet and safety flag
        self.last_update = 0.0
        self.is_safe_to_drive = False
        
    # Packet to ROS BatteryInfo with a message instance
    def pack_message_to_ros(self):
        msg = BatteryInfo()
        
        msg.manufacturer_id = self.manufacturerID
        msg.sku_code = self.sku_code
        msg.cells_voltage_mv = self.cells_voltage # mV
        msg.charge_discharge_current_ma = self.charge_discharge_current # mA
        msg.temperature_c = self.temperature  # Celsius
        msg.remaining_capacity_percent = self.remaining_capacity_percent # SOC %
        msg.cycle_life = self.cycle_life # times
        msg.health_status = self.health_status # % SOH (According to the battery chemical characteristics curve analysis)
        msg.cell_1_voltage_mv = self.cell_1_voltage # mV
        msg.cell_2_voltage_mv = self.cell_2_voltage # mV
        msg.cell_3_voltage_mv = self.cell_3_voltage # mV
        msg.cell_4_voltage_mv = self.cell_4_voltage # mV
        msg.cell_5_voltage_mv = self.cell_5_voltage # mV
        msg.cell_6_voltage_mv = self.cell_6_voltage # mV
        msg.cell_7_voltage_mv = self.cell_7_voltage # mV
        msg.cell_8_voltage_mv = self.cell_8_voltage # mV
        msg.cell_9_voltage_mv = self.cell_9_voltage # mV
        msg.cell_10_voltage_mv = self.cell_10_voltage # mV
        msg.cell_11_voltage_mv = self.cell_11_voltage # mV
        msg.cell_12_voltage_mv = self.cell_12_voltage # mV
        msg.standard_capacity_mah = self.standard_capacity # mAh
        msg.remaining_capacity_mah = self.remaining_capacity_mAh # mAh
        msg.error_code = self.error_information # Decimal format, convert to binary to read alarms

        return msg
        
    # Little endian sorted hex to decimal conversion with byte value data shifted 2 steps
    def update_from_frame(self, data):
        # Identification
        self.manufacturerID = (data[3] << 8) | data[2]
        self.sku_code =  (data[5] << 8) | data[4]

        # Voltage
        self.cells_voltage = (data[7] << 8) | data[6]  # mV

        # Discharge current
        self.charge_discharge_current = (data[9] << 8) | data[8]  # mA
        if self.charge_discharge_current & 0x8000:
            self.charge_discharge_current -= 0x10000

        # Temperature
        self.temperature = (data[11] << 8) | data[10]

        # Battery life and health status
        self.remaining_capacity_percent = (data[13] << 8) | data[12]
        self.cycle_life = (data[15] << 8) | data[14]
        self.health_status = (data[17] << 8) | data[16]

        # Cell data
        self.cell_1_voltage  = (data[19] << 8) | data[18]
        self.cell_2_voltage  = (data[21] << 8) | data[20]
        self.cell_3_voltage  = (data[23] << 8) | data[22]
        self.cell_4_voltage  = (data[25] << 8) | data[24]
        self.cell_5_voltage  = (data[27] << 8) | data[26]
        self.cell_6_voltage  = (data[29] << 8) | data[28]
        self.cell_7_voltage  = (data[31] << 8) | data[30]
        self.cell_8_voltage  = (data[33] << 8) | data[32]
        self.cell_9_voltage  = (data[35] << 8) | data[34]
        self.cell_10_voltage = (data[37] << 8) | data[36]
        self.cell_11_voltage = (data[39] << 8) | data[38]
        self.cell_12_voltage = (data[41] << 8) | data[40]

        # Capacity
        self.standard_capacity      = (data[43] << 8) | data[42]
        self.remaining_capacity_mAh  = (data[45] << 8) | data[44]

        # Error flags; 3 and 4 are reserved according to data sheet, hence = 0
        self.temp_err1 = data[46]
        self.temp_err2 = data[47]
        self.temp_err3 = 0
        self.temp_err4 = 0

        self.error_information = (
            (self.temp_err4 << 24) |
            (self.temp_err3 << 16) |
            (self.temp_err2 << 8)  |
            self.temp_err1
        )
        
        self.last_update = time.time()
        self.decode_error()
        self.allow_drive()
        
    # Ensure that the motors are safe to drive
    # Ensure that the battery is working and ok to use
    def allow_drive(self):
        safe = True
        
        # If battery has not updated for one second flag as not safe
        if time.time() - self.last_update > 1.0:
            safe = False

        # If voltage exceeds 52V (52000 mV) flag as not safe
        if self.cells_voltage > 52000:
            safe = False
        
        # If temperature is less than 5 or higher than 50 flag as not safe
        if self.temperature < 5 or self.temperature > 50:
            safe = False
        
        # If current is over 100 Amps (100000 mA) flag as not safe            
        if abs(self.charge_discharge_current) > 100000:
            safe = False
        
        cells = [self.cell_1_voltage, self.cell_2_voltage, self.cell_3_voltage, self.cell_4_voltage, self.cell_5_voltage, self.cell_6_voltage, 
            self.cell_7_voltage, self.cell_8_voltage, self.cell_9_voltage, self.cell_10_voltage, self.cell_11_voltage, self.cell_12_voltage]
        
        # If any of the 12 cells is under 3.15 V or exceeds 3.3 V flag as not safe 
        for cell_voltage in cells:
            if cell_voltage < 3000 or cell_voltage > 4200:
                safe = False
        
        # If battery is less than 10 percent do not start
        if self.remaining_capacity_percent < 10:
            safe = False       
        
        # If health status is less than 20 flag as not safe
        if self.health_status < 20:
            safe = False

        # Check for active alarm flags      
        for name in self.error_flags:
            if getattr(self, name):
                safe = False
                break

        # All checks passed, safe to drive
        self.is_safe_to_drive = safe
        return safe

    # Loop through each error flag and extract the bit, one step at a time, and update that bit into the correct attribute
    def decode_error(self):
        for i, name in enumerate(self.error_flags):
            value = (self.error_information >> i) & 0x01
            setattr(self, name, int(value))

class BatteryManager:
    def __init__(self, bus):
        self.bus = bus

        # Battery ID for left and right
        self.batteries = {
            LEFT_CAN_ID: Battery(LEFT_CAN_ID, "left"),
            RIGHT_CAN_ID: Battery(RIGHT_CAN_ID, "right")
        }   
        
        # Buffer for the batteries since it is multi-frame
        self.buffers = {
            LEFT_CAN_ID: [], 
            RIGHT_CAN_ID: []
        }

    # Extracts Start, End, Toggle, and Transfer-ID flags from the tail byte
    def extract_tail(self, byte):
        start = (byte >> 7) & 0x01      # Extract 1-bit from positions 7, 6, 5, and sets them into variables
        end = (byte >> 6) & 0x01
        toggle = (byte >> 5) & 0x01
        transfer_id = byte & 0x1F       # Transfer ID uses 5 bits, hence 0x1F
        return start, end, toggle, transfer_id

    # Receives CAN frames, rebuilds multi-frame packets, and updates batteries
    def receive_packet(self):
        while True:
            # recv python-can library, it waits for CAN frame to arrive on can1 socket
            batt_data = self.bus.recv(timeout=0.1)
            print("Starting to receive messages")
            # If nothing happens skip
            if batt_data is None:
                print("Empty message")
                return None

            # Skip all CAN messages that do not come from a LEFT or RIGHT battery
            if batt_data.arbitration_id  not in self.batteries:
                continue
            
            # Selects which battery that packet belongs to
            batt_id = batt_data.arbitration_id
            # List data
            frame = list(batt_data.data)
            print(frame)
            # Get the last byte of the CAN frame and decode tail byte
            tail = frame[-1]
            start, end, toggle, transfer_id = self.extract_tail(tail)

            # Start of transfer, begin new packet
            if start == 1:
                self.buffers[batt_id] = []

            # Append from 0 to 7 bytes, where tail is 7.
            self.buffers[batt_id] += frame[:7]

            # End of transfer, full packet received and is 48 bytes long.
            packet = self.buffers[batt_id]

            # If packet length is equal to or more than 48 update values and send, then empty buffert for next packet
            if len(packet) >= 48:
                self.batteries[batt_id].update_from_frame(packet)
                self.buffers[batt_id] = []
                return batt_id, packet
            else:
                print("Incorrect packet length:", len(packet))
