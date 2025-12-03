#!/usr/bin/env python
import time
import can
    
# Battery uses CAN Extended Frame (29-bit), same as motors
MESSAGE_TYPE = 0x1092
LEFT_NODE_ID = 0x16    # default CAN node ID = 0x16 according to documentation (p.23)
RIGHT_NODE_ID = 0x17

# Construct the full 29-bit CAN ID:
LEFT_CAN_ID = (MESSAGE_TYPE << 8) | LEFT_NODE_ID  # = 0x109216 adding them together
RIGHT_CAN_ID = (MESSAGE_TYPE << 8) | RIGHT_NODE_ID  # = 0x109217

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
        self.remaning_capacity_percent = 0.0 # %
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
        self.remaning_capacity_mAh = 0.0 # mAh
        self.error_information = 0.0

    def send_status_to_ros(self):
        print(self.manufacturerID)
        print(self.sku_code)
        print(self.cells_voltage) # mV
        print(self.charge_discharge_current) # mA
        print(self.temperature)  # Celsius
        print(self.remaning_capacity_percent) # %
        print(self.cycle_life) # times
        print(self.health_status) #% According to the battery chemical characteristics curve analysis
        print(self.cell_1_voltage) # mV
        print(self.cell_2_voltage) # mV
        print(self.cell_3_voltage) # mV
        print(self.cell_4_voltage) # mV
        print(self.cell_5_voltage) # mV
        print(self.cell_6_voltage) # mV
        print(self.cell_7_voltage) # mV
        print(self.cell_8_voltage) # mV
        print(self.cell_9_voltage) # mV
        print(self.cell_10_voltage) # mV
        print(self.cell_11_voltage) # mV
        print(self.cell_12_voltage) # mV
        print(self.standard_capacity) # mAh
        print(self.remaning_capacity_mAh) # mAh
        print(self.error_information)

    def update_from_frame(self, data):
        # All data is little endian
        self.manufacturerID = (data[1] << 8) | data[0]
        self.sku_code =  (data[3] << 8) | data[2]
        self.cells_voltage = (data[5] << 8) | data[4] # mV
        self.charge_discharge_current = (data[7] << 8) | data[6] # mA: Positive = charging, negative = discharging
        if self.charge_discharge_current & 0x8000:
            self.charge_discharge_current -= 0x10000
        self.temperature = (data[9] << 8) | data[8]  # Celsius
        self.remaning_capacity_percent = (data[11] << 8) | data[10] # %
        self.cycle_life = (data[13] << 8) | data[12] # times
        self.health_status = (data[15] << 8) | data[14] #% According to the battery chemical characteristics curve analysis
        self.cell_1_voltage = (data[17] << 8) | data[16] # mV
        self.cell_2_voltage = (data[19] << 8) | data[18] # mV
        self.cell_3_voltage = (data[21] << 8) | data[20] # mV
        self.cell_4_voltage = (data[23] << 8) | data[22] # mV
        self.cell_5_voltage = (data[25] << 8) | data[24] # mV
        self.cell_6_voltage = (data[27] << 8) | data[26] # mV
        self.cell_7_voltage = (data[29] << 8) | data[28] # mV
        self.cell_8_voltage = (data[31] << 8) | data[30] # mV
        self.cell_9_voltage = (data[33] << 8) | data[32] # mV
        self.cell_10_voltage = (data[35] << 8) | data[34] # mV
        self.cell_11_voltage = (data[37] << 8) | data[36] # mV
        self.cell_12_voltage = (data[39] << 8) | data[38] # mV
        self.standard_capacity = (data[41] << 8) | data[40] # mAh
        self.remaning_capacity_mAh = (data[43] << 8) | data[42] # mAh
        self.error_information = (data[45] << 8) | data[44]

        self.send_status_to_ros()

        self.last_update = time.time()

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
    def run(self):
        while True:
            # recv python-can library, it waits for CAN frame to arrive on can1 socket
            msg = self.bus.recv() 
            print("Starting to receive messages")
            # If nothing happens skip
            if msg is None:
                continue
            
            # Skip all CAN messages that do not come from a LEFT or RIGHT battery
            if msg.arbitration_id  not in self.batteries:
                continue

            # Selects which battery that packet belongs to
            batt_id = msg.arbitration_id
            # List data
            data = list(msg.data)
            # Get the last byte of the CAN frame and decode tail byte
            tail = data[-1]
            start, end, toggle, transfer_id = self.extract_tail(tail)

            # Start of transfer, begin new packet
            if start == 1:
                self.buffers[batt_id] = []

            # Append from 0 to 7 bytes, where tail is 7.
            self.buffers[batt_id] += data[:7]
            
            # End of transfer, full packet received and is 48 bytes long.
            if end == 1:
                packet = self.buffers[batt_id]
                if len(packet) >= 46:
                    self.batteries[batt_id].update_from_frame(packet)
                    break

    
