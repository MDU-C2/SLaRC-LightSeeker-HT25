#!/usr/bin/env python
import time
import subprocess
import can

BATTERY_ID = {
    "battery_left":  0x01,
    "battery_right": 0x02
  
Class Battery:
  def __init__(self, bus, battery):
      self.id = battery_id
      self.bus = bus
      self.name = name
      self.voltage = 0.0
      self.current = 0.0
      self.temperature = 0.0
      self.status = 0
      self.cells = []
      self.last_update = 0

  def update_handle(self, voltage=None, current=None, temperature=None, status=None)
      if voltage is not None:
        self.voltage = voltage
      if current is not None:
        self.current = current
      if temperature is not None:
        self.temperature = temperature
      if cells is not None:
        self.cells = cells

      self.last_update = time.time()

Class BatteryManager:
  def __init__(self, bus, battery):

            
#Receive Messages

#Unpack Messages

#Send Messages to Health node
