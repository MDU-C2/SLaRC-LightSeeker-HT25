
<!-- Invisible comments -->
<!-- 
Here is some help functions: 
# Main Title
## Section Title
### Subsection Title
Italic: *text* or _text_
Bold: **text** or __text__
Break line: <br>

Lists: 
- Item one
- Item two
1. First item 
2. Second item 
or that markdown automatically numbers lists
1. First item 
1. Second item 

Links: 
[Link text](https://example.com)

Images:
![Alt text](image.png) 
If the image is placed in the same folder as .md file.
From URL:
![Logo](https://example.com/logo.png)

Tabel: 
| Name     | Age | City       |
|:---------|:---:|-----------:|
|Anna      | 28  |   Stockholm|
|Björn     | 35  |  Gothenburg|
|Cecilia   | 42  |       Malmö|

Left-aligned: :---
Centered: :---:
Right-aligned: ---:

IMPORTANT! 
Name of files shuld follow this base: DOC-xxx-Name_Of_File

-->

# Documentation for the platform in SLaRC
**Version:** 1.0
**Release Date:** 2026-00-00
<br> Mälardalen University 

<!-- Table for abbreviations?-->
---
## Table of Contents
1. Introduction 
1. System Overview
1. Platform Features
1. Hardware Setup 
1. Software Installation
1. Troubleshooting
1. Support & Contact
---

## 1. Introduction 
Welcome to the SLaRC UGV user guide. This document will help you to set up, use, and troubleshoot the UGV platform.

**Who is this for?**
<br> This guide is intended for new users, technicians and administrators.

## 2. System Overview
The platform is a Unmanned Ground Vehicle (UGV) and is a part of the SLaRC project. ...

### 1. Bill of Materials
All components and spare parts are listed in the file [DOC-001-Bill_Of_Materials](https://github.com/MDU-C2/SLaRC-LightSeeker-HT25/blob/main/docs/DOC-Files/DOC-001-Bill_Of_Materials.xlsx)
### 2. Mechanical Design
<!-- e.g. solidwork, images of platform/sensors-->

### 3. Power System 
<!-- e.g. How the PDB is designed and all it functions, all other components-->
#### 1. Battery
The UGV is powered by two Li-Po batteries. The platform can be driven by either one och two batteries at the same time. This gives the user the opportunity to charge one battery while still using the platform. 

Specs:
Capacity: 22000mAh,
Voltage: 44.4V,
Discharge Rate: 25C,
Weight: 6058g,
Size: 237*173*116mm

WARNING: Lithium-Polymer (LiPo) batteries can be dangerous if mishandled. Failure to follow these safety instructions may result in fire, explosion, personal injury, or damage to equipment.

General Safety: 

- Always read and follow safety intructions before use. 
- 

Charging:
- Always use the intened charger for the specific battery.
- Never charge the battery unattended. 
- Inspect the battery from swelling, damage and torn cabled before use. Never charge the battery if any damage is identified. 

Handling and Operation:
- Do not short-cut the terminals.
- Never expose the battery for heat, direct sunlight, water or mechanical shock.
- Be careful not to crush or pierce the battery pack.

Storage:
- When the platfrom is not used, store the batteries in the intended safe-battery cabinet at C2 (building 326 at finslätten).

In Case of Fire:
- Use the intended lithium fire extinguisher for battery fire avalible at C2 (building 326 at finslätten). 

#### 2. Main Switch

The UGV:S main switch is located at the back of the platform. Its main purpose is to disconnect/open the ciurcut from the battery to the rest of the circuit. The two states is ON (green field), and OFF (red field).

#### 3. Emergency Stop Button

#### 4. Shunt Regulator

#### 5. Conductor

#### 6. High Voltage PCB
The high-voltage power distribution board is used for components that require 48V DC input. The PDB has an input from the battery, four outputs to the motors, and one output to the router.

It is designed to make a maximum current of 30A from the motors and 20A to the router.
The card also has extra space for CAN communication, including two USB-to-CAN adapters and two CAN bus splitters.
<!-- Add drawings and pictures of the pdb--->

#### 7. Low Voltage PCB
The low-voltage power distribution board is used for components with different DC inputs. The PDB accepts a 48V DC input and provides three output voltage levels through three DC-DC converters. The first is for all sensors and components that need a 24V DC input. The middle is made for the NUC computer that needs 20V DC, and the last is flexible between 5-12V for future sensors. 
<!-- Add drawings and pictures of the pdb--->

#### 8. Motor


### 4. CAN communication

#### 1. Motors
Transmitting messages to control the motors 
- CAN Protocol: CAN Bus 2.0B a baud rate @ 1Mbps 
- Identifier CAN ID: Each motor needs a unique ID which can be set using CubeMars - Upper Computer software. Note that using this software requires UART-USB connection. Instructions are available in [DS-CubeMars-AK10-9](https://github.com/MDU-C2/SLaRC-LightSeeker-HT25/blob/main/docs/DOC-Data_sheets/DS-CubeMars-AK10-9.pdf). These IDs should be specified in “motor_ws/s_robot/utils/motor/motors.py”.  
- Identifier: Control Mode ID + Driver ID 
- Frame Type: 29-bit Extended format 
- Frame Format: DATA 
- DLC:  8 bytes 
- Endian: Big Endian 
- Control Mode: Servo, Velocity mode (3) is the implemented control mode. 
Note: Velocity is set in ERPM and not RPM. ERPM has a range of –100000 -> 100000 while RPM has a range of 0 -> 235. Code example in [DS-CubeMars-AK10-9, p.35](https://github.com/MDU-C2/SLaRC-LightSeeker-HT25/blob/main/docs/DOC-Data_sheets/DS-CubeMars-AK10-9.pdf). 

Receiving messages with information from the motors 
The motors transmit single frame messages at a frequency set in the CubeMars software (1-500 Hz). The size of each message is 8 bytes. Each transfer will contain: 
- Position (2 bytes) 
- Speed (2 bytes) 
- Current (2 bytes) 
- Motor Temperature (1 byte) 
- Error Code (1 byte) 
Ranges, units and error codes is available in [DS-CubeMars-AK10-9, p.42](https://github.com/MDU-C2/SLaRC-LightSeeker-HT25/blob/main/docs/DOC-Data_sheets/DS-CubeMars-AK10-9.pdf) as well as an example of message reception. 

#### 1. Battery
The CAN communication for the battery uses CAN Bus 2.0B, meaning a 29-bit extended frames format and has a sending frequency of 4 Hz (0.25 s). 

In the case of a message broadcast transfer, the CAN ID field of every frame of the transfer will contain: 
    - Priority field – 5 bits 
    - Message Type ID – 16 bits 
    - Service or message – 1 bit 
    - Source node ID – 7 bits 
where the CAN ID consists of the values of the Message Type ID + BMS Source Node ID which gives: 
    - Battery 1: 0x01109216 (Default) 
    - Battery 2: 0x01109217 (Source Node ID = Default + 1) 

Each CAN frame ends with 1 tail byte, and that byte is structured as follows: 
- bit 7: Start of transfer 
- bit 6: End of transfer 
- bit 5: Toggle bit 
- bit 0-4: Transfer ID 
    
Each data field from the battery is encoded as a 2 byte / 16-bit little-endian value. The low byte comes first, and the high byte comes after (example data[0], ..., data[1]). All data fields use 2 bytes, except for the error of information where every code means the status of an error. All of these are shown in the list below: 
- Manufacturer ID (2 bytes) 
- SKU code (2 bytes) 
- Cells Voltage (2 bytes) 
- Charge and discharge current (2 bytes) 
- Temperature (2 bytes) 
- Remaining Capacity (2 bytes) 
- Cycle Life (2 bytes) 
- Health Status (2 bytes) 
- Cell 1 – 12 (2 bytes each) 
- Standard Capacity (2 bytes) 
- Remaining Capacity (2 bytes) 
- Error Information (4 bytes) 
    
Genstattu batteries utilize a multi-frame transfer protocol. The BMS requires 48 bytes of data to complete a status report, and one CAN frame can hold up to 8 bytes. Therefore, one status report is split up to several CAN frames, the structure is shown below: 
- Frame 1: bytes 0 – 6 
- Frame 2: bytes 7 – 13 
- Frame 3: bytes 14 – 20... 
- Final frame: bytes ...42 - 48 contains the last bytes and a tail-byte flagging the end byte to 1. 
In this battery, this results in 7 full frames. 

### 5. Sensors


### 6. Computer


### 7. Router


### 7. RTK


## 3. Platform Features
<!-- e.g. reqirements or functions that the device can perform -->
### 1. Battery Charging

### 2. Main Switch
The platfrom is equipted with a main switch with the purpose of disconnect/open the circuit from the battery to the rest of the circuit. 

#### 1. Open the circuit:
Turn the knob clockwise to the green ON position


### 3. Emergency Stop

### 4. Add/remove sensors to the PDB

### 5. Change Output Voltage on PDB



## 4. Hardware Setup
<!-- e.g. step by step how to set up something-->

## 5. Software Installation
<!-- e.g. step by step how to install or do something -->

## 6. Troubleshooting
<!-- e.g. Issue. UGV Fails to respond to commands. -->

## 7. Support & Contact
<!-- e.g. a list of all students in the SLaRC team and what role they had/what knowledge they possessed about-->
