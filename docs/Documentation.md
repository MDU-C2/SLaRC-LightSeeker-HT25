
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
1. [Introduction](#1-introduction)
1. [System Overview](#2-system-overview)
    1. [Bill of Materials](#21-bill-of-materials)
    1. [Mechanical Design](#22-mechanical-design)
        1. [Wheels](#221-wheels)
        1. [Suspension](#22-mechanical-design)
        1. [Chassis](#223-chassis)
        1. [Sensor Placement](#224-sensor-placement)
1. [Platform Features]()
    1. [Battery Charging](#31-battery-charging)
    1. [Main Switch](#32-main-switch)
    1. [Emergency Stop](#33-emergency-stop)
    1. [Add or Remove Components to the PDB](#34-add-or-remove-components-to-the-pdb)
1. [Hardware Setup]()
    1. [Power System](#43-power-system)
        1. [Battery](#431-battery)
        1. [Main Switch](#432-main-switch)
        1. [Emergency Stop Button](#433-emergency-stop-button)
        1. [Shunt Regulator](#434-shunt-regulator)
        1. [Conductor](#435-conductor)
        1. [High-Voltage PDB](#436-high-voltage-pdb)
        1. [Low-Voltage PDB](#437-low-voltage-pdb)
        1. [Motors](#438-motors)
    1. [CAN Communication](#44-can-communication)
        1. [Motors](#441-motors)
        1. [Battery](#442-battery)
1. [Software Installation](#5-software-installation)
1. [Platform Start-Up & Operation](#6-platform-start-up-and-operation)
1. [Troubleshooting](#7-troubleshooting)
1. [Support & Contact](#8-support--contact)
---

## 1. Introduction 
Welcome to the SLaRC UGV user guide. This document will help you to set up, use, and troubleshoot the UGV platform. It also provides detailed information on all components included in the UGV, describing how they are interconnected as well as their intended functions and applications. A comprehensive Bill of Materials (BOM) is included, accompanied by datasheets for each component within the platform. Should any information be missing, contact details are provided at the end of the document.

**Who is this for?**
<br> This guide is intended for new users, technicians and administrators. For safety reasons, always read through this document before using the platform for the first time. 

## 2. System Overview
The platform is a Unmanned Ground Vehicle (UGV) and is a part of the SLaRC project at Mälardalen University. 

### 2.1 Bill of Materials
All components and spare parts are listed in the file [DOC-001-Bill_Of_Materials](https://github.com/MDU-C2/SLaRC-LightSeeker-HT25/blob/main/docs/DOC-Files/DOC-001-Bill_Of_Materials.xlsx)

### 2.2 Mechanical Design
<!-- e.g. solidwork, images of platform/sensors-->
The overall mechanical design of the UGV is divided into four main sections. The following paragraphs provide detailed information on each section, outlining its functionality and purpose.

#### 2.2.1 Wheels

#### 2.2.2 Suspension 

#### 2.2.3 Chassis

#### 2.2.4 Sensor Placement

## 3 Platform Features
<!-- e.g. reqirements or functions that the device can perform -->
### 3.1 Battery Charging
The batteries in this UGV should **ONLY** be charged using the intended charger. Detaild information about the charger can be found in the BOM.

The charger has its own user manual provided by the manufacturer and is linked here.  
For safety reasons, read this document thoroughly before charging the batteries!

### 3.2 Main Switch
The platfrom is equipted with a main switch with the purpose of disconnect/open the circuit from the battery to the rest of the circuit. 

Open the circuit:
Turn the knob clockwise to the green ON position.

Close the Circuit:
Turn the knob counter - clockwise to the red OFF position.

### 3.3 Emergency Stop
The UGV is equipped with a red emergency stop bottom. It is located at the back of the platform together with the main switch. This is to be pressed in any case of emergency. 

Enable emergency stop: When pressing down the bottom, the power to the motors are cut of. All other components are still supplied with power from the batteries.

Reset emergency stop: Twist the button clockwise to close the circuit and thereby enable power to the motors. **WARNING:** Be cautious, as the motors may start up at the same speed they previously stopped at. 


### 3.4 Add or Remove Components to the PDB

The units connected to the PDB:s can be exchanged as long as the voltage and current limits are not exceeded. Find out if the new unit is suitable before connecting it. Here follows information on the limits for each PDB.

LowVoltage: It is equipped with three different dc-dc converters. One providing power to six units connected to 24V, and maximum 10A together. The next provides two units with 20V and a maximum of 10A. The last one provides power for three units and the voltage can be manually configured by turning the potentiometer higher/lower. It also has a limit of delivering no more than 10A. The fuses can be exchanged to match the new units. 

HighVoltage: The motors have one individual connector for each motor. Each connector tolerates 40V and 30A. Apart from motor connectors, it has an additional connector that can provide a unit with 48V and a maximum of 20A. 


### 3.5 Change Output Voltage on PDB

## 4. Hardware Setup
<!-- e.g. step by step how to set up something-->
### 4.3 Power System 
<!-- e.g. How the PDB is designed and all it functions, all other components-->
#### 4.3.1 Battery
The UGV is powered by two Li-Po batteries. The platform can be driven by either one och two batteries at the same time. This gives the user the opportunity to charge one battery while still using the platform. 

Specs:
Capacity: 22000mAh,
Voltage: 44.4V,
Discharge Rate: 25C,
Weight: 6058g,
Size: 237*173*116mm

**WARNING:** Lithium-Polymer (LiPo) batteries can be dangerous if mishandled. Failure to follow these safety instructions may result in fire, explosion, personal injury, or damage to equipment.

General Safety: 

- Always read and follow safety intructions before use. 
- If you are uncertain about how maintenance should be performed, ensure you obtain the necessary information before proceeding

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

#### 4.3.2 Main Switch

The UGV:S main switch is located at the back of the platform. Its main purpose is to disconnect/open the ciurcut from the battery to the rest of the circuit. The two states is ON (green field), and OFF (red field).

#### 4.3.3 Emergency Stop Button
The emergency stop button controlls the conductor. It is normaly closed and thereby brakes the circuit only when the button is pressed. When the button is relised the circuit closes again.

#### 4.3.4 Shunt Regulator

#### 4.3.5 Conductor
The conductor works as a relay and controls the power provided to the motors. The emergency stop button is connected to the conductor's control signal. 
The state of the coil is normally open. This means that if the control signal circuit is interrupted by any reason, the motors will automatically stop

If the control signal is OFF (emergency stop is pressed), the coil is open, and the current can't flow through the conductor to the motors. 

If the control signal is ON (emergency stop not pressed), the conductor is closed and current flows. 


#### 4.3.6 High-Voltage PDB
The high-voltage power distribution board is used for components that require 48V DC input. The PDB has an input from the battery, four outputs to the motors, and one output to the router. Every motor connector can handle 48V and up to 30A. The router connector is designed to tolerate 48V and up to 20A. 
The board also has a dedicated space for CAN communication, including two USB-to-CAN adapters and two CAN bus splitters.

The board is equipped with components to ensure a safe and stable power supply for all connected devices. Diodes are used to guarantee that current flows in the correct direction and to protect against reversed polarity when connecting the battery. The diodes are mounted on a heat sink for effective cooling. In addition, each connector is fitted with a fuse, allowing faults to be interrupted and thereby enhancing system safety
<!-- Add drawings and pictures of the pdb--->

#### 4.3.7 Low-Voltage PDB
The low-voltage power distribution board is used for components with different DC inputs. The PDB accepts a 48V DC input and provides three different output voltage levels through three DC-DC converters. 

For the intended use, the first dc-dc converter is for all sensors and components that requires a 24V DC input. The middle is made for the NUC computer that needs 20V DC, and the last is free to regulate between 5-12V for future needs. In total, a single DC‑DC converter can deliver up to 10A. The main idea is to keep the board as modular och flexible as possible to easily be able to change the setup in the future. 

As for the high-voltage PDB, this is also equiped with diodes and fuses for safe operation.
<!-- Add drawings and pictures of the pdb--->

#### 4.3.8 Motors
The UGV is powered by four brushless DC motors from the manufacturer Cube Mars, with one integrated into each rim. The motor driver board is seamlessly installed within the motor. 
The motors have two different types of cables connected. One is power and is connected to the High‑Voltage PDB, ensuring a reliable and stable power supply. The second is the CAN cable, linked to the CAN splitter that is also integrated into the High‑Voltage PDB, enabling efficient communication and control.

In the event of a motor failure, replacement is straightforward and can be carried out quickly. The project currently operates with six motors in total, of which two are kept as reserves.

**WARNING:** The motors are not IP tested and must therefore be used with caution in wet or dirty terrain.

Some basic motor specifications include: 
- Weight: 960g
- Communication: CAN
- Rated voltage: 48V
- Rated torque: 18Nm, Peak torque: 53 Nm
- Rated speed: 235 rpm
- Rated current: 10.7 ADC, Peak current: 31.9 ADC

More information can be found in the BOM or on CubeMars website. 



### 4.4 CAN communication

#### 4.4.1 Motors
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

#### 4.4.2 Battery
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
Further information is available in [TAA22K12SP25A-Tattu-Plus-1.0-22Ah-12S-Specification-Sheet](https://github.com/MDU-C2/SLaRC-LightSeeker-HT25/blob/main/docs/DOC-Data_sheets/TAA22K12SP25A-Tattu-Plus-1.0-22Ah-12S-Specification-Sheet.pdf).

### 4.4.3 Software Architecture
The software architecture structure is shown in the illustration below.

<img width="255" height="245" alt="systemarchitecture" src="https://github.com/user-attachments/assets/a33c9468-ea76-47c3-9f39-3d2b243eb5f6" />
It is divided in two categories: motor and battery. In the motor-subfolder we have:
- can_bus_motor: Starts a can bus communication channel on socket can0 for the four motors.
- motor_api: low-level handling, velocity control, send commands, and RPM conversion.
- motor_group: Groups the motors on left and right side for skid-steer implementation with ID's corresponding to 1 & 3 for the left side and 2 & 4 for the right side.
- motor_telemetry: Showcases motor health values and interesting parameters such as voltage, amps, RPM/eRPM and
- motor_odometry: Calculates the UGVs position and sends data to sensors.
- motors: Sets parameters such as motor ID's and bitrate at 1 Mbit/s
The battery section:
- can_bus_battery: Starts a can bus communication channel on socket can1 for the two batteries.
- communication_handler: Handles battery packets, decoding, and safety measures. Sends packet accordingly:
CAN frame -> Buffer -> Full packet -> update_from_frame() -> Battery-objekt

The CAN communication, as well as the implemented functionality, was done in C++ and the source code can be found under the motor and battery branches respectively.
The motors and battery requires each CAN interface (USB-CAN / ODrive adapter) to support two separate CAN busses.

(KANSKE SKA BORT?)
Cube Mars provided motor cables and CAN cables intertwined in one singular short cable. To lengthen them similar cables was soldered on both the motor cables and CAN cables respectively. Custom-made CAN cables was assembled by combining a [JST-GH Head](https://se.rs-online.com/web/p/wire-housings-plugs/7521731), a [JST-GH Crimp](https://se.rs-online.com/web/p/crimp-contacts/7521725), and 28 AWG cables.

To run the program, following commands needs to be ran:
source install/setup.bash -> Makes ROS nodes and messages available in the shell
colcon build -> Compiles all ROS2 packages
ros2 run s_robot motor_controller -> Runs the program

### 4.5 Sensors


### 4.6 Computer


### 4.7 Router


### 4.8 RTK
## 5. Software Installation
<!-- e.g. step by step how to install or do something -->
### 5.1 ROS Installation
Prerequisites:
- Ubuntu 24.04
- ROS2 Jazzy

The ROS2 Installation was set up following [THIS](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html) guide.
### C++ Libraries
The following libraries is used for this application:
- can

## 6. Platform Start-Up and Operation

Here follows a step-by-step checklist on how to start and operate the UGV.

1. Read through the entire user guide
1. Install the battery/batteries.
1. Set main switch to ON mode.
1. 


## 7. Troubleshooting
<!-- e.g. Issue. UGV Fails to respond to commands. -->

**System is running but no power to the motors?**

Make sure the emergency stop botton is not pressed. If not pressed, check if any cable leading to the botton is broke or lose. 

**Component/unit not getting any power?**

Check the corresponding fuse.

## 8. Support & Contact
<!-- e.g. a list of all students in the SLaRC team and what role they had/what knowledge they possessed about-->
