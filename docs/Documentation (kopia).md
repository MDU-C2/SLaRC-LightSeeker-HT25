
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
1. [Platform Features]()
    1. [Battery Charging](#31-battery-charging)
    1. [Main Switch](#32-main-switch)
    1. [Open the circuit](#321-open-the-circuit)
1. [Hardware Setup]()
    1. [power-system](#41-power-system)
    1. [battery](#411-battery)
    1. [main-switch](#412-main-switch)
    1. [emergency-stop-button](#413-emergency-stop-button)
    1. [shunt-regulator](#414-shunt-regulator)
    1. [conductor](#415-conductor)
    1. [high-voltage-pcb](#416-high-voltage-pcb)
    1. [low-voltage-pcb](#417-low-voltage-pcb)
    1. [motor](#418-motor)
    1. [can-communication](#42-can-communication)
    1. [motors](#421-motors)
    1. [battery](#422-battery)
1. [Software Installation](#5-software-installation)
1. [Troubleshooting](#6-troubleshooting)
1. [Support & Contact](#7-support--contact)
---

## 1. Introduction 
Welcome to the SLaRC UGV user guide. This document will help you to set up, use, and troubleshoot the UGV platform.

**Who is this for?**
<br> This guide is intended for new users, technicians and administrators.

## 2. System Overview
The platform is a Unmanned Ground Vehicle (UGV) and is a part of the SLaRC project. ...

### 2.1 Bill of Materials
All components and spare parts are listed in the file [DOC-001-Bill_Of_Materials](https://github.com/MDU-C2/SLaRC-LightSeeker-HT25/blob/main/docs/DOC-Files/DOC-001-Bill_Of_Materials.xlsx)
### 2.2 Mechanical Design
<!-- e.g. solidwork, images of platform/sensors-->
## 3. Platform Features
<!-- e.g. reqirements or functions that the device can perform -->
### 3.1 Battery Charging

### 3.2 Main Switch
The platfrom is equipted with a main switch with the purpose of disconnect/open the circuit from the battery to the rest of the circuit. 

#### 3.2.1 Open the circuit:
Turn the knob clockwise to the green ON position
## 4. Hardware Setup
<!-- e.g. step by step how to set up something-->
### 4.1 Power System 
<!-- e.g. How the PDB is designed and all it functions, all other components-->
#### 4.1.1 Battery
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

#### 4.1.2 Main Switch

The UGV:S main switch is located at the back of the platform. Its main purpose is to disconnect/open the ciurcut from the battery to the rest of the circuit. The two states is ON (green field), and OFF (red field).

#### 4.1.3 Emergency Stop Button

#### 4.1.4 Shunt Regulator

#### 4.1.5 Conductor

#### 4.1.6 High Voltage PCB
The high-voltage power distribution board is used for components that require 48V DC input. The PDB has an input from the battery, four outputs to the motors, and one output to the router.

It is designed to make a maximum current of 30A from the motors and 20A to the router.
The card also has extra space for CAN communication, including two USB-to-CAN adapters and two CAN bus splitters.
<!-- Add drawings and pictures of the pdb--->

#### 4.1.7 Low Voltage PCB
The low-voltage power distribution board is used for components with different DC inputs. The PDB accepts a 48V DC input and provides three output voltage levels through three DC-DC converters. The first is for all sensors and components that need a 24V DC input. The middle is made for the NUC computer that needs 20V DC, and the last is flexible between 5-12V for future sensors. 
<!-- Add drawings and pictures of the pdb--->

#### 4.1.8 Motor

### 4.2 CAN communication
<!-- Add step by step how you did. Don't forget the downloaded things--->
#### 4.2.1 Motors

#### 4.2.2 Battery

### 4.3 Sensors

### 4.4 Computer

### 4.5 Router

### 4.6 RTK

### 4.7 Emergency Stop

### 4.8 Add/remove sensors to the PDB

### 4.9 Change Output Voltage on PDB

## 5. Software Installation
<!-- e.g. step by step how to install or do something -->

## 6. Troubleshooting
<!-- e.g. Issue. UGV Fails to respond to commands. -->

## 7. Support & Contact
<!-- e.g. a list of all students in the SLaRC team and what role they had/what knowledge they possessed about-->

