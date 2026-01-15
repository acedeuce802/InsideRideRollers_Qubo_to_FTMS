# InsideRideRollers_Qubo_to_FTMS
Reprogramming the original InsideRide Smart Rollers with Qubo Smart Unit to be updated to FTMS protocol

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [How to use the Smart Rollers](#how-to-use-the-smart-rollers)
4. [How to use Web Server](#how-to-use-the-web-server)
5. [How to install code](#how-to-install-code)
6. [How to modify code](#how-to-modify-code)
7. [How to view Serial Data](#how-to-view-serial-data)
9. [High level code flow](#high-level-code-flow)
10. [Calibration](#calibration)
11. [LED States](#led-states)
12. [Hardware](#hardware)
13. [Changelog](#changelog)

## Introduction
Cycling rollers have 2 rollers in the rear for the rear wheel, and 1 roller in the front.  They allow cyclists to ride indoors while balancing, to practice and train in a more realistic environment.  A Smart trainer is a device that connects to a training software (Zwift, Trainerroad, MyWhoosh, etc) which can control it's resistance to either simulate how a grade would feel, or to closed loop control power.  Most smart trainers are fixed, either by holding the rear axle with the rear tire on a roller, or the rear wheel is removed and the frame is clamped onto the trainer, but a small percentage of the trainers out there are smart rollers.  The older version of the InsideRide E-Motion Smart Rollers use an adapted smart unit from an Elite Qubo trainer (see picture below).  They use an older protocol that doesn't work on newer cycling programs and are very slow to react.  My biggest gripe is in the virtual game, Zwift, when feeling grade changes takes 4-5 seconds.  During a race, when competitors surge power at the base of a hill, I'm left in the dust.  This project is to replace the circuit board with one that adapts an ESP32 and stepper motor controller, can drop right in place of the old PCB, and significantly speeds up the rollers.

![40143475285_08258a0821_c](https://github.com/user-attachments/assets/8bd6478c-a1c2-408c-916d-897ce8922b6e)

## Features
- Communication at 20hz with cycling programs
- Quick reaction of the stepper motor
- Removed software power limit (use at own risk, original limit of 900w)
- Code allows to calibrate power versus speed and stepper position, for an estimated power output (more accurate than the original 1x3 calibration)
- Code allows to calibrate stepper position versus speed and grade, you can decide if you want to ramp up resistance hard during fast sprints

## How to use the Smart Rollers
* These are intended to work very similarly to any smart trainer, so only unique instructions/tips/tricks are included
* When connecting to cycling software, connect as a Power Meter first, then as a Controllable Trainer (sometimes both are selected automatically), then remove the device as a Power Meter, and select the power meter on your bike
     - I'm working to fix, but currently if a physical power meter is connected first, this device won't be able to be connected as a Controllable Trainer
* In SIM mode:
     - This works just as any smart trainer
     - I enjoy riding Zwift at 50% Trainer Difficulty, meaning a 10% grade in Zwift actually translates to a 5% grade getting sent to the device
     - Note: This device was calibrated by feel to what feels right, 50% Trainer Difficulty doesn't necessarily mean it'll feel half as easy as it does outside
* In ERG mode:
     - Smart rollers have a narrower power band than a fixed trainer (like a Wahoo Kickr)
     - Fixed trainers can often complete all workouts without needing to shift gears
     - Depending on current gear, wheel speed, and power target, smart rollers may either be at minimum resistance and you are making more power than the target, or vice versa     -
     - See graph below to get an idea of the tuning band at each speed, and based on your FTP and workout selected (ultimately your min/max power), you can get an idea of what gear(s) you should be in for the workout

## How to use the Web Server
1. Whenever the power is on, the Wi-Fi SSID "InsideRideCal" will be able to be connected to
2. Wi-Fi Password is "insideride"
3. Navigate in a browser to 192.168.4.1
4. Diagnostics will update real-time
5. Use the "Go-To" function to set the resistance to a fixed value
     - Use "Enable stepper" before executing a move
     - During this time, cycling software will stay connected but resistance commands will be ignored
     - See calibration to get an idea of which stepper motor position and wheel speed will translate to a certain power
     - Press "Resume Software Control" to make cycling software command resistance

<img width="418" height="794" alt="image" src="https://github.com/user-attachments/assets/f7df38b4-52ae-4321-9147-8cbced5f589d" />

## How to install code
### Via Web Server
1. [Download the latest .bin file from here](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/tree/TestLimits/build/esp32.esp32.XIAO_ESP32C6)
2. Navigate to the Web Server and click "Firmware Update"
     - In order to update firmware, the CAL button needs pressed, and BLE must not be connected
     - CAL button is the button nearest the 3-pin connector
     - If CAL button is not pressed, Web Server will read "OTA is LOCKED. Press CAL to unlock for 60 seconds, then refresh /update."
3. Click "Choose File" and upload the .bin file
4. Click "Update"
5. The Web Server will indicate that the firmware is updating
6. Wi-Fi will disconnect on restart, so you will not get a final confirmation that firmware was uploaded
7. Reconnect to Wi-Fi, navigate back to 192.184.4.1 and confirm the "FW:" version at the bottom matches the .bin file name

<img width="334" height="195" alt="image" src="https://github.com/user-attachments/assets/9d1386bf-b0ef-4e37-af2a-232395936b46" />

### Via Arduino IDE
1. Download Arduino IDE software
2. Navigate to "Boards Manager" and download "ESP32"
3. [Download "boards.local.txt" and "xiao_c6_ota_16m.csv" from here](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/tree/TestLimits/partition)
4. In Windows Explorer, navigate to "C:\Users\*user*\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.3" (you may have to allow viewing hidden files)
5. Place "boards.local.txt" into this ESP folder, to confirm you are in the right spot, you should also see "boards.txt"
6. Place "xiao_c6_ota_16m.csv" into "C:\Users\*user*\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.3\tools\partitions
7. [Download and open "InsideRideRollers_Qubo_to_FTMS.ino" from here, and open with Arduino IDE](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/tree/TestLimits)
8. Plug in the USB-C cable to the ESP32 (PCB V2 has tight clearance from the cable to power input and capacitor, be careful)
9. Select the COM port at the top of the Arduino screen, click "Select Other Board and Port" and select "XIAO_ESP32C6"
10. Go to Tools->Partition Scheme and select "OTA (1.6MB APP/1.6MB OTA/0.85MB SPIFFS)"
     - If this option isn't available, close Arduino IDE, navigate to "C:\Users\*user*\AppData\Roaming", delete the "arduino-ide" folder, and reopen Arduino IDE
11. Click "Upload" in the top left

## How to modify code
1. [Follow the guide here to setup Arduino IDE](#via-arduino-ide)
2. Open the .ino file from the above guide
3. Modify code
4. If choosing to upload via Web Server, go to Sketch->Export Compiled Binary, the .ino.bin file will be located in your Arduino project folder, then follow the [Via Web Server instructions](#via-web-server)
5. If choosing to upload via Arduino IDE, plug into the ESP32 via USB-C and upload

## How to view Serial Data
If any issues arise during OTA updates, or the Web Server isn't working to show live data (like validating stepper motor position), serial commands are printed to help with diagnostics.
### Via Arduino IDE
1. Tools->Serial Monitor
2. Select 115200 baud

### Via PuTTy
1. Download PuTTy
2. On "Session" screen (default screen), select "Serial" under "Connection Type"
3. Enter COM port under "Serial line" (see Device Manager or Arduino IDE to help find COM port)
4. Enter "115200" under "Speed"
5. If you wish, you can save the session so you don't have to remember the COM port or baud rate
6. Click "Open" and a new screen will print Serial messages
7. The "Logging" tab can also be used if saving Serial messages to a file is needed


## High level code flow

## Calibration

## LED States
* OTA in progress: rapid blink (10 Hz)
* OTA unlocked window: double-blink every ~2s
* Fault / rehome requested / limit hit: SOS-like triple blink repeating
* Homing: medium blink (2 Hz)
* BLE connected: “heartbeat” (short on, long off)  
##### Mode indication:
* ERG: solid ON
* SIM: slow blink (1 Hz)
* IDLE: OFF

## Hardware


## Changelog
*Only including FW versions since I started exporting .bin files as unique FW names

- 2026-01-14_01 - Added LED states
- 2026-01-14_02 - Fixed gMode LED states
- 2026-01-14_03 - Added resistance curve when SIM or ERG not active
