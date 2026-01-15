# InsideRideRollers_Qubo_to_FTMS
Reprogramming the original InsideRide Smart Rollers with Qubo Smart Unit to be updated to FTMS protocol

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [How to use the Smart Rollers](#how-to-use-the-smart-rollers)
4. [How Smart Rollers work](#how-smart-rollers-work)
5. [How to use Web Server](#how-to-use-the-web-server)
6. [How to install code](#how-to-install-code)
7. [How to modify code](#how-to-modify-code)
8. [How to view Serial Data](#how-to-view-serial-data)
9. [High level code flow](#high-level-code-flow)
10. [Calibration](#calibration)
11. [LED States](#led-states)
12. [Hardware](#hardware)
13. [Changelog](#changelog)

## Introduction
Cycling rollers have 2 rollers in the rear for the rear wheel, and 1 roller in the front.  They allow cyclists to ride indoors while balancing, to practice and train in a more realistic environment.  A Smart trainer is a device that connects to a training software (Zwift, Trainerroad, MyWhoosh, etc) which can control it's resistance to either simulate how a grade would feel, or to closed loop control power.  Most smart trainers are fixed, either by holding the rear axle with the rear tire on a roller, or the rear wheel is removed and the frame is clamped onto the trainer, but a small percentage of the trainers out there are smart rollers.  The older version of the InsideRide E-Motion Smart Rollers use an adapted smart unit from an Elite Qubo trainer (see picture below).  They use an older protocol that doesn't work on newer cycling programs and are very slow to react.  My biggest gripe is in the virtual game, Zwift, when feeling grade changes takes 4-5 seconds.  During a race, when competitors surge power at the base of a hill, I'm left in the dust.  This project is to replace the circuit board with one that adapts an ESP32 and stepper motor controller, can drop right in place of the old PCB, and significantly speeds up the rollers.

![40143475285_08258a0821_c](https://github.com/user-attachments/assets/8bd6478c-a1c2-408c-916d-897ce8922b6e)

<img width="956" height="1270" alt="image" src="https://github.com/user-attachments/assets/2dd5cecf-59f9-4ea9-abed-5e18dc795f92" />


## Features
- Communication at 20hz with cycling programs
- Quick reaction of the stepper motor
- Removed software power limit (use at own risk, original limit of 900w)
- Code allows to calibrate stepper position versus speed and grade, you can decide if you want to ramp up resistance hard during fast sprints, have it mimic outside power vs. speed vs. grade, etc
- Calibration table with more resolution than the Qubo unit (Qubo calibrates power to 3x1 speed x resistance table, this calibrates to 7x5 speed x resistance)
- 9-30V input (only tested with 12V supply, same as Qubo module comes with)

## How to use the Smart Rollers
* These are intended to work very similarly to any smart trainer, so only unique instructions/tips/tricks are included
* When connecting to cycling software, connect as a Power Meter first, then as a Controllable Trainer (sometimes both are selected automatically), then remove the device as a Power Meter, and select the power meter on your bike
     - I'm working to fix, but currently if a physical power meter is connected first, this device won't be able to be connected as a Controllable Trainer
* These will output an estimated power to the cycling software, be aware that estimating power on rollers is difficult, any variation in system weight, tire pressure, tire temperature, etc, will skew the results
* See the [Calibration section](#calibration) to get the calibration as close as possible, but it's best not to rely on this as it will only be close, not accurate (true for any smart roller)
* In SIM mode:
     - This works just as any smart trainer
     - I enjoy riding Zwift at 50% Trainer Difficulty, meaning a 10% grade in Zwift actually translates to a 5% grade getting sent to the device
     - Note: This device was calibrated by feel to what feels right, 50% Trainer Difficulty doesn't necessarily mean it'll feel half as easy as it does outside
     - In SIM mode, the only function is using grade and roller speed to calculate resistance level
     - The only function of [calibration](#calibration) in SIM mode is for estimated power
* In ERG mode:
     - Calibration is important here, there is a [calibration](#calibration) table that takes target power and current roller speed, and outputs stepper motor position (resistance)
     - Apps like Zwift and Trainerroad (any app with a Power Match function), will compensate if this calibration isn't perfect
     - If for example, Zwift commands 300w, you are spinning at 15mph, the stepper motor goes to position=300, and your power meter reads 330w, Zwift will slowly ramp down it's target power until 300w is achieved
     - The closer your calibration is, the quicker ERG mode will react to new power levels
     - Smart rollers have a narrower power band than a fixed trainer (like a Wahoo Kickr)
     - Fixed trainers can often complete all workouts without needing to shift gears
     - Depending on current gear, wheel speed, and power target, smart rollers may either be at minimum resistance and you are making more power than the target, or vice versa     -
     - See graph below to get an idea of the tuning band at each speed, and based on your FTP and workout selected (ultimately your min/max power), you can get an idea of what gear(s) you should be in for the workout
     - Example: FTP of 300w, doing a VO2 workout with intervals between 50% and 110% FTP (150w and 330w), the workout could be done at 10mph, but any variation of cadence could push you under or over the tuning band
     - This may be better done at 8mph for the rest intervals, and then do 1 or 2 upshifts to 12-15mph for the VO2 intervals, which helps get you in the middle of the tuning band

<img width="750" height="464" alt="image" src="https://github.com/user-attachments/assets/b7175e7f-262d-413d-aa35-3675efdd25b5" />

## How Smart Rollers Work
* Most cycling trainers work off the Eddy Current theory, that magnets placed near moving ferrous material will create resistance
* The Qubo smart unit uses a flywheel connected to the rear rollers by a belt, 2 magnets on a plastic sled, and a stepper motor to control the position
* The closer the magnets are to being fully placed over the flywheel, the more resistance is generated, causing more power required at the pedals
* As flywheel speed increases, required power also increases
* In ERG mode for example, to keep a constant power, as the cyclist shifts up or increase cadence, magnetic resistance will need to be lowered to compensate, and vice versa
* In SIM mode, there is some calibration table that sets magnet position based on grade (most trainers ignore wind, rolling resistance, etc, those only affect speed in the game)
* This whole project is simply just: take ERG or SIM inputs, and tell the stepper motor what position to go to

##### Magnet moving in relation to flywheel
![magnet](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/blob/TestLimits/gifs/Magnet_v2.gif)

##### Fast jog move of stepper motor
![jogging](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/blob/TestLimits/gifs/Jogging.gif)

#### Homing using the limit switch
![Homing](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/blob/TestLimits/gifs/Homing.gif)

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
#### Setup (runs once when power turned on)
1. Controller pins, speed sensor, etc are all initialized
2. Controller homes the stepper motor, by driving it toward the limit switch, until the switch is presed
3. The stepper motor backs off slightly, slows down, and represses the switch to get a more accurate zero
4. BLE is exposed and allowed to be connected to by cycling apps
5. Web Server is called and Wi-Fi enabled

#### Loop (constantly running)
*Note: This loop runs at thousands of times per second, so the order doesn't really matter.  If you update target stepper position first and then call the stepper motor to move, compared to calling the stepper motor first and then update the target after, the response time difference will be imperceivable, because either way the stepper position target and the call to move are happening within a millisecond of each other  

1. BLE messages relating to SIM and ERG mode are only read when Zwift sends them, any functional code should be done outside of the BLE code
     - Example, if there is code within the SIM mode code to update the stepper position based on grade and roller speed, and the grade is not changing in the cycling software, there will not be a new message, and therefore this stepper position function will not be called (see code block "0x11" to see how minimal the SIM mode message code is)
     - What happens with this data (grade, target power, etc) happens every loop, but updating the cycling software variables (grade, target power) only occurs when the software sends updated messages
3. Speed sensor is measured (can measure up to tens/hundreds of thousands of RPM)
4. Speed is run through a filter
5. If ERG mode: target power is updated when an "0x05" BLE packet comes in, then target power and roller speed will compute the stepper position
6. If SIM mode: grade is updated when an "0x11" BLE packet comes in, then grade and roller speed will compute the stepper position
7. If neither: a progressive speed versus stepper position curve is used, in case BLE drops while riding, you can at least hit all your desired powers based on wheel speed
8. Stepper motor move command initiated
9. Every 10hz, estimated power and cadence are sent back to the cycling software (cadence is only 0 or 90, in order to make auto pause work)

##### Other functions that run in the background (less critical to main code flow)
* Constantly checking if OTA is active, to shut down other features, such that the controller isn't overloaded
* CAL button is constantly monitored
* The limit switch is always being monitored, if the motor misses steps and the controller no longer knows true position, the stepper will rehome if the limit switch is pressed
* Every second, a Serial message is printed with diagnostics (see function printDiag to add/remove messages)
* Code checks if Web Server "Go-To" function is active, in order to ignore ERG/SIM commands

## Calibration
* Working to make spreadsheet cleaner, will then upload a spreadsheet to turn test data into calibration tables, and make the descriptions more english

#### Use the speed and stepper position targets, along with the Web Server readout and some way to read power (Zwift, Garmin, etc), and fill in the calibration table (Note: my initial spreadsheet will assume you can exert enough power to hit all these targets, in the future I will refine the spreadsheet to make the calibration table based on whatever data you give it)
<img width="515" height="133" alt="image" src="https://github.com/user-attachments/assets/42d8e861-e392-4805-9346-c76a7ff4a14d" />

#### Spreadsheet will extrapolate into a larger table to supply to the function powerFromSpeedPos() with x-axis stepper position, y-axis speed, and z-output power, I did no riding at max stepper position because the resistance is so great, and I obviously didn't reach 50mph to calibrate, but extrapolated that far so I didn't have to extrapolate in Arduino code
<img width="616" height="177" alt="image" src="https://github.com/user-attachments/assets/81bc5640-7105-483f-8466-2abc16836f68" />

#### Spreadsheet will back calculate a new table to supply to the function stepFromPowerSpeed() with x-axis target power, y-axis speed, and z-putput stepper position.  Anything below 0 was clamped to 0, and anything above 1000 was clamped to 1000
<img width="1015" height="175" alt="image" src="https://github.com/user-attachments/assets/f8f9d1f2-57ec-453a-8201-e04f75d1d41c" />

#### This table was made from testing out different Trainer Difficulties, finding what grade vs. stepper position I liked, and then ramping up with speed so that you don't have to spin so fast to get high power on flats.  This supplies to the function gradeToSteps() with x-axis grade, y-axis speed, and z-output stepper position
<img width="814" height="197" alt="image" src="https://github.com/user-attachments/assets/88826368-dddb-47ad-84c3-26f48c430932" />


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
See PCB schematic and design below
<img width="781" height="541" alt="image" src="https://github.com/user-attachments/assets/6f08b609-51e6-4172-a5f8-a1dbfc3bb7c9" />
<img width="1086" height="388" alt="Screenshot 2026-01-15 153518" src="https://github.com/user-attachments/assets/46805f0a-811e-409a-9c82-0e54e627dd44" />

#### Hardware List
* PCB
* Xiao ESP32-C6
* Pololu DRV8825 Stepper Motor Driver
* Mini360 Buck Converter (5-30V -> 5V)
* 1x 100uF capacitor
* 2x 0.1uF capacitor
* S6B-PH-K-S (6-pin connector)
* S3B-PH-K-S (3-pin connector)
* S2B-PH-K-S (2-pin connector)
* PJ-037A (power input)
* RUEF185 (1.85A fuse)
* 1N5819 (schottky diode for 5v input protection)
* MJTP1243 (CAL switch, poor fit with V2 PCB)
* WP934EW/GD (green LED)
* 330ohm 1/4w resistor (for driving LED, poor fit with V2 PCB)
* Ring terminal and wire for motor ground strap


## Changelog
*Only including FW versions since I started exporting .bin files as unique FW names

- 2026-01-14_01 - Added LED states
- 2026-01-14_02 - Fixed gMode LED states
- 2026-01-14_03 - Added resistance curve when SIM or ERG not active
