# InsideRideRollers_Qubo_to_FTMS
Reprogramming the original InsideRide Smart Rollers with Qubo Smart Unit to use the FTMS protocol.

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [How to Use the Smart Rollers](#how-to-use-the-smart-rollers)
4. [How Smart Rollers Work](#how-smart-rollers-work)
5. [How to Use the Web Server](#how-to-use-the-web-server)
6. [How to Install Code](#how-to-install-code)
7. [How to Modify Code](#how-to-modify-code)
8. [How to View Serial Data](#how-to-view-serial-data)
9. [High-Level Code Flow](#high-level-code-flow)
10. [Calibration](#calibration)
11. [LED States](#led-states)
12. [Hardware](#hardware)
13. [Changelog](#changelog)

## Introduction
Cycling rollers have 2 rollers in the rear for the rear wheel, and 1 roller in the front. They allow cyclists to ride indoors while balancing, to practice and train in a more realistic environment. A smart trainer is a device that connects to training software (Zwift, TrainerRoad, MyWhoosh, etc.) which can control its resistance to either simulate how a grade would feel, or to closed-loop control power. Most smart trainers are fixed, either by holding the rear axle with the rear tire on a roller, or the rear wheel is removed and the frame is clamped onto the trainer, but a small percentage of trainers out there are smart rollers. The older version of the InsideRide E-Motion Smart Rollers use an adapted smart unit from an Elite Qubo trainer (see picture below). They use an older protocol that doesn't work on newer cycling programs and are very slow to react. My biggest gripe is in the virtual game, Zwift, where feeling grade changes takes 4-5 seconds. During a race, when competitors surge power at the base of a hill, I'm left in the dust. This project is to replace the circuit board with one that adapts an ESP32 and stepper motor controller, can drop right in place of the old PCB, and significantly speeds up the rollers.

![40143475285_08258a0821_c](https://github.com/user-attachments/assets/8bd6478c-a1c2-408c-916d-897ce8922b6e)

<img width="956" height="1270" alt="image" src="https://github.com/user-attachments/assets/2dd5cecf-59f9-4ea9-abed-5e18dc795f92" />


## Features
- Communication at 20 Hz with cycling programs
- Quick reaction of the stepper motor
- Removed software power limit (use at own risk, original limit of 900 W)
- Code allows calibration of stepper position versus speed and grade - you can decide if you want to ramp up resistance hard during fast sprints, have it mimic outside power vs. speed vs. grade, etc.
- Calibration table with more resolution than the Qubo unit (Qubo calibrates power to 3x1 speed x resistance table, this calibrates to 7x5 speed x resistance)
- 9-30V input (only tested with 12V supply, same as Qubo module comes with)
- WiFi Access Point with web server for diagnostics and configuration
- WiFi client mode to connect to your home network
- mDNS support (access via `http://insideride.local`)
- WebSocket for real-time live updates (5 Hz)
- Over-the-Air (OTA) firmware updates with rollback support
- Manual grade override from web interface
- IDLE curve calibration adjustable via web interface
- Automatic motor enable/disable based on speed (enables at 2.3 mph, disables at 2.0 mph)
- BLE connection keep-alive with automatic advertising restart

## How to Use the Smart Rollers
* These are intended to work very similarly to any smart trainer, so only unique instructions/tips/tricks are included.
* When connecting to cycling software, connect as a Power Meter first, then as a Controllable Trainer (sometimes both are selected automatically), then remove the device as a Power Meter, and select the power meter on your bike.
     - I'm working to fix, but currently if a physical power meter is connected first, this device won't be able to be connected as a Controllable Trainer.
* These will output an estimated power to the cycling software. Be aware that estimating power on rollers is difficult; any variation in system weight, tire pressure, tire temperature, etc., will skew the results.
* See the [Calibration section](#calibration) to get the calibration as close as possible, but it's best not to rely on this as it will only be close, not accurate (true for any smart roller).
* In SIM mode:
     - This works just as any smart trainer.
     - I enjoy riding Zwift at 50% Trainer Difficulty, meaning a 10% grade in Zwift actually translates to a 5% grade getting sent to the device.
     - Note: This device was calibrated by feel to what feels right. 50% Trainer Difficulty doesn't necessarily mean it'll feel half as easy as it does outside.
     - In SIM mode, the only function is using grade and roller speed to calculate resistance level.
     - The only function of [calibration](#calibration) in SIM mode is for estimated power.
* In ERG mode:
     - Calibration is important here. There is a [calibration](#calibration) table that takes target power and current roller speed, and outputs stepper motor position (resistance).
     - Apps like Zwift and TrainerRoad (any app with a Power Match function), will compensate if this calibration isn't perfect.
     - If for example, Zwift commands 300 W, you are spinning at 15 mph, the stepper motor goes to position=300, and your power meter reads 330 W, Zwift will slowly ramp down its target power until 300 W is achieved. Maybe Zwift sends 275 W target power and the new stepper position is 290 steps; Zwift will keep rechecking. If power is now 305 W, Zwift will send 273 W, the new stepper position is 289 steps, etc.
     - The closer your calibration is, the quicker ERG mode will react to new power levels.
     - Smart rollers have a narrower power band than a fixed trainer (like a Wahoo Kickr).
     - Fixed trainers can often complete all workouts without needing to shift gears.
     - Depending on current gear, wheel speed, and power target, smart rollers may either be at minimum resistance and you are making more power than the target, or vice versa.
     - See graph below to get an idea of the tuning band at each speed, and based on your FTP and workout selected (ultimately your min/max power), you can get an idea of what gear(s) you should be in for the workout.
     - Example: FTP of 300 W, doing a VO2 workout with intervals between 50% and 110% FTP (150 W and 330 W). The workout could be done at 10 mph, but any variation of cadence could push you under or over the tuning band. This may be better done at 8 mph for the rest intervals, and then do 1 or 2 upshifts to 12-15 mph for the VO2 intervals, which helps get you in the middle of the tuning band.

<img width="750" height="464" alt="image" src="https://github.com/user-attachments/assets/b7175e7f-262d-413d-aa35-3675efdd25b5" />

## How Smart Rollers Work
* Most cycling trainers work off the Eddy Current theory: magnets placed near moving ferrous material will create resistance.
* The Qubo smart unit uses a flywheel connected to the rear rollers by a belt, 2 magnets on a plastic sled, and a stepper motor to control the position.
* The closer the magnets are to being fully placed over the flywheel, the more resistance is generated, causing more power required at the pedals.
* As flywheel speed increases, required power also increases.
* In ERG mode for example, to keep a constant power, as the cyclist shifts up or increases cadence, magnetic resistance will need to be lowered to compensate, and vice versa.
* In SIM mode, there is some calibration table that sets magnet position based on grade (most trainers ignore wind, rolling resistance, etc.; those only affect speed in the game).
* This whole project is simply just: take ERG or SIM inputs, and tell the stepper motor what position to go to.

##### Magnet moving in relation to flywheel
![magnet](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/blob/TestLimits/gifs/Magnet_v2.gif)

##### Fast jog move of stepper motor
![jogging](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/blob/TestLimits/gifs/Jogging.gif)

#### Homing using the limit switch
![Homing](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/blob/TestLimits/gifs/Homing.gif)

## How to Use the Web Server

### Connecting to the Web Server

#### Option 1: Access Point Mode (Default)
1. Power on the device.
2. Connect to the WiFi network **"InsideRideCal"** (password: **insideride**).
3. Navigate to `http://192.168.4.1` or `http://insideride.local` in a browser.

#### Option 2: Home WiFi Mode
1. First connect via Access Point mode (above).
2. In the **WiFi Settings** section, enter your home network SSID and password.
3. Click **Save**, wait up to 60 seconds (the page may become unresponsive - this is normal).
4. Click **Restart**.
5. Reconnect to your home WiFi network.
6. Navigate to `http://insideride.local` or the IP address shown in the serial logs.

### Web Server Features

- **Live Diagnostics**: Real-time display of speed, power, position, target, mode, and BLE connection status (updates at 5 Hz via WebSocket).
- **Manual Control**:
  - **Go To Position**: Set a specific resistance position (0-1000).
  - **Go To Grade**: Simulate a specific grade (-4% to +10%).
  - **Resume App Control**: Return control to the cycling software.
- **IDLE Curve Calibration**: Adjust the speed-to-position polynomial curve coefficients (a, b, c, d).
- **WiFi Settings**: Configure home WiFi credentials for client mode.
- **OTA Firmware Update**: Upload new firmware via the web interface.
- **Firmware Rollback**: Roll back to the previous firmware version if needed.

<img width="418" height="794" alt="image" src="https://github.com/user-attachments/assets/f7df38b4-52ae-4321-9147-8cbced5f589d" />

## How to Install Code

### Via Web Server (OTA Update)
1. [Download the latest .bin file from here](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/tree/main/build/esp32.esp32.XIAO_ESP32C6).
2. Navigate to the Web Server.
3. **Important**: BLE must not be connected. Disconnect any cycling apps before updating.
4. In the **OTA Firmware Update** section, click **Choose File** and select the .bin file.
5. Click **Upload Firmware**.
6. The web server will indicate that the firmware is updating.
7. WiFi will disconnect on restart, so you will not get a final confirmation that firmware was uploaded.
8. Reconnect to WiFi, navigate back to the web server and confirm the firmware version matches the .bin file name.

<img width="334" height="195" alt="image" src="https://github.com/user-attachments/assets/9d1386bf-b0ef-4e37-af2a-232395936b46" />

### Via Arduino IDE
1. Download Arduino IDE software.
2. Navigate to **Boards Manager** and install **ESP32**.
3. [Download "boards.local.txt" and "xiao_c6_ota_16m.csv" from here](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/tree/main/partition).
4. In Windows Explorer, navigate to `C:\Users\<user>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.x.x` (you may have to allow viewing hidden files).
5. Place `boards.local.txt` into this ESP folder. To confirm you are in the right spot, you should also see `boards.txt`.
6. Place `xiao_c6_ota_16m.csv` into `C:\Users\<user>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.x.x\tools\partitions`.
7. [Download and open "InsideRideRollers_Qubo_to_FTMS.ino" from here](https://github.com/acedeuce802/InsideRideRollers_Qubo_to_FTMS/tree/main), and open with Arduino IDE.
8. Plug in the USB-C cable to the ESP32 (PCB V2 has tight clearance from the cable to power input and capacitor, be careful).
9. Select the COM port at the top of the Arduino screen, click **Select Other Board and Port** and select **XIAO_ESP32C6**.
10. Go to **Tools > Partition Scheme** and select **OTA (1.6MB APP/1.6MB OTA/0.85MB SPIFFS)**.
     - If this option isn't available, close Arduino IDE, navigate to `C:\Users\<user>\AppData\Roaming`, delete the `arduino-ide` folder, and reopen Arduino IDE.
11. Click **Upload** in the top left.

## How to Modify Code
1. [Follow the guide here to set up Arduino IDE](#via-arduino-ide).
2. Open the .ino file from the above guide.
3. Modify code.
4. If choosing to upload via Web Server, go to **Sketch > Export Compiled Binary**. The .ino.bin file will be located in your Arduino project folder, then follow the [Via Web Server instructions](#via-web-server-ota-update).
5. If choosing to upload via Arduino IDE, plug into the ESP32 via USB-C and upload.

## How to View Serial Data
If any issues arise during OTA updates, or the Web Server isn't working to show live data (like validating stepper motor position), serial commands are printed to help with diagnostics.

### Via Arduino IDE
1. Go to **Tools > Serial Monitor**.
2. Select **115200** baud.

### Via PuTTY
1. Download PuTTY.
2. On the **Session** screen (default screen), select **Serial** under **Connection Type**.
3. Enter the COM port under **Serial line** (see Device Manager or Arduino IDE to help find COM port).
4. Enter **115200** under **Speed**.
5. If you wish, you can save the session so you don't have to remember the COM port or baud rate.
6. Click **Open** and a new screen will print serial messages.
7. The **Logging** tab can also be used if saving serial messages to a file is needed.


## High-Level Code Flow

### Setup (runs once when power turned on)
1. Controller pins, speed sensor, etc. are all initialized.
2. Calibration and WiFi settings are loaded from NVS (non-volatile storage).
3. Controller homes the stepper motor by driving it toward the limit switch until the switch is pressed.
4. The stepper motor backs off slightly, slows down, and represses the switch to get a more accurate zero.
5. BLE is exposed and allowed to be connected to by cycling apps.
6. WiFi is started (tries home WiFi first if configured, falls back to AP mode).
7. Web Server and WebSocket server are initialized.

### Loop (constantly running)
*Note: This loop runs thousands of times per second, so the order doesn't really matter. If you update target stepper position first and then call the stepper motor to move, compared to calling the stepper motor first and then update the target after, the response time difference will be imperceptible, because either way the stepper position target and the call to move are happening within a millisecond of each other.*

1. BLE messages relating to SIM and ERG mode are only read when Zwift sends them; any functional code should be done outside of the BLE code.
     - Example: if there is code within the SIM mode code to update the stepper position based on grade and roller speed, and the grade is not changing in the cycling software, there will not be a new message, and therefore this stepper position function will not be called.
     - What happens with this data (grade, target power, etc.) happens every loop, but updating the cycling software variables (grade, target power) only occurs when the software sends updated messages.
2. Speed sensor is measured (can measure up to tens/hundreds of thousands of RPM).
3. Speed is run through a filter.
4. If ERG mode: target power is updated when a `0x05` BLE packet comes in, then target power and roller speed will compute the stepper position.
5. If SIM mode: grade is updated when a `0x11` BLE packet comes in, then grade and roller speed will compute the stepper position.
6. If neither (IDLE mode): a progressive speed versus stepper position curve is used, in case BLE drops while riding, you can at least hit all your desired powers based on wheel speed.
7. Motor auto-enable/disable based on speed (enables at 2.3 mph, disables at 2.0 mph for safety and noise reduction).
8. Stepper motor move command initiated.
9. Every 50 ms (20 Hz), estimated power and cadence are sent back to the cycling software (cadence is only 0 or 90, in order to make auto-pause work).
10. WebSocket broadcasts diagnostics at 5 Hz to connected web clients.

### Other functions that run in the background
* Constantly checking if OTA is active, to shut down other features so the controller isn't overloaded.
* CAL button is constantly monitored.
* The limit switch is always being monitored; if the motor misses steps and the controller no longer knows true position, the stepper will rehome if the limit switch is pressed.
* Every second, a serial message is printed with diagnostics (see function `printDiag` to add/remove messages).
* Code checks if Web Server manual control is active, in order to ignore ERG/SIM commands.
* BLE advertising is restarted periodically to maintain connection availability.

## Calibration
* Working to make spreadsheet cleaner. Will then upload a spreadsheet to turn test data into calibration tables, and make the descriptions more English-friendly.

#### Use the speed and stepper position targets, along with the Web Server readout and some way to read power (Zwift, Garmin, etc.), and fill in the calibration table
*(Note: my initial spreadsheet will assume you can exert enough power to hit all these targets; in the future I will refine the spreadsheet to make the calibration table based on whatever data you give it)*

<img width="515" height="133" alt="image" src="https://github.com/user-attachments/assets/42d8e861-e392-4805-9346-c76a7ff4a14d" />

#### Spreadsheet will extrapolate into a larger table to supply to the function `powerFromSpeedPos()` with x-axis stepper position, y-axis speed, and z-output power
I did no riding at max stepper position because the resistance is so great, and I obviously didn't reach 50 mph to calibrate, but extrapolated that far so I didn't have to extrapolate in Arduino code.

<img width="616" height="177" alt="image" src="https://github.com/user-attachments/assets/81bc5640-7105-483f-8466-2abc16836f68" />

#### Spreadsheet will back-calculate a new table to supply to the function `stepFromPowerSpeed()` with x-axis target power, y-axis speed, and z-output stepper position
Anything below 0 was clamped to 0, and anything above 1000 was clamped to 1000.

<img width="1015" height="175" alt="image" src="https://github.com/user-attachments/assets/f8f9d1f2-57ec-453a-8201-e04f75d1d41c" />

#### This table was made from testing out different Trainer Difficulties, finding what grade vs. stepper position I liked, and then ramping up with speed so that you don't have to spin so fast to get high power on flats
This supplies to the function `gradeToSteps()` with x-axis grade, y-axis speed, and z-output stepper position.

<img width="814" height="197" alt="image" src="https://github.com/user-attachments/assets/88826368-dddb-47ad-84c3-26f48c430932" />

#### IDLE Curve Calibration
The IDLE mode uses a cubic polynomial to determine stepper position based on speed:
```
position = a + b×speed + c×speed² + d×speed³
```
These coefficients can be adjusted via the Web Server's **IDLE Curve Calibration** section. Changes are saved to NVS and persist across reboots.


## LED States
| State | LED Behavior |
|-------|--------------|
| OTA in progress | Rapid blink (10 Hz) |
| OTA unlocked window | Double-blink every ~2s |
| Fault / rehome requested / limit hit | SOS-like triple blink repeating |
| Homing | Medium blink (2 Hz) |
| BLE connected | "Heartbeat" (short on, long off) |

### Mode Indication
| Mode | LED Behavior |
|------|--------------|
| ERG | Solid ON |
| SIM | Slow blink (1 Hz) |
| IDLE | OFF |

## Hardware
See PCB schematic and design below.

<img width="781" height="541" alt="image" src="https://github.com/user-attachments/assets/6f08b609-51e6-4172-a5f8-a1dbfc3bb7c9" />
<img width="1086" height="388" alt="Screenshot 2026-01-15 153518" src="https://github.com/user-attachments/assets/46805f0a-811e-409a-9c82-0e54e627dd44" />

### Hardware List
| Component | Description |
|-----------|-------------|
| PCB | Custom PCB |
| Xiao ESP32-C6 | Microcontroller |
| Pololu DRV8825 | Stepper Motor Driver |
| Mini360 Buck Converter | 5-30V to 5V |
| 1x 100µF capacitor | Power filtering |
| 2x 0.1µF capacitor | Decoupling |
| S6B-PH-K-S | 6-pin connector |
| S3B-PH-K-S | 3-pin connector |
| S2B-PH-K-S | 2-pin connector |
| PJ-037A | Power input jack |
| RUEF185 | 1.85A resettable fuse |
| 1N5819 | Schottky diode for 5V input protection |
| MJTP1243 | CAL switch (poor fit with V2 PCB) |
| WP934EW/GD | Green LED |
| 330Ω 1/4W resistor | LED current limiting (poor fit with V2 PCB) |
| Ring terminal and wire | Motor ground strap |


## Changelog
*Only including FW versions since I started exporting .bin files as unique FW names*

| Version | Changes |
|---------|---------|
| 2026-01-14_01 | Added LED states |
| 2026-01-14_02 | Fixed gMode LED states |
| 2026-01-14_03 | Added resistance curve when SIM or ERG not active |
| 2026-01-15_01 | Stepper disable if target doesn't move beyond threshold |
| 2026-01-15_02 | BLE keep-alive, boot loop fixes, safety features |
| 2026-01-15_03 | OTA improvements, calibration page |
| 2026-01-15_04 | Grade override control |
| 2026-01-16_01 | WiFi client mode with NVS storage, mDNS support |
| 2026-01-23_01 | WebSocket for live updates, OTA rollback support |
| 2026-02-02_01 | WiFi settings via web UI, improved NVS reliability |
