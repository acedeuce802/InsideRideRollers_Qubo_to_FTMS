# InsideRideRollers_Qubo_to_FTMS
Reprogramming the original InsideRide Smart Rollers with Qubo Smart Unit to be updated to FTMS protocol

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. How to use Web Server
4. [How to install code](#how-to-install-code)
5. How to use the Smart Rollers
6. High level code flow
7. Calibration
8. LED States
9. Hardware

## Introduction
Cycling rollers have 2 rollers in the rear for the rear wheel, and 1 roller in the front.  They allow cyclists to ride indoors while balancing, to practice and train in a more realistic environment.  A Smart trainer is a device that connects to a training software (Zwift, Trainerroad, MyWhoosh, etc) which can control it's resistance to either simulate how a grade would feel, or to closed loop control power.  Most smart trainers are fixed, either by holding the rear axle with the rear tire on a roller, or the rear wheel is removed and the frame is clamped onto the trainer, but a small percentage of the trainers out there are smart rollers.  A few years back (2023? ) I started using rollers and eventually added fixed resistance with magnets, based on the Eddy current theory.  I had the idea of putting the magnets on a linear bearing and controlling them with a servo motor.  Luckily a few people have converted spin bikes to smart bikes already, so I at least had some guidance on the BLE connection.  This is my old project here: https://github.com/acedeuce802/DIY-Smart-Roller.  I had completed the code for ERG mode and hadn't yet written code for simulation mode, whcn a set of InsideRide E-motion Smart Rollers popped up for sale for a too-good-to-be-true price.  Shouldn't a production smart roller be much better than a DIY hack setup?  The hardware is great, they are on fore-aft linear bearings, are very smooth, and have a flywheel to better simulate road feel.  The software leaves a lot to be desired, these are an older version that uses an adapted wheel-on fixed trainer smart module, with a belt run to the rollers.  They use an older protocol that doesn't work on newer cycling programs and are very slow to react.  My biggest gripe is in the virtual game, Zwift, when feeling grade changes takes 4-5 seconds.  During a race, when competitors surge power at the base of a hill, I'm left in the dust.  This project is to replace the circuit board with one that adapts an ESP32 and stepper motor controller, can drop right in place of the old PCB, and significantly speeds up the rollers.

# Features
- Communication at 10hz with cycling programs
- Quick reaction of the stepper motor
- Removed software power limit (use at own risk, original limit of 900w)
- Code allows to calibrate power versus speed and stepper position, for an estimated power output (more accurate than the original 1x3 calibration)
- Code allows to calibrate stepper position versus speed and grade, you can decide if you want to ramp up resistance hard during fast sprints

# How to use the Web Server
















































































## How to install code


