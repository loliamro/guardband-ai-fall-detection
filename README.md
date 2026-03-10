# GuardBand AI
Smart Wrist-Worn Fall Detection & AI Anomaly Monitoring System

IoT Systems Design Project  
Abu Dhabi University – College of Engineering  
Author: Amro Kamal Jallad  
Date: March 2026


## Overview
GuardBand AI is a wearable IoT fall-detection and anomaly-monitoring system designed for elderly safety.  
The system runs entirely on an ESP32 microcontroller and uses an MPU6050 accelerometer/gyroscope to monitor body motion in real time.

The device detects dangerous events using a finite state machine (FSM) combined with adaptive AI-style anomaly detection based on statistical Z-scores.  
Sensor data is streamed over Wi-Fi to a Python dashboard where caregivers can observe movement graphs, alerts, and system status.

The system operates completely offline and requires no cloud infrastructure.


## Key Features

• Real-time fall detection using motion pattern recognition  
• AI-based anomaly monitoring with personalized calibration  
• OLED display for live device feedback  
• Red and blue LED alert indicators  
• Wi-Fi streaming to a real-time monitoring dashboard  
• Local edge computing — no cloud required  
• Hardware cost under $15


## System Architecture

The project consists of two main components.

### Wearable Device
Runs on the ESP32 and performs all sensing and decision making.

Hardware components:
- ESP32 Dev Module
- MPU6050 accelerometer/gyroscope
- 0.96" OLED display (SSD1306)
- Red LED alert indicator
- Blue LED status indicator

Functions:
- Reads sensor data at 100 Hz
- Runs fall detection finite state machine
- Performs AI calibration and anomaly detection
- Displays system status on OLED
- Streams JSON data over Wi-Fi


### Monitoring Dashboard
A Python desktop application used for real-time monitoring.

Features:
- Live acceleration graphs (Ax, Ay, Az)
- Combined acceleration magnitude plot
- Event log for anomalies and emergencies
- Patient status indicator
- Real-time sensor bars
- Wi-Fi connection to ESP32


## Fall Detection Algorithm

The system models the physical pattern of a fall using a six-state Finite State Machine.

States:

CALIBRATING  
Initial 60-second learning phase to establish motion baseline.

IDLE  
Normal monitoring state.

FREEFALL  
Triggered when acceleration drops below 0.6g.

IMPACT  
Triggered when acceleration spikes above 1.8g.

FALL CONFIRMED  
Detected impact followed by stillness.

EMERGENCY  
Triggered if fall confirmation countdown expires.


## AI Anomaly Detection

During calibration the system learns the user's normal movement pattern.

Two parameters are calculated:

μ  (mean acceleration)  
σ  (standard deviation)

Each new reading is evaluated using a Z-score:

Z = |a − μ| / σ

The system detects four anomaly types:

Unusual Movement  
Acceleration outside normal range.

Tremor  
Rapid oscillation detected through rolling standard deviation.

Slow Collapse  
Sustained drop in acceleration relative to baseline.

No Movement  
Sensor remains nearly static for an extended period.


## Hardware Wiring

MPU6050  
SDA → GPIO21  
SCL → GPIO22  

OLED SSD1306  
SDA → GPIO4  
SCL → GPIO5  

Red LED  
GPIO13 with 330Ω resistor  

Blue LED  
GPIO12 with 330Ω resistor  

Two independent I2C buses are used to avoid address conflicts.


## Running the Dashboard

Install Python and required libraries:
