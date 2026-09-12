<h1 align="center">🤖 17-SERVO HUMANOID ROBOT</h1>

<p align="center">
  <img src="images/robot-final.jpg.jpg" width="500">
</p>

<h1 align="center">17-SERVO HUMANOID ROBOT</h1>

<p align="center">
  <strong>A DIY humanoid robot built from mechanical parts, electronics, code, and a questionable amount of patience.</strong>
</p>

<p align="center">
  Mechanical Design • Electronics • Programming • Wireless Control • Robotics
</p>

<p align="center">
  <a href="#overview">Overview</a> •
  <a href="#features">Features</a> •
  <a href="#hardware">Hardware</a> •
  <a href="#circuit-diagrams">Circuits</a> •
  <a href="#software">Software</a> •
  <a href="#robot-gallery">Gallery</a> •
  <a href="#build-process">Build</a>
</p>

<p align="center">

![Status](https://img.shields.io/badge/Status-Working%20Prototype-orange)
![Servos](https://img.shields.io/badge/Servos-17-blue)
![Controller](https://img.shields.io/badge/Main%20Controller-ESP32-green)
![Transmitter](https://img.shields.io/badge/Hand%20Controller-ESP8266-purple)
![License](https://img.shields.io/badge/License-MIT-yellow)

</p>

---

## 🧠 Overview

The **17-Servo Humanoid Robot** is a DIY humanoid robotics project built to explore the beautiful intersection of **mechanical design, embedded electronics, wireless communication, servo control, sensors, and programmable movement**.

At the center of the system is an **ESP32-based receiver** responsible for controlling the robot's electronics and servo system.

The robot is paired with an **ESP8266-based hand controller** that uses a **flex sensor** to translate physical hand movement into wireless commands.

In simpler words:

> **Bend the controller → send the command → move the robot.**

No magic. Just electronics, mathematics, wires, and several opportunities to discover that one connector was plugged in upside down.

This repository contains the project's:

- Source code
- Circuit diagrams
- Robot photographs
- Mechanical design files
- Receiver firmware
- Transmitter firmware
- Build information
- Calibration information
- Project documentation

The goal is to make the project understandable, reproducible, and easy for other makers and robotics enthusiasts to explore.

---

# ⚡ Features

<table>
<tr>
<td width="50%">

### 🤖 Robot

- 17-servo humanoid robot
- Custom mechanical structure
- Programmable humanoid movements
- 16-channel servo control system
- Dedicated servo power distribution

</td>
<td width="50%">

### 🧠 Electronics

- ESP32 robot receiver
- ESP8266 wireless transmitter
- Flex-sensor based control
- OLED display
- Audio amplifier
- Speaker system
- PCA9685 servo driver

</td>
</tr>
</table>

### System Highlights

- 17-servo humanoid robot
- ESP32-based robot receiver
- ESP8266-based wireless hand controller
- Flex-sensor based control
- OLED display
- Audio amplifier / speaker system
- 16 servo control system
- Dedicated power distribution for servos
- Wireless communication between controller and robot
- Custom 3D-printed mechanical structure
- Programmable humanoid movements

---

# 📋 Robot Specifications

| Specification | Details |
|---|---|
| **Robot Type** | Humanoid |
| **Servo Motors** | 17 |
| **Main Controller** | ESP32 |
| **Hand Controller** | ESP8266 |
| **Input** | Flex Sensors |
| **Display** | OLED |
| **Audio** | Amplifier + Speaker |
| **Servo Control** | PCA9685 |
| **Communication** | Wireless |
| **Mechanical Parts** | 3D Printed |
| **Project Status** | Working Prototype |

---

# 🔩 Hardware

## 🤖 Robot

The robot consists of:

- ESP32 development board
- PCA9685 servo driver
- 17 servo motors
- OLED display
- Audio amplifier
- Speaker
- Power supply
- Wiring and connectors
- Custom 3D-printed mechanical parts

### The Robot's Job

The ESP32 acts as the main brain of the robot.

It receives commands, processes them, and controls the connected electronics and servo motors.

The PCA9685 handles the servo-control side of the system, allowing multiple servo motors to be controlled from the ESP32.

---

## ✋ Hand Controller

The wireless controller consists of:

- ESP8266 development board
- Flex sensor
- Supporting resistors
- Power supply
- Control wiring

The flex sensor provides the physical input used to control the robot wirelessly.

The controller reads the sensor movement and sends the corresponding commands to the robot receiver.

---

# 🔌 Circuit Diagrams

All circuit diagrams are organized inside the [`diagrams`](diagrams) folder.

The diagrams are separated into **Receiver** and **Transmitter** sections so that the electronics can be understood without digging through a pile of wires that looks like it has developed its own ecosystem.

---

## 📡 Receiver Diagrams

### 1. ESP32 + OLED + Amplifier

This diagram shows the main receiver electronics, including the **ESP32, OLED display, amplifier, and related connections**.

<p align="center">
  <img src="diagrams/receiver/esp32-oled-amplifier.png.png" width="650">
</p>

---

### 2. Servo Connections

This diagram shows the **signal connections between the PCA9685 servo driver and the servo motors**.

<p align="center">
  <img src="diagrams/receiver/servo-connections.png.png" width="650">
</p>

---

### 3. Servo Power Supply

This diagram shows how power is distributed to the **servo motors**.

<p align="center">
  <img src="diagrams/receiver/servo-power-supply.png.png" width="650">
</p>

> **Important:** The servo power system should be wired carefully and checked before connecting the complete servo load.

---

## ✋ Transmitter Diagram

### Flex Sensor Hand Controller

This diagram shows the **ESP8266 hand controller and flex-sensor circuit**.

<p align="center">
  <img src="diagrams/transmitter/flex-sensor-transmitter.png.png" width="650">
</p>

---

# 💻 Software

The project contains two main software components.

---

## 01 — Humanoid Robot Receiver

The receiver software runs on the **ESP32** and controls the robot's servo motors and other connected electronics.

The receiver is the main control system of the humanoid robot.

**Source code:**

👉 [Open ESP32 Receiver Code](17-Servo-Humanoid-Robot/ESP32_Humanoid_Robot_RECEIVER)

---

## 02 — Hand Controller Transmitter

The transmitter software runs on the **ESP8266**.

It reads the flex-sensor input and sends control commands wirelessly to the robot.

**Source code:**

👉 [Open ESP8266 Transmitter Code](17-Servo-Humanoid-Robot/ESP8266_Hand_Controller_TRANSMITTER_Y4_BUTTON)

---

# 🛠️ Installation

## ESP32 Receiver

1. Install the Arduino IDE.
2. Install ESP32 board support.
3. Open the receiver `.ino` file.
4. Install the required libraries.
5. Select the correct ESP32 board.
6. Select the correct COM port.
7. Connect the ESP32.
8. Upload the program.
9. Open the Serial Monitor.
10. Verify that the receiver starts correctly.

---

## ESP8266 Transmitter

1. Install the Arduino IDE.
2. Install ESP8266 board support.
3. Open the transmitter `.ino` file.
4. Install the required libraries.
5. Select the correct ESP8266 board.
6. Select the correct COM port.
7. Connect the ESP8266.
8. Upload the program.
9. Open the Serial Monitor.
10. Verify wireless communication.

---

# ⚙️ Servo Configuration

The robot uses **17 servo motors** distributed across the humanoid structure.

Servo positions and movement ranges are defined in the receiver software.

Before operating the robot, make sure:

- Servo horns are installed in the correct position.
- Servo power is supplied separately from the controller when required.
- PCA9685 connections are correct.
- Servo limits are properly calibrated.
- The robot is mechanically stable.
- Wiring is secure.
- The power supply can handle the servo load.

### ⚠️ Before You Let It Move

A humanoid robot does not understand the concept of:

> "Let's just try it once."

So always test movements carefully before running a complete sequence.

---

# 🎯 Calibration

Servo calibration is important before performing full robot movements.

### Recommended Procedure

**01. Power**

Power the electronics and verify the system.

**02. Check**

Check the servo driver and controller connections.

**03. Test**

Test each servo individually.

**04. Neutral Position**

Adjust each servo to its intended neutral position.

**05. Mechanical Alignment**

Verify that the servo horns and mechanical joints are correctly aligned.

**06. Movement Limits**

Set safe movement limits.

**07. Individual Movement**

Test individual robot movements.

**08. Full Movement**

Only after everything looks correct, test complete movement sequences.

> ⚠️ **Never force a servo against its mechanical limit.**

---

# 🦾 Robot Gallery

A small collection of the robot during development, testing, electronics integration, and final assembly.

---

## Front View

<p align="center">
  <img src="images/robot-front.jpg.jpg" width="450">
</p>

<p align="center">
  <em>The humanoid structure from the front.</em>
</p>

---

## Final Robot

<p align="center">
  <img src="images/robot-final.jpg.jpg" width="450">
</p>

<p align="center">
  <em>The completed robot prototype.</em>
</p>

---

## Display / Face

<p align="center">
  <img src="images/robot-display.jpg.jpg" width="450">
</p>

<p align="center">
  <em>Robot head and display assembly.</em>
</p>

---

## Electronics

<p align="center">
  <img src="images/robot-electronics.jpg.jpg" width="450">
</p>

<p align="center">
  <em>Electronics, wiring, and internal hardware.</em>
</p>

---

## Testing

<p align="center">
  <img src="images/robot-testing.jpg.jpg" width="450">
</p>

<p align="center">
  <em>Testing the robot's mechanical and electronic systems.</em>
</p>

---

## Night Test

<p align="center">
  <img src="images/robot-night-test.jpg.jpg" width="450">
</p>

<p align="center">
  <em>A late-night test. Because apparently robots don't believe in bedtime.</em>
</p>

---

# 🧩 Mechanical Design

The humanoid robot uses **custom-designed mechanical parts**.

The mechanical structure was designed specifically for this project and assembled around the servo-driven joints and electronic system.

The complete robot design files are provided separately because the design archive is large.

---

## 📦 3D Design Files

The complete `Robo_design.zip` package is available through **GitHub Releases**.

### Download

> 🔗 **Release download link will be added here.**

The package contains the complete robot design files required for the mechanical side of the project.

---

# 🏗️ Build Process

The robot can be built in several stages.

---

## 01. Mechanical Assembly

Assemble the 3D-printed body, arms, legs, head, and other structural components.

---

## 02. Servo Installation

Install and align the servo motors according to their respective positions.

Correct mechanical alignment is important for both movement quality and servo safety.

---

## 03. Electronics

Connect:

- ESP32
- PCA9685
- OLED
- Amplifier
- Speaker
- Servo system
- Supporting electronics

Follow the provided receiver circuit diagrams.

---

## 04. Power System

Connect the servo power distribution system.

Make sure the power supply is suitable for the total servo load.

> ⚠️ **Do not assume the controller's power source can safely power all servos.**

---

## 05. Hand Controller

Build the ESP8266 hand controller and connect the flex sensor according to the transmitter circuit diagram.

---

## 06. Programming

Upload:

- ESP32 receiver firmware
- ESP8266 transmitter firmware

---

## 07. Calibration

Calibrate the servos and test individual movements.

---

## 08. Final Testing

Test wireless control and complete humanoid movements.

At this point, you should have a robot that is considerably more interesting than the average pile of electronics on a table.

---

# 📥 Downloads

## Robot Design Files

The complete 3D design package is available through **GitHub Releases**.

**Download:**

> 🔗 Release link will be added here.

---

## Source Code

### ESP32 Receiver

👉 [Open ESP32 Humanoid Robot Receiver](17-Servo-Humanoid-Robot/ESP32_Humanoid_Robot_RECEIVER)

### ESP8266 Transmitter

👉 [Open ESP8266 Hand Controller Transmitter](17-Servo-Humanoid-Robot/ESP8266_Hand_Controller_TRANSMITTER_Y4_BUTTON)

---

## Circuit Diagrams

All diagrams are available here:

- 👉 [Receiver Diagrams](diagrams/receiver)
- 👉 [Transmitter Diagram](diagrams/transmitter)

---

# 🎬 Videos

Project videos and demonstrations will be added here.

---

## 🤖 Robot Demonstration

> 🎥 YouTube link will be added here.

---

## 🔧 Build / Assembly

> 🎥 YouTube link will be added here.

---

## ✋ Hand Controller Demonstration

> 🎥 YouTube link will be added here.

---

# 📁 Project Structure

```text
17-Servo-Humanoid-Robot/
│
├── 17-Servo-Humanoid-Robot/
│   │
│   ├── ESP32_Humanoid_Robot_RECEIVER/
│   │   └── 2ESP32_Humanoid_Robot_RECEIVER/
│   │
│   └── ESP8266_Hand_Controller_TRANSMITTER_Y4_BUTTON/
│       └── ESP8266_Hand_Controller_TRANSMITTER_Y4_BUTTON/
│
├── diagrams/
│   │
│   ├── receiver/
│   │   ├── esp32-oled-amplifier.png.png
│   │   ├── servo-connections.png.png
│   │   └── servo-power-supply.png.png
│   │
│   └── transmitter/
│       └── flex-sensor-transmitter.png.png
│
├── images/
│   ├── robot-display.jpg.jpg
│   ├── robot-electronics.jpg.jpg
│   ├── robot-final.jpg.jpg
│   ├── robot-front.jpg.jpg
│   ├── robot-night-test.jpg.jpg
│   └── robot-testing.jpg.jpg
│
├── LICENSE
└── README.md
