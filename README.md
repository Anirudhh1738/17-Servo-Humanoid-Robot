<div align="center">

# 🤖 17-SERVO HUMANOID ROBOT

### **A DIY Humanoid Robot Built From Servos, Sensors, Wires & Questionable Amounts of Patience.**

<p>
  <img src="images/robot-final.jpg.jpg" width="520">
</p>

<p>
  <strong>MECHANICAL DESIGN</strong> •
  <strong>ELECTRONICS</strong> •
  <strong>EMBEDDED SYSTEMS</strong> •
  <strong>WIRELESS CONTROL</strong>
</p>

<p>
  <a href="#-overview">Overview</a> •
  <a href="#-features">Features</a> •
  <a href="#-hardware">Hardware</a> •
  <a href="#-circuit-diagrams">Circuits</a> •
  <a href="#-software">Software</a> •
  <a href="#-gallery">Gallery</a> •
  <a href="#-build-process">Build</a>
</p>

<p>
  <img src="https://img.shields.io/badge/Robot-Humanoid-00A8FF?style=for-the-badge">
  <img src="https://img.shields.io/badge/Servos-17-7B61FF?style=for-the-badge">
  <img src="https://img.shields.io/badge/ESP32-Receiver-FF6B35?style=for-the-badge">
  <img src="https://img.shields.io/badge/ESP8266-Controller-00C853?style=for-the-badge">
</p>

<p>
  <img src="https://img.shields.io/badge/Status-Working%20Prototype-2EA44F?style=flat-square">
  <img src="https://img.shields.io/github/license/Anirudhh1738/17-Servo-Humanoid-Robot?style=flat-square">
</p>

</div>

---

## ⚡ Overview

What started as a collection of motors, wires, plastic parts and several
excellent opportunities to question life eventually became this:

**a 17-servo humanoid robot controlled wirelessly through a flex-sensor
hand controller.**

The robot uses an **ESP32** as its main controller and an **ESP8266**
as the wireless hand-controller platform.

The system combines:

- Servo-based humanoid movement
- Flex-sensor input
- Wireless communication
- ESP32 embedded control
- ESP8266 hand-controller input
- PCA9685 servo control
- OLED feedback
- Audio electronics
- Custom mechanical construction

This repository documents the project from the electronics and firmware
to the mechanical build and testing process.

> **Warning:** 17 servos may look peaceful in a photo.  
> Power them incorrectly and they suddenly become extremely confident.

---

# ✨ Features

<table>
<tr>
<td width="50%">

### 🦾 Humanoid Mechanics

- 17 servo motors
- Custom mechanical structure
- 3D-printed components
- Multiple articulated joints
- Custom feet and body structure

</td>

<td width="50%">

### 🧠 Control System

- ESP32 robot receiver
- ESP8266 hand controller
- Flex-sensor input
- Wireless communication
- Programmable movements

</td>
</tr>

<tr>
<td>

### 🔌 Electronics

- PCA9685 servo control
- Dedicated servo power distribution
- OLED display
- Audio amplifier
- Speaker system
- Custom wiring

</td>

<td>

### 🛠️ Documentation

- Source code
- Circuit diagrams
- Build process
- Calibration information
- Project photographs
- Demonstration videos

</td>
</tr>
</table>

---

# 🧩 How the System Works

The project is divided into two main electronic systems.

```text
                 ┌─────────────────────────┐
                 │   FLEX SENSOR GLOVE     │
                 │                         │
                 │       ESP8266           │
                 └────────────┬────────────┘
                              │
                              │ Wireless
                              ▼
                 ┌─────────────────────────┐
                 │      ESP32 RECEIVER     │
                 │                         │
                 │  Robot Control System   │
                 └────────────┬────────────┘
                              │
                              ▼
                 ┌─────────────────────────┐
                 │       PCA9685           │
                 │    SERVO CONTROLLER     │
                 └────────────┬────────────┘
                              │
                              ▼
                 ┌─────────────────────────┐
                 │      17 SERVO MOTORS    │
                 │                         │
                 │   HUMANOID MOVEMENT     │
                 └─────────────────────────┘
