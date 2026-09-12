<h1 align="center">🤖 17-SERVO HUMANOID ROBOT</h1>

<p align="center">
  <strong>DIY Humanoid Robotics • 17 Servos • ESP32 • ESP8266</strong>
</p>

<p align="center">
  <em>Where metal, code, and a questionable amount of wiring decided to become a robot.</em>
</p>

<p align="center">
  <img src="images/robot-final.jpg.jpg" width="420">
</p>

<p align="center">
  <img src="https://img.shields.io/badge/17-Servo%20Humanoid-0969DA?style=for-the-badge">
  <img src="https://img.shields.io/badge/ESP32-Receiver-E34F26?style=for-the-badge">
  <img src="https://img.shields.io/badge/ESP8266-Transmitter-2EA44F?style=for-the-badge">
  <img src="https://img.shields.io/badge/Status-Working-8957E5?style=for-the-badge">
</p>

<p align="center">
  <a href="#-overview">Overview</a> •
  <a href="#-features">Features</a> •
  <a href="#-specifications">Specifications</a> •
  <a href="#-hardware">Hardware</a> •
  <a href="#-circuit-diagrams">Circuits</a> •
  <a href="#-software">Software</a> •
  <a href="#-build-process">Build</a> •
  <a href="#-robot-gallery">Gallery</a> •
  <a href="#-downloads">Downloads</a>
</p>

---

<h2 id="-overview">🚀 Overview</h2>

The <strong>17-Servo Humanoid Robot</strong> is a DIY humanoid robotics project built around an ESP32 robot controller and an ESP8266 wireless hand controller.

The controller uses a flex sensor to translate hand movement into commands. Those commands travel wirelessly to the robot, where the ESP32 takes over the difficult part:

<strong>making 17 servos cooperate instead of starting a tiny mechanical rebellion.</strong>

The project combines:

- 🤖 Humanoid mechanical design
- ⚙️ 17 servo motors
- 🧠 ESP32 control
- 📡 Wireless communication
- 🖐️ Flex-sensor input
- 📟 OLED display
- 🔊 Audio electronics
- 🖨️ Custom 3D-printed parts
- 💻 Arduino-based firmware

This repository contains the software, circuit diagrams, photographs, build information, and project resources.

---

<h2 id="-features">✨ Features</h2>

<table align="center">
<tr>
<td align="center"><h3>🤖</h3><strong>Humanoid</strong></td>
<td align="center"><h3>⚙️</h3><strong>17 Servos</strong></td>
<td align="center"><h3>📡</h3><strong>Wireless</strong></td>
<td align="center"><h3>🖐️</h3><strong>Flex Sensor</strong></td>
</tr>
<tr>
<td align="center"><h3>🧠</h3><strong>ESP32</strong></td>
<td align="center"><h3>🎮</h3><strong>ESP8266</strong></td>
<td align="center"><h3>📟</h3><strong>OLED</strong></td>
<td align="center"><h3>🔊</h3><strong>Audio</strong></td>
</tr>
</table>

---

<h2 id="-specifications">📋 Specifications</h2>

| Specification | Details |
|---|---|
| 🤖 Robot Type | Humanoid |
| ⚙️ Servo Motors | 17 |
| 🧠 Main Controller | ESP32 |
| 🎮 Hand Controller | ESP8266 |
| 🖐️ Sensor | Flex Sensor |
| 📟 Display | OLED |
| 🔊 Audio | Amplifier + Speaker |
| ⚡ Servo Driver | PCA9685 |
| 📡 Communication | Wireless |
| 🖨️ Structure | Custom 3D Printed |

---

<h2 id="-hardware">🔩 Hardware</h2>

<h3>🤖 Robot</h3>

- ESP32 development board
- PCA9685 servo driver
- 17 servo motors
- OLED display
- Audio amplifier
- Speaker
- Servo power supply
- Wiring and connectors
- Custom 3D-printed mechanical structure

<h3>🎮 Hand Controller</h3>

- ESP8266 development board
- Flex sensor
- Supporting components
- Power supply
- Connecting wires

---

<h2 id="-circuit-diagrams">⚡ Circuit Diagrams</h2>

<p align="center">
  <em>Four diagrams. One robot. Zero excuses for mysterious wiring.</em>
</p>

<h3>🔵 Receiver • ESP32 + OLED + Amplifier</h3>

<p align="center">
  <img src="diagrams/receiver/esp32-oled-amplifier.png.png" width="560">
</p>

<h3>⚙️ Receiver • Servo Connections</h3>

<p align="center">
  <img src="diagrams/receiver/servo-connections.png.png" width="560">
</p>

<h3>🔋 Receiver • Servo Power Supply</h3>

<p align="center">
  <img src="diagrams/receiver/servo-power-supply.png.png" width="560">
</p>

<h3>🟢 Transmitter • Flex Sensor Controller</h3>

<p align="center">
  <img src="diagrams/transmitter/flex-sensor-transmitter.png.png" width="560">
</p>

---

<h2 id="-software">💻 Software</h2>

<h3>🔵 ESP32 Humanoid Robot Receiver</h3>

The ESP32 acts as the robot's main controller.

It handles the incoming wireless commands and controls the robot's servo system and connected electronics.

<p align="center">
  <a href="17-Servo-Humanoid-Robot/ESP32_Humanoid_Robot_RECEIVER">
    <strong>📂 OPEN ESP32 RECEIVER CODE →</strong>
  </a>
</p>

<h3>🟢 ESP8266 Hand Controller</h3>

The ESP8266 reads the flex-sensor input and sends the corresponding commands wirelessly to the robot.

<p align="center">
  <a href="17-Servo-Humanoid-Robot/ESP8266_Hand_Controller_TRANSMITTER_Y4_BUTTON">
    <strong>📂 OPEN ESP8266 TRANSMITTER CODE →</strong>
  </a>
</p>

---

<h2>🛠️ Installation</h2>

<h3>🔵 ESP32 Receiver</h3>

1. Install the Arduino IDE.
2. Install ESP32 board support.
3. Open the receiver `.ino` file.
4. Install the required libraries.
5. Select the correct ESP32 board.
6. Select the correct COM port.
7. Upload the firmware.
8. Open Serial Monitor.
9. Verify startup messages and communication.

<h3>🟢 ESP8266 Transmitter</h3>

1. Install the Arduino IDE.
2. Install ESP8266 board support.
3. Open the transmitter `.ino` file.
4. Install the required libraries.
5. Select the correct ESP8266 board.
6. Select the correct COM port.
7. Upload the firmware.
8. Open Serial Monitor.
9. Verify wireless communication.

<p align="center">
  <em>Then comes the exciting part: discovering which servo you forgot to connect.</em>
</p>

---

<h2>⚙️ Servo Configuration</h2>

The robot uses <strong>17 servo motors</strong> across its humanoid structure.

Before powering the complete system:

- Check servo wiring.
- Check PCA9685 connections.
- Verify servo power.
- Align servo horns correctly.
- Confirm neutral positions.
- Verify movement limits.
- Test individual servos first.

<blockquote>
⚠️ <strong>WARNING:</strong> Never force a servo against its mechanical limit.
</blockquote>

---

<h2>🎯 Calibration</h2>

Calibration should be performed before full robot operation.

```text
POWER ON
   │
   ▼
CHECK ELECTRONICS
   │
   ▼
CHECK PCA9685
   │
   ▼
TEST EACH SERVO
   │
   ▼
SET NEUTRAL POSITIONS
   │
   ▼
SET MOVEMENT LIMITS
   │
   ▼
TEST INDIVIDUAL MOVEMENTS
   │
   ▼
TEST COMPLETE ROBOT
