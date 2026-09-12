<h1 align="center">🤖 17-SERVO HUMANOID ROBOT</h1>

<p align="center">
  <strong>A DIY 17-Servo Humanoid Robotics Project</strong>
</p>

<p align="center">
  Mechanical Design • Electronics • Embedded Programming • Wireless Control
</p>

<p align="center">
  <img src="images/robot-final.jpg.jpg" width="450">
</p>

<p align="center">
  <img src="https://img.shields.io/badge/17-Servo%20Humanoid-blue?style=for-the-badge">
  <img src="https://img.shields.io/badge/ESP32-Receiver-red?style=for-the-badge">
  <img src="https://img.shields.io/badge/ESP8266-Transmitter-green?style=for-the-badge">
  <img src="https://img.shields.io/badge/Status-Working-success?style=for-the-badge">
</p>

<p align="center">
  <a href="#overview">Overview</a> •
  <a href="#features">Features</a> •
  <a href="#specifications">Specifications</a> •
  <a href="#hardware">Hardware</a> •
  <a href="#circuit-diagrams">Diagrams</a> •
  <a href="#software">Software</a> •
  <a href="#calibration">Calibration</a> •
  <a href="#robot-gallery">Gallery</a> •
  <a href="#downloads">Downloads</a>
</p>

---

<h2 id="overview">🚀 Overview</h2>

The <strong>17-Servo Humanoid Robot</strong> is a DIY humanoid robotics project developed to explore humanoid movement, servo control, embedded electronics, wireless communication, flex-sensor control, and programmable robotics.

The robot uses an <strong>ESP32-based receiver</strong> to control its servo motors and connected electronics.

A separate <strong>ESP8266-based hand controller</strong> uses a flex sensor to provide wireless control commands to the robot.

This repository contains the project's source code, circuit diagrams, robot photographs, mechanical design resources, and documentation.

---

<h2 id="features">✨ Features</h2>

<table align="center">
<tr>
<td align="center">🤖<br><strong>Humanoid Robot</strong></td>
<td align="center">⚙️<br><strong>17 Servos</strong></td>
<td align="center">📡<br><strong>Wireless Control</strong></td>
<td align="center">🖐️<br><strong>Flex Sensor</strong></td>
</tr>
<tr>
<td align="center">🧠<br><strong>ESP32</strong></td>
<td align="center">📟<br><strong>OLED Display</strong></td>
<td align="center">🔊<br><strong>Audio System</strong></td>
<td align="center">🖨️<br><strong>3D Printed</strong></td>
</tr>
</table>

---

<h2 id="specifications">📋 Specifications</h2>

| Specification | Details |
|---|---|
| 🤖 Robot Type | Humanoid |
| ⚙️ Servo Motors | 17 |
| 🧠 Robot Controller | ESP32 |
| 🎮 Hand Controller | ESP8266 |
| 🖐️ Sensor Input | Flex Sensor |
| 📟 Display | OLED |
| 🔊 Audio | Amplifier + Speaker |
| ⚡ Servo Driver | PCA9685 |
| 📡 Communication | Wireless |
| 🖨️ Mechanical Structure | Custom 3D Printed |

---

<h2 id="hardware">🔩 Hardware</h2>

<h3>🤖 Robot Hardware</h3>

- ESP32 development board
- PCA9685 servo driver
- 17 servo motors
- OLED display
- Audio amplifier
- Speaker
- Servo power supply
- Wiring and connectors
- Custom 3D-printed mechanical parts

<h3>🎮 Hand Controller Hardware</h3>

- ESP8266 development board
- Flex sensor
- Supporting components
- Power supply
- Connecting wires

---

<h2 id="circuit-diagrams">⚡ Circuit Diagrams</h2>

All circuit diagrams are organized inside the <code>diagrams/</code> directory.

<h3>🔵 Receiver</h3>

<h4>ESP32 + OLED + Amplifier</h4>

<p align="center">
  <img src="diagrams/receiver/esp32-oled-amplifier.png.png" width="700">
</p>

<h4>Servo Connections</h4>

<p align="center">
  <img src="diagrams/receiver/servo-connections.png.png" width="700">
</p>

<h4>Servo Power Supply</h4>

<p align="center">
  <img src="diagrams/receiver/servo-power-supply.png.png" width="700">
</p>

<h3>🟢 Transmitter</h3>

<h4>Flex Sensor Hand Controller</h4>

<p align="center">
  <img src="diagrams/transmitter/flex-sensor-transmitter.png.png" width="700">
</p>

---

<h2 id="software">💻 Software</h2>

<h3>🔵 ESP32 Humanoid Robot Receiver</h3>

The ESP32 receiver controls the robot's servo motors and connected electronics.

<a href="17-Servo-Humanoid-Robot/ESP32_Humanoid_Robot_RECEIVER">
<strong>📂 Open ESP32 Receiver Code →</strong>
</a>

<h3>🟢 ESP8266 Hand Controller Transmitter</h3>

The ESP8266 transmitter reads the flex sensor and sends movement commands wirelessly to the robot.

<a href="17-Servo-Humanoid-Robot/ESP8266_Hand_Controller_TRANSMITTER_Y4_BUTTON">
<strong>📂 Open ESP8266 Transmitter Code →</strong>
</a>

---

<h2>🛠️ Installation</h2>

<h3>🔵 ESP32 Receiver</h3>

1. Install the Arduino IDE.
2. Install ESP32 board support.
3. Open the ESP32 receiver <code>.ino</code> file.
4. Install the required libraries.
5. Select the correct ESP32 board.
6. Select the correct COM port.
7. Upload the program.
8. Open the Serial Monitor.
9. Verify the receiver startup and communication.

<h3>🟢 ESP8266 Transmitter</h3>

1. Install the Arduino IDE.
2. Install ESP8266 board support.
3. Open the ESP8266 transmitter <code>.ino</code> file.
4. Install the required libraries.
5. Select the correct ESP8266 board.
6. Select the correct COM port.
7. Upload the program.
8. Open the Serial Monitor.
9. Verify wireless communication.

---

<h2>⚙️ Servo Configuration</h2>

The robot uses <strong>17 servo motors</strong> distributed throughout its humanoid structure.

Before operating the robot:

- Check all servo wiring.
- Verify PCA9685 connections.
- Install servo horns correctly.
- Set neutral positions.
- Verify movement limits.
- Check servo power connections.
- Confirm mechanical alignment.

<blockquote>
⚠️ <strong>WARNING:</strong> Never force a servo against its mechanical limit.
</blockquote>

---

<h2 id="calibration">🎯 Calibration</h2>

Proper calibration is important for stable and safe robot movement.

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
