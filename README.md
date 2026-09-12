# 17-Servo Humanoid Robot

A DIY 17-servo humanoid robot built using ESP32, ESP8266, servo motors, flex sensors, and custom 3D-printed mechanical parts.

**Mechanical Design • Electronics • Programming • Wireless Control**

---

## Project Overview

The 17-Servo Humanoid Robot is a DIY robotics project focused on humanoid movement, servo control, wireless communication, and embedded electronics.

The robot is controlled wirelessly using an ESP8266-based hand controller with flex sensors. The robot itself uses an ESP32 as the main receiver/controller.

This repository contains the robot software, transmitter software, circuit diagrams, robot images, and other project resources.

---

## Robot

![17-Servo Humanoid Robot](images/robot-final.jpg.jpg)

### Robot Gallery

#### Front View

![Robot Front](images/robot-front.jpg.jpg)

#### Electronics

![Robot Electronics](images/robot-electronics.jpg.jpg)

#### Testing

![Robot Testing](images/robot-testing.jpg.jpg)

#### Display

![Robot Display](images/robot-display.jpg.jpg)

#### Night Test

![Robot Night Test](images/robot-night-test.jpg.jpg)

---

## Main Features

- 17-servo humanoid robot
- ESP32-based robot receiver
- ESP8266-based wireless hand controller
- Flex-sensor-based control
- PCA9685 servo control
- Custom mechanical design
- Wireless communication between controller and robot
- Multiple humanoid movements
- Custom electronics and power distribution

---

# Hardware

## Robot Receiver

The robot uses an ESP32 as the main controller.

Main components include:

- ESP32
- PCA9685 servo driver
- Servo motors
- OLED display
- Amplifier/audio electronics
- Power supply system
- Wiring and connectors

## Hand Controller

The wireless hand controller uses an ESP8266 and flex sensors to control the robot.

Main components include:

- ESP8266
- Flex sensors
- Control electronics
- Power supply
- Wireless communication components

---

# Circuit Diagrams

All circuit diagrams are available in the `diagrams` folder.

## Receiver Circuit

### ESP32, OLED & Amplifier

![ESP32 OLED Amplifier](diagrams/receiver/esp32-oled-amplifier.png.png)

### Servo Connections

![Servo Connections](diagrams/receiver/servo-connections.png.png)

### Servo Power Supply

![Servo Power Supply](diagrams/receiver/servo-power-supply.png.png)

The servo connection diagram shows how the servo signal wires are connected to the PCA9685 servo driver.

The power-supply diagram shows the power distribution used for the servo motors.

---

## Transmitter Circuit

### Flex Sensor Transmitter

![Flex Sensor Transmitter](diagrams/transmitter/flex-sensor-transmitter.png.png)

The transmitter uses flex sensors to detect hand/finger movement and sends the corresponding control information wirelessly to the robot.

---

# Software

The project contains two main programs:

1. **ESP32 Humanoid Robot Receiver**
2. **ESP8266 Flex Sensor Transmitter**

---

## ESP32 Receiver Code

The ESP32 program controls the robot and receives wireless commands from the hand controller.

**Location:**

```text
17-Servo-Humanoid-Robot/
└── ESP32_Humanoid_Robot_RECEIVER/
    └── 2ESP32_Humanoid_Robot_RECEIVER/
        └── 2ESP32_Humanoid_Robot_RECEIVER.ino
