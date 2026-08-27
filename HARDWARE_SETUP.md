# Smart Car Hardware Architecture & ESP32 Setup Guide

**Platform**: 4-Wheel Independent Swerve Drive with Dual Stepper Steering and DC Drive  
**Microcontroller**: ESP32 DevKit V1 (38-pin / 30-pin)  
**Host Computer**: Raspberry Pi 4 Model B (4GB/8GB) / Onboard Linux Compute  

---

## 1. Electrical System & Power Architecture

```
[2S/3S LiPo Battery: 7.4V - 11.1V, 2200-5000mAh]
   ├── [Main Power Switch & Fuse (15A)]
   ├── [Step-Down Buck Converter (5V 5A)] ──────────> Raspberry Pi 4 (USB-C / GPIO 5V)
   │                                     ──────────> ESP32 DevKit V1 (5V / VIN pin)
   ├── [12V Step-Up / Raw Battery Rail] ───────────> 4× DRV8825 Stepper Drivers (VMOT)
   └── [Raw Battery Rail] ──────────────────────────> 2× TB6612FNG Dual DC Motor Drivers (VM)
```

---

## 2. Microcontroller Pinout (ESP32 DevKit V1)

### A. 4× Stepper Motors (Steering Pods) via DRV8825
| Module / Corner | DRV8825 STEP Pin | DRV8825 DIR Pin | Hardware Timer / Engine | Microstepping Setting |
|---|:---:|:---:|---|:---:|
| **Front-Left (FL)** | `GPIO 26` | `GPIO 27` | FastAccelStepper Engine | Full Step / 1/16 |
| **Front-Right (FR)** | `GPIO 14` | `GPIO 12` | FastAccelStepper Engine | Full Step / 1/16 |
| **Rear-Left (BL/RL)** | `GPIO 32` | `GPIO 33` | FastAccelStepper Engine | Full Step / 1/16 |
| **Rear-Right (BR/RR)** | `GPIO 25` | `GPIO 13` | FastAccelStepper Engine | Full Step / 1/16 |

### B. 4× DC Drive Motors via TB6612FNG Dual H-Bridges
| Module / Corner | Driver IN1 (PWM) | Driver IN2 (PWM) | PWM Frequency | PWM Resolution |
|---|:---:|:---:|:---:|:---:|
| **Front-Left (FL)** | `GPIO 15` (Ch 0) | `GPIO 2` (Ch 1) | 5.0 kHz | 8-bit (0 – 255) |
| **Front-Right (FR)** | `GPIO 0` (Ch 2) | `GPIO 4` (Ch 3) | 5.0 kHz | 8-bit (0 – 255) |
| **Rear-Left (BL/RL)** | `GPIO 16` (Ch 4) | `GPIO 17` (Ch 5) | 5.0 kHz | 8-bit (0 – 255) |
| **Rear-Right (BR/RR)** | `GPIO 5` (Ch 6) | `GPIO 18` (Ch 7) | 5.0 kHz | 8-bit (0 – 255) |

### C. 4× Incremental Optical / Hall Encoders (Drive Feedback)
| Module / Corner | Encoder Channel A | Encoder Channel B | Decoding Method |
|---|:---:|:---:|---|
| **Front-Left (FL)** | `GPIO 36` (VP) | `GPIO 39` (VN) | Half-Quad Hardware PCNT |
| **Front-Right (FR)** | `GPIO 34` | `GPIO 35` | Half-Quad Hardware PCNT |
| **Rear-Left (BL/RL)** | `GPIO 19` | `GPIO 21` | Half-Quad Hardware PCNT |
| **Rear-Right (BR/RR)** | `GPIO 22` | `GPIO 23` | Half-Quad Hardware PCNT |

---

## 3. Host-to-ESP32 Serial Communication Protocol

* **Physical Interface**: USB-to-UART bridge (CP2102 / CH340) via `/dev/ttyUSB0` or `/dev/ttyACM0`.
* **Baud Rate**: `115200` baud, 8 data bits, no parity, 1 stop bit (8-N-1).
* **Command Syntax**:
  ```text
  SWERVE:angle_fr,speed_fr,angle_fl,speed_fl,angle_rl,speed_rl,angle_rr,speed_rr\n
  ```
  * `angle_*`: Desired steering angle in degrees $[0.0, 360.0)^\circ$.
  * `speed_*`: Drive motor PWM command $[-255, 255]$.
* **Acknowledgement**:
  * ESP32 returns `OK\n` upon successful command receipt and parser execution.

---

## 4. Flashing ESP32 Firmware via PlatformIO

```bash
# Navigate to firmware directory
cd /home/diya/smart_car/esp32_firmware

# Compile and upload to connected ESP32
pio run --target upload

# Open serial monitor to verify boot logs
pio device monitor --baud 115200
```
