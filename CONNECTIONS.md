# ESP32-CAM Wiring Guide (L298N + MOSFET)

## ⚠️ CRITICAL CHANGE
**DO NOT use GPIO 16**. It causes the ESP32-CAM to crash (boot loop) because it conflicts with PSRAM.
We have moved the **Weapon/Shoot** function to **GPIO 4** (previously Light).

---

## 1. Motor Driver (L298N)
| ESP32 Pin | L298N Pin | Function |
| :--- | :--- | :--- |
| **GPIO 12** | **ENA** | Motor A Speed (PWM) |
| **GPIO 13** | **IN1** | Motor A Direction |
| **GPIO 15** | **IN2** | Motor A Direction |
| **GPIO 12** | **ENB** | Motor B Speed (PWM) |
| **GPIO 14** | **IN3** | Motor B Direction |
| **GPIO 2** | **IN4** | Motor B Direction |
| **GND** | **GND** | **Common Ground** (Must connect to ESP32 GND) |

> **Note:** Both motors share **GPIO 12** for speed control.

---

## 2. MOSFET Trigger (WEAPON)
| ESP32 Pin | MOSFET Module | Function |
| :--- | :--- | :--- |
| **GPIO 4** | **SIG / Gate** | Trigger Signal (High = Fire) |
| **GND** | **GND** | Common Ground |
| **5V / VCC** | **VCC** | Logic Power (if module requires it) |

> **Changed from GPIO 16 to GPIO 4 to prevent crash.**

---

## 3. Flash Light / Illumination
**DISABLED** in firmware to free up GPIO 4 for the weapon.
(GPIO 4 is the onboard flash LED).

---

## 4. Power
- **5V** to ESP32-CAM **5V** pin.
- **GND** to ESP32-CAM **GND**.
- Ensure **all grounds are connected**.
