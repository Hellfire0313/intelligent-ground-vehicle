<p align="center">
  <strong>SARATHI</strong><br/>
  <em>Autonomous Lane-Following Robot</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="License"/>
  <img src="https://img.shields.io/badge/platform-RPi_5_+_Arduino_Mega-orange.svg" alt="Platform"/>
  <img src="https://img.shields.io/badge/patent-pending-yellow.svg" alt="Patent"/>
  <img src="https://img.shields.io/badge/competition-IGVC_Robofest_5.0-red.svg" alt="Competition"/>
</p>

---

Built for [IGVC Robofest 5.0](https://robofest.gujarat.gov.in/SchemeDetails/15), Sarathi is a 60 cm wide autonomous robot that **sees the road, reads markers, and navigates junctions** in real time. A Raspberry Pi 5 runs the full vision pipeline — yellow-line detection, ArUco marker commands, and RealSense depth-based obstacle avoidance — and streams drive commands to an Arduino Mega that handles motor control, 4-wheel steering, and encoder RPM feedback.

> **Patent application filed** for the navigation and control methods described in this project.

---

## How It Works

```
┌────────────────────────────────────────────────────────────────┐
│                        SENSOR LAYER                            │
│                                                                │
│   ┌─────────────────────┐     ┌───────────────────────────┐   │
│   │     USB Webcam      │     │  Intel RealSense D455     │   │
│   │  Yellow-line +      │     │  Depth camera             │   │
│   │  ArUco detection    │     │  Obstacle avoidance       │   │
│   └──────────┬──────────┘     └────────────┬──────────────┘   │
│              │  OpenCV / pyrealsense2       │                  │
│   ┌──────────┴──────────────────────────────┴──────────────┐   │
│   │                   RASPBERRY PI 5                       │   │
│   │                                                        │   │
│   │   ┌─────────────┐  ┌───────────────┐  ┌────────────┐  │   │
│   │   │  Lane Vision│  │ ArUco Nav     │  │ Obstacle   │  │   │
│   │   │             │  │               │  │ SM         │  │   │
│   │   │  Tier 1     │  │  ID1 Turn L   │  │            │  │   │
│   │   │  Both walls │  │  ID2 Turn R   │  │  DODGE     │  │   │
│   │   │             │  │  ID3 Stop     │  │  ESCAPE    │  │   │
│   │   │  Tier 2     │  │  ID4 Resume   │  │  ESCAPE2   │  │   │
│   │   │  One wall   │  │               │  │  RECOVER   │  │   │
│   │   │             │  │  Auto-exit    │  │            │  │   │
│   │   │  Tier 3     │  │  detection    │  │            │  │   │
│   │   │  Lost/creep │  │               │  │            │  │   │
│   │   └──────┬──────┘  └───────┬───────┘  └─────┬──────┘  │   │
│   │          └─────────────────┴────────────────┘          │   │
│   │                    PD Controller                        │   │
│   │               speed + steer commands                   │   │
│   └──────────────────────┬─────────────────────────────────┘   │
│                          │ UART Serial (9600 baud)              │
│   ┌──────────────────────┴─────────────────────────────────┐   │
│   │                   ARDUINO MEGA                         │   │
│   │                                                        │   │
│   │   ┌─────────────────────┐   ┌─────────────────────┐   │   │
│   │   │  6× Cytron DC Motor │   │  4× Servo           │   │   │
│   │   │  PWM + DIR control  │   │  4-wheel steering   │   │   │
│   │   └─────────────────────┘   │  front + rear opp.  │   │   │
│   │                             └─────────────────────┘   │   │
│   │   ┌─────────────────────────────────────────────────┐  │   │
│   │   │  6× Quadrature Encoders → RPM feedback @ 200ms  │  │   │
│   │   └─────────────────────────────────────────────────┘  │   │
│   └────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────┘
```

### Lane Following — 3-Tier System

The RPi processes each webcam frame and picks the highest-confidence tier available.

| Tier | Condition | Strategy |
|------|-----------|----------|
| **Tier 1** | Both yellow walls visible | PD control on midpoint between walls |
| **Tier 2** | One wall visible | Estimates centre using learned lane width (EMA) |
| **Tier 3** | No walls | Hold last steer → creep straight → stop |

### ArUco Marker Navigation

The same webcam feed is scanned for `DICT_4X4_50` markers each frame. Detected commands are held until a matching wall gap is confirmed before the turn executes — preventing premature turns at false junctions.

Only 4 marker IDs are used in the current implementation. The dictionary supports up to 50 unique IDs — any of the remaining markers can be mapped to custom behaviours by extending `ARUCO_ID_MAP` in the config block.

| ID | Current action | Example extension |
|----|----------------|-------------------|
| 1 | Turn left | — |
| 2 | Turn right | — |
| 3 | Stop | — |
| 4 | Resume / suppress false exit | — |
| 5–50 | *(unused)* | Speed zones, checkpoints, U-turn, lap counter … |

### False-Exit Suppression

| Strategy | How |
|----------|-----|
| **Opposite-turn marker** | Place ID 2 at a false left exit (or ID 1 at a false right exit). Robot holds straight and waits for the correct opening. |
| **RESUME marker (ID 4)** | Place ID 4 facing the false gap. Seeing it opens a timed suppression window during which automatic exit turns are blocked. |

### Obstacle Avoidance

| Threshold | Distance | Behaviour |
|-----------|----------|-----------|
| `OBSTACLE_DETECT_M` | 0.90 m | Enter DODGE — steer around |
| `OBSTACLE_WARN_M` | 0.70 m | ESCAPE2 — gentle reverse + turn |
| `OBSTACLE_ESTOP_M` | 0.35 m | ESCAPE — hard reverse + turn |

After clearing, the robot blends back to lane following through a RECOVER state. If the RealSense is not connected, obstacle avoidance is silently disabled and lane following runs normally.

---

## Hardware

| Component | Part | Role |
|-----------|------|------|
| **SBC** | Raspberry Pi 5 | Vision pipeline, navigation, serial comms |
| **MCU** | Arduino Mega | Motor drive, steering, encoder feedback |
| **Lane camera** | USB webcam (640×480) | Yellow-line detection + ArUco scanning |
| **Depth camera** | Intel RealSense D455 | Obstacle detection and avoidance |
| **Drive** | 6× DC motors + Cytron driver | 6-wheel drive |
| **Steering** | 4× servos | 4-wheel steering, rear opposite to front |
| **Encoders** | 6× quadrature encoders | Per-motor RPM feedback |

### Arduino Pin Map

| Pin(s) | Function |
|--------|----------|
| 4 – 9 | Motor PWM (6 channels) |
| 30 – 35 | Motor DIR (6 channels) |
| 10 – 13 | Servo signal (S1–S4) |
| 2, 3, 18, 19, 20, 21 | Encoder A — interrupt pins |
| 22 – 27 | Encoder B — direction pins |

---

## Serial Protocol

**RPi → Arduino** (every control cycle):
```
<speed> <steer>\n
```
- `speed`: −255 to 255 (negative = reverse)
- `steer`: −100 to 100 (negative = left, positive = right)

**Arduino → RPi** (every 200 ms):
```
ENC <rpm0> <rpm1> <rpm2> <rpm3> <rpm4> <rpm5>
```
Motor order: RearRight, MidRight, FrontRight, RearLeft, MidLeft, FrontLeft.

---

## Patent

| | |
|---|---|
| **Status** | Application filed |
| **Institution** | Birla Vishvakarma Mahavidyalaya Engineering College, Gujarat |

A patent application has been filed for the navigation and control methods described in this project. Unauthorized reproduction, distribution, or commercial use of this work may constitute patent infringement.

---

## Quick Start

```bash
# 1. Clone
git clone https://github.com/Hellfire0313/intelligent-ground-vehicle.git
cd intelligent-ground-vehicle

# 2. Install Python dependencies (on RPi)
pip install opencv-python numpy pyserial
pip install pyrealsense2   # optional — for obstacle avoidance

# 3. Flash Arduino
#    Open arduino/Robofest_5.ino in Arduino IDE
#    Select board: Arduino Mega 2560 → Upload

# 4. Run (autonomous)
python python/Robofest5.py

# 4b. Run (manual PS4 controller)
python python/ps4_robot_controller.py
```

**Controls (GUI window must be focused):**

| Key | Action |
|-----|--------|
| `L` | Start AUTO mode |
| `M` | MANUAL mode (WASD) |
| `Q` | Quit |
| `W/A/S/D` | Manual drive |

The **Controls window** (opens on launch) provides live trackbars for proximity-bar offset, reverse/forward speed, steer, and duration — no code edits needed during testing.

---

## Project Structure

```
sarathi-robot/
├── arduino/
│   └── Robofest_5.ino                # Arduino Mega — motor drive, steering, encoder RPM reporting
├── python/
│   ├── Robofest5.py                  # RPi controller — lane following, ArUco, obstacle avoidance
│   └── ps4_robot_controller.py       # PS4 controller — manual drive over Bluetooth
├── LICENSE
└── README.md
```

---

## Dependencies

| Library | Purpose |
|---------|---------|
| [`opencv-python`](https://pypi.org/project/opencv-python/) | Vision pipeline, ArUco detection, GUI |
| [`numpy`](https://numpy.org/) | Array math and image processing |
| [`pyserial`](https://pypi.org/project/pyserial/) | UART communication with Arduino |
| [`pyrealsense2`](https://pypi.org/project/pyrealsense2/) | RealSense D455 depth frames *(optional)* |
| [Arduino `Servo.h`](https://www.arduino.cc/reference/en/libraries/servo/) | Servo control (built-in) |

---

## License

MIT — see `LICENSE` for details.
