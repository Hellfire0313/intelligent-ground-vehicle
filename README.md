# Sarathi — Autonomous Lane-Following Robot

> RPi 5 → Arduino Mega (Serial) | 60 cm wide | Ackermann steering | 6 DC motors (Cytron)

---

## Overview

`combined_robot_aruco.py` is the main control script for the Sarathi robot. It merges a full lane-following pipeline with ArUco marker–based navigation into a single file that runs on a Raspberry Pi 5 and communicates with an Arduino Mega over serial.

Key capabilities:

- **3-tier lane following** using yellow-line detection (Hough transforms, HSV masking)
- **ArUco marker navigation** for commanded turns and stops
- **Automatic exit detection** — turns toward a wall gap when one side opens up
- **False-exit suppression** via RESUME markers or opposite-turn markers
- **Obstacle avoidance** using an Intel RealSense D455 depth camera (optional)
- **Proximity bar** — configurable wall-touch detection with reverse/forward escape sequence
- **3-panel live GUI** with camera feed, yellow mask, and depth overlay

---

## Hardware

| Component | Details |
|-----------|---------|
| SBC | Raspberry Pi 5 |
| Microcontroller | Arduino Mega (Serial @ 9600 baud) |
| Lane camera | USB webcam (index 0, 640×480) |
| Depth camera | Intel RealSense D455 *(optional)* |
| Drive | 6× DC motors via Cytron driver (PWM pins 4–9, DIR pins 30–35) |
| Steering | 4-servo 4-wheel steering — front + rear opposite (servo pins 10–13) |
| Encoders | 6× quadrature encoders (interrupt pins 2, 3, 18–21; direction pins 22–27) |

---

## Software Requirements

```bash
pip install opencv-python numpy pyserial
# Optional — for obstacle avoidance:
pip install pyrealsense2
```

Python 3.8+ is recommended.

---

## Running

```bash
python combined_robot_aruco.py
```

### Keyboard controls (GUI window must be focused)

| Key | Action |
|-----|--------|
| `L` | Start **AUTO** mode |
| `M` | **MANUAL** mode (WASD driving) |
| `Q` | Quit |
| `W/A/S/D` | Manual drive (forward / left / back / right) |

---

## ArUco Marker Map

Dictionary: `DICT_4X4_50`

Only 4 marker IDs are used in the current implementation, but the `DICT_4X4_50` dictionary supports up to 50 unique IDs — additional markers can be mapped to any custom behaviour by extending `ARUCO_ID_MAP` in the config section.

### Currently used

| Marker ID | Action |
|-----------|--------|
| 1 | Turn Left |
| 2 | Turn Right |
| 3 | Stop |
| 4 | Resume / suppress false-exit |

### Ideas for additional markers

| Marker ID | Suggested use |
|-----------|---------------|
| 5 | Speed up (increase `BASE_SPEED`) |
| 6 | Slow down / caution zone |
| 7 | U-turn |
| 8 | Checkpoint / lap counter |
| 9 | Switch to manual override |
| 10+ | Course-specific waypoints or task triggers |

To add a new marker, register it in `ARUCO_ID_MAP` and handle its action name in the AR command processing block inside `main()`.

Place markers facing the camera. Commands are held until the matching wall gap is detected (turn commands) or until a RESUME marker clears a stop.

---

## Serial Protocol

The RPi and Arduino communicate over UART at 9600 baud.

**RPi → Arduino** (each frame):
```
<speed> <steer>\n
```
- `speed`: −255 to 255 (negative = reverse)
- `steer`: −100 to 100 (negative = left, positive = right)

**Arduino → RPi** (every 200 ms):
```
ENC <rpm0> <rpm1> <rpm2> <rpm3> <rpm4> <rpm5>
```
RPM values are reported for all 6 motors in order: RearRight, MidRight, FrontRight, RearLeft, MidLeft, FrontLeft. The Arduino also handles quadrature encoder direction using the B-channel pins.

---

## Steering Notes

All 4 servos are driven independently with calibrated limits (adjustable at the top of the Arduino sketch). The rear axle steers in the **opposite direction** to the front — this reduces the turning radius significantly compared to front-only Ackermann steering.

---

The controller picks the highest-confidence tier available each frame.

**Tier 1 — Both walls visible**  
Uses the midpoint between left and right yellow lines as the target. A lookahead blend (`LOOKAHEAD_BLEND`) smooths the path ahead.

**Tier 2 — Single wall**  
Estimates the lane centre using a learned lane width (EMA-filtered from Tier 1 observations). Falls back to a default of 280 px until calibrated.

**Tier 3 — No walls**  
Holds last steering for `LOST_HOLD_FRAMES`, then creeps straight, then stops. If a horizontal line was previously seen, triggers a pivot-scan to re-acquire the lane.

---

## Obstacle Avoidance (RealSense)

Depth ROI covers the robot's forward corridor. Three thresholds:

| Threshold | Distance | Behaviour |
|-----------|----------|-----------|
| `OBSTACLE_DETECT_M` | 0.90 m | Enter DODGE state — steer around |
| `OBSTACLE_WARN_M` | 0.70 m | Trigger ESCAPE2 (gentle reverse + turn) |
| `OBSTACLE_ESTOP_M` | 0.35 m | Trigger ESCAPE (hard reverse + turn) |

After clearing the obstacle the robot enters a RECOVER blend back to lane following. If the RealSense is not connected, obstacle avoidance is silently disabled and lane following runs normally.

---

## False-Exit Suppression

Two strategies to prevent the robot from turning into wall gaps that are not real junctions:

**Strategy 1 — Opposite-turn marker**  
Place a `ID 2` (TURN RIGHT) marker at a false *left* exit, or `ID 1` (TURN LEFT) at a false *right* exit. The robot holds straight and waits for the correct exit.

**Strategy 2 — RESUME marker**  
Place an `ID 4` (RESUME) marker facing the false exit gap. Seeing it starts a suppression window (`RESUME_SUPPRESS_DURATION_S`, default 6 s) during which automatic exit turns are blocked on both sides.

---

## Tuning Reference

All constants are grouped at the top of the file. Commonly adjusted values:

```python
BASE_SPEED      = 55       # forward cruise speed (0–255)
EXIT_CONFIRM_FRAMES = 3    # raise to reduce false exit triggers
RESUME_SUPPRESS_DURATION_S = 6.0  # seconds to block auto-exit after ID 4

KP_CENTER = 0.15           # lane-centre proportional gain
KD_CENTER = 0.05           # lane-centre derivative gain
MAX_STEER = 55             # maximum steering angle (degrees)

OBSTACLE_DETECT_M = 0.90   # obstacle detection range
OBSTACLE_ESTOP_M  = 0.35   # emergency stop range
```

The **Controls window** (opens on launch) provides live trackbars for the proximity-bar offset, Y position, reverse/forward speed, steer, and duration — no code changes needed during testing.

---

## GUI Layout

```
┌─────────────────┬─────────────────┬─────────────────┐
│   Camera feed   │   Yellow mask   │   Depth map     │
│  (with overlays)│  (lane lines)   │  (RealSense)    │
├─────────────────┴─────────────────┴─────────────────┤
│  HUD: mode | state label | speed | steer | bar      │
└─────────────────────────────────────────────────────┘
```

HUD colour codes: green = AUTO lane, purple = AR/auto-turn active, dark red = AR-STOP, teal = false-exit suppression, blue = bar-touch sequence.

---

## File Structure

```
.
├── combined_robot_aruco.py   # RPi controller — lane following, ArUco, obstacle avoidance
└── arduino/
    └── motor_servo_encoder.ino   # Arduino Mega — motor drive, steering, encoder RPM reporting
```

---

## License

MIT — see `LICENSE` for details.
