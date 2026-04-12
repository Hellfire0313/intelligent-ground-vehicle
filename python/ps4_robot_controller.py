#!/usr/bin/env python3
"""
PS4 Controller → Arduino Robot Controller
==========================================
Left  Joystick Y-axis  →  Forward / Backward  (speed  -50 to +50 PWM)
Right Joystick X-axis  →  Left / Right         (steer -100 to +100)

Requires:  pip install pygame pyserial
Run with:  python3 ps4_robot_controller.py
"""

import pygame
import serial
import time
import sys

# ─────────────────────────────────────────────────────────────
# CONFIGURATION  ← edit these if needed
# ─────────────────────────────────────────────────────────────
SERIAL_PORT   = "/dev/ttyUSB0"   # change to /dev/ttyACM0 if needed
BAUD_RATE     = 9600

MAX_SPEED     = 50               # max PWM sent to Arduino
MAX_STEER     = 100              # max steering value sent to Arduino

DEADZONE      = 0.08             # joystick dead-zone (0.0 – 1.0)
SEND_RATE_HZ  = 20               # how often to send commands per second

# PS4 axis indices (pygame default mapping)
LEFT_Y_AXIS   = 1                # left  stick Y  (up = negative)
RIGHT_X_AXIS  = 3                # right stick X  (right = positive)


# ─────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────
def apply_deadzone(value: float, dz: float) -> float:
    """Return 0 inside dead-zone, rescale linearly outside."""
    if abs(value) < dz:
        return 0.0
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - dz) / (1.0 - dz)


def axis_to_speed(axis_val: float) -> int:
    """Left stick Y: up = positive speed, down = negative speed."""
    val = apply_deadzone(-axis_val, DEADZONE)   # invert: up is negative on most sticks
    return int(round(val * MAX_SPEED))


def axis_to_steer(axis_val: float) -> int:
    """Right stick X: right = positive steer."""
    val = apply_deadzone(axis_val, DEADZONE)
    return int(round(val * MAX_STEER))


def send_command(ser: serial.Serial, speed: int, steer: int):
    """Send  'speed steer\n'  to Arduino."""
    msg = f"{speed} {steer}\n"
    ser.write(msg.encode())


def read_arduino(ser: serial.Serial):
    """Print any RPM lines received from Arduino."""
    while ser.in_waiting:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(f"  [Arduino] {line}")


# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────
def main():
    # ── Serial ──────────────────────────────────────────────
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
        print(f"[OK] Serial opened on {SERIAL_PORT} at {BAUD_RATE} baud")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open serial port: {e}")
        print("  Try:  ls /dev/ttyUSB*  or  ls /dev/ttyACM*")
        sys.exit(1)

    time.sleep(2)   # wait for Arduino to reset after serial open

    # ── Pygame / Controller ──────────────────────────────────
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[ERROR] No joystick detected.")
        print("  • Connect PS4 controller via USB  or")
        print("  • Pair via Bluetooth then run:  ds4drv  (pip install ds4drv)")
        ser.close()
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[OK] Controller detected: {joy.get_name()}")
    print()
    print("  Left  stick  Y → forward / backward")
    print("  Right stick  X → left / right steering")
    print("  Press  Ctrl-C  to stop and centre the wheels\n")

    interval  = 1.0 / SEND_RATE_HZ
    last_send = time.monotonic()
    last_speed, last_steer = None, None   # track changes to avoid noise

    try:
        while True:
            # Process pygame events (required to update axis values)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            now = time.monotonic()
            if now - last_send >= interval:
                last_send = now

                speed = axis_to_speed(joy.get_axis(LEFT_Y_AXIS))
                steer = axis_to_steer(joy.get_axis(RIGHT_X_AXIS))

                # Only print/send when values change (reduces console spam)
                if speed != last_speed or steer != last_steer:
                    send_command(ser, speed, steer)
                    print(f"\r  Speed: {speed:+4d} PWM  |  Steer: {steer:+4d}    ",
                          end="", flush=True)
                    last_speed, last_steer = speed, steer
                else:
                    send_command(ser, speed, steer)   # keep sending same vals

                # Print any RPM feedback from Arduino
                read_arduino(ser)

            time.sleep(0.001)   # yield CPU

    except KeyboardInterrupt:
        print("\n\n[INFO] Stopping robot...")
        send_command(ser, 0, 0)   # stop motors, centre steering
        time.sleep(0.5)

    finally:
        ser.close()
        pygame.quit()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
