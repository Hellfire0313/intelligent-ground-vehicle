# -*- coding: utf-8 -*-

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
import threading
import sys

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("[WARN] pyrealsense2 not found — obstacle avoidance disabled.")


# ==============================================================================
#  CONFIGURATION
# ==============================================================================

BAUD_RATE    = 9600
CAMERA_INDEX = 0

BASE_SPEED      = 55
TURN_SPEED      = 42
LOST_SPEED      = 28
DODGE_SPEED_MAX = 50
DODGE_SPEED_MIN = 32
RECOVER_SPEED   = 45
ESCAPE_REV_SPD  = -38
ESCAPE_FWD_SPD  =  42

KP_CENTER = 0.15
KD_CENTER = 0.05
KP_WALL   = 0.20
KD_WALL   = 0.06
MAX_STEER = 55

WALL_GUARD_MARGIN_PX  = 18
WALL_GUARD_MAX_STEER  = 40

FRAME_WIDTH          = 640
FRAME_HEIGHT         = 480
WALL_MARGIN_RIGHT_PX = 331
WALL_MARGIN_LEFT_PX  = FRAME_WIDTH - WALL_MARGIN_RIGHT_PX

DEFAULT_LANE_WIDTH_PX = 280
LANE_WIDTH_EMA_ALPHA  = 0.12

YELLOW_HSV_LOW  = np.array([19,  50,  102], dtype=np.uint8)
YELLOW_HSV_HIGH = np.array([41, 255, 255], dtype=np.uint8)

ROI_TOP_FRAC      = 0.50
ROI_TOP_FRAC_WIDE = 0.20
HOUGH_THRESH       = 30
HOUGH_MIN_LINE_LEN = 40
HOUGH_MAX_LINE_GAP = 25
MIN_LINE_ANGLE_DEG = 15
LOST_HOLD_FRAMES   = 8
LOST_FRAME_THRESH  = 25
BOTH_LOST_TIMEOUT  = 3.0

HLINE_ANGLE_DEG   = 20
HLINE_MIN_LEN_PX  = 80
HLINE_ROI_FRAC    = 0.60
HLINE_SCAN_STEER  = 25
HLINE_SCAN_FRAMES = 40
HLINE_CRAWL_SPEED = 30

REALSENSE_W   = 640
REALSENSE_H   = 480
REALSENSE_FPS = 30
DEPTH_ROI_TOP = 0.25
DEPTH_ROI_BOT = 0.60

ROBOT_CORRIDOR_FRAC  = 0.70
DEPTH_MIN_VALID_M    = 0.10
OBSTACLE_DETECT_M    = 0.90
OBSTACLE_WARN_M      = 0.70
OBSTACLE_ESTOP_M     = 0.35
OBSTACLE_CLEAR_M     = 0.30
CONFIRM_FRAMES       = 4
CLEAR_CONFIRM_FRAMES = 4
SIDE_BLOCK_M         = 0.55
COOLDOWN_FRAMES      = 18

DODGE_STEER_MIN = 30
DODGE_STEER_MAX = 52

ESCAPE_STEER      = 38
ESCAPE_REVERSE_MS = 500
ESCAPE_FORWARD_MS = 600

ESCAPE2_STEER      = 33
ESCAPE2_REV_SPD    = -30
ESCAPE2_FWD_SPD    =  38
ESCAPE2_REVERSE_MS = 420
ESCAPE2_FORWARD_MS = 520

RECOVER_RETURN_STEER  = 22
RECOVER_RETURN_SPEED  = 30
RECOVER_BLEND_FRAMES  = 12
RECOVER_CENTRE_TOL_PX = 40

RIGHT_BIAS_DEPTH_M = 0.08

MANUAL_SPEED = 60
MANUAL_STEER = 35

MODE_IDLE   = "IDLE"
MODE_AUTO   = "AUTO"
MODE_MANUAL = "MANUAL"

MID_EMA_ALPHA   = 0.30
LOOKAHEAD_BLEND = 0.20

PATH_SCAN_ROWS     = [0.85, 0.60, 0.35]
PATH_ROW_WEIGHTS   = [3.0,  2.0,  1.0]
PATH_LOOKAHEAD_ROW = 0.20
PATH_EMA_ALPHA     = 0.35
PATH_MIN_CONF      = 0.30
PATH_BLEND_WALL    = True
PATH_LINE_COLOR    = (0, 255, 128)
PATH_PT_COLOR      = (255, 128, 0)
PATH_LA_COLOR      = (0, 255, 255)
PATH_LINE_THICK    = 2

BAR_OFFSET_DEFAULT      = 120
BAR_THICKNESS           = 10
CENTRE_LINE_THICK       = 1
BAR_COLOR               = (0, 220, 220)
CENTRE_LINE_COLOR       = (255, 255, 255)
BAR_REVERSE_SPEED       = -50
BAR_REVERSE_STEER       = 55
BAR_REVERSE_DURATION_MS = 500
BAR_CONFIRM_FRAMES      = 0.5
BAR_FORWARD_SPEED       = 42
BAR_FORWARD_STEER       = 40
BAR_FORWARD_DURATION_MS = 600

# ==============================================================================
#  AR CODE CONFIGURATION
# ==============================================================================

ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50

ARUCO_ID_MAP = {
    1: "TURN_LEFT",
    2: "TURN_RIGHT",
    3: "STOP",
    4: "RESUME",
}

ARUCO_STOP_TIMEOUT = 5.0
ARUCO_CMD_TIMEOUT  = 15.0
BANNER_DURATION    = 3.0

# EXIT TURN CONFIG — used for BOTH AR-commanded and automatic exit turns
EXIT_CONFIRM_FRAMES        = 3      # raise to reduce false positives at wall gaps
EXIT_TURN_STEER            = 50
EXIT_TURN_SPEED            = 38
EXIT_TURN_DURATION_MS      = 1800
RESUME_SUPPRESS_DURATION_S = 6.0   # auto-exit blocked for N seconds after RESUME seen

# ==============================================================================
#  GUI PANEL CONFIGURATION
# ==============================================================================

GUI_PANEL_W = 320
GUI_PANEL_H = 240
GUI_HUD_H   =  48

GUI_WALL_COLOR     = (0, 255, 255)
GUI_CENTRE_COLOR   = (255, 0, 200)
GUI_FRAME_CX_COLOR = (220, 220, 220)
GUI_ERROR_COLOR    = (0, 140, 255)


# ==============================================================================
#  SERIAL
# ==============================================================================

def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    keywords = ["arduino", "ch340", "ch341", "ftdi", "uno",
                "mega", "nano", "leonardo", "usb serial"]
    for p in ports:
        if any(kw in (p.description + p.hwid).lower() for kw in keywords):
            print("[INFO] Arduino on %s (%s)" % (p.device, p.description))
            return p.device
    for p in ports:
        if "ACM" in p.device or "USB" in p.device or "COM" in p.device:
            return p.device
    return ports[0].device if ports else None


class ArduinoSerial:
    def __init__(self, baud):
        port = find_arduino_port()
        if port is None:
            raise serial.SerialException("No serial port found.")
        self.ser   = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2)
        self._lock = threading.Lock()
        threading.Thread(target=self._reader, daemon=True).start()
        print("[INFO] Serial on %s @ %d baud." % (port, baud))

    def send(self, speed, steer):
        speed = int(np.clip(speed, -255, 255))
        steer = int(np.clip(steer,  -90,  90))
        with self._lock:
            self.ser.write(("%d %d\n" % (speed, steer)).encode())

    def _reader(self):
        while True:
            try:
                self.ser.readline()
            except Exception:
                pass

    def stop(self):
        self.send(0, 0)


# ==============================================================================
#  DEPTH CAMERA
# ==============================================================================

class DepthCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, REALSENSE_W, REALSENSE_H,
                          rs.format.z16, REALSENSE_FPS)
        profile        = self.pipeline.start(cfg)
        sensor         = profile.get_device().first_depth_sensor()
        self._scale    = sensor.get_depth_scale()
        self._spatial  = rs.spatial_filter()
        self._temporal = rs.temporal_filter()
        self._frame    = None
        self._lock     = threading.Lock()
        self._running  = True
        threading.Thread(target=self._loop, daemon=True).start()
        print("[INFO] RealSense D455 ready (scale=%.5f m/unit)." % self._scale)

    def _loop(self):
        while self._running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=300)
                df = frames.get_depth_frame()
                if not df:
                    continue
                df  = self._spatial.process(df)
                df  = self._temporal.process(df)
                img = np.asanyarray(df.get_data()) * self._scale
                with self._lock:
                    self._frame = img
            except Exception as e:
                print("[WARN] RealSense: %s" % e)

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self):
        self._running = False
        self.pipeline.stop()


# ==============================================================================
#  DEPTH ANALYSIS
# ==============================================================================

def _col_min_valid(roi):
    masked  = np.where(roi > DEPTH_MIN_VALID_M, roi.astype(np.float32), np.nan)
    col_p10 = np.nanpercentile(masked, 10, axis=0)
    return np.where(np.isnan(col_p10), np.inf, col_p10)


def analyse_depth(depth_img):
    h, w = depth_img.shape
    rt   = int(h * DEPTH_ROI_TOP)
    rb   = int(h * DEPTH_ROI_BOT)
    cx   = w // 2
    half = int(w * ROBOT_CORRIDOR_FRAC / 2)
    cl   = max(0,     cx - half)
    cr   = min(w - 1, cx + half)
    roi  = depth_img[rt:rb, :]

    col_p10   = _col_min_valid(roi)
    corr_cols = col_p10[cl:cr]
    min_dist  = float(np.min(corr_cols)) if corr_cols.size > 0 else np.inf
    estop     = min_dist < OBSTACLE_ESTOP_M
    warn      = (OBSTACLE_ESTOP_M <= min_dist < OBSTACLE_WARN_M)

    full_blocked = int(np.sum(col_p10 < OBSTACLE_DETECT_M))
    width_frac   = float(full_blocked) / w
    obstacle     = (OBSTACLE_ESTOP_M <= min_dist < OBSTACLE_DETECT_M)

    w_score    = float(np.clip(width_frac, 0.0, 1.0))
    dist_range = OBSTACLE_DETECT_M - OBSTACLE_ESTOP_M
    d_score    = float(np.clip(
        (OBSTACLE_DETECT_M - min_dist) / (dist_range + 1e-6), 0.0, 1.0))
    severity   = 0.50 * w_score + 0.50 * d_score

    left_cols  = col_p10[:cl] if cl > 0     else np.array([np.inf])
    right_cols = col_p10[cr:] if cr < w - 1 else np.array([np.inf])

    def _med(cols):
        v = cols[np.isfinite(cols)]
        return float(np.median(v)) if v.size > 0 else np.inf

    clear_left  = _med(left_cols)
    clear_right = _med(right_cols) + RIGHT_BIAS_DEPTH_M

    disp = (np.clip(depth_img / 4.0, 0, 1) * 255).astype(np.uint8)
    dbg  = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

    col = ((0,0,255) if estop else (0,100,255) if warn
           else (0,60,255) if obstacle else (0,220,60))
    cv2.rectangle(dbg, (cl, rt), (cr, rb), col, 2)
    label = ("ESTOP %.2fm" % min_dist if estop else
             "WARN  %.2fm -> ESC2" % min_dist if warn else
             "OBS %.2fm sev=%.2f" % (min_dist, severity) if obstacle else
             ("clear %.2fm" % min_dist if np.isfinite(min_dist) else "clear"))
    cv2.putText(dbg, label, (cl+4, rt+22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
    bar = int(width_frac * w)
    cv2.rectangle(dbg, (cx-bar//2, rb+4), (cx-bar//2+bar, rb+10), col, -1)
    if estop:
        cv2.putText(dbg, "!! ESTOP !!", (w//2-80, h//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)
    elif warn:
        cv2.putText(dbg, "!! WARNING !!", (w//2-90, h//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,100,255), 3)

    return dict(obstacle=obstacle, estop=estop, warn=warn,
                min_dist=min_dist, severity=severity,
                clear_left=clear_left, clear_right=clear_right,
                width_frac=width_frac, dbg=dbg)


# ==============================================================================
#  LANE VISION
# ==============================================================================

def get_yellow_mask(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(l)
    frame_eq = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    hsv  = cv2.cvtColor(frame_eq, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_HSV_LOW, YELLOW_HSV_HIGH)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   np.ones((5,5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3,3), np.uint8))
    return mask


def detect_lines(mask):
    h, _ = mask.shape
    edges = cv2.Canny(mask, 50, 150)
    segs  = cv2.HoughLinesP(edges, 1, np.pi/180, HOUGH_THRESH,
                             minLineLength=HOUGH_MIN_LINE_LEN,
                             maxLineGap=HOUGH_MAX_LINE_GAP)
    lines = []
    if segs is None:
        return lines
    for seg in segs:
        x1, y1, x2, y2 = seg[0]
        angle = abs(np.degrees(np.arctan2(abs(y2-y1), abs(x2-x1)+1e-9)))
        if angle < MIN_LINE_ANGLE_DEG:
            continue
        dy = y2 - y1
        if dy == 0:
            continue
        slope = (x2 - x1) / float(dy)
        x_bot = x1 + slope * (h - 1 - y1)
        lines.append(x_bot)
    return lines


def cluster_lines(lines, frame_width):
    cx    = frame_width / 2.0
    left  = [x for x in lines if x <  cx]
    right = [x for x in lines if x >= cx]
    lx = float(np.mean(left))  if left  else None
    rx = float(np.mean(right)) if right else None
    return lx, rx


def try_detect(frame, roi_top_frac):
    h   = frame.shape[0]
    top = int(h * roi_top_frac)
    roi = frame[top:, :]
    fw  = roi.shape[1]
    mask = get_yellow_mask(roi)
    lx, rx = cluster_lines(detect_lines(mask), fw)
    return lx, rx, roi, mask, fw


def detect_hline(mask):
    h, w = mask.shape
    top  = int(h * (1.0 - HLINE_ROI_FRAC))
    edges = cv2.Canny(mask[top:, :], 50, 150)
    segs  = cv2.HoughLinesP(edges, 1, np.pi/180, HOUGH_THRESH,
                             minLineLength=HLINE_MIN_LEN_PX,
                             maxLineGap=HOUGH_MAX_LINE_GAP)
    if segs is None:
        return False
    for seg in segs:
        x1, y1, x2, y2 = seg[0]
        dx = abs(x2-x1); dy = abs(y2-y1)
        if (abs(np.degrees(np.arctan2(dy, dx+1e-9))) < HLINE_ANGLE_DEG
                and np.hypot(dx, dy) >= HLINE_MIN_LEN_PX):
            return True
    return False


# ==============================================================================
#  LANE WIDTH ESTIMATOR
# ==============================================================================

class LaneWidthEstimator:
    def __init__(self, default_px=DEFAULT_LANE_WIDTH_PX, alpha=LANE_WIDTH_EMA_ALPHA):
        self._alpha      = alpha
        self._width      = None
        self._default    = default_px
        self._calibrated = False

    def update(self, lx, rx):
        if lx is not None and rx is not None:
            raw = rx - lx
            if raw > 30:
                self._width = raw if self._width is None else (
                    self._alpha * raw + (1.0 - self._alpha) * self._width)
                self._calibrated = True

    @property
    def width(self):
        return self._width if self._width is not None else self._default

    @property
    def calibrated(self):
        return self._calibrated

    def centre_from_right(self, rx):
        return rx - self.width / 2.0

    def centre_from_left(self, lx):
        return lx + self.width / 2.0

    def reset(self):
        self._width = None; self._calibrated = False


# ==============================================================================
#  MIDPOINT EMA SMOOTHER
# ==============================================================================

class MidpointSmoother:
    def __init__(self, alpha=MID_EMA_ALPHA):
        self._alpha    = alpha
        self._smooth   = None
        self._had_both = False

    def update(self, lx, rx):
        if lx is not None and rx is not None:
            raw = (lx + rx) / 2.0
            if self._smooth is None or not self._had_both:
                self._smooth = raw
            else:
                self._smooth = (self._alpha * raw
                                + (1.0 - self._alpha) * self._smooth)
            self._had_both = True
            return self._smooth, True
        else:
            self._had_both = False
            return self._smooth, False

    def inject(self, estimated_centre):
        if estimated_centre is None:
            return
        if self._smooth is None:
            self._smooth = estimated_centre
        else:
            alpha_weak = self._alpha * 0.5
            self._smooth = (alpha_weak * estimated_centre
                            + (1.0 - alpha_weak) * self._smooth)

    def reset(self):
        self._smooth = None; self._had_both = False


# ==============================================================================
#  VIRTUAL PATH TRACKER
# ==============================================================================

def _row_centroid(yellow_mask, y_row, lx, rx, fw):
    row     = yellow_mask[y_row, :]
    x_left  = max(0,      int(lx) + 2) if lx is not None else 0
    x_right = min(fw - 1, int(rx) - 2) if rx is not None else fw - 1
    if x_right <= x_left + 4:
        return fw / 2.0, False
    segment = row[x_left:x_right]
    free    = (segment == 0).astype(np.float32)
    total   = float(np.sum(free))
    if total < 8:
        return fw / 2.0, False
    indices = np.arange(x_left, x_right, dtype=np.float32)
    cx = float(np.sum(free * indices)) / total
    return cx, True


class VirtualPathTracker:
    def __init__(self):
        self.smooth_x   = None
        self.confidence = 0.0

    def update(self, yellow_mask, lx, rx, fw, fh):
        scan_points = []
        xs, ys, ws  = [], [], []
        valid_count = 0

        for frac, wt in zip(PATH_SCAN_ROWS, PATH_ROW_WEIGHTS):
            y_row = int(np.clip(frac * fh, 0, fh - 1))
            cx, ok = _row_centroid(yellow_mask, y_row, lx, rx, fw)
            if ok:
                xs.append(cx); ys.append(float(y_row)); ws.append(wt)
                scan_points.append((int(cx), y_row))
                valid_count += 1
            else:
                fb_cx = self._fallback_cx(lx, rx, fw)
                if fb_cx is not None:
                    xs.append(fb_cx); ys.append(float(y_row)); ws.append(wt * 0.4)

        lookahead_x = None
        raw_target  = self._fallback_cx(lx, rx, fw)
        if len(xs) >= 2:
            raw_target, lookahead_x = self._fit_and_project(xs, ys, ws, fh)
        elif len(xs) == 1:
            raw_target = xs[0]
        if raw_target is None:
            raw_target = float(fw) / 2.0

        boundary_bonus  = (0.2 if lx is not None else 0.0) + \
                          (0.2 if rx is not None else 0.0)
        row_score       = valid_count / float(len(PATH_SCAN_ROWS))
        self.confidence = float(np.clip(row_score * 0.6 + boundary_bonus, 0.0, 1.0))

        if self.smooth_x is None:
            self.smooth_x = raw_target
        else:
            self.smooth_x = (PATH_EMA_ALPHA * raw_target
                             + (1.0 - PATH_EMA_ALPHA) * self.smooth_x)
        return self.smooth_x, self.confidence, scan_points, lookahead_x

    def reset(self):
        self.smooth_x = None; self.confidence = 0.0

    @staticmethod
    def _fallback_cx(lx, rx, fw):
        if lx is not None and rx is not None:
            return (lx + rx) / 2.0
        if rx is not None:
            return rx - (fw - WALL_MARGIN_LEFT_PX)
        if lx is not None:
            return lx + (fw - WALL_MARGIN_RIGHT_PX)
        return None

    @staticmethod
    def _fit_and_project(xs, ys, ws, fh):
        ws_arr = np.array(ws, dtype=np.float64)
        xs_arr = np.array(xs, dtype=np.float64)
        ys_arr = np.array(ys, dtype=np.float64)
        W = np.sum(ws_arr)
        if W < 1e-9:
            return xs_arr[0], None
        y_bar = np.sum(ws_arr * ys_arr) / W
        x_bar = np.sum(ws_arr * xs_arr) / W
        num   = np.sum(ws_arr * (ys_arr - y_bar) * (xs_arr - x_bar))
        denom = np.sum(ws_arr * (ys_arr - y_bar) ** 2)
        if abs(denom) < 1e-6:
            return x_bar, x_bar
        m = num / denom
        b = x_bar - m * y_bar
        la_y        = PATH_LOOKAHEAD_ROW * fh
        lookahead_x = float(m * la_y + b)
        return x_bar, lookahead_x


# ==============================================================================
#  VIRTUAL PATH OVERLAY
# ==============================================================================

def draw_virtual_path(frame, scan_points, lookahead_x, smooth_x, fh, fw,
                      confidence, tier1_active=False):
    cx_int = int(np.clip(smooth_x, 0, fw - 1))
    if tier1_active:
        cv2.line(frame, (cx_int, 0), (cx_int, fh), (255, 0, 200), 2)
    else:
        dash_on = True
        for y in range(0, fh, 10):
            if dash_on:
                cv2.line(frame, (cx_int, y), (cx_int, min(y+8, fh-1)),
                         PATH_LINE_COLOR, PATH_LINE_THICK)
            dash_on = not dash_on
    for (px, py) in scan_points:
        cv2.circle(frame, (px, py), 5, PATH_PT_COLOR, -1)
        cv2.line(frame, (cx_int, py), (px, py), PATH_PT_COLOR, 1)
    if lookahead_x is not None:
        la_x = int(np.clip(lookahead_x, 0, fw - 1))
        la_y = int(PATH_LOOKAHEAD_ROW * fh)
        cv2.drawMarker(frame, (la_x, la_y), PATH_LA_COLOR, cv2.MARKER_CROSS, 16, 2)
        cv2.putText(frame, "LA", (la_x+6, la_y-4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, PATH_LA_COLOR, 1)
    bar_w   = int(confidence * 80)
    bar_col = (0, int(255*confidence), int(255*(1-confidence)))
    cv2.rectangle(frame, (4, fh-10), (4+bar_w, fh-4), bar_col, -1)
    cv2.rectangle(frame, (4, fh-10), (84, fh-4), (80,80,80), 1)
    tier_lbl = "T1-CTR" if tier1_active else "T2/T3"
    cv2.putText(frame, "%s %.0f%%" % (tier_lbl, confidence*100), (4, fh-13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, bar_col, 1)


# ==============================================================================
#  PROXIMITY BAR OVERLAY + TOUCH DETECTION
# ==============================================================================

def draw_proximity_bars(frame, bar_offset, bar_y_frac, lx, rx, fw, fh,
                        left_touch, right_touch):
    cx = fw // 2
    cv2.line(frame, (cx, 0), (cx, fh), CENTRE_LINE_COLOR, CENTRE_LINE_THICK)
    bar_y     = int(fh * bar_y_frac)
    bar_y     = max(BAR_THICKNESS, min(fh - BAR_THICKNESS, bar_y))
    left_end  = max(0,      cx - bar_offset)
    right_end = min(fw - 1, cx + bar_offset)
    bar_color = (0, 0, 255) if (left_touch or right_touch) else BAR_COLOR
    cv2.line(frame, (left_end, bar_y), (right_end, bar_y), bar_color, BAR_THICKNESS)
    cv2.line(frame, (cx, bar_y-BAR_THICKNESS), (cx, bar_y+BAR_THICKNESS),
             CENTRE_LINE_COLOR, CENTRE_LINE_THICK)
    cv2.putText(frame, "Bar+-%dpx  Y:%d%%" % (bar_offset, int(bar_y_frac*100)),
                (cx-52, bar_y-BAR_THICKNESS-4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200,200,200), 1)
    for lx_mark in [lx, rx]:
        if lx_mark is not None:
            mx = int(lx_mark)
            cv2.line(frame, (mx, bar_y-10), (mx, bar_y+10), (180,255,0), 2)


def check_bar_touch(lx, rx, fw, bar_offset):
    cx          = fw // 2
    left_end    = cx - bar_offset
    right_end   = cx + bar_offset
    left_touch  = (lx is not None) and (left_end  <= lx)
    right_touch = (rx is not None) and (right_end >= rx)
    return left_touch, right_touch


# ==============================================================================
#  ARUCO DETECTOR
# ==============================================================================

class ArucoDetector:
    """
    Inline ArUco detection on the shared lane camera.
      detect_frame(frame): call each loop on the FULL frame.
        Draws detected markers in-place, updates id/timestamp.
      latest_id(max_age): returns (id, age_s) or (None, inf) if stale.
    Thread-safe.
    """
    def __init__(self):
        self._dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        self._params = cv2.aruco.DetectorParameters()
        self._id     = None
        self._seen_t = 0.0
        self._lock   = threading.Lock()
        print("[AR] ArucoDetector ready (dict=DICT_4X4_50)")

    def detect_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._dict, parameters=self._params)
        if ids is not None:
            mid = int(ids[0][0])
            with self._lock:
                self._id     = mid
                self._seen_t = time.time()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return frame

    def latest_id(self, max_age=1.0):
        with self._lock:
            if self._id is None:
                return None, float("inf")
            age = time.time() - self._seen_t
            if age > max_age:
                return None, age
            return self._id, age


# ==============================================================================
#  EXIT DETECTOR
# ==============================================================================

class ExitDetector:
    """
    Tracks consecutive frames where one lane wall is absent while the
    other is present — indicates an exit/branch on that side.

    Used for BOTH AR-commanded turns (confirmation) and automatic
    no-AR exit detection. Adjust EXIT_CONFIRM_FRAMES to tune sensitivity.
    """
    def __init__(self):
        self.left_count  = 0
        self.right_count = 0

    def update(self, lx, rx):
        lw = lx is not None
        rw = rx is not None
        if lw and rw:
            # Both walls present — decay counters
            self.left_count  = max(0, self.left_count  - 1)
            self.right_count = max(0, self.right_count - 1)
        elif not lw and rw:
            # Only right wall visible — left exit opening up
            self.left_count  += 1
            self.right_count  = 0
        elif lw and not rw:
            # Only left wall visible — right exit opening up
            self.right_count += 1
            self.left_count   = 0
        else:
            # Both walls gone — decay (TIER 3 lost, not an exit)
            self.left_count  = max(0, self.left_count  - 1)
            self.right_count = max(0, self.right_count - 1)

        left_ready  = self.left_count  >= EXIT_CONFIRM_FRAMES
        right_ready = self.right_count >= EXIT_CONFIRM_FRAMES
        return left_ready, right_ready

    def reset(self):
        self.left_count = 0; self.right_count = 0

    def draw_overlay(self, frame, fw, fh):
        lf = min(1.0, self.left_count / max(EXIT_CONFIRM_FRAMES, 1))
        lc = (0, int(255*lf), int(255*(1-lf)))
        cv2.rectangle(frame, (4, 4), (4+int(50*lf), 12), lc, -1)
        cv2.rectangle(frame, (4, 4), (54, 12), (80,80,80), 1)
        cv2.putText(frame, "Lexit", (4, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.28, lc, 1)
        rf = min(1.0, self.right_count / max(EXIT_CONFIRM_FRAMES, 1))
        rc = (0, int(255*rf), int(255*(1-rf)))
        cv2.rectangle(frame, (fw-54, 4), (fw-4, 12), (80,80,80), 1)
        cv2.rectangle(frame, (fw-4-int(50*rf), 4), (fw-4, 12), rc, -1)
        cv2.putText(frame, "Rexit", (fw-54, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.28, rc, 1)


# ==============================================================================
#  CONTROLS WINDOW
# ==============================================================================

CONTROLS_WIN = "Controls  (Bar Offset)"

def create_controls_window():
    cv2.namedWindow(CONTROLS_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS_WIN, 480, 340)
    cv2.createTrackbar("Bar Half-Length (px)", CONTROLS_WIN,
                       BAR_OFFSET_DEFAULT, FRAME_WIDTH//2, lambda v: None)
    cv2.createTrackbar("Bar Y Position (%)", CONTROLS_WIN,
                       75, 100, lambda v: None)
    cv2.createTrackbar("Bar Touch Logic  0=OFF  1=ON", CONTROLS_WIN,
                       0, 1, lambda v: None)
    cv2.createTrackbar("REV Speed (1-100)", CONTROLS_WIN,
                       int(abs(BAR_REVERSE_SPEED)), 100, lambda v: None)
    cv2.createTrackbar("REV Steer (deg)", CONTROLS_WIN,
                       int(BAR_REVERSE_STEER), 90, lambda v: None)
    cv2.createTrackbar("REV Duration (ms)", CONTROLS_WIN,
                       int(BAR_REVERSE_DURATION_MS), 3000, lambda v: None)
    cv2.createTrackbar("FWD Speed (1-100)", CONTROLS_WIN,
                       int(BAR_FORWARD_SPEED), 100, lambda v: None)
    cv2.createTrackbar("FWD Steer (deg)", CONTROLS_WIN,
                       int(BAR_FORWARD_STEER), 90, lambda v: None)
    cv2.createTrackbar("FWD Duration (ms)", CONTROLS_WIN,
                       int(BAR_FORWARD_DURATION_MS), 3000, lambda v: None)

def get_bar_offset():
    return max(10, cv2.getTrackbarPos("Bar Half-Length (px)", CONTROLS_WIN))
def get_bar_y_frac():
    return max(1, cv2.getTrackbarPos("Bar Y Position (%)", CONTROLS_WIN)) / 100.0
def get_bar_touch_enabled():
    return cv2.getTrackbarPos("Bar Touch Logic  0=OFF  1=ON", CONTROLS_WIN) == 1
def get_bar_rev_speed():
    return -max(1, cv2.getTrackbarPos("REV Speed (1-100)", CONTROLS_WIN))
def get_bar_rev_steer():
    return max(1, cv2.getTrackbarPos("REV Steer (deg)", CONTROLS_WIN))
def get_bar_rev_duration():
    return max(100, cv2.getTrackbarPos("REV Duration (ms)", CONTROLS_WIN))
def get_bar_fwd_speed():
    return max(1, cv2.getTrackbarPos("FWD Speed (1-100)", CONTROLS_WIN))
def get_bar_fwd_steer():
    return max(1, cv2.getTrackbarPos("FWD Steer (deg)", CONTROLS_WIN))
def get_bar_fwd_duration():
    return max(100, cv2.getTrackbarPos("FWD Duration (ms)", CONTROLS_WIN))


# ==============================================================================
#  PD CONTROLLER
# ==============================================================================

class PD:
    def __init__(self, kp, kd):
        self.kp, self.kd = kp, kd
        self._prev = 0.0

    def compute(self, error):
        d = error - self._prev
        self._prev = error
        return self.kp * error + self.kd * d

    def reset(self):
        self._prev = 0.0


# ==============================================================================
#  LANE WALL GUARD
# ==============================================================================

def apply_wall_guard(steer, lx, rx, fw):
    cx = fw / 2.0
    if rx is not None:
        room_right = rx - cx
        if room_right < WALL_GUARD_MARGIN_PX:
            max_right = max(0, int(WALL_GUARD_MAX_STEER *
                                   (room_right / max(WALL_GUARD_MARGIN_PX, 1))))
            steer = min(steer, max_right)
    if lx is not None:
        room_left = cx - lx
        if room_left < WALL_GUARD_MARGIN_PX:
            max_left = max(0, int(WALL_GUARD_MAX_STEER *
                                  (room_left / max(WALL_GUARD_MARGIN_PX, 1))))
            steer = max(steer, -max_left)
    return steer


# ==============================================================================
#  DODGE DIRECTOR
# ==============================================================================

def compute_dodge(depth_data, lx, rx, fw, locked_dir=None):
    cl  = depth_data["clear_left"]
    cr  = depth_data["clear_right"]
    sev = depth_data["severity"]

    left_blocked  = cl < SIDE_BLOCK_M
    right_blocked = cr < SIDE_BLOCK_M

    cx         = fw / 2.0
    lane_left  = max(0.0, cx - lx) if lx is not None else cx
    lane_right = max(0.0, rx - cx) if rx is not None else cx

    if left_blocked and not right_blocked:
        new_dir = +1
    elif right_blocked and not left_blocked:
        new_dir = -1
    elif left_blocked and right_blocked:
        new_dir = -1 if cl >= cr else +1
    else:
        total_depth = cl + cr
        d_left  = cl / total_depth if total_depth > 0 else 0.5
        d_right = cr / total_depth if total_depth > 0 else 0.5
        total_lane = lane_left + lane_right
        l_left  = lane_left  / total_lane if total_lane > 0 else 0.5
        l_right = lane_right / total_lane if total_lane > 0 else 0.5
        score_left  = 0.65 * d_left  + 0.35 * l_left
        score_right = 0.65 * d_right + 0.35 * l_right
        new_dir = -1 if score_left >= score_right else +1

    if locked_dir is not None:
        if locked_dir == -1 and not left_blocked:
            new_dir = -1
        elif locked_dir == +1 and not right_blocked:
            new_dir = +1

    steer_mag   = int(DODGE_STEER_MIN + sev * (DODGE_STEER_MAX - DODGE_STEER_MIN))
    dodge_steer = steer_mag * new_dir
    dodge_speed = int(DODGE_SPEED_MAX - sev * (DODGE_SPEED_MAX - DODGE_SPEED_MIN))
    return new_dir, dodge_steer, dodge_speed


# ==============================================================================
#  MAIN CONTROLLER STATE MACHINE
# ==============================================================================

class RobotSM:
    ST_LANE   = "LANE_FOLLOW"
    ST_DODGE  = "DODGE"
    ST_RECOV  = "RECOVER"
    ST_ESC_R  = "ESCAPE_REV"
    ST_ESC_F  = "ESCAPE_FWD"
    ST_ESC2_R = "ESCAPE2_REV"
    ST_ESC2_F = "ESCAPE2_FWD"

    def __init__(self):
        self.state      = self.ST_LANE
        self._confirm   = 0
        self._clear_cnt = 0
        self._cooldown  = 0
        self._dodge_dir = 0
        self._rec_phase = "A"
        self._rec_blend = 0
        self._esc_t0    = 0.0
        self._esc_dir   = +1

    def update(self, depth_data, lx, rx, fw, lane_steer, lane_speed):
        obs   = depth_data["obstacle"] if depth_data else False
        estop = depth_data["estop"]    if depth_data else False
        warn  = depth_data["warn"]     if depth_data else False

        if self._cooldown > 0 and self.state == self.ST_LANE:
            self._cooldown -= 1
            obs = False; estop = False; warn = False

        now = time.time()

        if estop and self.state not in (self.ST_ESC_R, self.ST_ESC_F,
                                        self.ST_ESC2_R, self.ST_ESC2_F):
            self._esc_dir = self._pick_escape_dir(depth_data)
            self.state    = self.ST_ESC_R
            self._esc_t0  = now
            self._confirm = 0
            print("[SM] ESTOP %.2fm -> ESCAPE dir=%s" % (
                depth_data["min_dist"], "L" if self._esc_dir < 0 else "R"))

        if self.state == self.ST_ESC_R:
            s, st, l = self._do_escape_rev(depth_data, now, lx, rx, fw)
            return s, st, l, False
        if self.state == self.ST_ESC_F:
            s, st, l = self._do_escape_fwd(depth_data, now, lx, rx, fw)
            return s, st, l, False
        if self.state == self.ST_ESC2_R:
            s, st, l = self._do_escape2_rev(depth_data, now, lx, rx, fw)
            return s, st, l, False
        if self.state == self.ST_ESC2_F:
            s, st, l = self._do_escape2_fwd(depth_data, now, lx, rx, fw)
            return s, st, l, False
        if self.state == self.ST_LANE:
            s, st, l = self._do_lane(obs, depth_data, lx, rx, fw,
                                     lane_steer, lane_speed)
            return s, st, l, False
        if self.state == self.ST_DODGE:
            s, st, l = self._do_dodge(obs, warn, depth_data, lx, rx, fw, now)
            return s, st, l, False
        if self.state == self.ST_RECOV:
            return self._do_recover(obs, depth_data, lx, rx, fw,
                                    lane_steer, lane_speed)
        return lane_speed, lane_steer, "LANE(default)", False

    def _do_lane(self, obs, depth_data, lx, rx, fw, lane_steer, lane_speed):
        if obs:
            self._confirm += 1
            if self._confirm >= CONFIRM_FRAMES:
                self._dodge_dir, _, _ = compute_dodge(depth_data, lx, rx, fw)
                self._confirm = 0; self._clear_cnt = 0
                self.state = self.ST_DODGE
                side = "L" if self._dodge_dir < 0 else "R"
                print("[SM] Obstacle -> DODGE %s  dist=%.2fm sev=%.2f" % (
                    side, depth_data["min_dist"], depth_data["severity"]))
                s, st, _ = self._do_dodge(obs, False, depth_data, lx, rx, fw,
                                          time.time())
                return s, st, "DODGE->%s" % side
        else:
            self._confirm = max(0, self._confirm - 1)
        return lane_speed, lane_steer, "LANE"

    def _do_dodge(self, obs, warn, depth_data, lx, rx, fw, now):
        min_dist = depth_data["min_dist"] if depth_data else np.inf
        if warn and depth_data:
            self._esc_dir = self._pick_escape_dir(depth_data)
            self.state    = self.ST_ESC2_R
            self._esc_t0  = now
            self._confirm = 0
            print("[SM] WARN %.2fm -> ESCAPE2 dir=%s" % (
                min_dist, "L" if self._esc_dir < 0 else "R"))
            return self._do_escape2_rev(depth_data, now, lx, rx, fw)

        if depth_data and min_dist >= OBSTACLE_CLEAR_M:
            self._clear_cnt += 1
            if self._clear_cnt >= CLEAR_CONFIRM_FRAMES:
                self._clear_cnt = 0
                self._rec_phase = "A"; self._rec_blend = 0
                self.state = self.ST_RECOV
                print("[SM] Corridor clear -> RECOVER")
        else:
            self._clear_cnt = 0

        if not depth_data:
            return BASE_SPEED, 0, "DODGE(no-depth)"

        new_dir, dodge_steer, dodge_speed = compute_dodge(
            depth_data, lx, rx, fw, locked_dir=self._dodge_dir)
        if new_dir != self._dodge_dir:
            self._dodge_dir = new_dir
        side = "L" if self._dodge_dir < 0 else "R"
        return (dodge_speed, dodge_steer,
                "DODGE->%s sev=%.2f st=%+d" % (
                    side, depth_data["severity"], dodge_steer))

    def _do_recover(self, obs, depth_data, lx, rx, fw, lane_steer, lane_speed):
        if obs and depth_data:
            self._confirm = CONFIRM_FRAMES; self._clear_cnt = 0
            self._rec_phase = "A"; self._rec_blend = 0
            self.state = self.ST_DODGE
            print("[SM] New obstacle in RECOVER -> DODGE")
            s, st, l = self._do_dodge(obs, depth_data.get("warn", False),
                                      depth_data, lx, rx, fw, time.time())
            return s, st, l, False

        return_dir   = -self._dodge_dir
        return_steer = int(RECOVER_RETURN_STEER * return_dir)
        side_label   = "L" if return_dir < 0 else "R"

        if self._rec_phase == "A":
            centred = False
            if lx is not None and rx is not None:
                mid_error = abs((lx + rx) / 2.0 - fw / 2.0)
                centred   = mid_error <= RECOVER_CENTRE_TOL_PX
            if centred:
                self._rec_phase = "B"; self._rec_blend = 0
                print("[SM] RECOVER-A done -> blend")
            else:
                return (RECOVER_RETURN_SPEED, return_steer,
                        "RECOVER-A ->%s" % side_label, True)

        t       = min(1.0, self._rec_blend / max(RECOVER_BLEND_FRAMES, 1))
        blended = int(return_steer + t * (lane_steer - return_steer))
        self._rec_blend += 1

        if t >= 1.0:
            self._cooldown = COOLDOWN_FRAMES
            self._rec_phase = "A"; self._rec_blend = 0
            self.state = self.ST_LANE; self._confirm = 0
            print("[SM] RECOVER done -> LANE_FOLLOW")
            return lane_speed, lane_steer, "LANE", False

        return (RECOVER_SPEED, blended,
                "RECOVER-B %.0f%%" % (t * 100), False)

    def _do_escape_rev(self, depth_data, now, lx, rx, fw):
        elapsed = now - self._esc_t0
        steer   = int(ESCAPE_STEER * -self._esc_dir)
        if elapsed * 1000 >= ESCAPE_REVERSE_MS:
            self.state = self.ST_ESC_F; self._esc_t0 = now
        return (ESCAPE_REV_SPD, steer,
                "ESCAPE-REV %.0f/%.0fms" % (elapsed*1000, ESCAPE_REVERSE_MS))

    def _do_escape_fwd(self, depth_data, now, lx, rx, fw):
        if depth_data and depth_data["estop"]:
            self.state = self.ST_ESC_R; self._esc_t0 = now
            return (ESCAPE_REV_SPD, int(ESCAPE_STEER * -self._esc_dir),
                    "ESCAPE-REV(retry)")
        elapsed = now - self._esc_t0
        steer   = int(ESCAPE_STEER * self._esc_dir)
        if elapsed * 1000 >= ESCAPE_FORWARD_MS:
            self._confirm = 0; self._clear_cnt = 0
            self.state = self.ST_DODGE
            print("[SM] Escape done -> DODGE")
        return (ESCAPE_FWD_SPD, steer,
                "ESCAPE-FWD %.0f/%.0fms" % (elapsed*1000, ESCAPE_FORWARD_MS))

    def _do_escape2_rev(self, depth_data, now, lx, rx, fw):
        if depth_data and depth_data["estop"]:
            self.state = self.ST_ESC_R; self._esc_t0 = now
            print("[SM] ESCAPE2 escalated -> ESCAPE")
            return (ESCAPE_REV_SPD, int(ESCAPE_STEER * -self._esc_dir),
                    "ESCAPE-REV(escalated)")
        elapsed = now - self._esc_t0
        steer   = int(ESCAPE2_STEER * -self._esc_dir)
        if elapsed * 1000 >= ESCAPE2_REVERSE_MS:
            self.state = self.ST_ESC2_F; self._esc_t0 = now
        return (ESCAPE2_REV_SPD, steer,
                "ESCAPE2-REV %.0f/%.0fms" % (elapsed*1000, ESCAPE2_REVERSE_MS))

    def _do_escape2_fwd(self, depth_data, now, lx, rx, fw):
        if depth_data and depth_data["estop"]:
            self.state = self.ST_ESC_R; self._esc_t0 = now
            print("[SM] ESCAPE2-FWD escalated -> ESCAPE")
            return (ESCAPE_REV_SPD, int(ESCAPE_STEER * -self._esc_dir),
                    "ESCAPE-REV(escalated)")
        elapsed = now - self._esc_t0
        steer   = int(ESCAPE2_STEER * self._esc_dir)
        if elapsed * 1000 >= ESCAPE2_FORWARD_MS:
            self._confirm = 0; self._clear_cnt = 0
            self.state = self.ST_DODGE
            print("[SM] ESCAPE2 done -> DODGE")
        return (ESCAPE2_FWD_SPD, steer,
                "ESCAPE2-FWD %.0f/%.0fms" % (elapsed*1000, ESCAPE2_FORWARD_MS))

    def _pick_escape_dir(self, depth_data):
        cl = depth_data["clear_left"]  if depth_data else np.inf
        cr = depth_data["clear_right"] if depth_data else np.inf
        return -1 if cl >= cr else +1

    def reset(self):
        self.__init__()


# ==============================================================================
#  MANUAL KEY STATE
# ==============================================================================

class KeyState:
    HOLD_MS = 250

    def __init__(self):
        self._t = {}

    def press(self, ch):
        self._t[ch] = time.time() * 1000

    def held(self, ch):
        return (time.time() * 1000 - self._t.get(ch, 0)) < self.HOLD_MS

    def command(self):
        spd = (MANUAL_SPEED  if (self.held('w') and not self.held('s')) else
               -MANUAL_SPEED if (self.held('s') and not self.held('w')) else 0)
        st  = (-MANUAL_STEER if (self.held('a') and not self.held('d')) else
                MANUAL_STEER if (self.held('d') and not self.held('a')) else 0)
        return spd, st


# ==============================================================================
#  AR BANNER HELPERS
# ==============================================================================

_AR_BANNER_CFG = {
    "TURN_LEFT":  ((0, 180, 255), "<<  TURN LEFT"),
    "TURN_RIGHT": ((0, 180, 255), "TURN RIGHT  >>"),
    "STOP":       ((0,   0, 220), "!! STOP !!"),
    "RESUME":     ((0, 160,   0), ">> RESUME <<"),
}


def _draw_ar_banner(img, banner_id):
    """Draw the centred ArUco action banner on img (in-place, any resolution)."""
    ar_action_name = ARUCO_ID_MAP.get(banner_id, "UNKNOWN")
    color, action_msg = _AR_BANNER_CFG.get(ar_action_name,
                                           ((120,120,0), ar_action_name))
    h, fw = img.shape[:2]
    font      = cv2.FONT_HERSHEY_DUPLEX
    big_scale = 0.90; big_thick = 2
    sm_scale  = 0.45; sm_thick  = 1
    id_text   = "ArUco ID: %d" % banner_id

    (bw, bh), _ = cv2.getTextSize(action_msg, font, big_scale, big_thick)
    (sw, sh), _ = cv2.getTextSize(id_text,    font, sm_scale,  sm_thick)
    banner_h = bh + sh + 20
    banner_y = h // 2 - banner_h // 2

    cv2.rectangle(img, (0, banner_y-10), (fw, banner_y+banner_h+10), color, -1)
    cv2.putText(img, id_text,
                ((fw-sw)//2, banner_y+sh),
                font, sm_scale, (255,255,255), sm_thick)
    cv2.putText(img, action_msg,
                ((fw-bw)//2, banner_y+sh+bh+10),
                font, big_scale, (255,255,255), big_thick)


# ==============================================================================
#  3-PANEL DEBUG DISPLAY
# ==============================================================================

_GUI_WIN = "Team Sarathi  |  Robot Vision  (L=Auto  M=Manual  Q=Quit)"


def _make_panel(img, W=GUI_PANEL_W, H=GUI_PANEL_H):
    if img is None:
        blank = np.zeros((H, W, 3), dtype=np.uint8)
        cv2.putText(blank, "No frame", (8, H//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60,60,60), 1)
        return blank
    if len(img.shape) == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return cv2.resize(img, (W, H))


def _overlay_lane_lines(panel, lx, rx, fw, fh, target_x, steer_error_px,
                         steer_cmd=0, W=GUI_PANEL_W, H=GUI_PANEL_H):
    sx = W / max(fw, 1)
    cx = W // 2
    for y in range(0, H, 14):
        cv2.line(panel, (cx, y), (cx, min(y+9, H-1)), GUI_FRAME_CX_COLOR, 1)
    if lx is not None:
        lxp = int(np.clip(lx * sx, 0, W-1))
        cv2.line(panel, (lxp, 0), (lxp, H), GUI_WALL_COLOR, 2)
        cv2.putText(panel, "L", (max(0, lxp-12), 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, GUI_WALL_COLOR, 1)
    if rx is not None:
        rxp = int(np.clip(rx * sx, 0, W-1))
        cv2.line(panel, (rxp, 0), (rxp, H), GUI_WALL_COLOR, 2)
        cv2.putText(panel, "R", (min(W-14, rxp+3), 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, GUI_WALL_COLOR, 1)
    if target_x is not None:
        txp = int(np.clip(target_x * sx, 0, W-1))
        cv2.line(panel, (txp, 0), (txp, H), GUI_CENTRE_COLOR, 2)
        cv2.putText(panel, "TARGET", (max(0, txp-22), H-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, GUI_CENTRE_COLOR, 1)
        MAX_ARROW_PX = W // 4
        arrow_len    = int(steer_cmd / max(MAX_STEER, 1) * MAX_ARROW_PX)
        arrow_y      = H // 2
        if abs(arrow_len) > 3:
            tip_x = int(np.clip(cx + arrow_len, 0, W-1))
            cv2.arrowedLine(panel, (cx, arrow_y), (tip_x, arrow_y),
                            GUI_ERROR_COLOR, 2, tipLength=0.25)


def draw_debug_v2(lane_roi, mask, depth_dbg, state_label,
                  speed, steer, mode, sm_state,
                  lx, rx, fw, fh,
                  target_x, steer_error_px,
                  lane_width, lane_width_calibrated,
                  bar_active=False,
                  ar_active=False,
                  ar_stopped=False,
                  resume_suppressed=False,
                  banner_id=None):
    W, H = GUI_PANEL_W, GUI_PANEL_H

    # Panel 1: Camera — AR banner drawn at ROI resolution before resize
    cam_raw = lane_roi.copy() if lane_roi is not None else None
    if cam_raw is not None and banner_id is not None:
        _draw_ar_banner(cam_raw, banner_id)
    cam = _make_panel(cam_raw)
    _overlay_lane_lines(cam, lx, rx, fw, fh, target_x, steer_error_px,
                        steer_cmd=steer)
    cv2.putText(cam, "CAMERA", (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,220,220), 1)
    if ar_active:
        cv2.putText(cam, "AR-TURN", (W-70, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200,0,255), 1)
    elif ar_stopped:
        cv2.putText(cam, "AR-STOP", (W-70, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,0,220), 1)
    elif resume_suppressed:
        # Teal label — auto-exit suppressed by RESUME marker
        cv2.putText(cam, "EXIT-OFF", (W-70, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,200,180), 1)

    # Panel 2: Yellow mask
    msk = _make_panel(mask)
    _overlay_lane_lines(msk, lx, rx, fw, fh, target_x, steer_error_px,
                        steer_cmd=steer)
    cv2.putText(msk, "YELLOW MASK", (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255,255,80), 1)
    lw_col = (80,255,80) if lane_width_calibrated else (80,80,255)
    lw_txt = ("LaneW=%.0fpx" % lane_width if lane_width_calibrated
              else "LaneW~%.0fpx(default)" % lane_width)
    cv2.putText(msk, lw_txt, (4, H-6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, lw_col, 1)

    # Panel 3: Depth
    dep = _make_panel(depth_dbg)
    cv2.putText(dep, "DEPTH", (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,165,255), 1)

    top_row = np.hstack([cam, msk, dep])
    total_w = top_row.shape[1]

    # HUD bar
    hud = np.zeros((GUI_HUD_H, total_w, 3), dtype=np.uint8)
    if ar_active:
        hud_bg = (180, 0, 200)
    elif ar_stopped:
        hud_bg = (0, 0, 160)
    elif resume_suppressed:
        hud_bg = (0, 120, 110)   # teal — false exit suppression active
    elif bar_active:
        hud_bg = (0, 80, 220)
    else:
        hud_bg = {
            RobotSM.ST_DODGE:   (0,  30, 180),
            RobotSM.ST_RECOV:   (0, 110,  45),
            RobotSM.ST_ESC_R:   (0,   0, 150),
            RobotSM.ST_ESC_F:   (0,   0, 150),
            RobotSM.ST_ESC2_R:  (0,  60, 180),
            RobotSM.ST_ESC2_F:  (0,  60, 180),
        }.get(sm_state, (24, 24, 24))
    hud[:] = hud_bg

    mode_col = {MODE_AUTO:   (0, 220, 50),
                MODE_MANUAL: (0, 165, 255)}.get(mode, (140,140,140))
    cv2.putText(hud, "[%s]  %s" % (mode, state_label[:55]),
                (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.46, mode_col, 1)
    cv2.putText(hud, "spd=%+d   steer=%+d   err=%+.0fpx" % (
                    speed, steer, steer_error_px),
                (6, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200,200,200), 1)

    _bar_en = get_bar_touch_enabled()
    tog_col = (0,220,80) if _bar_en else (80,80,80)
    cv2.putText(hud, "BAR-TOUCH: ON " if _bar_en else "BAR-TOUCH: OFF",
                (6, GUI_HUD_H-4), cv2.FONT_HERSHEY_SIMPLEX, 0.34, tog_col, 1)

    bar_x0 = total_w - 220
    bar_cx  = total_w - 120
    bar_y0  = 10; bar_y1 = 26
    cv2.rectangle(hud, (bar_x0, bar_y0), (total_w-20, bar_y1), (55,55,55), -1)
    cv2.line(hud, (bar_cx, bar_y0-2), (bar_cx, bar_y1+2), (255,255,255), 1)
    steer_fill = int(steer / 90.0 * 100)
    fill_col   = (0,200,255) if steer > 0 else (255,180,0)
    if steer > 0:
        cv2.rectangle(hud, (bar_cx, bar_y0+2),
                      (min(total_w-20, bar_cx+steer_fill), bar_y1-2), fill_col, -1)
    elif steer < 0:
        cv2.rectangle(hud, (max(bar_x0, bar_cx+steer_fill), bar_y0+2),
                      (bar_cx, bar_y1-2), fill_col, -1)
    cv2.putText(hud, "STEER", (bar_x0, bar_y1+14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (170,170,170), 1)

    spd_fill = int(abs(speed)/255.0*100)
    spd_col  = (0,220,50) if speed >= 0 else (50,50,220)
    cv2.rectangle(hud, (bar_x0, 30), (bar_x0+spd_fill, 40), spd_col, -1)
    cv2.rectangle(hud, (bar_x0, 30), (total_w-20, 40), (55,55,55), 1)
    cv2.putText(hud, "SPD", (bar_x0, GUI_HUD_H-4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (170,170,170), 1)

    cv2.imshow(_GUI_WIN, np.vstack([top_row, hud]))


# ==============================================================================
#  MAIN
# ==============================================================================

def main():
    print("[INFO] Connecting to Arduino ...")
    try:
        arduino = ArduinoSerial(BAUD_RATE)
    except serial.SerialException as e:
        print("[ERROR] %s" % e); sys.exit(1)

    depth_cam = None
    if REALSENSE_AVAILABLE:
        print("[INFO] Starting RealSense D455 ...")
        try:
            depth_cam = DepthCamera()
        except Exception as e:
            print("[WARN] RealSense: %s — obstacle avoidance disabled." % e)

    print("[INFO] Opening webcam ...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        print("[ERROR] Cannot open webcam."); arduino.stop(); sys.exit(1)

    create_controls_window()
    cv2.namedWindow(_GUI_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_GUI_WIN, GUI_PANEL_W * 3, GUI_PANEL_H + GUI_HUD_H)

    robot       = RobotSM()
    pd_center   = PD(KP_CENTER, KD_CENTER)
    pd_wall     = PD(KP_WALL,   KD_WALL)
    keys        = KeyState()
    path_track  = VirtualPathTracker()
    mid_smooth  = MidpointSmoother(alpha=MID_EMA_ALPHA)
    lane_w_est  = LaneWidthEstimator()
    ar_detector = ArucoDetector()
    exit_det    = ExitDetector()
    mode        = MODE_IDLE

    # Lane / lost state
    lost_frames         = 0
    last_steer          = 0
    last_speed          = BASE_SPEED
    prev_both_visible   = False
    first_vanished_side = None
    hline_active        = False
    hline_scan_dir      = +1
    hline_scan_count    = 0
    both_lost_since     = None

    # Proximity bar state
    bar_touch_count    = 0
    bar_touch_dir      = 0
    bar_reverse_active = False
    bar_reverse_t0     = 0.0
    bar_forward_active = False
    bar_forward_t0     = 0.0

    # AR / auto-exit navigation state
    ar_stopped            = False
    ar_stop_time          = 0.0
    ar_turn_active        = False
    ar_turn_dir           = 0
    ar_turn_t0_ms         = 0.0
    pending_turn          = None
    pending_turn_time     = 0.0
    banner_id             = None
    banner_until          = 0.0
    resume_suppress_until = 0.0   # blocks auto-exit after RESUME marker is seen

    print("[INFO] Ready — L=AUTO  M=MANUAL  Q=Quit")
    print("[INFO] Thresholds: DETECT=%.2fm  WARN=%.2fm  ESTOP=%.2fm" % (
        OBSTACLE_DETECT_M, OBSTACLE_WARN_M, OBSTACLE_ESTOP_M))
    print("[INFO] Lane: TIER1=centre-both  TIER2=single-wall-estimate  TIER3=lost")
    print("[INFO] AR codes: ID1=TURN_LEFT  ID2=TURN_RIGHT  ID3=STOP  ID4=RESUME")
    print("[INFO] Auto-exit: EXIT_CONFIRM_FRAMES=%d  (raise to reduce false triggers)"
          % EXIT_CONFIRM_FRAMES)
    print("[INFO] False-exit suppression: RESUME_SUPPRESS_DURATION_S=%.1fs"
          % RESUME_SUPPRESS_DURATION_S)

    try:
        while True:
            quit_flag = False
            k = cv2.waitKey(1) & 0xFF
            if k != 255:
                ch = chr(k).lower() if k < 128 else ''
                if ch == 'q':
                    quit_flag = True
                elif ch == 'l' and mode != MODE_AUTO:
                    mode = MODE_AUTO
                    robot.reset(); pd_center.reset(); pd_wall.reset()
                    path_track.reset(); mid_smooth.reset()
                    lane_w_est.reset(); exit_det.reset()
                    lost_frames = 0
                    bar_touch_count = 0; bar_reverse_active = False
                    bar_forward_active = False
                    ar_stopped = False; ar_turn_active = False
                    pending_turn = None; resume_suppress_until = 0.0
                    print("[INFO] -- AUTO --")
                elif ch == 'm' and mode != MODE_MANUAL:
                    mode = MODE_MANUAL
                    robot.reset(); arduino.send(0, 0)
                    path_track.reset(); mid_smooth.reset()
                    lane_w_est.reset(); exit_det.reset()
                    bar_touch_count = 0; bar_reverse_active = False
                    bar_forward_active = False
                    ar_stopped = False; ar_turn_active = False
                    pending_turn = None; resume_suppress_until = 0.0
                    print("[INFO] -- MANUAL --")
                if ch in ('w', 'a', 's', 'd'):
                    keys.press(ch)
            if quit_flag:
                break

            bar_offset = get_bar_offset()
            bar_y_frac = get_bar_y_frac()

            # ── Depth ─────────────────────────────────────────────────────────
            depth_data = None
            if depth_cam is not None:
                raw = depth_cam.get_frame()
                if raw is not None:
                    depth_data = analyse_depth(raw)
            depth_dbg = depth_data["dbg"] if depth_data else None

            # ── Capture frame ─────────────────────────────────────────────────
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Webcam frame failed."); continue

            # ── AR DETECTION — inline on full frame, before ROI crop ──────────
            frame    = ar_detector.detect_frame(frame)
            ar_id, _ = ar_detector.latest_id(max_age=0.5)
            ar_action = ARUCO_ID_MAP.get(ar_id) if ar_id is not None else None

            now    = time.time()
            now_ms = now * 1000

            # Update banner
            if ar_id is not None:
                banner_id    = ar_id
                banner_until = now + BANNER_DURATION
            if now > banner_until:
                banner_id = None

            # Process AR command
            if ar_action == "STOP" and not ar_stopped:
                ar_stopped   = True
                ar_stop_time = now
                pending_turn = None
                print("[AR] STOP command received.")

            elif ar_action == "RESUME":
                # Always set suppression window when RESUME is visible —
                # covers false exits where the robot was never stopped.
                if ar_stopped:
                    ar_stopped = False
                    print("[AR] RESUME command received — AR-STOP cleared.")
                if now >= resume_suppress_until:
                    # Only print once per suppression window start
                    print("[AR] RESUME seen — auto-exit suppressed for %.1fs"
                          % RESUME_SUPPRESS_DURATION_S)
                resume_suppress_until = now + RESUME_SUPPRESS_DURATION_S

            elif ar_action == "TURN_LEFT" and pending_turn != "TURN_LEFT":
                pending_turn      = "TURN_LEFT"
                pending_turn_time = now
                print("[AR] Stored junction command: TURN_LEFT")
            elif ar_action == "TURN_RIGHT" and pending_turn != "TURN_RIGHT":
                pending_turn      = "TURN_RIGHT"
                pending_turn_time = now
                print("[AR] Stored junction command: TURN_RIGHT")

            # Auto-expire AR commands
            if ar_stopped and (now - ar_stop_time) > ARUCO_STOP_TIMEOUT:
                ar_stopped = False
                print("[AR] STOP timeout — resuming.")
            if (pending_turn is not None and
                    (now - pending_turn_time) > ARUCO_CMD_TIMEOUT):
                print("[AR] Turn command '%s' expired." % pending_turn)
                pending_turn = None

            armed_cmd = ('LEFT'  if pending_turn == "TURN_LEFT"  else
                         'RIGHT' if pending_turn == "TURN_RIGHT" else None)

            # Convenience flag for GUI / Sub-case B
            resume_suppressed = (now < resume_suppress_until)

            # ── Lane detection ────────────────────────────────────────────────
            lx, rx, lane_roi, mask, fw = try_detect(frame, ROI_TOP_FRAC)
            hline_seen = detect_hline(mask)
            if lx is None and rx is None and not hline_seen:
                lx, rx, lane_roi, mask, fw = try_detect(frame, ROI_TOP_FRAC_WIDE)

            if not hline_active:
                if prev_both_visible:
                    if   lx is None and rx is not None:
                        first_vanished_side = "left"
                    elif rx is None and lx is not None:
                        first_vanished_side = "right"
            prev_both_visible = (lx is not None and rx is not None)

            fh = lane_roi.shape[0]
            lane_w_est.update(lx, rx)
            smooth_mid, tier1_valid = mid_smooth.update(lx, rx)
            smooth_x, path_conf, scan_pts, la_x = path_track.update(
                mask, lx, rx, fw, fh)

            # ── Exit detection ────────────────────────────────────────────────
            left_exit_ready, right_exit_ready = exit_det.update(lx, rx)

            # ── Bar touch ─────────────────────────────────────────────────────
            left_touch, right_touch = check_bar_touch(lx, rx, fw, bar_offset)
            bar_touch_enabled = get_bar_touch_enabled()

            if bar_touch_enabled:
                if not bar_reverse_active and not bar_forward_active:
                    if left_touch or right_touch:
                        bar_touch_count += 1
                        bar_touch_dir    = -1 if left_touch else +1
                    else:
                        bar_touch_count  = 0
                    if bar_touch_count >= BAR_CONFIRM_FRAMES:
                        bar_reverse_active = True
                        bar_reverse_t0     = now_ms
                        side_lbl = "L" if bar_touch_dir < 0 else "R"
                        print("[BAR] Touch (%s) — REV %dms -> FWD %dms" % (
                            side_lbl, get_bar_rev_duration(), get_bar_fwd_duration()))
                if bar_reverse_active:
                    if now_ms - bar_reverse_t0 >= get_bar_rev_duration():
                        bar_reverse_active = False
                        bar_forward_active = True
                        bar_forward_t0     = now_ms
                        print("[BAR] REV done -> FWD")
                if bar_forward_active:
                    if now_ms - bar_forward_t0 >= get_bar_fwd_duration():
                        bar_forward_active = False
                        bar_touch_count    = 0
                        bar_touch_dir      = 0
                        print("[BAR] FWD done — sequence complete")
            else:
                if bar_reverse_active or bar_forward_active:
                    bar_reverse_active = False; bar_forward_active = False
                    bar_touch_count    = 0;     bar_touch_dir      = 0
                    print("[BAR] Touch logic disabled — sequence cancelled.")

            # ── Draw overlays ─────────────────────────────────────────────────
            lane_roi_display = lane_roi.copy()
            display_cx = (smooth_mid if (tier1_valid and smooth_mid is not None)
                          else (smooth_x if smooth_x is not None else fw / 2.0))
            draw_virtual_path(lane_roi_display, scan_pts, la_x,
                              display_cx, fh, fw, path_conf,
                              tier1_active=tier1_valid)
            draw_proximity_bars(lane_roi_display, bar_offset, bar_y_frac,
                                lx, rx, fw, fh, left_touch, right_touch)
            exit_det.draw_overlay(lane_roi_display, fw, fh)

            # ==================================================================
            # IDLE
            # ==================================================================
            if mode == MODE_IDLE:
                arduino.send(0, 0)
                draw_debug_v2(lane_roi_display, mask, depth_dbg,
                              "IDLE", 0, 0, mode, robot.state,
                              lx, rx, fw, fh,
                              target_x=smooth_mid, steer_error_px=0.0,
                              lane_width=lane_w_est.width,
                              lane_width_calibrated=lane_w_est.calibrated,
                              resume_suppressed=resume_suppressed,
                              banner_id=banner_id)
                continue

            # ==================================================================
            # MANUAL
            # ==================================================================
            if mode == MODE_MANUAL:
                speed, steer = keys.command()
                arduino.send(speed, steer)
                wi = "%" if keys.held('w') else "."
                ai = "%" if keys.held('a') else "."
                si = "%" if keys.held('s') else "."
                di = "%" if keys.held('d') else "."
                draw_debug_v2(lane_roi_display, mask, depth_dbg,
                              "MANUAL W=%s A=%s S=%s D=%s" % (wi,ai,si,di),
                              speed, steer, mode, robot.state,
                              lx, rx, fw, fh,
                              target_x=None, steer_error_px=float(steer),
                              lane_width=lane_w_est.width,
                              lane_width_calibrated=lane_w_est.calibrated,
                              resume_suppressed=resume_suppressed,
                              banner_id=banner_id)
                continue

            # ==================================================================
            # AUTO — priority stack
            # ==================================================================

            gui_target_x     = None
            gui_steer_error  = 0.0
            estimated_centre = None

            # ── Priority 0a: Bar REVERSE ──────────────────────────────────────
            if bar_reverse_active:
                bar_steer = int(get_bar_rev_steer() * bar_touch_dir)
                bar_speed = get_bar_rev_speed()
                side_str  = "LEFT" if bar_touch_dir < 0 else "RIGHT"
                arduino.send(bar_speed, bar_steer)
                draw_debug_v2(lane_roi_display, mask, depth_dbg,
                              "BAR-REV->%s  spd=%d st=%+d" % (
                                  side_str, bar_speed, bar_steer),
                              bar_speed, bar_steer, mode, robot.state,
                              lx, rx, fw, fh,
                              target_x=None, steer_error_px=float(bar_steer),
                              lane_width=lane_w_est.width,
                              lane_width_calibrated=lane_w_est.calibrated,
                              bar_active=True,
                              resume_suppressed=resume_suppressed,
                              banner_id=banner_id)
                continue

            # ── Priority 0b: Bar FORWARD ──────────────────────────────────────
            if bar_forward_active:
                bar_steer = int(get_bar_fwd_steer() * -bar_touch_dir)
                bar_speed = get_bar_fwd_speed()
                side_str  = "LEFT" if -bar_touch_dir < 0 else "RIGHT"
                arduino.send(bar_speed, bar_steer)
                draw_debug_v2(lane_roi_display, mask, depth_dbg,
                              "BAR-FWD->%s  spd=%d st=%+d" % (
                                  side_str, bar_speed, bar_steer),
                              bar_speed, bar_steer, mode, robot.state,
                              lx, rx, fw, fh,
                              target_x=None, steer_error_px=float(bar_steer),
                              lane_width=lane_w_est.width,
                              lane_width_calibrated=lane_w_est.calibrated,
                              bar_active=True,
                              resume_suppressed=resume_suppressed,
                              banner_id=banner_id)
                continue

            # ── Priority 1: AR STOP ───────────────────────────────────────────
            if ar_stopped:
                arduino.send(0, 0)
                draw_debug_v2(lane_roi_display, mask, depth_dbg,
                              "AR-STOPPED  (show ID4/RESUME to go)",
                              0, 0, mode, robot.state,
                              lx, rx, fw, fh,
                              target_x=None, steer_error_px=0.0,
                              lane_width=lane_w_est.width,
                              lane_width_calibrated=lane_w_est.calibrated,
                              ar_stopped=True,
                              resume_suppressed=resume_suppressed,
                              banner_id=banner_id)
                continue

            # ── Priority 2: Exit turn trigger + active turn ───────────────────
            #
            # Three sub-cases handled in one block:
            #   A) AR command pending + matching wall gone  (AR-guided)
            #   B) No AR command      + single exit ready  (automatic)
            #      — skipped entirely if resume_suppressed is True
            #      — also skipped when armed_cmd is set (opposite-turn strategy)
            #   C) Turn already active                     (continue turning)
            #
            if not ar_turn_active:

                # Sub-case A — AR command waiting for matching exit
                left_exit_now  = ((lx is None) if armed_cmd == 'LEFT'
                                  else left_exit_ready)
                right_exit_now = ((rx is None) if armed_cmd == 'RIGHT'
                                  else right_exit_ready)

                if armed_cmd == 'LEFT' and left_exit_now:
                    ar_turn_active = True
                    ar_turn_dir    = -1
                    ar_turn_t0_ms  = now_ms
                    pending_turn   = None
                    exit_det.reset()
                    robot.reset()
                    print("[AR] LEFT wall gone -> AR_TURN_LEFT")

                elif armed_cmd == 'RIGHT' and right_exit_now:
                    ar_turn_active = True
                    ar_turn_dir    = +1
                    ar_turn_t0_ms  = now_ms
                    pending_turn   = None
                    exit_det.reset()
                    robot.reset()
                    print("[AR] RIGHT wall gone -> AR_TURN_RIGHT")

                # Sub-case B — No AR command, but a clear single-sided exit.
                # Blocked when:
                #   • armed_cmd is set (opposite-turn marker strategy — inherent)
                #   • resume_suppressed is True (RESUME marker strategy)
                # Only fires when EXACTLY one wall is gone (not both — that
                # is TIER 3 lost, handled below).
                elif armed_cmd is None and not resume_suppressed:
                    if left_exit_ready and not right_exit_ready:
                        ar_turn_active = True
                        ar_turn_dir    = -1
                        ar_turn_t0_ms  = now_ms
                        exit_det.reset()
                        robot.reset()
                        print("[EXIT] Left wall gone (no AR) -> AUTO_TURN_LEFT")

                    elif right_exit_ready and not left_exit_ready:
                        ar_turn_active = True
                        ar_turn_dir    = +1
                        ar_turn_t0_ms  = now_ms
                        exit_det.reset()
                        robot.reset()
                        print("[EXIT] Right wall gone (no AR) -> AUTO_TURN_RIGHT")

                elif armed_cmd is None and resume_suppressed:
                    # Log once per suppression window so it's visible in console
                    pass  # exit_det counters still increment; reset happens on
                          # entry to the next real exit after suppression expires

            # Sub-case C — actively executing a turn (AR or auto)
            if ar_turn_active:
                elapsed_turn_ms = now_ms - ar_turn_t0_ms
                if elapsed_turn_ms < EXIT_TURN_DURATION_MS:
                    ar_steer = int(EXIT_TURN_STEER * ar_turn_dir)
                    side_lbl = "LEFT" if ar_turn_dir < 0 else "RIGHT"
                    arduino.send(EXIT_TURN_SPEED, ar_steer)
                    draw_debug_v2(lane_roi_display, mask, depth_dbg,
                                  "TURN %s  %.0f/%.0fms" % (
                                      side_lbl, elapsed_turn_ms,
                                      EXIT_TURN_DURATION_MS),
                                  EXIT_TURN_SPEED, ar_steer,
                                  mode, robot.state,
                                  lx, rx, fw, fh,
                                  target_x=None,
                                  steer_error_px=float(ar_steer),
                                  lane_width=lane_w_est.width,
                                  lane_width_calibrated=lane_w_est.calibrated,
                                  ar_active=True,
                                  resume_suppressed=resume_suppressed,
                                  banner_id=banner_id)
                    continue
                else:
                    ar_turn_active  = False
                    lost_frames     = 0
                    both_lost_since = None
                    path_track.reset()
                    mid_smooth.reset()
                    print("[TURN] Complete (%.0fms) -> LANE_FOLLOW"
                          % elapsed_turn_ms)

            # ==================================================================
            # TIER 1 — Both walls visible
            # ==================================================================
            if tier1_valid and smooth_mid is not None:
                if LOOKAHEAD_BLEND > 0 and la_x is not None:
                    target_x = ((1.0 - LOOKAHEAD_BLEND) * smooth_mid
                                + LOOKAHEAD_BLEND * la_x)
                    tier_lbl = "T1-CTR+LA"
                else:
                    target_x = smooth_mid
                    tier_lbl = "T1-CTR"

                error      = target_x - fw / 2.0
                lane_steer = -pd_center.compute(error)
                lane_speed = BASE_SPEED;  lane_state = tier_lbl
                lost_frames = 0; last_steer = lane_steer
                last_speed  = lane_speed;  both_lost_since = None
                gui_target_x    = target_x
                gui_steer_error = error

            # ==================================================================
            # TIER 2 — Single wall: estimate centre from learned lane width
            # ==================================================================
            elif rx is not None:
                estimated_centre = lane_w_est.centre_from_right(rx)
                mid_smooth.inject(estimated_centre)
                error      = estimated_centre - fw / 2.0
                lane_steer = -pd_center.compute(error)
                lane_speed = TURN_SPEED
                lane_state = "T2-CTR(R-wall) LW=%.0f" % lane_w_est.width
                lost_frames = 0; last_steer = lane_steer
                last_speed  = lane_speed;  both_lost_since = None
                gui_target_x    = estimated_centre
                gui_steer_error = error

            elif lx is not None:
                estimated_centre = lane_w_est.centre_from_left(lx)
                mid_smooth.inject(estimated_centre)
                error      = estimated_centre - fw / 2.0
                lane_steer = -pd_center.compute(error)
                lane_speed = TURN_SPEED
                lane_state = "T2-CTR(L-wall) LW=%.0f" % lane_w_est.width
                lost_frames = 0; last_steer = lane_steer
                last_speed  = lane_speed;  both_lost_since = None
                gui_target_x    = estimated_centre
                gui_steer_error = error

            # ==================================================================
            # TIER 3 — No walls
            # ==================================================================
            else:
                lost_frames += 1
                if both_lost_since is None:
                    both_lost_since = time.time()
                elapsed = time.time() - both_lost_since

                if elapsed >= BOTH_LOST_TIMEOUT:
                    lane_steer = 0; lane_speed = 0; lane_state = "LOST-STOP"
                elif lost_frames <= LOST_HOLD_FRAMES:
                    lane_steer = last_steer; lane_speed = LOST_SPEED
                    lane_state = "LOST-HOLD %.1fs" % elapsed
                elif lost_frames < LOST_FRAME_THRESH:
                    lane_steer = 0; lane_speed = LOST_SPEED
                    lane_state = "LOST-CREEP %.1fs" % elapsed
                else:
                    if first_vanished_side and not hline_active:
                        hline_active     = True
                        hline_scan_dir   = (-1 if first_vanished_side == "left"
                                            else +1)
                        hline_scan_count = 0
                    lane_steer = 0; lane_speed = 0; lane_state = "LOST-STOP"

                gui_steer_error = float(lane_steer)

            # ── AR straight-hold override ─────────────────────────────────────
            # When an AR turn command is armed and the OPPOSITE wall has gone
            # (false exit on that side), Tier 2's centre estimate would drift
            # the robot toward the gap. Override to steer=0 so it goes straight
            # and waits for the correct exit to open.
            #
            #   armed RIGHT, left wall gone  → only right wall remains → go straight
            #   armed LEFT,  right wall gone → only left  wall remains → go straight
            #
            # Tier 1 (both walls) and Tier 3 (no walls) are unaffected.
            if armed_cmd == 'RIGHT' and lx is None and rx is not None:
                lane_steer = 0
                lane_speed = BASE_SPEED
                lane_state = "STRAIGHT(wait-R-exit)"
                gui_steer_error = 0.0
            elif armed_cmd == 'LEFT' and rx is None and lx is not None:
                lane_steer = 0
                lane_speed = BASE_SPEED
                lane_state = "STRAIGHT(wait-L-exit)"
                gui_steer_error = 0.0

            # ── Wall guard ────────────────────────────────────────────────────
            lane_steer = int(np.clip(lane_steer, -MAX_STEER, MAX_STEER))
            lane_steer = apply_wall_guard(lane_steer, lx, rx, fw)

            # ── H-line handling ───────────────────────────────────────────────
            if hline_seen or hline_active:
                if not hline_active:
                    hline_scan_dir   = (-1 if first_vanished_side == "left"
                                        else +1)
                    hline_active     = True
                    hline_scan_count = 0
                    print("[INFO] H-line — pivot %s" % (
                        "L" if hline_scan_dir < 0 else "R"))
                if lx is not None or rx is not None:
                    hline_active        = False
                    first_vanished_side = None
                    prev_both_visible   = False
                    lost_frames         = 0
                    both_lost_since     = None
                else:
                    hline_scan_count += 1
                    if hline_scan_count > HLINE_SCAN_FRAMES:
                        hline_scan_dir *= -1; hline_scan_count = 0
                    steer = int(np.clip(HLINE_SCAN_STEER * hline_scan_dir,
                                        -MAX_STEER, MAX_STEER))
                    arduino.send(HLINE_CRAWL_SPEED, steer)
                    draw_debug_v2(lane_roi_display, mask, depth_dbg,
                                  "HLINE-PIVOT %s" % (
                                      "R" if hline_scan_dir > 0 else "L"),
                                  HLINE_CRAWL_SPEED, steer,
                                  mode, robot.state,
                                  lx, rx, fw, fh,
                                  target_x=None, steer_error_px=float(steer),
                                  lane_width=lane_w_est.width,
                                  lane_width_calibrated=lane_w_est.calibrated,
                                  resume_suppressed=resume_suppressed,
                                  banner_id=banner_id)
                    continue

            # ── Obstacle state machine ────────────────────────────────────────
            speed, steer, state_label, bypass_guard = robot.update(
                depth_data, lx, rx, fw, lane_steer, lane_speed)

            # ── Final wall guard ──────────────────────────────────────────────
            if not bypass_guard:
                steer = apply_wall_guard(steer, lx, rx, fw)
            steer = int(np.clip(steer, -MAX_STEER, MAX_STEER))

            # Append pending AR / suppression info to HUD label
            if armed_cmd in ('LEFT', 'RIGHT'):
                remaining = max(0.0, ARUCO_CMD_TIMEOUT -
                                (time.time() - pending_turn_time))
                state_label += " [AR:%s %.0fs]" % (armed_cmd, remaining)
            if resume_suppressed:
                remaining_sup = max(0.0, resume_suppress_until - time.time())
                state_label += " [EXIT-OFF %.0fs]" % remaining_sup

            arduino.send(speed, steer)
            draw_debug_v2(lane_roi_display, mask, depth_dbg,
                          state_label, speed, steer,
                          mode, robot.state,
                          lx, rx, fw, fh,
                          target_x=gui_target_x,
                          steer_error_px=gui_steer_error,
                          lane_width=lane_w_est.width,
                          lane_width_calibrated=lane_w_est.calibrated,
                          bar_active=False,
                          resume_suppressed=resume_suppressed,
                          banner_id=banner_id)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
    finally:
        print("[INFO] Stopping.")
        arduino.stop()
        if depth_cam is not None:
            depth_cam.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()