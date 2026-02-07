"""
Privacy-Preserving Smart-Home Semantic Camera (NO ML)

Board   : Adafruit MEMENTO (ESP32-S3 + OV5640)
Output  : Serial Monitor + Wi-Fi UDP (LAN only)

This system performs on-device semantic interpretation of human
behavior for smart-home adaptation, without storing or transmitting
any visual data.

The camera functions as a semantic sensor, not a recorder.
"""

import time
import gc
import wifi
import socketpool
import adafruit_pycamera

# =========================================================
# NETWORK CONFIGURATION
# =========================================================

WIFI_SSID = "AITTEST"
WIFI_PASS = "12345678"

UDP_TARGET_IP = "192.168.1.255"
UDP_PORT = 5005

# =========================================================
# SYSTEM CONSTANTS
# =========================================================

FRAME_INTERVAL = 5.0              # Sampling period (seconds)
SUBSAMPLE_STEP = 10               # Spatial downsampling factor

WINDOW = 6                        # Motion history window
POSTURE_WINDOW = 5                # Posture median window
TREND_WINDOW = 4                  # Motion trend window
RESP_WINDOW = 10                  # Responsiveness window

OCCUPANCY_THRESHOLD = 8
STAND_CY = 70                     # Centroid threshold for standing
SIT_CY = 110                      # Centroid threshold for sitting

MOTION_ALPHA = 0.4                # EMA smoothing factor

FALL_SPIKE = 85                   # Sudden motion spike threshold
UNRESPONSIVE_TIME = 20            # Seconds of inactivity

# =========================================================
# STATE VARIABLES
# =========================================================

prev_frame = None

motion_hist = []
centroid_hist = []
posture_hist = []

motion_min = None
motion_max = None
motion_smooth = None

presence_start = None
last_active = None

# =========================================================
# NETWORK INITIALIZATION
# =========================================================

wifi.radio.enabled = True
wifi.radio.connect(WIFI_SSID, WIFI_PASS)
pool = socketpool.SocketPool(wifi.radio)
sock = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)

# =========================================================
# CAMERA INITIALIZATION
# =========================================================

pycam = adafruit_pycamera.PyCamera()
pycam.mode = 0                   # Grayscale
pycam.resolution = 1             # 320 × 240
pycam.effect = 2                 # Black & White
pycam.led_level = 0              # LED off (privacy)
pycam.tone(1000, 0.05)

# =========================================================
# LOW-LEVEL SIGNAL EXTRACTION
# =========================================================

def frame_difference(a, b):
    """
    Motion Energy Computation

    Formula:
    --------
    M = (1 / N) * Σ | I_t(x,y) − I_(t−1)(x,y) |

    where:
    I_t(x,y)      = pixel intensity at time t
    I_(t−1)(x,y)  = pixel intensity at previous frame
    N             = number of sampled pixels
    """
    s = 0
    n = 0
    for y in range(0, a.height, SUBSAMPLE_STEP):
        for x in range(0, a.width, SUBSAMPLE_STEP):
            s += abs(a[x, y] - b[x, y])
            n += 1
    return s // n if n else 0


def normalize_motion(v):
    """
    Adaptive Motion Normalization

    Formula:
    --------
    M_norm = 100 * (M − M_min) / (M_max − M_min)

    This allows self-calibration across different rooms
    and lighting conditions.
    """
    global motion_min, motion_max
    motion_min = v if motion_min is None else min(motion_min, v)
    motion_max = v if motion_max is None else max(motion_max, v)
    span = max(1, motion_max - motion_min)
    return int(100 * (v - motion_min) / span)


def smooth_motion(v):
    """
    Exponential Moving Average (EMA)

    Formula:
    --------
    M_s(t) = α · M(t) + (1 − α) · M_s(t−1)

    where:
    α = MOTION_ALPHA
    """
    global motion_smooth
    motion_smooth = v if motion_smooth is None else (
        MOTION_ALPHA * v + (1 - MOTION_ALPHA) * motion_smooth
    )
    return int(motion_smooth)


def centroid_y(frame):
    """
    Vertical Intensity Centroid

    Formula:
    --------
    C_y = Σ ( y · I(x,y) ) / Σ I(x,y)

    Used as a geometric proxy for posture estimation.
    """
    w = p = 0
    for y in range(0, frame.height, SUBSAMPLE_STEP):
        for x in range(0, frame.width, SUBSAMPLE_STEP):
            val = frame[x, y]
            if val > 20:                 # Noise threshold
                w += y * val
                p += val
    return w // p if p else None

# =========================================================
# SEMANTIC STATE INFERENCE
# =========================================================

def posture(cy_hist):
    """
    Posture Classification using Median Filtering

    Median(C_y) over POSTURE_WINDOW is compared
    against calibrated thresholds.
    """
    if len(cy_hist) < POSTURE_WINDOW:
        return "UNKNOWN"
    m = sorted(cy_hist[-POSTURE_WINDOW:])[POSTURE_WINDOW // 2]
    if m < STAND_CY: return "STANDING"
    if m < SIT_CY: return "SITTING"
    return "LYING"


def activity(m):
    """
    Activity Level based on Motion Magnitude
    """
    if m < 20: return "IDLE"
    if m < 50: return "ACTIVE"
    return "INTENSE"


def motion_trend(hist):
    """
    Motion Trend Estimation

    Formula:
    --------
    ΔM = M_last − M_first
    """
    if len(hist) < TREND_WINDOW:
        return "STABLE"
    d = hist[-1] - hist[0]
    if d > 10: return "INCREASING"
    if d < -10: return "DECREASING"
    return "STABLE"


def affect_state(hist):
    """
    Affective State Proxy using Motion Variance

    Formula:
    --------
    σ² = (1/N) * Σ ( M_i − μ )²
    """
    if len(hist) < 4:
        return "UNKNOWN"
    mean = sum(hist) / len(hist)
    var = sum((x - mean) ** 2 for x in hist) / len(hist)
    if var < 25: return "CALM"
    if var < 120: return "STRESSED"
    return "AGITATED"


def fatigue_state(hist, posture_hist):
    """
    Fatigue Estimation using Sustained Low Motion
    and Posture Degradation
    """
    if len(hist) < WINDOW:
        return "UNKNOWN"
    if max(hist[-WINDOW:]) < 15 and posture_hist[-1] in ("SITTING", "LYING"):
        return "HIGH"
    if sum(hist[-WINDOW:]) / WINDOW < 25:
        return "MEDIUM"
    return "LOW"


def responsiveness(last_active):
    """
    Responsiveness Score

    Formula:
    --------
    RESP = max(0, 100 − k · Δt)

    where:
    Δt = time since last detected activity
    """
    if not last_active:
        return 0
    dt = time.monotonic() - last_active
    return max(0, 100 - int(dt * 5))


def posture_transition_rate(hist):
    """
    Posture Transition Rate

    Measures number of posture changes over time
    to infer stability or agitation.
    """
    if len(hist) < 4:
        return "STABLE"
    changes = sum(hist[i] != hist[i-1] for i in range(1, len(hist)))
    if changes <= 1: return "STABLE"
    if changes <= 3: return "FREQUENT"
    return "ERRATIC"


def environment_intent(activity, affect, fatigue, safety):
    """
    High-Level Environmental Control Intent
    """
    if safety != "NORMAL": return "ALERT"
    if fatigue == "HIGH": return "DIM_WARM"
    if affect == "AGITATED": return "CALM_ENV"
    if activity == "INTENSE": return "COOL_BRIGHT"
    return "NO_ACTION"

# =========================================================
# MAIN LOOP
# =========================================================

last_tick = time.monotonic()

while True:
    frame = pycam.continuous_capture()
    pycam.blit(frame)

    if prev_frame:
        raw = frame_difference(frame, prev_frame)
        motion = smooth_motion(normalize_motion(raw))
        now = time.monotonic()

        if motion > OCCUPANCY_THRESHOLD:
            last_active = now
            if presence_start is None:
                presence_start = now

        cy = centroid_y(frame)
        if cy is not None:
            centroid_hist.append(cy)
            centroid_hist = centroid_hist[-20:]

        motion_hist.append(motion)
        motion_hist = motion_hist[-WINDOW:]

        post = posture(centroid_hist)
        posture_hist.append(post)
        posture_hist = posture_hist[-10:]

        act = activity(motion)
        trend = motion_trend(motion_hist)
        affect = affect_state(motion_hist)
        fatigue = fatigue_state(motion_hist, posture_hist)
        resp = responsiveness(last_active)
        post_tr = posture_transition_rate(posture_hist)

        safety = (
            "FALL_RISK" if motion > FALL_SPIKE and post == "LYING"
            else "UNRESPONSIVE" if resp < 30
            else "NORMAL"
        )

        env_int = environment_intent(act, affect, fatigue, safety)
        presence_time = int(now - presence_start) if presence_start else 0
        confidence = min(100, resp + (10 if post != "UNKNOWN" else 0))

        payload = (
            f"OCC={int(motion > OCCUPANCY_THRESHOLD)},"
            f"MOT={motion},"
            f"ACT={act},"
            f"POST={post},"
            f"TREND={trend},"
            f"AFFECT={affect},"
            f"FATIGUE={fatigue},"
            f"RESP={resp},"
            f"POST_TR={post_tr},"
            f"SAFE={safety},"
            f"PRES_T={presence_time},"
            f"CONF={confidence},"
            f"ENV_INT={env_int}"
        )

        sock.sendto(payload.encode(), (UDP_TARGET_IP, UDP_PORT))
        print("TX:", payload)

    prev_frame = frame
    gc.collect()

    sleep = FRAME_INTERVAL - (time.monotonic() - last_tick)
    if sleep > 0:
        time.sleep(sleep)
    last_tick = time.monotonic()
