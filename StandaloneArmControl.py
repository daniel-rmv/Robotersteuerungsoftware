#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArmPi Pro Standalone (ROS-frei, GPIO Half-Duplex)

CLI Highlights:
  --home / --set-home / --clear-home / --factory-home
  --hello                 (winken)
  --pickup                (nur aufheben)
  --getobject             (aufheben + rechts abstellen)
  --grip-object
  --gripper-open / --gripper-close / --gripper-object

Einzelgelenk-Flags:
  --roll-zero / --roll-left-max / --roll-right-max / --roll-percent X
  --pitch-zero / --pitch-percent X
  --elbow-zero / --elbow-percent X
  --shoulder-zero / --shoulder-percent X
  --base-zero / --base-left-90 / --base-right-90 / --base-left-45
  --base-right-45 / --base-left-max / --base-right-max / --base-percent X

Postures:
  --list-poses (alias: --pose-list / --list_poses)
  --pose pose_base_zero              (öffnet vorher)
  --pose pose_base_zero_closed       (öffnet erst unten)
  ... alle Base-Richtungen gibt’s als offen & _closed

Power:
  --unload / --load
"""

import os
import time
import json
import struct
import sqlite3 as sql
from typing import Dict, Tuple, Union

import serial
import RPi.GPIO as GPIO

# -------------------- Protokoll --------------------
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31

# -------------------- Joint-Config -----------------
JOINTS: Dict[str, int] = {
    'gripper'    : 1,
    'wrist_roll' : 2,
    'wrist_pitch': 3,
    'elbow'      : 4,
    'shoulder'   : 5,
    'base'       : 6,
}

DIR_SIGN: Dict[str, int] = {
    'gripper'    : 1,
    'wrist_roll' : 1,
    'wrist_pitch': 1,
    'elbow'      : 1,
    'shoulder'   : 1,
    'base'       : 1,
}

# Gripper Puls-Werte
GRIPPER_OPEN_PULSE   = 32
GRIPPER_CLOSE_PULSE  = 607
GRIPPER_OBJECT_PULSE = 422

# Wrist Roll (ID2)
WRIST_ROLL_LEFT_MAX_PULSE  = 25
WRIST_ROLL_RIGHT_MAX_PULSE = 995
WRIST_ROLL_ZERO_PULSE      = 500

# Wrist Pitch (ID3)
WRIST_PITCH_DOWN_MAX_PULSE = 55
WRIST_PITCH_UP_MAX_PULSE   = 970
WRIST_PITCH_ZERO_PULSE     = 500

# Elbow (ID4)
ELBOW_DOWN_MAX_PULSE = 1008
ELBOW_UP_MAX_PULSE   = 0
ELBOW_ZERO_PULSE     = 500

# Shoulder (ID5)
SHOULDER_DOWN_MAX_PULSE = 655
SHOULDER_UP_MAX_PULSE   = 877
SHOULDER_ZERO_PULSE     = 500

# Base (ID6)
BASE_ZERO_PULSE       = 500
BASE_LEFT_MAX_PULSE   = 1000
BASE_RIGHT_MAX_PULSE  = 0
BASE_LEFT_45_PULSE    = 693
BASE_RIGHT_45_PULSE   = 305
BASE_LEFT_90_PULSE    = 868
BASE_RIGHT_90_PULSE   = 122

# Front-Down Pose (gleich für alle Basen)
POSTURE_FRONT_DOWN = {
    'wrist_pitch': 192,
    'elbow'      : 707,
    'shoulder'   : 268,
    'wrist_roll' : 500,
}

def _ppair(a, b):
    return (min(a, b), max(a, b))

JOINT_PERCENT_LIMITS = {
    'wrist_roll'  : _ppair(WRIST_ROLL_RIGHT_MAX_PULSE,  WRIST_ROLL_LEFT_MAX_PULSE),
    'wrist_pitch' : _ppair(WRIST_PITCH_DOWN_MAX_PULSE,  WRIST_PITCH_UP_MAX_PULSE),
    'elbow'       : _ppair(ELBOW_UP_MAX_PULSE,          ELBOW_DOWN_MAX_PULSE),
    'shoulder'    : _ppair(SHOULDER_DOWN_MAX_PULSE,     SHOULDER_UP_MAX_PULSE),
    'base'        : _ppair(BASE_RIGHT_MAX_PULSE,        BASE_LEFT_MAX_PULSE),
}

PULSE_RANGE: Dict[str, Tuple[int, int]] = {j: (0, 1000) for j in JOINTS}
DEG_RANGE:   Dict[str, Tuple[float, float]] = {j: (0.0, 240.0) for j in JOINTS}
SOFT_LIMITS: Dict[str, Tuple[float, float]] = DEG_RANGE.copy()

DEFAULT_DURATION_MS = 800
DEFAULT_BAUD = 115200

RX_PIN = 4
TX_PIN = 27

HOME_FILE = os.path.expanduser('~/.armpi_home.json')
CUSTOM_POSE_FILE = os.path.expanduser('~/.armpi_postures.json')
DEBUG = False

Number   = Union[int, float]
JointKey = Union[str, int]

# ---------- Helper ----------
def clamp(v: Number, lo: Number, hi: Number) -> Number:
    return lo if v < lo else hi if v > hi else v

def deg_to_pulse(joint: str, deg: float) -> int:
    p0, p1 = PULSE_RANGE[joint]
    d0, d1 = DEG_RANGE[joint]
    deg = clamp(deg, d0, d1)
    return int(round((deg - d0) * (p1 - p0) / (d1 - d0) + p0))

def pulse_to_deg(joint: str, pulse: int) -> float:
    p0, p1 = PULSE_RANGE[joint]
    d0, d1 = DEG_RANGE[joint]
    pulse = clamp(pulse, p0, p1)
    return (pulse - p0) * (d1 - d0) / (p1 - p0) + d0

def _mk_pose(base_pulse, gripper_pulse):
    return {'base': base_pulse, **POSTURE_FRONT_DOWN, 'gripper': gripper_pulse}

# ---------- POSTURES ----------
POSTURES: Dict[str, Dict[str, int]] = {
    # offen Varianten
    'pose_base_zero'      : _mk_pose(BASE_ZERO_PULSE,      GRIPPER_OPEN_PULSE),
    'pose_base_left_45'   : _mk_pose(BASE_LEFT_45_PULSE,   GRIPPER_OPEN_PULSE),
    'pose_base_left_90'   : _mk_pose(BASE_LEFT_90_PULSE,   GRIPPER_OPEN_PULSE),
    'pose_base_left_max'  : _mk_pose(BASE_LEFT_MAX_PULSE,  GRIPPER_OPEN_PULSE),
    'pose_base_right_45'  : _mk_pose(BASE_RIGHT_45_PULSE,  GRIPPER_OPEN_PULSE),
    'pose_base_right_90'  : _mk_pose(BASE_RIGHT_90_PULSE,  GRIPPER_OPEN_PULSE),
    'pose_base_right_max' : _mk_pose(BASE_RIGHT_MAX_PULSE, GRIPPER_OPEN_PULSE),

    # closed Varianten
    'pose_base_zero_closed'      : _mk_pose(BASE_ZERO_PULSE,      GRIPPER_CLOSE_PULSE),
    'pose_base_left_45_closed'   : _mk_pose(BASE_LEFT_45_PULSE,   GRIPPER_CLOSE_PULSE),
    'pose_base_left_90_closed'   : _mk_pose(BASE_LEFT_90_PULSE,   GRIPPER_CLOSE_PULSE),
    'pose_base_left_max_closed'  : _mk_pose(BASE_LEFT_MAX_PULSE,  GRIPPER_CLOSE_PULSE),
    'pose_base_right_45_closed'  : _mk_pose(BASE_RIGHT_45_PULSE,  GRIPPER_CLOSE_PULSE),
    'pose_base_right_90_closed'  : _mk_pose(BASE_RIGHT_90_PULSE,  GRIPPER_CLOSE_PULSE),
    'pose_base_right_max_closed' : _mk_pose(BASE_RIGHT_MAX_PULSE, GRIPPER_CLOSE_PULSE),
}

# Custom Posen mergen (falls vorhanden)
if os.path.exists(CUSTOM_POSE_FILE):
    try:
        with open(CUSTOM_POSE_FILE, 'r') as f:
            custom = json.load(f)
        POSTURES.update(custom)
    except Exception as e:
        print("[WARN] Konnte custom poses nicht laden:", e)

# -------------------- Low Level --------------------
class BusServo:
    def __init__(self, port='/dev/serial0', baud=DEFAULT_BAUD,
                 timeout=0.1, write_timeout=0.8, use_gpio=True):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout, write_timeout=write_timeout)
        self.use_gpio = use_gpio
        if self.use_gpio:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(RX_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(TX_PIN, GPIO.OUT, initial=GPIO.HIGH)

    def _port_write(self):
        if self.use_gpio:
            GPIO.output(TX_PIN, GPIO.HIGH)
            GPIO.output(RX_PIN, GPIO.LOW)

    def _port_read(self):
        if self.use_gpio:
            GPIO.output(RX_PIN, GPIO.HIGH)
            GPIO.output(TX_PIN, GPIO.LOW)

    @staticmethod
    def _checksum(buf: bytes) -> int:
        s = sum(buf) - 0x55 - 0x55
        return (~s) & 0xFF

    def _packet(self, sid: int, cmd: int, dat1=None, dat2=None) -> bytes:
        b = bytearray([0x55, 0x55, sid])
        if dat1 is None and dat2 is None:
            b += bytes([3, cmd])
        elif dat1 is not None and dat2 is None:
            b += bytes([4, cmd, dat1 & 0xFF])
        else:
            b += bytes([7, cmd,
                       dat1 & 0xFF, (dat1 >> 8) & 0xFF,
                       dat2 & 0xFF, (dat2 >> 8) & 0xFF])
        b.append(self._checksum(b))
        return bytes(b)

    def write_cmd(self, sid, cmd, dat1=None, dat2=None):
        sid = 0xFE if sid is None else sid
        pkt = self._packet(sid, cmd, dat1, dat2)
        if DEBUG:
            print("TX:", pkt.hex())
        self._port_write()
        self.ser.write(pkt)
        time.sleep(0.00034)

    def read_cmd(self, sid, cmd):
        sid = 0xFE if sid is None else sid
        pkt = self._packet(sid, cmd)
        if DEBUG:
            print("TX(R):", pkt.hex())
        self._port_write()
        self.ser.write(pkt)
        time.sleep(0.00034)

    def get_reply(self, expect_cmd):
        self.ser.flushInput()
        self._port_read()
        time.sleep(0.005)
        if self.ser.in_waiting == 0:
            return None
        data = self.ser.read(self.ser.in_waiting)
        if DEBUG and data:
            print("RX:", data.hex())
        if len(data) < 7:
            return None
        try:
            idx = data.index(b"\x55\x55")
        except ValueError:
            return None
        frame = data[idx:]
        if len(frame) < 7:
            return None
        length = frame[3]
        _cmd   = frame[4]
        if _cmd != expect_cmd:
            return None
        if length == 4:
            return struct.unpack('b', frame[5:6])[0]
        if length == 5:
            val = frame[5] | (frame[6] << 8)
            return struct.unpack('<h', struct.pack('<H', val))[0]
        if length == 7:
            v1 = frame[5] | (frame[6] << 8)
            v2 = frame[7] | (frame[8] << 8)
            return (struct.unpack('<h', struct.pack('<H', v1))[0],
                    struct.unpack('<h', struct.pack('<H', v2))[0])
        return None

    def move_time_write(self, sid: int, pulse: int, t_ms: int):
        self.write_cmd(sid, LOBOT_SERVO_MOVE_TIME_WRITE,
                       int(clamp(pulse, 0, 1000)), int(clamp(t_ms, 0, 30000)))

    def stop(self, sid=None):
        self.write_cmd(sid, LOBOT_SERVO_MOVE_STOP)

    def read_pos(self, sid: int):
        self.read_cmd(sid, LOBOT_SERVO_POS_READ)
        return self.get_reply(LOBOT_SERVO_POS_READ)

    def unload(self, sid: int):
        self.write_cmd(sid, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

    def load(self, sid: int):
        self.write_cmd(sid, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 1)

    def cleanup(self):
        if self.use_gpio:
            GPIO.cleanup()

# -------------------- High Level --------------------
class ArmPiPro:
    def __init__(self, port='/dev/serial0', baud=DEFAULT_BAUD, ids=None, use_gpio=True):
        self.bus = BusServo(port, baud, use_gpio=use_gpio)
        self.joints = ids if ids else JOINTS.copy()
        self.home_pose = self._load_home_file()

    # --- Basic ---
    def move_joint_deg(self, joint, angle_deg, duration_ms=DEFAULT_DURATION_MS):
        jname = self._name(joint)
        angle_deg = clamp(angle_deg, *SOFT_LIMITS[jname])
        pulse = deg_to_pulse(jname, angle_deg)
        self.move_joint_pulse(joint, pulse, duration_ms)

    def move_joint_pulse(self, joint, pulse, duration_ms=DEFAULT_DURATION_MS):
        self.bus.move_time_write(self._sid(joint), int(pulse), int(duration_ms))

    def move_pose_deg(self, angles, duration_ms=DEFAULT_DURATION_MS):
        if isinstance(angles, dict):
            for j, a in angles.items():
                self.move_joint_deg(j, a, duration_ms)
        else:
            for jname, a in zip(self.joints.keys(), angles):
                self.move_joint_deg(jname, a, duration_ms)

    def move_joint_pulses(self, pulses: Dict[str, int], duration_ms=DEFAULT_DURATION_MS):
        for j, p in pulses.items():
            self.move_joint_pulse(j, p, duration_ms)

    # --- Gripper ---
    def open_gripper(self, duration_ms=500):
        self.move_joint_pulse('gripper', GRIPPER_OPEN_PULSE, duration_ms)

    def close_gripper(self, duration_ms=500):
        self.move_joint_pulse('gripper', GRIPPER_CLOSE_PULSE, duration_ms)

    def grip_object(self, duration_ms=500):
        self.move_joint_pulse('gripper', GRIPPER_OBJECT_PULSE, duration_ms)

    def set_gripper_percent(self, percent: float, duration_ms=500):
        percent = clamp(percent, 0, 100)
        pulse = int(GRIPPER_OPEN_PULSE + (GRIPPER_CLOSE_PULSE - GRIPPER_OPEN_PULSE) * (percent/100.0))
        self.move_joint_pulse('gripper', pulse, duration_ms)

    # --- Wrist Roll ---
    def wrist_roll_zero(self, duration_ms=500):
        self.move_joint_pulse('wrist_roll', WRIST_ROLL_ZERO_PULSE, duration_ms)

    def wrist_roll_left_max(self, duration_ms=500):
        self.move_joint_pulse('wrist_roll', WRIST_ROLL_LEFT_MAX_PULSE, duration_ms)

    def wrist_roll_right_max(self, duration_ms=500):
        self.move_joint_pulse('wrist_roll', WRIST_ROLL_RIGHT_MAX_PULSE, duration_ms)

    def wrist_roll_percent(self, percent: float, duration_ms=500):
        percent = clamp(percent, 0, 100)
        pmin, pmax = JOINT_PERCENT_LIMITS['wrist_roll']
        pulse = int(pmin + (pmax - pmin) * (percent/100.0))
        self.move_joint_pulse('wrist_roll', pulse, duration_ms)

    # --- Wrist Pitch ---
    def wrist_pitch_zero(self, duration_ms=500):
        self.move_joint_pulse('wrist_pitch', WRIST_PITCH_ZERO_PULSE, duration_ms)

    def wrist_pitch_percent(self, percent: float, duration_ms=500):
        percent = clamp(percent, 0, 100)
        pmin, pmax = JOINT_PERCENT_LIMITS['wrist_pitch']
        pulse = int(pmin + (pmax - pmin) * (percent/100.0))
        self.move_joint_pulse('wrist_pitch', pulse, duration_ms)

    # --- Elbow ---
    def elbow_zero(self, duration_ms=500):
        self.move_joint_pulse('elbow', ELBOW_ZERO_PULSE, duration_ms)

    def elbow_percent(self, percent: float, duration_ms=500):
        percent = clamp(percent, 0, 100)
        pmin, pmax = JOINT_PERCENT_LIMITS['elbow']
        pulse = int(pmin + (pmax - pmin) * (percent/100.0))
        self.move_joint_pulse('elbow', pulse, duration_ms)

    # --- Shoulder ---
    def shoulder_zero(self, duration_ms=500):
        self.move_joint_pulse('shoulder', SHOULDER_ZERO_PULSE, duration_ms)

    def shoulder_percent(self, percent: float, duration_ms=500):
        percent = clamp(percent, 0, 100)
        pmin, pmax = JOINT_PERCENT_LIMITS['shoulder']
        pulse = int(pmin + (pmax - pmin) * (percent/100.0))
        self.move_joint_pulse('shoulder', pulse, duration_ms)

    # --- Base ---
    def base_zero(self, duration_ms=600):
        self.move_joint_pulse('base', BASE_ZERO_PULSE, duration_ms)

    def base_left_90(self, duration_ms=700):
        self.move_joint_pulse('base', BASE_LEFT_90_PULSE, duration_ms)

    def base_right_90(self, duration_ms=700):
        self.move_joint_pulse('base', BASE_RIGHT_90_PULSE, duration_ms)

    def base_left_45(self, duration_ms=600):
        self.move_joint_pulse('base', BASE_LEFT_45_PULSE, duration_ms)

    def base_right_45(self, duration_ms=600):
        self.move_joint_pulse('base', BASE_RIGHT_45_PULSE, duration_ms)

    def base_left_max(self, duration_ms=800):
        self.move_joint_pulse('base', BASE_LEFT_MAX_PULSE, duration_ms)

    def base_right_max(self, duration_ms=800):
        self.move_joint_pulse('base', BASE_RIGHT_MAX_PULSE, duration_ms)

    def base_percent(self, percent: float, duration_ms=700):
        percent = clamp(percent, 0, 100)
        pmin, pmax = JOINT_PERCENT_LIMITS['base']
        pulse = int(pmin + (pmax - pmin) * (percent/100.0))
        self.move_joint_pulse('base', pulse, duration_ms)

    # --- Readback ---
    def read_joint_pulse(self, joint):
        return self.bus.read_pos(self._sid(joint))

    def read_joint_deg(self, joint):
        raw = self.read_joint_pulse(joint)
        return None if raw is None else pulse_to_deg(self._name(joint), raw)

    # --- Power ---
    def stop_all(self):
        self.bus.stop(None)

    def unload_all(self):
        for sid in self.joints.values():
            self.bus.unload(sid)

    def load_all(self):
        for sid in self.joints.values():
            self.bus.load(sid)

    def unload_joint(self, joint):
        self.bus.unload(self._sid(joint))

    def load_joint(self, joint):
        self.bus.load(self._sid(joint))

    def cleanup(self):
        self.bus.cleanup()

    # --- Helpers ---
    def _sid(self, joint: JointKey) -> int:
        if isinstance(joint, int):
            return joint
        return self.joints[joint]

    def _name(self, joint: JointKey) -> str:
        if isinstance(joint, str):
            return joint
        for k, v in self.joints.items():
            if v == joint:
                return k
        return str(joint)

    # --- Relative Moves ---
    def _rel_move_joint_deg(self, joint, delta_deg, duration_ms=DEFAULT_DURATION_MS):
        jname = self._name(joint)
        cur = self.read_joint_deg(joint)
        if cur is None:
            lo, hi = SOFT_LIMITS[jname]
            cur = (lo + hi) / 2.0
        self.move_joint_deg(joint, cur + DIR_SIGN.get(jname, 1) * delta_deg, duration_ms)

    def turn_left(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('base', +abs(deg), duration_ms)

    def turn_right(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('base', -abs(deg), duration_ms)

    def shoulder_up(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('shoulder', +abs(deg), duration_ms)

    def shoulder_down(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('shoulder', -abs(deg), duration_ms)

    def elbow_up(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('elbow', +abs(deg), duration_ms)

    def elbow_down(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('elbow', -abs(deg), duration_ms)

    def wrist_pitch_up(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('wrist_pitch', +abs(deg), duration_ms)

    def wrist_pitch_down(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('wrist_pitch', -abs(deg), duration_ms)

    def wrist_roll_left(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('wrist_roll', +abs(deg), duration_ms)

    def wrist_roll_right(self, deg, duration_ms=DEFAULT_DURATION_MS):
        self._rel_move_joint_deg('wrist_roll', -abs(deg), duration_ms)

    # --- Home ---
    def home(self, duration_ms=1200):
        # 1) In Home-Position fahren
        if self.home_pose:
            self.move_pose_deg(self.home_pose, duration_ms)
        else:
            mids = {j: (SOFT_LIMITS[j][0] + SOFT_LIMITS[j][1]) / 2 for j in self.joints}
            self.move_pose_deg(mids, duration_ms)
        # 2) danach kurz warten und Gripper schliessen
        time.sleep(duration_ms / 1000.0 + 0.2)
        self.close_gripper(400)

    def set_home_from_current(self, save=True, path=HOME_FILE):
        pose = {}
        for j in self.joints:
            ang = self.read_joint_deg(j)
            if ang is None:
                raw = self.read_joint_pulse(j)
                ang = pulse_to_deg(j, raw) if raw is not None else (SOFT_LIMITS[j][0] + SOFT_LIMITS[j][1]) / 2
            pose[j] = float(ang)
        self.home_pose = pose
        if save:
            with open(path, 'w') as f:
                json.dump(pose, f, indent=2)
        return pose

    def clear_home_file(self, path=HOME_FILE):
        if os.path.exists(path):
            os.remove(path)

    def _load_home_file(self, path=HOME_FILE):
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    d = json.load(f)
                return {k: float(v) for k, v in d.items() if k in JOINTS}
            except Exception:
                return None
        return None

    def factory_home(self, duration_ms=1200):
        mids = {}
        for j in self.joints:
            pulse = GRIPPER_OPEN_PULSE if j == 'gripper' else 500
            mids[j] = pulse_to_deg(j, pulse)
        self.move_pose_deg(mids, duration_ms)

    # --- Wave ---
    def wave(self, reps=5, amp=25, duration_ms=300, prep_shoulder=40, prep_elbow=60, settle_ms=600):
        self.shoulder_up(prep_shoulder, settle_ms)
        self.elbow_up(prep_elbow, settle_ms)
        time.sleep(settle_ms / 1000)
        for _ in range(reps):
            self.wrist_roll_left(amp, duration_ms)
            time.sleep(duration_ms / 1000)
            self.wrist_roll_right(amp, duration_ms)
            time.sleep(duration_ms / 1000)

    # --- Pickup only ---
    def pickup_object(self,
                      base_to_pick=0.0,
                      sh_down=18.0,
                      el_down=35.0,
                      lift=35.0,
                      dur_travel=1200,
                      dur_small=700,
                      grip_time=600,
                      grip_percent_pick=80.0,
                      final_close_percent=100.0):
        self.load_all()
        self.open_gripper(grip_time)
        time.sleep(grip_time / 1000)
        if base_to_pick != 0:
            (self.turn_left if base_to_pick > 0 else self.turn_right)(abs(base_to_pick), dur_travel)
            time.sleep(dur_travel / 1000)
        self.shoulder_down(sh_down, dur_small)
        self.elbow_down(el_down, dur_small)
        time.sleep(dur_small / 1000)
        self.set_gripper_percent(grip_percent_pick, grip_time)
        time.sleep(grip_time / 1000)
        self.elbow_up(lift, dur_small)
        self.shoulder_up(sh_down, dur_small)
        time.sleep(dur_small / 1000)
        self.set_gripper_percent(final_close_percent, grip_time)
        time.sleep(grip_time / 1000)

    # --- Pick & Place ---
    def getobject(self,
                  base_to_pick=0.0,
                  sh_down=18.0,
                  el_down=35.0,
                  lift=35.0,
                  base_to_place=-60.0,
                  place_sh_down=15.0,
                  place_el_down=25.0,
                  dur_travel=1200,
                  dur_small=700,
                  grip_time=600,
                  grip_percent_pick=80.0):
        self.load_all()
        self.open_gripper(grip_time)
        time.sleep(grip_time / 1000)
        if base_to_pick != 0:
            (self.turn_left if base_to_pick > 0 else self.turn_right)(abs(base_to_pick), dur_travel)
            time.sleep(dur_travel / 1000)
        self.shoulder_down(sh_down, dur_small)
        self.elbow_down(el_down, dur_small)
        time.sleep(dur_small / 1000)
        self.set_gripper_percent(grip_percent_pick, grip_time)
        time.sleep(grip_time / 1000)
        self.elbow_up(lift, dur_small)
        self.shoulder_up(sh_down, dur_small)
        time.sleep(dur_small / 1000)
        if base_to_place != 0:
            (self.turn_right if base_to_place < 0 else self.turn_left)(abs(base_to_place), dur_travel)
            time.sleep(dur_travel / 1000)
        self.shoulder_down(place_sh_down, dur_small)
        self.elbow_down(place_el_down, dur_small)
        time.sleep(dur_small / 1000)
        self.open_gripper(grip_time)
        time.sleep(grip_time / 1000)
        self.elbow_up(place_el_down, dur_small)
        self.shoulder_up(place_sh_down, dur_small)
        time.sleep(dur_small / 1000)

    # --- Postures ---
    def list_poses(self):
        return list(POSTURES.keys())

    def apply_posture(self, name: str, duration_ms=800):
        if name not in POSTURES:
            raise ValueError(f"Posture '{name}' nicht gefunden.")
        pose = POSTURES[name]
        closed_mode = name.endswith('_closed')

        # Vor-Handling Gripper
        if closed_mode:
            self.close_gripper(400); time.sleep(0.4)
        else:
            if 'gripper' in pose and pose['gripper'] == GRIPPER_OPEN_PULSE:
                self.open_gripper(400); time.sleep(0.4)

        # Base zuerst
        if 'base' in pose:
            self.move_joint_pulse('base', pose['base'], duration_ms)
            time.sleep(duration_ms / 1000.0 + 0.1)

        # restliche Gelenke (ohne gripper)
        pulses = {j: p for j, p in pose.items() if j not in ('base', 'gripper')}
        self.move_joint_pulses(pulses, duration_ms)
        time.sleep(duration_ms / 1000.0 + 0.1)

        # Nach-Handling Gripper
        if closed_mode:
            self.move_joint_pulse('gripper', GRIPPER_OPEN_PULSE, 400)
        else:
            if 'gripper' in pose:
                self.move_joint_pulse('gripper', pose['gripper'], 400)

# ---------- ActionGroup Player ----------
class ActionGroupPlayer:
    def __init__(self, arm: ArmPiPro, folder='/home/pi/ArmPi/ActionGroups'):
        self.arm = arm
        self.folder = folder
        self.running = False
        self.stop_flag = False

    def stop(self):
        self.stop_flag = True

    def play(self, name: str, times: int = 1):
        path = os.path.join(self.folder, name + '.d6a')
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        for _ in range(times if times > 0 else 10**9):
            if not self._play_once(path):
                break

    def _play_once(self, path: str) -> bool:
        self.stop_flag = False
        self.running = True
        conn = sql.connect(path)
        cu = conn.cursor()
        cu.execute('select * from ActionGroup')
        row = cu.fetchone()
        while row:
            if self.stop_flag:
                break
            duration = row[1]
            for idx, pulse in enumerate(row[2:], start=1):
                if pulse is None:
                    continue
                if idx in self.arm.joints.values():
                    self.arm.move_joint_pulse(idx, pulse, duration)
            time.sleep(duration / 1000.0)
            row = cu.fetchone()
        cu.close()
        conn.close()
        self.running = False
        return not self.stop_flag

# -------------------- CLI --------------------
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='ArmPi Pro Standalone (GPIO half-duplex)')
    parser.add_argument('port', nargs='?', default='/dev/serial0')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--no-gpio', action='store_true', help='kein GPIO-Umschalten (USB-TTL)')

    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--action', type=str)
    parser.add_argument('--times', type=int, default=1)

    # Home
    parser.add_argument('--home', action='store_true')
    parser.add_argument('--set-home', action='store_true')
    parser.add_argument('--clear-home', action='store_true')
    parser.add_argument('--factory-home', action='store_true')

    # Funktions-Flags
    parser.add_argument('--hello', action='store_true')
    parser.add_argument('--pickup', action='store_true')
    parser.add_argument('--getobject', action='store_true')
    parser.add_argument('--grip-object', action='store_true')

    # Extra Gripper Flags
    parser.add_argument('--gripper-open', action='store_true')
    parser.add_argument('--gripper-close', action='store_true')
    parser.add_argument('--gripper-object', action='store_true')

    # Wrist Roll
    parser.add_argument('--roll-zero', action='store_true')
    parser.add_argument('--roll-left-max', action='store_true')
    parser.add_argument('--roll-right-max', action='store_true')
    parser.add_argument('--roll-percent', type=float)

    # Wrist Pitch
    parser.add_argument('--pitch-zero', action='store_true')
    parser.add_argument('--pitch-percent', type=float)

    # Elbow
    parser.add_argument('--elbow-zero', action='store_true')
    parser.add_argument('--elbow-percent', type=float)

    # Shoulder
    parser.add_argument('--shoulder-zero', action='store_true')
    parser.add_argument('--shoulder-percent', type=float)

    # Base
    parser.add_argument('--base-zero', action='store_true')
    parser.add_argument('--base-left-90', action='store_true')
    parser.add_argument('--base-right-90', action='store_true')
    parser.add_argument('--base-left-45', action='store_true')
    parser.add_argument('--base-right-45', action='store_true')
    parser.add_argument('--base-left-max', action='store_true')
    parser.add_argument('--base-right-max', action='store_true')
    parser.add_argument('--base-percent', type=float)

    # Postures
    parser.add_argument('--pose', type=str)
    parser.add_argument('--list-poses', action='store_true')
    parser.add_argument('--pose-list', action='store_true', dest='list_poses')
    parser.add_argument('--list_poses', action='store_true', dest='list_poses')

    # Power
    parser.add_argument('--unload', action='store_true')
    parser.add_argument('--load', action='store_true')

    args = parser.parse_args()
    DEBUG = args.debug

    arm = ArmPiPro(args.port, args.baud, use_gpio=not args.no_gpio)
    print("Connected")

    try:
        # Home actions
        if args.clear_home:
            arm.clear_home_file()
            print("Home-Datei gelöscht.")
        elif args.factory_home:
            arm.factory_home()
        elif args.set_home:
            pose = arm.set_home_from_current(save=True)
            print("Home gespeichert:", pose)
        elif args.home:
            arm.home()

        # Gripper single actions
        elif args.gripper_open:
            arm.open_gripper()
        elif args.gripper_close:
            arm.close_gripper()
        elif args.gripper_object:
            arm.grip_object()

        # Funktions-Flags
        elif args.hello:
            arm.wave()
        elif args.pickup:
            arm.pickup_object()
        elif args.getobject:
            arm.getobject()
        elif args.grip_object:
            arm.grip_object()

        # Wrist Roll
        elif args.roll_zero:
            arm.wrist_roll_zero()
        elif args.roll_left_max:
            arm.wrist_roll_left_max()
        elif args.roll_right_max:
            arm.wrist_roll_right_max()
        elif args.roll_percent is not None:
            arm.wrist_roll_percent(args.roll_percent)

        # Wrist Pitch
        elif args.pitch_zero:
            arm.wrist_pitch_zero()
        elif args.pitch_percent is not None:
            arm.wrist_pitch_percent(args.pitch_percent)

        # Elbow
        elif args.elbow_zero:
            arm.elbow_zero()
        elif args.elbow_percent is not None:
            arm.elbow_percent(args.elbow_percent)

        # Shoulder
        elif args.shoulder_zero:
            arm.shoulder_zero()
        elif args.shoulder_percent is not None:
            arm.shoulder_percent(args.shoulder_percent)

        # Base
        elif args.base_zero:
            arm.base_zero()
        elif args.base_left_90:
            arm.base_left_90()
        elif args.base_right_90:
            arm.base_right_90()
        elif args.base_left_45:
            arm.base_left_45()
        elif args.base_right_45:
            arm.base_right_45()
        elif args.base_left_max:
            arm.base_left_max()
        elif args.base_right_max:
            arm.base_right_max()
        elif args.base_percent is not None:
            arm.base_percent(args.base_percent)

        # Postures
        elif args.list_poses:
            print("Verfügbare Posen:")
            for n in arm.list_poses():
                print("  ", n)
        elif args.pose:
            arm.apply_posture(args.pose)

        # Power
        elif args.unload:
            arm.unload_all()
            print("Servos unloaded.")
        elif args.load:
            arm.load_all()
            print("Servos loaded.")

        # ActionGroup
        elif args.action:
            ActionGroupPlayer(arm).play(args.action, args.times)

        # Demo
        elif args.demo:
            print("Base -> 120°")
            arm.move_joint_deg('base', 120, 1200)
            time.sleep(1.2)
            print("Shoulder up/down")
            arm.shoulder_up(30, 800)
            time.sleep(0.8)
            arm.shoulder_down(30, 800)
            time.sleep(0.8)
            print("Open gripper")
            arm.open_gripper(600)
            time.sleep(0.6)
            print("Done")
        else:
            print("Options: --home --set-home --clear-home --factory-home "
                  "--hello --pickup --getobject --grip-object "
                  "--gripper-open --gripper-close --gripper-object "
                  "--roll-zero --roll-left-max --roll-right-max --roll-percent X "
                  "--pitch-zero --pitch-percent X "
                  "--elbow-zero --elbow-percent X "
                  "--shoulder-zero --shoulder-percent X "
                  "--base-zero --base-left-90 --base-right-90 --base-left-45 --base-right-45 "
                  "--base-left-max --base-right-max --base-percent X "
                  "--pose NAME --list-poses "
                  "--unload --load --demo --action NAME")
    finally:
        arm.cleanup()
