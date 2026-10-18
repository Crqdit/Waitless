#!/usr/bin/env python3
"""
Smart Crossing Simulator - version 2 (two in-road sensors, switchable fixes)
===========================================================================

A T-junction with two zebra crossings:

  Crossing A - across the main road.
  Crossing B - across the side road. Westbound cars turning right into the
               side road must cross it.

Turning and straight cars share the westbound lane. Two in-road sensors
(blue on the map) watch the junction:
  sensor 1 - in the westbound lane at the junction mouth
  sensor 2 - in the side road, between the junction and crossing B

When crossing B's countdown ends, cars already on the sensors are "committed".
Depending on the hold rule, B's walk signal waits until those cars are through:
straight cars just leave sensor 1 (wait-and-see), turners pass crossing B.

Fixes you can switch on:
  1. Move crossing B two car lengths up the side road
  2. End B's walk early once the crossing is empty
  3. Line up B's walk with A's walk (main road is stopped then)
  4. Wait for a gap in traffic, using an upstream sensor (sensor 0)

Run the window:      python crossing_sim.py
Compare every fix:   python crossing_sim.py --compare               (30 sim minutes each)
                     python crossing_sim.py --compare 60 25 40 8    (minutes, cars/min each
                                                     way, right-turn %, people/min)

Results are appended to results_log.csv and the comparison is written to
comparison_report.txt, both next to this file.

Needs only standard Python 3 (tkinter comes with it).
"""

import bisect
import csv
import datetime
import math
import os
import random
import sys
import time

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:          # lets --compare run without a display
    tk = ttk = None

# =====================================================================
# SETTINGS - change these to experiment
# =====================================================================
COUNTDOWN_START = 25.0      # countdown before anyone is counted (s)
SECONDS_PER_PERSON = 2.0    # countdown gets this much shorter per person (s)
COUNTDOWN_MIN = 10.0        # countdown never goes below this (s)
MIN_CAR_GREEN = 12.0        # cars always get at least this much green between walks (s)
AMBER_TIME = 3.0
ALL_RED_TIME = 1.0
WALK_TIME = 8.0
CLEAR_TIME = 4.0            # flashing walk: finish crossing, nobody new starts

YOUR_RULE_TOTAL_CAP = 60.0  # "60 s total cap": max total wait, from the first person
SHORT_HOLD_CAP = 12.0       # "12 s cap": max hold after the countdown ends

# fix settings
SETBACK_CAR_LENGTHS = 2     # fix 1: how far crossing B moves up the side road
MIN_WALK_B = 5.0            # fix 2: shortest walk phase at B (s)
SHORT_CLEAR = 1.0           # fix 2: clearance when the crossing is confirmed empty (s)
PAIR_MAX_WAIT = 5.0         # fix 3: B waits for A's walk only if it's due within this (s)
GAP_NEEDED = 2.5            # fix 4: no car on sensor 0 for this long counts as a gap (s)
GAP_MAX_WAIT = 4.0          # fix 4: longest wait for a gap (s)

# fix 5 (Claude's fix): coordinated, storage-aware timing
PULL_A_WITHIN = 15.0        # pull A's walk forward to share B's if A is due within this (s)
SPILL_RISK_OK = 0.10        # walk B alone only if the chance of spill-back is below this
SMART_MAX_WAIT = 8.0        # longest B will wait for a safer moment (s)
TURN_RATE_WINDOW = 180.0    # turning rate is learned from the last this-many seconds
TURN_RATE_PRIOR = 0.07      # starting guess, turners per second (about 4 per minute)

DWELL_TIME = 1.0            # must stand on a plate this long to be counted (s)
MIN_OBJECT_KG = 15.0        # lighter clusters are ignored (bags, dogs...)
KG_PER_PERSON_MIN = 20.0    # a cluster can't count as more people than kg / this
FOOT_PAIR_KG = (40.0, 250.0)  # two lone "feet" one tile apart in this range = one person
IMPATIENT_AFTER = 40.0      # people waiting longer than this often cross on red
BREAKDOWN_TIME = 45.0       # how long a broken-down car blocks crossing B (s)

CAR_SPEED = 80.0            # px per second
CAR_ACCEL = 120.0
CAR_LEN, CAR_WID, CAR_GAP = 26, 14, 8
PED_SPEED = 30.0

COMPARE_MINUTES = 30        # default length of each comparison run
COMPARE_SEED = 12345
COMPARE_BREAKDOWN_AT = 600.0  # one breakdown in every comparison run, at this time (s)

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_CSV = os.path.join(HERE, "results_log.csv")
REPORT_TXT = os.path.join(HERE, "comparison_report.txt")

# =====================================================================
# FIXED GEOMETRY (pixels)
# =====================================================================
W, H = 960, 600
ROAD_N, LANE_WB_Y, LANE_DIV_Y, LANE_EB_Y, ROAD_S = 300, 320, 340, 360, 380
SIDE_W_X, SIDE_E_X, SIDE_MID_X, SIDE_NB_X = 400, 520, 460, 490
CROSS_A_X0, CROSS_A_X1 = 640, 680
STOP_A_WB_X, STOP_A_EB_X = 688, 632
S1_X0, S1_X1 = 520, 620     # sensor 1, westbound lane
S0_X0, S0_X1 = 860, 920     # sensor 0 (fix 4), westbound lane upstream
HALF = CAR_LEN / 2
TILE_COLS, TILE_ROWS, TILE_PX = 6, 4, 11

MODES = ["No hold", "Hold, 60 s total cap", "Hold, 12 s cap"]
FIX_NAMES = {
    "fix1": "Move crossing B back 2 car lengths",
    "fix2": "End B's walk early when empty",
    "fix3": "Line up B's walk with A's",
    "fix4": "Wait for a traffic gap (sensor 0)",
    "fix5": "Claude's fix: coordinated, storage-aware",
}


# =====================================================================
# PATHS - cars move along these; s = distance travelled
# =====================================================================
class Path:
    def __init__(self, pts, step=2.0):
        dense = [pts[0]]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
            for i in range(1, n + 1):
                t = i / n
                dense.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        self.pts = dense
        self.cum = [0.0]
        for a, b in zip(dense, dense[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.cum[-1]

    def pos(self, s):
        s = max(0.0, min(s, self.length))
        i = min(bisect.bisect_right(self.cum, s) - 1, len(self.pts) - 2)
        seg = (self.cum[i + 1] - self.cum[i]) or 1e-9
        t = (s - self.cum[i]) / seg
        (x0, y0), (x1, y1) = self.pts[i], self.pts[i + 1]
        return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, math.atan2(y1 - y0, x1 - x0)

    def s_at(self, pred):
        for p, c in zip(self.pts, self.cum):
            if pred(p):
                return c
        return self.length


def arc_points(cx, cy, r, a0, a1, steps=18):
    out = []
    for i in range(steps + 1):
        a = math.radians(a0 + (a1 - a0) * i / steps)
        out.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return out


class Geometry:
    """Everything that moves when crossing B is set back (fix 1)."""

    def __init__(self, setback):
        self.setback = setback
        self.cross_b_y0 = 190 - setback
        self.cross_b_y1 = 230 - setback
        self.stop_b_y = 238 - setback

        self.path_wb = Path([(W + 40, LANE_WB_Y), (-40, LANE_WB_Y)])
        self.path_eb = Path([(-40, LANE_EB_Y), (W + 40, LANE_EB_Y)])
        self.path_turn = Path([(W + 40, LANE_WB_Y), (550, LANE_WB_Y)]
                              + arc_points(550, 260, 60, 90, 180)[1:] + [(SIDE_NB_X, -40)])
        pw, pe, pt = self.path_wb, self.path_eb, self.path_turn
        side = lambda p: p[0] < SIDE_E_X

        self.s_wb_stop_a = pw.s_at(lambda p: p[0] <= STOP_A_WB_X + HALF)
        self.s_eb_stop_a = pe.s_at(lambda p: p[0] >= STOP_A_EB_X - HALF)
        self.s_turn_stop_a = pt.s_at(lambda p: p[0] <= STOP_A_WB_X + HALF)
        self.s_arc_start = pt.s_at(lambda p: p[0] <= 550)
        self.s_turn_leave_wb = pt.s_at(lambda p: side(p) and p[1] <= ROAD_N - HALF)
        self.s_turn_stop_b = pt.s_at(lambda p: side(p) and p[1] <= self.stop_b_y + HALF)
        self.s_turn_clear_b = pt.s_at(lambda p: side(p) and p[1] <= self.cross_b_y0 - HALF)

        # sensors as ranges of s where the car body is over them
        self.s1_wb = (pw.s_at(lambda p: p[0] <= S1_X1 + HALF), pw.s_at(lambda p: p[0] <= S1_X0 - HALF))
        self.s1_turn = (pt.s_at(lambda p: p[0] <= S1_X1 + HALF), self.s_turn_leave_wb)
        self.s2 = (pt.s_at(side), pt.s_at(lambda p: side(p) and p[1] <= self.stop_b_y - HALF))
        # how many turners fit between the main road and crossing B
        self.storage_cap = int((self.s_turn_stop_b - self.s_turn_leave_wb) // (CAR_LEN + CAR_GAP)) + 1
        self.s0 = (pw.s_at(lambda p: p[0] <= S0_X1 + HALF), pw.s_at(lambda p: p[0] <= S0_X0 - HALF))


# =====================================================================
# PEOPLE AND PRESSURE PLATES
# =====================================================================
FOOTPRINTS = {   # kind: (tiles relative to anchor, mean kg, +/- kg)
    "Adult": ([(0, 0), (1, 0)], 75, 12),
    "Child": ([(0, 0)], 25, 6),
    "Heavy adult": ([(0, 0), (2, 0)], 150, 25),
    "Bag / object": ([(0, 0)], 8, 3),
}


class Person:
    def __init__(self, kind, plate, tiles, t, rng):
        _, kg, spread = FOOTPRINTS[kind]
        self.kind, self.plate, self.tiles = kind, plate, tiles
        self.weight = rng.uniform(kg - spread, kg + spread)
        self.arrive = t
        self.is_person = kind != "Bag / object"
        centres = [plate.tile_center(c, r) for c, r in tiles]
        self.x = sum(p[0] for p in centres) / len(centres)
        self.y = sum(p[1] for p in centres) / len(centres)
        self.tx = self.ty = None
        self.cross = plate.crossing


class Plate:
    def __init__(self, name, crossing, x0, y0, dest):
        self.name, self.crossing = name, crossing
        self.x0, self.y0 = x0, y0
        self.x1, self.y1 = x0 + TILE_COLS * TILE_PX, y0 + TILE_ROWS * TILE_PX
        self.dest = dest
        self.people = []

    def contains(self, x, y):
        return self.x0 <= x < self.x1 and self.y0 <= y < self.y1

    def tile_at(self, x, y):
        return int((x - self.x0) // TILE_PX), int((y - self.y0) // TILE_PX)

    def tile_center(self, c, r):
        return self.x0 + (c + 0.5) * TILE_PX, self.y0 + (r + 0.5) * TILE_PX

    def load(self, t=None):
        out = {}
        for p in self.people:
            if t is not None and t - p.arrive < DWELL_TIME:
                continue
            for tile in p.tiles:
                out[tile] = out.get(tile, 0.0) + p.weight / len(p.tiles)
        return out

    def place(self, kind, t, rng, anchor=None):
        shape = FOOTPRINTS[kind][0]
        used = {tile for p in self.people for tile in p.tiles}
        spots = []
        for r in range(TILE_ROWS):
            for c in range(TILE_COLS):
                tiles = [(c + dc, r + dr) for dc, dr in shape]
                if all(0 <= x < TILE_COLS and 0 <= y < TILE_ROWS and (x, y) not in used
                       for x, y in tiles):
                    spots.append(tiles)
        if not spots:
            return False
        if anchor:
            spots.sort(key=lambda ts: abs(ts[0][0] - anchor[0]) + abs(ts[0][1] - anchor[1]))
            tiles = spots[0]
        else:
            tiles = rng.choice(spots)
        self.people.append(Person(kind, self, tiles, t, rng))
        return True


def detect_count(plate, t):
    """Tiles -> clusters -> ignore light ones -> estimate -> merge split feet."""
    load = plate.load(t)
    seen, clusters = set(), []
    for start in load:
        if start in seen:
            continue
        seen.add(start)
        stack, tiles = [start], []
        while stack:
            c, r = stack.pop()
            tiles.append((c, r))
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    nb = (c + dc, r + dr)
                    if nb in load and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        clusters.append((tiles, sum(load[x] for x in tiles)))
    clusters = [cl for cl in clusters if cl[1] >= MIN_OBJECT_KG]
    count = 0
    for tiles, kg in clusters:
        count += min(max(1, int(len(tiles) / 2 + 0.5)), max(1, int(kg // KG_PER_PERSON_MIN)))
    singles = [cl for cl in clusters if len(cl[0]) == 1]
    used = set()
    for i in range(len(singles)):
        for j in range(i + 1, len(singles)):
            if i in used or j in used:
                continue
            (a,), (b,) = singles[i][0], singles[j][0]
            if (max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 2
                    and FOOT_PAIR_KG[0] <= singles[i][1] + singles[j][1] <= FOOT_PAIR_KG[1]):
                used |= {i, j}
                count -= 1
    return count


def countdown_for(n):
    return max(COUNTDOWN_MIN, COUNTDOWN_START - SECONDS_PER_PERSON * n)


# =====================================================================
# CARS
# =====================================================================
class Car:
    next_id = 0

    def __init__(self, kind, t, geo):
        Car.next_id += 1
        self.id = Car.next_id
        self.kind = kind                      # "wb", "turn" or "eb"
        self.path = {"wb": geo.path_wb, "turn": geo.path_turn, "eb": geo.path_eb}[kind]
        self.s = 0.0
        self.speed = CAR_SPEED
        self.spawn_t = t
        self.broken_until = -1.0
        self.will_break = False
        self.was_blocked = False
        self.seen_s2 = False

    def occupies(self, geo):
        if self.kind == "eb":
            return {"EB"}
        if self.kind == "wb" or self.s < geo.s_arc_start:
            return {"WB"}
        if self.s < geo.s_turn_leave_wb:
            return {"WB", "SIDE"}         # mid-turn: still sticking out into the lane
        return {"SIDE"}

    def watches(self, geo):
        if self.kind == "turn":
            return {"WB", "SIDE"} if self.s < geo.s_turn_leave_wb else {"SIDE"}
        return self.occupies(geo)


class Crossing:
    def __init__(self, name):
        self.name = name
        self.state = "GREEN"
        self.t_state = 0.0
        self.wait_start = None
        self.last_end = -1e9
        self.detected = 0
        self.actual = 0
        self.committed = set()
        self.short_clear = False

    def go(self, state, t):
        self.state, self.t_state = state, t


# =====================================================================
# THE SIMULATION (no drawing here)
# =====================================================================
DEFAULT_CONFIG = dict(mode=2, fix1=False, fix2=False, fix3=False, fix4=False, fix5=False,
                      auto_cars=True, car_rate=12, turn_pct=35,
                      auto_people=True, people_rate=6)


class Sim:
    def __init__(self, config=None, seed=None):
        self.cfg = dict(DEFAULT_CONFIG)
        if config:
            self.cfg.update(config)
        self.reset(seed)

    def reset(self, seed=None):
        self.seed = seed if seed is not None else random.randrange(10 ** 6)
        self.rng_arrivals = random.Random(self.seed)       # same traffic for every setup
        self.rng = random.Random(self.seed + 1)
        self.geo = Geometry(SETBACK_CAR_LENGTHS * (CAR_LEN + CAR_GAP) if self.cfg["fix1"] else 0)
        g = self.geo
        self.t = 0.0
        self.cars, self.walkers = [], []
        self.pending = {"west": [], "east": []}
        self.plates = [
            Plate("A north", "A", 627, 254, (None, 440)),
            Plate("A south", "A", 627, 384, (None, 296)),
            Plate("B west", "B", 330, g.cross_b_y0 - 2, (545, None)),
            Plate("B east", "B", 524, g.cross_b_y0 - 2, (375, None)),
        ]
        self.A, self.B = Crossing("A"), Crossing("B")
        self.last_s0 = -1e9
        self.turn_log = []          # times turners reached sensor 2 (fix 5 learns from this)
        self.st = dict(peds_A=0, wait_A=0.0, peds_B=0, wait_B=0.0, ped_max=0.0, impatient=0,
                       cars_wb=0, delay_wb=0.0, cars_turn=0, delay_turn=0.0,
                       cars_eb=0, delay_eb=0.0, car_max=0.0,
                       blocked_s=0.0, blocked_cars=0,
                       holds=0, holds_false=0, hold_time=0.0, hold_max=0.0,
                       walks_B=0, walk_time_B=0.0, main_red=0.0,
                       walks_A=0, merged=0, spill_cuts=0)

    # ---------- adding things ----------
    def add_car(self, kind, breaks=False):
        self.pending["east" if kind == "eb" else "west"].append((kind, breaks))

    def add_person(self, plate, kind, anchor=None):
        return plate.place(kind, self.t, self.rng, anchor)

    def breakdown(self):
        self.add_car("turn", breaks=True)    # this car will stall at crossing B

    def plates_of(self, name):
        return [p for p in self.plates if p.crossing == name]

    # ---------- sensors ----------
    def on_s1(self):
        g = self.geo
        return [c for c in self.cars if (c.kind == "wb" and g.s1_wb[0] <= c.s <= g.s1_wb[1])
                or (c.kind == "turn" and g.s1_turn[0] <= c.s <= g.s1_turn[1])]

    def on_s2(self):
        g = self.geo
        return [c for c in self.cars if c.kind == "turn" and g.s2[0] <= c.s <= g.s2[1]]

    def on_s0(self):
        g = self.geo
        return [c for c in self.cars if c.kind in ("wb", "turn") and g.s0[0] <= c.s <= g.s0[1]]

    def sensor_candidates(self):
        """Cars the junction sensors can see right now (sensor 1 through sensor 2)."""
        g = self.geo
        return {c.id for c in self.cars
                if (c.kind == "wb" and g.s1_wb[0] <= c.s <= g.s1_wb[1])
                or (c.kind == "turn" and g.s1_turn[0] <= c.s <= g.s2[1])}

    # ---------- signals ----------
    def light_a(self):
        return {"GREEN": "g", "AMBER": "y"}.get(self.A.state, "r")

    def light_b(self):
        if self.B.state in ("ALLRED", "WALK", "CLEAR"):
            return "r"
        return "y" if self.B.state == "AMBER" else "g"

    def go_time(self, cr):
        return max(cr.wait_start + countdown_for(cr.detected), cr.last_end + MIN_CAR_GREEN)

    # ---------- fix 5: coordinated, storage-aware timing ----------
    def storage_count(self):
        g = self.geo
        return sum(1 for c in self.cars if c.kind == "turn" and g.s_arc_start <= c.s < g.s_turn_clear_b)

    def turn_rate(self, t):
        recent = sum(1 for x in self.turn_log if x > t - TURN_RATE_WINDOW)
        window = min(TURN_RATE_WINDOW, t)
        prior_s = 30.0                     # the starting guess counts as 30 s of data
        return (recent + TURN_RATE_PRIOR * prior_s) / (window + prior_s)

    def spill_risk(self, t):
        """Chance that more turners arrive during B's red than the side road can hold."""
        room = self.geo.storage_cap - self.storage_count()
        if room < 0:
            return 1.0
        expected_red = AMBER_TIME + ALL_RED_TIME + MIN_WALK_B + SHORT_CLEAR + 2.0
        lam = self.turn_rate(t) * expected_red
        p_ok = sum(math.exp(-lam) * lam ** k / math.factorial(k) for k in range(room + 1))
        return 1.0 - p_ok

    def smart_decide(self, t, waited):
        """Returns True when B should start now. May pull A's walk forward to share it."""
        A = self.A
        if A.state in ("AMBER", "ALLRED"):
            return True                    # A's walk is starting anyway: join it for free
        if A.state == "GREEN" and A.wait_start is not None:
            due = self.go_time(A) - t
            if due <= PULL_A_WITHIN:
                if t >= A.last_end + MIN_CAR_GREEN:
                    self.start_a(t, join=False)   # one shared stop instead of two
                    self.st["merged"] += 1
                    return True
                if waited < SMART_MAX_WAIT:
                    return False           # A becomes free to go in a moment: wait for it
        if self.spill_risk(t) <= SPILL_RISK_OK:
            return True
        if self.storage_count() == 0 and not self.on_s1():
            return True                    # cheapest moment: nothing in or entering the turn
        return waited >= SMART_MAX_WAIT

    def start_a(self, t, join=True):
        A, B = self.A, self.B
        if A.wait_start is None:
            A.wait_start = t
        A.go("AMBER", t)
        self.st["walks_A"] += 1
        joiners = self.cfg["fix3"] or self.cfg["fix5"]
        if join and joiners and B.state in ("GREEN", "SMARTWAIT", "PAIRWAIT") and B.detected > 0:
            if B.wait_start is None:
                B.wait_start = t
            if self.cfg["fix5"]:
                self.b_check(t)
            elif B.state == "GREEN":
                self.b_after_pair(t)

    def b_begin(self, t):
        A, B = self.A, self.B
        if self.cfg["fix5"]:
            if self.smart_decide(t, 0.0):
                self.b_check(t)
            else:
                B.go("SMARTWAIT", t)
            return
        if (self.cfg["fix3"] and A.state == "GREEN" and A.wait_start is not None
                and self.go_time(A) - t <= PAIR_MAX_WAIT):
            B.go("PAIRWAIT", t)
        else:
            self.b_after_pair(t)

    def b_after_pair(self, t):
        if self.cfg["fix4"]:
            self.B.go("GAPWAIT", t)
        else:
            self.b_check(t)

    def b_check(self, t):
        B = self.B
        cands = self.sensor_candidates()
        if self.cfg["mode"] == 0 or not cands:
            B.go("AMBER", t)
            return
        B.committed = cands
        B.go("HOLD", t)
        self.st["holds"] += 1
        alive = {c.id: c for c in self.cars}
        if not any(alive[i].kind == "turn" for i in cands if i in alive):
            self.st["holds_false"] += 1      # only straight cars: wait-and-see

    def hold_done(self, t):
        B, g = self.B, self.geo
        alive = {c.id: c for c in self.cars}
        left = set()
        for i in B.committed:
            c = alive.get(i)
            if c is None:
                continue
            if c.kind == "turn" and c.s > g.s_turn_clear_b:
                continue
            if c.kind == "wb" and c.s > g.s1_wb[1]:
                continue
            left.add(i)
        B.committed = left
        if not left:
            return True
        if self.cfg["mode"] == 1:
            return t - (B.wait_start or B.t_state) >= YOUR_RULE_TOTAL_CAP
        return t - B.t_state >= SHORT_HOLD_CAP

    def gap_ok(self, t):
        return not self.on_s0() and t - self.last_s0 >= GAP_NEEDED

    def update_signals(self, t):
        A, B, cfg = self.A, self.B, self.cfg

        # ----- crossing A (main road) -----
        el = t - A.t_state
        if A.state == "GREEN":
            if A.detected > 0:
                if A.wait_start is None:
                    A.wait_start = t
                if t >= self.go_time(A):
                    self.start_a(t)               # B may join (fix 3 / fix 5)
            else:
                A.wait_start = None
        elif A.state == "AMBER" and el >= AMBER_TIME:
            A.go("ALLRED", t)
        elif A.state == "ALLRED" and el >= ALL_RED_TIME:
            A.go("WALK", t)
        elif A.state == "WALK" and el >= WALK_TIME:
            A.go("CLEAR", t)
        elif A.state == "CLEAR" and el >= CLEAR_TIME:
            A.go("GREEN", t)
            A.last_end, A.wait_start = t, None

        # ----- crossing B (side road) -----
        el = t - B.t_state
        if B.state == "GREEN":
            if B.detected > 0:
                if B.wait_start is None:
                    B.wait_start = t
                if t >= self.go_time(B):
                    self.b_begin(t)
            else:
                B.wait_start = None
        elif B.state == "SMARTWAIT":
            if self.smart_decide(t, el):
                self.b_check(t)
        elif B.state == "PAIRWAIT":
            if A.state in ("AMBER", "ALLRED", "WALK") or el >= PAIR_MAX_WAIT:
                self.b_after_pair(t)
        elif B.state == "GAPWAIT":
            if self.gap_ok(t) or el >= GAP_MAX_WAIT:
                self.b_check(t)
        elif B.state == "HOLD":
            if self.hold_done(t):
                self.st["hold_time"] += el
                self.st["hold_max"] = max(self.st["hold_max"], el)
                B.go("AMBER", t)
        elif B.state == "AMBER" and el >= AMBER_TIME:
            B.go("ALLRED", t)
        elif B.state == "ALLRED" and el >= ALL_RED_TIME:
            B.go("WALK", t)
        elif B.state == "WALK":
            empty = (not any(p.is_person for pl in self.plates_of("B") for p in pl.people)
                     and not any(w.cross == "B" for w in self.walkers))
            if (cfg["fix2"] or cfg["fix5"]) and el >= MIN_WALK_B and empty:
                B.short_clear = True
            elif cfg["fix5"] and el >= MIN_WALK_B and self.storage_count() >= self.geo.storage_cap:
                B.short_clear = False          # side road is full: stop new walkers now
                self.st["spill_cuts"] += 1
            elif el >= WALK_TIME:
                B.short_clear = False
            else:
                return
            self.st["walks_B"] += 1
            self.st["walk_time_B"] += el
            B.go("CLEAR", t)
        elif B.state == "CLEAR" and el >= (SHORT_CLEAR if B.short_clear else CLEAR_TIME):
            B.go("GREEN", t)
            B.last_end, B.wait_start = t, None

    # ---------- main step ----------
    def step(self, dt):
        self.t += dt
        t = self.t
        self.auto_spawn(dt)
        if self.on_s0():
            self.last_s0 = t

        for cr in (self.A, self.B):
            plates = self.plates_of(cr.name)
            cr.detected = sum(detect_count(p, t) for p in plates)
            cr.actual = sum(1 for p in plates for q in p.people if q.is_person)
        self.update_signals(t)
        if self.light_a() != "g":
            self.st["main_red"] += dt

        for cr in (self.A, self.B):
            if cr.state != "WALK":
                continue
            for plate in self.plates_of(cr.name):
                for p in plate.people:
                    if not p.is_person:
                        continue
                    w = t - p.arrive
                    self.st["peds_" + cr.name] += 1
                    self.st["wait_" + cr.name] += w
                    self.st["ped_max"] = max(self.st["ped_max"], w)
                    if w > IMPATIENT_AFTER:
                        self.st["impatient"] += 1
                    if cr.name == "A":
                        p.tx, p.ty = self.rng.uniform(CROSS_A_X0 + 6, CROSS_A_X1 - 6), plate.dest[1]
                    else:
                        p.tx = plate.dest[0]
                        p.ty = self.rng.uniform(self.geo.cross_b_y0 + 6, self.geo.cross_b_y1 - 6)
                    self.walkers.append(p)
                plate.people = []

        for p in self.walkers[:]:
            dx, dy = p.tx - p.x, p.ty - p.y
            d = math.hypot(dx, dy)
            if d < PED_SPEED * dt:
                self.walkers.remove(p)
            else:
                p.x += dx / d * PED_SPEED * dt
                p.y += dy / d * PED_SPEED * dt

        self.spawn_pending()
        self.move_cars(dt, t)
        for c in self.cars:
            if c.kind == "turn" and not c.seen_s2 and c.s >= self.geo.s2[0]:
                c.seen_s2 = True
                self.turn_log.append(t)

    def auto_spawn(self, dt):
        r = self.rng_arrivals
        a, b, c, d, e, f = (r.random() for _ in range(6))   # always 6 draws: same traffic every run
        cfg = self.cfg
        if cfg["auto_cars"]:
            p = cfg["car_rate"] / 60.0 * dt
            if a < p:
                self.add_car("turn" if b < cfg["turn_pct"] / 100 else "wb")
            if c < p:
                self.add_car("eb")
        if cfg["auto_people"] and d < cfg["people_rate"] / 60.0 * dt:
            kinds = list(FOOTPRINTS)
            kind = kinds[0] if e < 0.70 else kinds[1] if e < 0.85 else kinds[2] if e < 0.95 else kinds[3]
            self.add_person(self.plates[min(3, int(f * 4))], kind)

    def spawn_pending(self):
        for side, queue in self.pending.items():
            if not queue:
                continue
            same = ("eb",) if side == "east" else ("wb", "turn")
            if all(c.s >= CAR_LEN + CAR_GAP + 4 for c in self.cars if c.kind in same):
                kind, breaks = queue.pop(0)
                car = Car(kind, self.t, self.geo)
                car.will_break = breaks
                self.cars.append(car)

    def stops_for(self, c):
        g = self.geo
        if c.kind == "wb":
            return [(g.s_wb_stop_a, self.light_a())]
        if c.kind == "eb":
            return [(g.s_eb_stop_a, self.light_a())]
        return [(g.s_turn_stop_a, self.light_a()), (g.s_turn_stop_b, self.light_b())]

    def move_cars(self, dt, t):
        g = self.geo
        la = self.light_a()
        occ = {c: c.occupies(g) for c in self.cars}
        limits = {}
        for c in self.cars:
            limit = c.path.length + 100
            looks = c.watches(g)
            leader = None
            for o in self.cars:
                if o is not c and o.s > c.s and (occ[o] & looks):
                    if leader is None or o.s < leader.s:
                        leader = o
            if leader:
                limit = min(limit, leader.s - CAR_LEN - CAR_GAP)
            for stop_s, light in self.stops_for(c):
                if c.s <= stop_s + 0.5:
                    if light == "r" or (light == "y" and stop_s - c.s > c.speed * 0.5 + 6):
                        limit = min(limit, stop_s)
            if c.will_break and c.s >= g.s_turn_stop_b - 1:
                c.broken_until = t + BREAKDOWN_TIME
                c.will_break = False
            if t < c.broken_until:
                limit = c.s
            # a straight car stuck behind a turning car although its own light is green
            if (c.kind == "wb" and c.speed < 1 and leader is not None and leader.kind == "turn"
                    and leader.speed < 1 and (la == "g" or c.s > g.s_wb_stop_a + 1)):
                self.st["blocked_s"] += dt
                c.was_blocked = True
            limits[c] = limit

        for c in self.cars[:]:
            limit = limits[c]
            target = min(CAR_SPEED, max(0.0, limit - c.s) * 2.5)
            c.speed = min(target, c.speed + CAR_ACCEL * dt) if target > c.speed else target
            c.s = min(c.s + c.speed * dt, max(c.s, limit))
            if c.s >= c.path.length - 2:
                self.cars.remove(c)
                delay = (t - c.spawn_t) - c.path.length / CAR_SPEED
                self.st["cars_" + c.kind] += 1
                self.st["delay_" + c.kind] += delay
                self.st["car_max"] = max(self.st["car_max"], delay)
                if c.was_blocked:
                    self.st["blocked_cars"] += 1

    # ---------- results ----------
    def metrics(self):
        s = self.st
        avg = lambda total, n: total / n if n else 0.0
        peds = s["peds_A"] + s["peds_B"]
        return dict(
            minutes=self.t / 60,
            ped_wait_A=avg(s["wait_A"], s["peds_A"]),
            ped_wait_B=avg(s["wait_B"], s["peds_B"]),
            ped_wait_max=s["ped_max"],
            ped_over40_pct=100 * avg(s["impatient"], peds),
            straight_delay=avg(s["delay_wb"], s["cars_wb"]),
            turn_delay=avg(s["delay_turn"], s["cars_turn"]),
            east_delay=avg(s["delay_eb"], s["cars_eb"]),
            car_delay_max=s["car_max"],
            blocked_car_s=s["blocked_s"],
            blocked_pct=100 * avg(s["blocked_cars"], s["cars_wb"]),
            holds=s["holds"],
            holds_straight_only=s["holds_false"],
            hold_avg=avg(s["hold_time"], s["holds"]),
            hold_max=s["hold_max"],
            walk_B_avg=avg(s["walk_time_B"], s["walks_B"]),
            main_red_pct=100 * avg(s["main_red"], self.t),
            main_stops_per_h=3600 * avg(s["walks_A"], self.t),
            people_crossed=peds,
            cars_through=s["cars_wb"] + s["cars_turn"] + s["cars_eb"],
        )

    def config_label(self):
        fixes = [k[-1] for k in ("fix1", "fix2", "fix3", "fix4", "fix5") if self.cfg[k]]
        return MODES[self.cfg["mode"]] + (" + fix " + ",".join(fixes) if fixes else ", no fixes")

    def status(self, cr):
        t, el = self.t, self.t - cr.t_state
        n = cr.name
        if cr.state == "GREEN":
            if cr.wait_start is None:
                return f"Crossing {n}: nobody waiting"
            return f"Crossing {n}: {cr.detected} counted, walk in {max(0, self.go_time(cr) - t):.0f} s"
        return {
            "PAIRWAIT": f"Crossing {n}: waiting to go with A ({el:.0f} s)",
            "SMARTWAIT": f"Crossing {n}: waiting for a safer moment ({el:.0f} s)",
            "GAPWAIT": f"Crossing {n}: waiting for a gap ({el:.0f} s)",
            "HOLD": f"Crossing {n}: letting {len(cr.committed)} car(s) through ({el:.0f} s)",
            "AMBER": f"Crossing {n}: cars on amber",
            "ALLRED": f"Crossing {n}: all red",
            "WALK": f"Crossing {n}: WALK ({el:.0f} s)",
            "CLEAR": f"Crossing {n}: finish crossing",
        }[cr.state]


# =====================================================================
# LOGGING AND COMPARISON
# =====================================================================
CSV_FIELDS = ["time", "label", "mode", "fix1", "fix2", "fix3", "fix4", "fix5", "seed",
              "car_rate", "turn_pct", "people_rate"]
METRIC_FIELDS = ["minutes", "people_crossed", "ped_wait_A", "ped_wait_B", "ped_wait_max",
                 "ped_over40_pct", "cars_through", "straight_delay", "turn_delay", "east_delay",
                 "car_delay_max", "blocked_car_s", "blocked_pct", "holds", "holds_straight_only",
                 "hold_avg", "hold_max", "walk_B_avg", "main_red_pct", "main_stops_per_h"]


def log_row(sim, label):
    m = sim.metrics()
    new = not os.path.exists(LOG_CSV)
    with open(LOG_CSV, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(CSV_FIELDS + METRIC_FIELDS)
        cfg = sim.cfg
        w.writerow([datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), label, MODES[cfg["mode"]],
                    cfg["fix1"], cfg["fix2"], cfg["fix3"], cfg["fix4"], cfg["fix5"], sim.seed,
                    cfg["car_rate"], cfg["turn_pct"], cfg["people_rate"]]
                   + [round(m[k], 2) for k in METRIC_FIELDS])


PRESETS = [
    ("Baseline: no hold, no fixes", dict(mode=0)),
    ("Hold, 12 s cap", dict(mode=2)),
    ("Hold, 60 s total cap", dict(mode=1)),
    ("Hold 12 + fix 1 (B moved back)", dict(mode=2, fix1=True)),
    ("Hold 12 + fix 2 (early end)", dict(mode=2, fix2=True)),
    ("Hold 12 + fix 3 (pair with A)", dict(mode=2, fix3=True)),
    ("Hold 12 + fix 4 (gap wait)", dict(mode=2, fix4=True)),
    ("No hold + fix 1", dict(mode=0, fix1=True)),
    ("Hold 12 + fixes 1-3", dict(mode=2, fix1=True, fix2=True, fix3=True)),
    ("Hold 12 + fixes 1-4", dict(mode=2, fix1=True, fix2=True, fix3=True, fix4=True)),
    ("No hold + fix 5 (Claude)", dict(mode=0, fix5=True)),
    ("Hold 12 + fix 5 (Claude)", dict(mode=2, fix5=True)),
]

REPORT_COLS = [  # (metric, header, format)
    ("ped_wait_A", "WaitA", "{:6.1f}"), ("ped_wait_B", "WaitB", "{:6.1f}"),
    ("ped_over40_pct", ">40s%", "{:6.1f}"), ("straight_delay", "StrDly", "{:6.1f}"),
    ("turn_delay", "TrnDly", "{:6.1f}"), ("blocked_car_s", "Block", "{:6.0f}"),
    ("blocked_pct", "Blk%", "{:6.1f}"), ("holds", "Holds", "{:6.0f}"),
    ("hold_avg", "HoldAv", "{:6.1f}"), ("hold_max", "HoldMx", "{:6.1f}"),
    ("walk_B_avg", "WalkB", "{:6.1f}"), ("main_red_pct", "Red%", "{:6.1f}"),
    ("main_stops_per_h", "Stop/h", "{:6.0f}"),
]


def make_compare_sims(minutes, traffic):
    sims = []
    for label, cfg in PRESETS:
        c = dict(traffic)
        c.update(dict(mode=0, fix1=False, fix2=False, fix3=False, fix4=False, fix5=False))
        c.update(cfg)
        c["auto_cars"] = c["auto_people"] = True
        sims.append((label, Sim(c, seed=COMPARE_SEED)))
    return sims


def compare_step(sim, steps, minutes, dt=0.1):
    """Advance one comparison run; returns True when finished."""
    end = minutes * 60
    for _ in range(steps):
        if sim.t >= end:
            return True
        if not getattr(sim, "_broke", False) and sim.t >= COMPARE_BREAKDOWN_AT:
            sim._broke = True
            sim.breakdown()
        sim.step(dt)
    return sim.t >= end


def build_report(results, minutes, traffic):
    lines = [
        "SMART CROSSING - COMPARISON REPORT",
        f"Run at {datetime.datetime.now():%Y-%m-%d %H:%M}, {minutes} simulated minutes per setup, "
        f"seed {COMPARE_SEED} (same traffic in every run)",
        f"Traffic: {traffic['car_rate']} cars/min each way, {traffic['turn_pct']}% turn right, "
        f"{traffic['people_rate']} people/min. One car breaks down at crossing B at "
        f"{COMPARE_BREAKDOWN_AT / 60:.0f} min.",
        "",
        f"{'Setup':32}" + "".join(f"{h:>7}" for _, h, _ in REPORT_COLS),
        "-" * (32 + 7 * len(REPORT_COLS)),
    ]
    for label, m in results:
        lines.append(f"{label:32}" + "".join(" " + fmt.format(m[k]) for k, _, fmt in REPORT_COLS))
    base = results[0][1]
    lines += ["", "Compared with the baseline:"]
    for label, m in results[1:]:
        lines.append(
            f"  {label:32} blocking {m['blocked_car_s'] - base['blocked_car_s']:+7.0f} car-s, "
            f"straight delay {m['straight_delay'] - base['straight_delay']:+5.1f} s, "
            f"ped wait A {m['ped_wait_A'] - base['ped_wait_A']:+5.1f} s, "
            f"B {m['ped_wait_B'] - base['ped_wait_B']:+5.1f} s")
    lines += [
        "",
        "Columns:",
        "  WaitA, WaitB  average pedestrian wait at crossing A / B (s)",
        "  >40s%         % of pedestrians who waited over 40 s (likely to cross on red)",
        "  StrDly        average delay of straight westbound cars (s)",
        "  TrnDly        average delay of right-turning cars (s)",
        "  Block         total seconds straight cars spent stuck behind a turning car on green",
        "  Blk%          % of straight westbound cars that got stuck behind a turner",
        "  Holds         times B's walk was held for cars on the sensors",
        "  HoldAv/HoldMx average / longest hold (s)",
        "  WalkB         average length of B's walk phase (s)",
        "  Red%          % of time the main road had a red or amber light",
        "  Stop/h        times per hour the main road was stopped for crossing A",
        "Lower is better for every column except Holds, which is just a count.",
    ]
    return "\n".join(lines)


def run_compare_cli(minutes, car_rate=None, turn_pct=None, people_rate=None):
    traffic = {k: DEFAULT_CONFIG[k] for k in ("car_rate", "turn_pct", "people_rate")}
    for k, v in (("car_rate", car_rate), ("turn_pct", turn_pct), ("people_rate", people_rate)):
        if v is not None:
            traffic[k] = v
    results = []
    for label, sim in make_compare_sims(minutes, traffic):
        t0 = time.time()
        compare_step(sim, 10 ** 9, minutes)
        log_row(sim, "compare: " + label)
        results.append((label, sim.metrics()))
        print(f"  done: {label} ({time.time() - t0:.0f} s)")
    report = build_report(results, minutes, traffic)
    with open(REPORT_TXT, "w") as f:
        f.write(report + "\n")
    print()
    print(report)
    print(f"\nSaved to {REPORT_TXT} and {LOG_CSV}")


# =====================================================================
# WINDOW AND DRAWING
# =====================================================================
COL = dict(grass="#c9dbb8", road="#5b5f63", line="#f4f4f4", text="#1d2a20",
           g="#2ecc71", y="#f1c40f", r="#e74c3c",
           sensor="#1f78b4", sensor_on="#5ec8ff",
           wb="#3b7dd8", turn="#f08c2e", eb="#7b5cc4", broken="#c0392b",
           adult="#7d3c98", child="#e86fb0", heavy="#4a235a", bag="#8d6e4a")


class App:
    def __init__(self, root):
        self.root = root
        self.sim = Sim()
        self.paused = False
        self.compare_job = None
        root.title("Smart crossing simulator")

        self.canvas = tk.Canvas(root, width=W, height=H, bg=COL["grass"], highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="n")
        self.canvas.bind("<Button-1>", self.on_click)

        panel = ttk.Frame(root, padding=8)
        panel.grid(row=0, column=1, sticky="ns")
        self.build_panel(panel)

        self.draw_background()
        self.last = time.perf_counter()
        self.frame = 0
        self.tick()

    # ---------- control panel ----------
    def build_panel(self, p):
        box = ttk.LabelFrame(p, text="Hold rule at crossing B", padding=4)
        box.pack(fill="x", pady=(0, 6))
        self.mode_var = tk.IntVar(value=self.sim.cfg["mode"])
        for i, name in enumerate(MODES):
            ttk.Radiobutton(box, text=name, variable=self.mode_var, value=i,
                            command=self.settings_changed).pack(anchor="w")

        box = ttk.LabelFrame(p, text="Fixes", padding=4)
        box.pack(fill="x", pady=(0, 6))
        self.fix_vars = {}
        for i, (key, name) in enumerate(FIX_NAMES.items(), 1):
            v = tk.BooleanVar(value=False)
            ttk.Checkbutton(box, text=f"{i}. {name}", variable=v,
                            command=self.settings_changed).pack(anchor="w")
            self.fix_vars[key] = v
        ttk.Label(box, text="Changing the rule or a fix restarts the run.",
                  foreground="#555").pack(anchor="w")

        box = ttk.LabelFrame(p, text="Drop in people", padding=4)
        box.pack(fill="x", pady=(0, 6))
        self.kind_var = tk.StringVar(value="Adult")
        for i, k in enumerate(FOOTPRINTS):
            ttk.Radiobutton(box, text=k, variable=self.kind_var, value=k).grid(
                row=i // 2, column=i % 2, sticky="w")
        names = ["A north", "A south", "B west", "B east"]
        for i, n in enumerate(names):
            ttk.Button(box, text=f"Add at {n}", command=lambda k=i: self.add_person_button(k)).grid(
                row=2 + i // 2, column=i % 2, sticky="ew", pady=1)

        box = ttk.LabelFrame(p, text="Drop in cars", padding=4)
        box.pack(fill="x", pady=(0, 6))
        ttk.Button(box, text="Straight (west)", command=lambda: self.sim.add_car("wb")).grid(
            row=0, column=0, sticky="ew", pady=1)
        ttk.Button(box, text="Right turn", command=lambda: self.sim.add_car("turn")).grid(
            row=0, column=1, sticky="ew", pady=1)
        ttk.Button(box, text="Straight (east)", command=lambda: self.sim.add_car("eb")).grid(
            row=1, column=0, sticky="ew", pady=1)
        ttk.Button(box, text="Break down a turner", command=lambda: self.sim.breakdown()).grid(
            row=1, column=1, sticky="ew", pady=1)

        box = ttk.LabelFrame(p, text="Automatic traffic", padding=4)
        box.pack(fill="x", pady=(0, 6))
        self.auto_cars = tk.BooleanVar(value=True)
        self.auto_people = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="Cars", variable=self.auto_cars).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(box, text="People", variable=self.auto_people).grid(row=0, column=1, sticky="w")
        self.car_rate = self.slider(box, 1, "Cars / min each way", 0, 40, 12)
        self.turn_pct = self.slider(box, 2, "Right-turn %", 0, 100, 35)
        self.people_rate = self.slider(box, 3, "People / min", 0, 30, 6)
        self.speed = self.slider(box, 4, "Speed ×", 1, 5, 1)
        row = ttk.Frame(box)
        row.grid(row=5, column=0, columnspan=2, sticky="ew")
        self.pause_btn = ttk.Button(row, text="Pause", command=self.toggle_pause)
        self.pause_btn.pack(side="left", expand=True, fill="x")
        ttk.Button(row, text="Restart", command=self.restart).pack(side="left", expand=True, fill="x")

        box = ttk.LabelFrame(p, text="Results", padding=4)
        box.pack(fill="x")
        ttk.Button(box, text="Save this run to the log", command=self.save_run).pack(fill="x", pady=1)
        ttk.Button(box, text=f"Compare all setups ({COMPARE_MINUTES} min each)",
                   command=self.start_compare).pack(fill="x", pady=1)
        self.result_msg = ttk.Label(box, text="", foreground="#555", wraplength=240)
        self.result_msg.pack(anchor="w")

    def slider(self, parent, row, label, lo, hi, val):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        var = tk.IntVar(value=val)
        tk.Scale(parent, from_=lo, to=hi, orient="horizontal", variable=var,
                 length=110, showvalue=True, highlightthickness=0).grid(row=row, column=1)
        return var

    def current_config(self):
        cfg = dict(mode=self.mode_var.get(),
                   auto_cars=self.auto_cars.get(), car_rate=self.car_rate.get(),
                   turn_pct=self.turn_pct.get(), auto_people=self.auto_people.get(),
                   people_rate=self.people_rate.get())
        cfg.update({k: v.get() for k, v in self.fix_vars.items()})
        return cfg

    def settings_changed(self):
        self.restart()

    def restart(self):
        self.sim = Sim(self.current_config())
        self.draw_background()

    def add_person_button(self, i):
        self.sim.add_person(self.sim.plates[i], self.kind_var.get())

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="Resume" if self.paused else "Pause")

    def on_click(self, e):
        for plate in self.sim.plates:
            if plate.contains(e.x, e.y):
                self.sim.add_person(plate, self.kind_var.get(), plate.tile_at(e.x, e.y))
                return

    # ---------- logging ----------
    def save_run(self):
        log_row(self.sim, "manual: " + self.sim.config_label())
        self.result_msg.config(text=f"Saved {self.sim.t / 60:.1f} min of "
                                    f"'{self.sim.config_label()}' to results_log.csv")

    def start_compare(self):
        if self.compare_job:
            return
        self.paused = True
        self.pause_btn.config(text="Resume")
        traffic = dict(car_rate=self.car_rate.get(), turn_pct=self.turn_pct.get(),
                       people_rate=self.people_rate.get())
        self.compare_job = dict(sims=make_compare_sims(COMPARE_MINUTES, traffic), i=0,
                                results=[], traffic=traffic)
        self.compare_tick()

    def compare_tick(self):
        job = self.compare_job
        label, sim = job["sims"][job["i"]]
        if compare_step(sim, 1500, COMPARE_MINUTES):
            log_row(sim, "compare: " + label)
            job["results"].append((label, sim.metrics()))
            job["i"] += 1
            if job["i"] == len(job["sims"]):
                report = build_report(job["results"], COMPARE_MINUTES, job["traffic"])
                with open(REPORT_TXT, "w") as f:
                    f.write(report + "\n")
                self.compare_job = None
                self.result_msg.config(text="Done. Saved comparison_report.txt and results_log.csv.")
                self.show_report(report)
                return
        done = job["i"] + sim.t / (COMPARE_MINUTES * 60)
        self.result_msg.config(text=f"Comparing… {100 * done / len(job['sims']):.0f}% "
                                    f"({job['i'] + 1}/{len(job['sims'])}: {label})")
        self.root.after(1, self.compare_tick)

    def show_report(self, report):
        win = tk.Toplevel(self.root)
        win.title("Comparison report")
        text = tk.Text(win, width=118, height=40, font=("Courier", 9), wrap="none")
        text.insert("1.0", report)
        text.pack(fill="both", expand=True)

    # ---------- main loop ----------
    def tick(self):
        now = time.perf_counter()
        real = min(0.1, now - self.last)
        self.last = now
        sim = self.sim
        sim.cfg.update(auto_cars=self.auto_cars.get(), auto_people=self.auto_people.get(),
                       car_rate=self.car_rate.get(), turn_pct=self.turn_pct.get(),
                       people_rate=self.people_rate.get())
        if not self.paused:
            total = real * self.speed.get()
            n = max(1, math.ceil(total / 0.05))
            for _ in range(n):
                sim.step(total / n)
        self.draw_dynamic()
        self.frame += 1
        self.root.after(30, self.tick)

    # ---------- drawing ----------
    def draw_background(self):
        c, g = self.canvas, self.sim.geo
        c.delete("all")
        c.create_rectangle(0, ROAD_N, W, ROAD_S, fill=COL["road"], outline="")
        c.create_rectangle(SIDE_W_X, 0, SIDE_E_X, ROAD_N, fill=COL["road"], outline="")

        for x in range(0, W, 30):
            if not (SIDE_W_X - 10 <= x <= SIDE_E_X or CROSS_A_X0 - 10 <= x <= CROSS_A_X1):
                c.create_line(x, LANE_DIV_Y, x + 15, LANE_DIV_Y, fill=COL["line"], width=2)
        for y in range(0, ROAD_N - 20, 30):
            if not g.cross_b_y0 - 16 <= y <= g.cross_b_y1 + 4:
                c.create_line(SIDE_MID_X, y, SIDE_MID_X, y + 15, fill=COL["line"], width=2)

        for y in range(ROAD_N + 3, ROAD_S - 3, 12):
            c.create_rectangle(CROSS_A_X0, y, CROSS_A_X1, y + 6, fill=COL["line"], outline="")
        for x in range(SIDE_W_X + 3, SIDE_E_X - 3, 12):
            c.create_rectangle(x, g.cross_b_y0, x + 6, g.cross_b_y1, fill=COL["line"], outline="")

        c.create_line(STOP_A_WB_X, ROAD_N, STOP_A_WB_X, LANE_DIV_Y, fill=COL["line"], width=3)
        c.create_line(STOP_A_EB_X, LANE_DIV_Y, STOP_A_EB_X, ROAD_S, fill=COL["line"], width=3)
        c.create_line(SIDE_MID_X, g.stop_b_y, SIDE_E_X, g.stop_b_y, fill=COL["line"], width=3)

        f = ("Arial", 9)
        c.create_text(660, 446, text="Crossing A", font=("Arial", 10, "bold"), fill=COL["text"])
        c.create_text(322, g.cross_b_y0 + 20, text="Crossing B", anchor="e",
                      font=("Arial", 10, "bold"), fill=COL["text"])
        c.create_text(10, 10, anchor="nw", font=f, fill=COL["text"],
                      text="Pick a person type on the right, then click a plate tile to drop them in.")
        c.create_text(10, 26, anchor="nw", font=f, fill=COL["text"],
                      text="Blue boxes are road sensors; they light up when a car is on them.")
        y = 470
        for key, name in (("wb", "straight, heading west"), ("turn", "turning right"),
                          ("eb", "straight, heading east"), ("broken", "broken down")):
            c.create_rectangle(14, y, 30, y + 10, fill=COL[key], outline="")
            c.create_text(38, y + 5, text=name, anchor="w", font=f, fill=COL["text"])
            y += 16

    def head(self, x, y, light, label):
        c = self.canvas
        c.create_rectangle(x - 8, y - 8, x + 8, y + 8, fill="#222", outline="", tags="dyn")
        c.create_oval(x - 5, y - 5, x + 5, y + 5, fill=COL[light], outline="", tags="dyn")
        c.create_text(x + 12, y, text=label, anchor="w", font=("Arial", 7),
                      fill=COL["text"], tags="dyn")

    def sensor(self, x0, y0, x1, y1, on, label):
        c = self.canvas
        c.create_rectangle(x0, y0, x1, y1, fill=COL["sensor_on"] if on else COL["sensor"],
                           outline="", tags="dyn")
        c.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=label, fill="white",
                      font=("Arial", 10, "bold"), tags="dyn")

    def draw_dynamic(self):
        c, sim, g = self.canvas, self.sim, self.sim.geo
        c.delete("dyn")

        self.sensor(S1_X0 + 2, ROAD_N + 3, S1_X1, LANE_DIV_Y - 3, bool(sim.on_s1()), "1")
        self.sensor(SIDE_MID_X + 3, g.stop_b_y + 3, SIDE_E_X - 3, ROAD_N - 2, bool(sim.on_s2()), "2")
        if sim.cfg["fix4"]:
            self.sensor(S0_X0, ROAD_N + 3, S0_X1, LANE_DIV_Y - 3, bool(sim.on_s0()), "0")

        for plate in sim.plates:
            cr = sim.A if plate.crossing == "A" else sim.B
            if cr.state == "WALK":
                edge = COL["g"]
            elif cr.state == "CLEAR":
                edge = COL["g"] if int(sim.t * 3) % 2 else "#444"
            else:
                edge = COL["r"]
            load = plate.load()
            for r in range(TILE_ROWS):
                for col in range(TILE_COLS):
                    shade = max(0, 185 - int(load.get((col, r), 0) * 1.6))
                    x, y = plate.x0 + col * TILE_PX, plate.y0 + r * TILE_PX
                    c.create_rectangle(x, y, x + TILE_PX, y + TILE_PX,
                                       fill=f"#{shade:02x}{shade:02x}{min(255, shade + 10):02x}",
                                       outline="#8a8d91", tags="dyn")
            c.create_rectangle(plate.x0 - 2, plate.y0 - 2, plate.x1 + 2, plate.y1 + 2,
                               outline=edge, width=3, tags="dyn")
            for p in plate.people:
                self.draw_person(p)
        for p in sim.walkers:
            self.draw_person(p)

        self.head(704, 280, sim.light_a(), "main road")
        self.head(614, 398, sim.light_a(), "main road")
        self.head(530, g.stop_b_y + 12, sim.light_b(), "turners")

        for car in sim.cars:
            x, y, a = car.path.pos(car.s)
            dx, dy = math.cos(a), math.sin(a)
            px, py = -dy, dx
            hl, hw = CAR_LEN / 2, CAR_WID / 2
            pts = [x + dx * hl + px * hw, y + dy * hl + py * hw,
                   x + dx * hl - px * hw, y + dy * hl - py * hw,
                   x - dx * hl - px * hw, y - dy * hl - py * hw,
                   x - dx * hl + px * hw, y - dy * hl + py * hw]
            colour = COL["broken"] if sim.t < car.broken_until else COL[car.kind]
            c.create_polygon(pts, fill=colour, outline="#222", tags="dyn")

        f = ("Arial", 9, "bold")
        c.create_text(560, 240, text=sim.status(sim.A), anchor="w", font=f,
                      fill=COL["text"], tags="dyn")
        c.create_text(330, g.cross_b_y0 - 26, text=sim.status(sim.B), anchor="w", font=f,
                      fill=COL["text"], tags="dyn")

        m = sim.metrics()
        left = (f"Setup: {sim.config_label()}\n"
                f"Time: {sim.t / 60:5.1f} min\n"
                f"People crossed: {m['people_crossed']}\n"
                f"  avg wait A {m['ped_wait_A']:4.1f} s, B {m['ped_wait_B']:4.1f} s\n"
                f"  longest {m['ped_wait_max']:4.1f} s, over 40 s: {m['ped_over40_pct']:.0f}%\n"
                f"Waiting now: A {sim.A.actual} (counted {sim.A.detected}), "
                f"B {sim.B.actual} (counted {sim.B.detected})")
        right = (f"Cars through: {m['cars_through']}\n"
                 f"  avg delay: straight {m['straight_delay']:4.1f} s, turn {m['turn_delay']:4.1f} s\n"
                 f"Stuck behind a turner: {m['blocked_car_s']:.0f} car-s ({m['blocked_pct']:.0f}% of cars)\n"
                 f"Holds at B: {m['holds']} (avg {m['hold_avg']:.1f} s, longest {m['hold_max']:.1f} s)\n"
                 f"B walk phase avg: {m['walk_B_avg']:.1f} s\n"
                 f"Main road red/amber: {m['main_red_pct']:.0f}% of the time")
        if sim.cfg["fix5"]:
            right += (f"\nFix 5: side road {sim.storage_count()}/{g.storage_cap} cars, "
                      f"turns {60 * sim.turn_rate(sim.t):.1f}/min, risk {100 * sim.spill_risk(sim.t):.0f}%")
        c.create_text(230, 468, text=left, anchor="nw", font=("Courier", 9),
                      fill=COL["text"], tags="dyn")
        c.create_text(590, 468, text=right, anchor="nw", font=("Courier", 9),
                      fill=COL["text"], tags="dyn")

    def draw_person(self, p):
        c = self.canvas
        if p.kind == "Bag / object":
            c.create_rectangle(p.x - 3, p.y - 3, p.x + 3, p.y + 3, fill=COL["bag"],
                               outline="", tags="dyn")
            return
        r, col = {"Adult": (4, COL["adult"]), "Child": (3, COL["child"]),
                  "Heavy adult": (6, COL["heavy"])}[p.kind]
        c.create_oval(p.x - r, p.y - r, p.x + r, p.y + r, fill=col, outline="white", tags="dyn")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--compare":
        nums = [int(x) for x in sys.argv[2:6]] + [None] * 4
        minutes = nums[0] or COMPARE_MINUTES
        print(f"Running {len(PRESETS)} setups, {minutes} simulated minutes each…")
        run_compare_cli(minutes, nums[1], nums[2], nums[3])
        return
    if tk is None:
        print("tkinter is missing. On Linux: sudo apt install python3-tk")
        return
    root = tk.Tk()
    root.resizable(False, False)
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
