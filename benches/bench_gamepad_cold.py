"""Cold bench: gamepad mapping (no pygame) + the headless hot-plug cycle.

Known input -> known output in m/s, rad/s and seconds. Sections:
  A. axes_to_twist - known axis values -> known m/s and rad/s. Pure, no pygame.
  E. the MOSS envelope - the module fed moss_adapter's scale and envelope:
     full forward + full turn passes untouched, a stray strafe shares the rim.
  B. the module's hot-plug loop driven by a FAKE pygame: absent pad -> pad
     plugged in -> deadman released and pressed again -> pad pulled out ->
     pad back -> at 50 Hz, the 0.5 s brake then SILENCE. Still no pygame needed.
  H. brake cancel on a deadman press + the brake window knob (MOSS_BRAKE_S).
  C. real pygame with no display, if it is installed: the dummy SDL drivers
     have to make init()/event.pump() work on a headless computer.
  F. the safety nets: trust gate, absolute ceilings, rim envelope.
  G. the slew ramp.

Needs dimOS (the module imports it). No pad, no display, no robot.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_gamepad_cold.py
"""
import logging
import os
import subprocess
import sys
import time
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

os.environ.pop("MOSS_BRAKE_S", None)
for _var in ("MOSS_MPS_AT_LIMIT", "MOSS_TRACK_M", "MOSS_TURN_GAIN", "MOSS_MAX_PCT"):
    os.environ.pop(_var, None)

from moss_dimos import gamepad as gp  # noqa: E402
from moss_dimos.gamepad import GamepadTeleop, TeleopConfig, axes_to_twist  # noqa: E402

OK = KO = 0


def check(cond, label):
    global OK, KO
    if cond:
        OK += 1
    else:
        KO += 1
    print(("  OK  " if cond else "  KO  ") + label)


def close(a, b, tol=1e-12):
    return abs(a - b) <= tol


def twist_close(got, expected, tol=1e-12):
    return all(close(g, e, tol) for g, e in zip(got, expected))


def wait_for(predicate, timeout=6.0, tick=0.02):
    """Poll until true or timeout. The waiting loop re-scans every 2 s."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(tick)
    return bool(predicate())


# --- A. pure mapping: known axes -> known physical units ------------------
# Defaults: 0.6 m/s and 1.2 rad/s at full stick, deadzone 0.32, boost x2,
# ceilings 0.45 m/s and 0.8 rad/s.
CFG = TeleopConfig()
LIN, ANG = gp.CLAMP_LINEAR_MS, gp.CLAMP_ANGULAR_RADS
print("A. axes_to_twist (no pygame)")

check("pygame" not in sys.modules, "mapping is testable with pygame unimported")
check((LIN, ANG) == (0.45, 0.8), f"ceilings: {LIN} m/s, {ANG} rad/s")
# The pad contract as literals: every press below goes through
# gp.DEADMAN_BUTTON, so without these a moved constant would still pass.
check(gp.DEADMAN_BUTTON is None,
      f"no deadman by default: the sticks drive (MOSS_PAD_DEADMAN=r1 brings R1 back): {gp.DEADMAN_BUTTON}")
# the pad on its USB cable is an Xbox 360 pad to the kernel (xpad): R1 = button 5, right stick X = axis 3,
# triggers at rest = -1 on axes 2 and 5 (measured: js1 initial state axes 2 and 5 at -32767, sticks at 0)
check(gp.pad_layout({}) == {"deadman": None, "axis_right_x": 2, "axis_r2": 5}, "default layout = android, no deadman (None, 2, 5)")
check(gp.pad_layout({"MOSS_PAD_LAYOUT": "xinput"}) == {"deadman": None, "axis_right_x": 3, "axis_r2": 5}, "xinput layout = (None, 3, 5)")
check(gp.pad_layout({"MOSS_PAD_LAYOUT": "xinput", "MOSS_PAD_DEADMAN": "4"}) == {"deadman": 4, "axis_right_x": 3, "axis_r2": 5}, "MOSS_PAD_DEADMAN=4 -> that SDL button")
check(gp.pad_layout({"MOSS_PAD_DEADMAN": "r1"})["deadman"] == 7 and gp.pad_layout({"MOSS_PAD_LAYOUT": "xinput", "MOSS_PAD_DEADMAN": "R1"})["deadman"] == 5
      and gp.pad_layout({"MOSS_PAD_DEADMAN": "none"})["deadman"] is None and gp.pad_layout({"MOSS_PAD_DEADMAN": "x"})["deadman"] is None,
      "MOSS_PAD_DEADMAN=r1 -> the layout's R1 (7 android, 5 xinput); none / unparseable -> no deadman")
check(gp.pad_layout({"MOSS_PAD_LAYOUT": "nope", "MOSS_PAD_AXIS_R2": "x"}) == {"deadman": None, "axis_right_x": 2, "axis_r2": 5}, "unknown layout / bad number -> android")
check(gp.pad_index({}) == 0 and gp.pad_index({"MOSS_PAD_INDEX": "1"}) == 1 and gp.pad_index({"MOSS_PAD_INDEX": "-3"}) == 0, "MOSS_PAD_INDEX: 0 by default, clamped at 0")
check((gp.AXIS_LEFT_X, gp.AXIS_LEFT_Y, gp.AXIS_RIGHT_X, gp.AXIS_R2) == (0, 1, 2, 5),
      "strafe = left X (axis 0), throttle = left Y (axis 1), turn = right X (axis 2), boost = R2 (axis 5)")

# The module pins SDL with setdefault, so this cannot be asserted in THIS
# process: a shell that exported SDL_VIDEODRIVER=x11 would fail it, and
# honouring that export is the documented intent. Import it in a subprocess
# with a controlled environment instead - hermetic either way.
SDL_PROBE = ("import os, moss_dimos.gamepad; "
             "print(os.environ['SDL_VIDEODRIVER'], "
             "os.environ['SDL_AUDIODRIVER'])")


def sdl_after_import(**preset):
    """SDL_VIDEODRIVER/SDL_AUDIODRIVER once the module is imported fresh."""
    env = {k: v for k, v in os.environ.items()
           if k not in ("SDL_VIDEODRIVER", "SDL_AUDIODRIVER")}
    env["PYTHONPATH"] = str(REPO_ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    env.update(preset)
    done = subprocess.run([sys.executable, "-c", SDL_PROBE], env=env,
                          capture_output=True, text=True)
    if done.returncode != 0:
        print(done.stderr.strip()[-400:])
    # the last line: dimOS may log above it
    lines = done.stdout.strip().splitlines()
    return lines[-1] if lines else ""


unset = sdl_after_import()
check(unset == "dummy dummy",
      f"with SDL unset, importing the module pins the dummy (headless) "
      f"drivers: {unset!r}")
exported = sdl_after_import(SDL_VIDEODRIVER="x11")
check(exported == "x11 dummy",
      f"an operator who exported SDL_VIDEODRIVER=x11 keeps their display "
      f"(setdefault, not assignment): {exported!r}")

# SDL: stick up = -1. Full forward + R2 held -> 0.6 * 2.0 = 1.2 m/s asked.
check(twist_close(axes_to_twist(0.0, -1.0, 0.0, 1.0, CFG), (LIN, 0.0, 0.0)),
      f"stick fully forward + boost (1.2 m/s asked) -> vx CLAMPED at {LIN} m/s")
check(twist_close(axes_to_twist(0.0, -1.0, 0.0, -1.0, CFG), (LIN, 0.0, 0.0)),
      f"stick fully forward (0.6 m/s asked) -> vx CLAMPED at {LIN} m/s")
check(twist_close(axes_to_twist(0.0, -0.5, 0.0, -1.0, CFG), (0.3, 0.0, 0.0)),
      "half stick forward -> vx = 0.3 m/s (proportional below the ceiling)")
check(twist_close(axes_to_twist(0.0, 1.0, 0.0, -1.0, CFG), (-LIN, 0.0, 0.0)),
      f"stick fully back -> vx CLAMPED at -{LIN} m/s")

# Deadzone 0.32: below it the axis is dead, above it goes through unscaled.
check(twist_close(axes_to_twist(0.05, -0.10, 0.11, -1.0, CFG), (0.0, 0.0, 0.0)),
      "all sticks inside the deadzone -> 0.0 on every axis")
check(twist_close(axes_to_twist(0.0, -0.20, 0.0, -1.0, CFG), (0.0, 0.0, 0.0)),
      "0.20 is INSIDE the 0.32 deadzone -> 0.0")
check(twist_close(axes_to_twist(0.0, -0.35, 0.0, -1.0, CFG), (0.21, 0.0, 0.0)),
      "0.35 just outside the deadzone -> vx = 0.21 m/s (no rescaling)")

# Rotation: right stick right (+1) turns clockwise = negative wz.
check(twist_close(axes_to_twist(0.0, 0.0, 1.0, -1.0, CFG), (0.0, 0.0, -ANG)),
      f"right stick X = +1 (1.2 rad/s asked) -> wz CLAMPED at -{ANG} rad/s")
check(twist_close(axes_to_twist(0.0, 0.0, 1.0, 1.0, CFG), (0.0, 0.0, -ANG)),
      f"boost does not touch rotation -> wz stays at the -{ANG} rad/s ceiling")

# Strafe: left stick left (-1) strafes to +y.
check(twist_close(axes_to_twist(-1.0, 0.0, 0.0, -1.0, CFG), (0.0, LIN, 0.0)),
      f"left stick X = -1 -> vy CLAMPED at +{LIN} m/s (strafe left)")
check(twist_close(axes_to_twist(1.0, 0.0, 0.0, -1.0, CFG), (0.0, -LIN, 0.0)),
      f"left stick X = +1 -> vy CLAMPED at -{LIN} m/s (strafe right)")
check(twist_close(axes_to_twist(-0.5, -0.5, 0.5, -1.0, CFG),
                  (0.15, 0.15, -0.3)),
      "diagonal + turn (half sticks: 0.3 + 0.3 + 0.5 x 0.6 = 0.9 m/s rim) -> the "
      "default envelope (0.45 m/s) halves it to (0.15, 0.15, -0.3)")

# A non-default envelope must scale, nothing hard-coded.
SLOW = TeleopConfig(linear_speed=0.25, angular_speed=0.5, deadzone=0.2,
                    boost_multiplier=3.0)
vx, vy, wz = axes_to_twist(0.0, -1.0, -1.0, 1.0, SLOW)
rim = abs(vx) + abs(vy) + gp.DEFAULT_LEVER_M * abs(wz)
check(abs(rim - gp.DEFAULT_RIM_MS) < 1e-9 and abs(vx / wz - 0.45 / 0.5) < 1e-9,
      "custom cfg full fwd+turn+boost -> rim held at the envelope, gesture proportions kept")


# --- E. the MOSS envelope: moss_adapter's scale and ceilings --------------
print("\nE. the MOSS envelope (default geometry: track 0.266 m, the measured speed at 100 %, turn gain 2)")
from moss_dimos.moss_adapter import MossGeometry, teleop_envelope, teleop_speeds  # noqa: E402

G = MossGeometry.from_env()
V = G.mps_at_limit                    # the measured plateau at 100 % (0.32 m/s since the 90:1 motors)
LEVER = G.track_m / 2.0               # 0.133 m
m_lin, m_ang = teleop_speeds(G)
m_env = teleop_envelope(G)
check(close(m_lin, V, 1e-9) and close(m_ang, 2.0 * 2.0 * V / G.track_m, 1e-9),
      f"scale: {m_lin:.3f} m/s and {m_ang:.4f} rad/s at full stick (2 x the spin rate of the tracks)")
check(set(m_env) == {"linear", "angular", "lever", "rim"},
      f"teleop_envelope keys = TwistEnvelope's fields: {sorted(m_env)}")
moss_pad = GamepadTeleop(linear_speed=m_lin, angular_speed=m_ang, envelope=m_env)
RIM = V + LEVER * m_ang               # full forward + full turn sits exactly on the rim
check(isinstance(moss_pad.envelope, gp.TwistEnvelope)
      and close(moss_pad.envelope.rim, RIM, 1e-9) and close(moss_pad.envelope.lever, LEVER, 1e-9),
      f"the envelope dict (what a blueprint carries) becomes a TwistEnvelope: "
      f"lever {moss_pad.envelope.lever:.3f} m, rim {moss_pad.envelope.rim:.3f} m/s")
MCFG = moss_pad.cfg
got = axes_to_twist(0.0, -1.0, 1.0, -1.0, MCFG)
check(twist_close(got, (V, 0.0, -m_ang), 1e-9),
      f"full forward + full turn -> ({got[0]:.3f} m/s, {got[2]:.4f} rad/s): NOT scaled, "
      f"the rim ({V:.2f} + {LEVER:.3f} x {m_ang:.4f} = {RIM:.3f}) is exactly the envelope")
got = axes_to_twist(-1.0, -1.0, 1.0, -1.0, MCFG)
k = RIM / (2.0 * V + LEVER * m_ang)   # ... plus a full strafe: the whole command scales down to the rim
check(twist_close(got, (k * V, k * V, -k * m_ang), 1e-9),
      f"... plus full left stick X -> rim {2.0 * V + LEVER * m_ang:.2f}, the whole command x {k:.2f}: "
      f"({got[0]:.4f}, {got[1]:.4f}, {got[2]:.4f}); the tracks drop vy, the scale stays")
check(twist_close(axes_to_twist(0.0, -1.0, 0.0, 1.0, MCFG), (V, 0.0, 0.0), 1e-9),
      f"boost on the MOSS scale -> vx still {V:.2f} m/s (the base's limit is the ceiling)")


# --- B. hot-plug cycle on a fake pygame -----------------------------------
print("\nB. hot-plug loop (fake pygame, no display, no pad)")


class FakeError(Exception):
    """Stand-in for pygame.error."""


class FakePad:
    def __init__(self, name, axes, buttons=8):
        self.name, self.axes = name, axes
        self.buttons = [0] * buttons

    def init(self):
        pass

    def get_name(self):
        return self.name

    def get_numaxes(self):
        return len(self.axes)

    def get_axis(self, index):
        return self.axes[index]

    def get_numbuttons(self):
        return len(self.buttons)

    def get_button(self, index):
        return self.buttons[index]


class FakeJoystickModule:
    """SDL joystick subsystem stand-in; the test plugs `pad` in and out."""

    def __init__(self):
        self.pad = None
        self.rescans = 0

    def init(self):
        self.rescans += 1

    def quit(self):
        pass

    def get_count(self):
        return 0 if self.pad is None else 1

    def Joystick(self, index):  # noqa: N802 - pygame's own name
        if self.pad is None:
            raise FakeError("no joystick at index %d" % index)
        return self.pad


class LogSpy:
    """Records what the module logs (dimOS's logger is structlog, not stdlib)."""

    def __init__(self):
        self.lines = []

    def _record(self, msg, *args, **_kw):
        self.lines.append(msg % args if args else msg)

    info = warning = error = debug = _record

    def count(self, needle):
        return sum(needle in line for line in self.lines)


js = FakeJoystickModule()
fake_pygame = types.SimpleNamespace(
    error=FakeError,
    init=lambda: (5, 0),
    quit=lambda: None,
    event=types.SimpleNamespace(pump=lambda: None),
    joystick=js,
)
sys.modules["pygame"] = fake_pygame
spy = LogSpy()
real_logger, gp.logger = gp.logger, spy

pad_module = GamepadTeleop(rate_hz=200.0)
published = []
pad_module.tele_cmd_vel.subscribe(published.append)
pad_module.start()
check(pad_module._thread.name == "moss-gamepad", f"worker thread named {pad_module._thread.name!r}")

# 1. no pad at startup: the module waits instead of returning.
check(wait_for(lambda: spy.count("waiting for gamepad (index 0)") == 1, 3.0),
      "no pad at startup -> logs 'waiting for gamepad (index 0)'")
check(wait_for(lambda: js.rescans >= 3, 5.0),
      f"keeps re-scanning while waiting ({js.rescans} scans in ~5 s)")
check(pad_module._thread.is_alive() and not published,
      "loop still alive after several empty scans, nothing published")
check(spy.count("waiting for gamepad (index 0)") == 1,
      "the waiting line is logged ONCE, not once per scan")

# 2. pad appears WITH FULL-DEFLECTION AXES (an uninitialised userspace
# driver): the trust gate must hold everything at zero until neutral is seen.
js.pad = FakePad("Fake DS4", [0.0, -1.0, -1.0, 0.0, 0.0, -1.0])
check(wait_for(lambda: spy.count("Gamepad connected: Fake DS4") == 1, 5.0),
      "pad plugged in -> logs 'Gamepad connected: <name>' (the launcher greps it)")
time.sleep(0.3)
check(all(close(t.linear.x, 0.0) for t in published[-10:]) if published else True,
      "full deflection at connection -> ZEROS only (trust gate holds)")
# neutral seen once -> trust earned. Axis 2 is the rotation stick: its rest
# is 0.0.
js.pad.axes = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
check(wait_for(lambda: spy.count("axes seen at neutral") == 1, 3.0),
      "sticks at rest -> trust earned (logged once)")
# 2a. NO DEADMAN (the default): a stick out of its deadzone drives, centred sticks stop
n_rest = len(published)
time.sleep(0.3)
check(len(published) == n_rest, f"trusted, sticks at rest, no deadman -> nothing published ({len(published) - n_rest} messages)")
js.pad.axes = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0]
check(wait_for(lambda: published and close(published[-1].linear.x, LIN), 3.0),
      f"no deadman + stick forward -> vx = {LIN} m/s CEILING, no button held (got {published[-1].linear.x if published else None})")
js.pad.axes = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
check(wait_for(lambda: close(published[-1].linear.x, 0.0), 3.0), "sticks back to centre -> zeros published (the brake window)")
time.sleep(pad_module.brake_s + 0.3)
n_silent = len(published)
time.sleep(0.3)
check(len(published) == n_silent and close(published[-1].linear.x, 0.0),
      f"then SILENCE: no message after the brake window with the sticks centred ({len(published) - n_silent} extra)")
# From here on the bench drives the OPTIONAL deadman (MOSS_PAD_DEADMAN=r1 -> R1 = SDL button 7 on the
# MOSS pad): the module reads gp.DEADMAN_BUTTON on every tick, so the switch takes effect at once.
gp.DEADMAN_BUTTON = 7
# deadman NOT held: still zeros even with a deflected stick
js.pad.axes = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0]
time.sleep(0.3)
check(all(close(t.linear.x, 0.0) for t in published[-5:]),
      "deadman on, NOT held, stick forward -> still zeros")
# deadman held -> motion, CLAMPED at the ceiling
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
check(wait_for(lambda: published and close(published[-1].linear.x, LIN), 3.0),
      f"deadman held + stick forward -> vx = {LIN} m/s CEILING (got {published[-1].linear.x if published else None})")
check(close(published[-1].linear.y, 0.0) and close(published[-1].angular.z, 0.0),
      "and vy = 0.0 m/s, wz = 0.0 rad/s on the other axes")

# 2b. deadman RELEASED at speed, then re-pressed with the sticks centred: the
# slew state has to restart from rest. If prev_vx kept the last driven
# 0.45 m/s, the re-press would publish 0.438 m/s with the sticks at rest and
# the rover would leave on its own for ~0.75 s.
js.pad.buttons[gp.DEADMAN_BUTTON] = 0
js.pad.axes = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]   # sticks back to neutral
check(wait_for(lambda: close(published[-1].linear.x, 0.0), 3.0),
      "deadman released while driving at the ceiling -> zeros published")
re_press = len(published)                       # index taken BEFORE the re-press
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
check(wait_for(lambda: len(published) - re_press >= 20, 3.0),
      "deadman pressed again -> the loop keeps publishing")
after = published[re_press:]
peak = max(abs(m.linear.x) for m in after)
check(all(close(m.linear.x, 0.0) and close(m.linear.y, 0.0)
          and close(m.angular.z, 0.0) for m in after),
      f"re-press with the sticks centred -> 0.000 m/s from the FIRST command "
      f"(peak |vx| = {peak:.3f} m/s over {len(after)} messages; 0.438 with a stale ramp)")
# and the ramp climbs again from rest to the ceiling (drive state restored)
js.pad.axes = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0]
check(wait_for(lambda: close(published[-1].linear.x, LIN), 3.0),
      f"stick forward again -> ramps back up to the {LIN} m/s ceiling")

# 3. pad pulled out: exactly one zero Twist, then back to waiting.
sent_before_unplug = len(published)
js.pad = None
check(wait_for(lambda: spy.count("back to waiting") == 1, 3.0),
      "pad unplugged -> logs the loss once")
check(wait_for(lambda: close(published[-1].linear.x, 0.0), 2.0),
      "publishes a zero Twist on loss")
zeros = [m for m in published[sent_before_unplug:] if close(m.linear.x, 0.0)]
check(len(zeros) == 1, f"exactly ONE zero Twist on loss (got {len(zeros)})")
check(pad_module._thread.is_alive(), "loop survives the unplug")
check(spy.count("waiting for gamepad (index 0)") == 2,
      "goes back to the waiting state")

# 4. pad comes back: trust must be RE-EARNED at neutral, then deadman,
# then boost + turn - both CLAMPED at the ceilings.
js.pad = FakePad("Fake DS5", [0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
check(wait_for(lambda: spy.count("Gamepad connected: Fake DS5") == 1, 5.0),
      "pad plugged back in -> logs the new name")
check(wait_for(lambda: spy.count("axes seen at neutral") == 2, 3.0),
      "re-connection re-earns trust at neutral")
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
js.pad.axes = [0.0, -1.0, 1.0, 0.0, 0.0, 1.0]   # full fwd + full turn (axis 2) + boost
expected_k = gp.DEFAULT_RIM_MS / (LIN + gp.DEFAULT_LEVER_M * ANG)
check(wait_for(lambda: published and close(published[-1].linear.x, LIN * expected_k), 3.0),
      f"boost + full turn -> vx enveloped to {LIN * expected_k:.3f} m/s "
      f"(the rim, not each axis, is the ceiling; got {published[-1].linear.x if published else None})")
check(close(published[-1].angular.z, -ANG * expected_k),
      "... and wz enveloped with the SAME factor (proportions of the gesture kept)")

# 5. a pad that has no R2 axis at all must read as "no boost", not crash.
js.pad = None
check(wait_for(lambda: spy.count("waiting for gamepad (index 0)") == 3, 4.0),
      "pad pulled again -> waiting")
js.pad = FakePad("Fake 4-axis", [0.0, 0.0, 0.0, 0.0])
check(wait_for(lambda: spy.count("Gamepad connected: Fake 4-axis") == 1, 5.0),
      "4-axis pad (no R2) accepted")
check(wait_for(lambda: spy.count("axes seen at neutral") == 3, 3.0),
      "4-axis pad earns trust at neutral")
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
js.pad.axes = [0.0, -1.0, 0.0, 0.0]
check(wait_for(lambda: close(published[-1].linear.x, LIN), 3.0)
      and pad_module._thread.is_alive(),
      f"missing R2 = no boost; full stick -> the {LIN} m/s ceiling, loop alive")

t0 = time.monotonic()
pad_module.stop()
stop_s = time.monotonic() - t0
check(not pad_module._thread.is_alive() and stop_s < 2.0,
      f"stop() joins the loop in {stop_s:.2f} s")
check(close(published[-1].linear.x, 0.0) and close(published[-1].angular.z, 0.0),
      "last message on shutdown is a zero Twist")

# 6. BRAKE THEN SILENCE, at the real 50 Hz rate. dimOS's MovementManager reads
# EVERY tele_cmd_vel message - zeros included - as "a human is driving": it
# cancels the nav goal and mutes nav_cmd_vel for tele_cooldown_sec (1.0 s).
# A pad at rest publishing 50 zeros/s would stop exploration for good. The
# released pad brakes for 0.5 s (<= 25 messages at 50 Hz) and then says
# NOTHING at all.
js.pad = FakePad("Fake 50Hz", [0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
brake_module = GamepadTeleop(rate_hz=50.0)
brake_pub = []
brake_module.tele_cmd_vel.subscribe(brake_pub.append)
brake_module.start()
check(wait_for(lambda: spy.count("Gamepad connected: Fake 50Hz") == 1, 5.0),
      "50 Hz module: pad connected")
check(wait_for(lambda: spy.count("axes seen at neutral") == 4, 3.0),
      "50 Hz module: trust earned at neutral")
check(not brake_pub, f"deadman never held -> not a single message published "
                     f"(got {len(brake_pub)})")
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
js.pad.axes = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0]
check(wait_for(lambda: close(brake_pub[-1].linear.x, LIN) if brake_pub else False, 3.0),
      f"deadman held -> the 50 Hz flow ramps to the {LIN} m/s ceiling")
js.pad.buttons[gp.DEADMAN_BUTTON] = 0   # release, at speed
time.sleep(1.2)                          # 0.5 s of brake + 0.7 s past its end
last_drive = max(i for i, m in enumerate(brake_pub) if not close(m.linear.x, 0.0))
brake = brake_pub[last_drive + 1:]
check(all(close(m.linear.x, 0.0) and close(m.linear.y, 0.0)
          and close(m.angular.z, 0.0) for m in brake),
      f"release -> every message after the last driven one is a zero Twist "
      f"({len(brake)} of them)")
check(1 <= len(brake) <= 25,
      f"... and at most 25 of them (BRAKE_S {gp.BRAKE_S} s x 50 Hz): got {len(brake)}")
silent_from = len(brake_pub)
time.sleep(0.6)                          # 30 ticks at 50 Hz
check(len(brake_pub) == silent_from,
      f"then SILENCE: 0 messages in the next 0.6 s (got "
      f"{len(brake_pub) - silent_from}; 30 zeros would mute autonomy)")
# and the flow restarts on the next press - the pad is not disarmed, just quiet
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
check(wait_for(lambda: len(brake_pub) - silent_from >= 20, 2.0),
      f"deadman pressed again -> the 50 Hz flow resumes "
      f"({len(brake_pub) - silent_from} messages)")
restart = brake_pub[silent_from:]
step = gp.SLEW_LINEAR_MS2 / 50.0        # 0.012 m/s per tick at 50 Hz
check(abs(restart[0].linear.x) <= step + 1e-9,
      f"... from rest: first vx = {restart[0].linear.x:.3f} m/s <= one slew "
      f"step ({step:.3f}), not the 0.438 of a stale ramp")
check(wait_for(lambda: close(brake_pub[-1].linear.x, LIN), 3.0),
      f"... and ramps back to the {LIN} m/s ceiling (stick still forward)")
brake_module.stop()


# --- H. brake cancel on a deadman press + the brake window knob -----------
# Two behaviours for two complaints a pilot can have: a quick release and
# re-press must not leave a zero behind the new command, and a released pad
# can be told to keep ordering a stop for longer before it falls silent.
print("\nH. brake cancel on deadman + the brake window knob")

# H1. the decision itself, pure. A window 0.3 s in the future, and the three
# states a tick can be in.
WINDOW = time.monotonic() + 0.3
check(gp.BRAKE_CANCEL_ON_DEADMAN is True,
      "BRAKE_CANCEL_ON_DEADMAN ships True (False = the window always runs to its end)")
check(gp.brake_window_after_press(WINDOW, True, False) == 0.0,
      "RISING edge (released -> held) -> window cancelled to 0.0, i.e. the past: "
      "the brake path cannot publish one more zero behind the new command")
check(gp.brake_window_after_press(WINDOW, True, True) == WINDOW,
      "deadman simply HELD -> window untouched (each driving tick re-arms its "
      "own, and that one must live its full brake_s after the NEXT release)")
check(gp.brake_window_after_press(WINDOW, False, True) == WINDOW,
      "the release itself -> window untouched (the brake has to happen)")
check(gp.brake_window_after_press(WINDOW, False, False) == WINDOW,
      "pad at rest -> window untouched")

# H2. the flag OFF: identity function.
gp.BRAKE_CANCEL_ON_DEADMAN = False
try:
    check(gp.brake_window_after_press(WINDOW, True, False) == WINDOW,
          "BRAKE_CANCEL_ON_DEADMAN=False -> the rising edge changes NOTHING "
          "(the window always runs to its end)")
    check(all(gp.brake_window_after_press(WINDOW, d, p) == WINDOW
              for d in (True, False) for p in (True, False)),
          "... and neither does any other combination: pure identity")
finally:
    gp.BRAKE_CANCEL_ON_DEADMAN = True

# H3. the knob on the window length, in SECONDS. Known value in, known out.
check(gp.BRAKE_S_ENV == "MOSS_BRAKE_S", f"the knob is {gp.BRAKE_S_ENV}")
check(gp.resolve_brake_s() == gp.BRAKE_S == 0.5,
      f"no argument, no env -> {gp.BRAKE_S} s, the default")
check(gp.resolve_brake_s(1.0) == 1.0, "brake_s=1.0 -> 1.00 s")
check(gp.resolve_brake_s(None, {"MOSS_BRAKE_S": "1.0"}) == 1.0,
      "MOSS_BRAKE_S=1.0 -> 1.00 s (the operator's lever: dimOS builds this "
      "module from a blueprint)")
check(gp.resolve_brake_s(0.8, {"MOSS_BRAKE_S": "0.2"}) == 0.8,
      "an explicit argument beats the environment (0.8 s wins over 0.2 s)")
check(gp.resolve_brake_s(None, {"MOSS_BRAKE_S": "3.0"}) == gp.BRAKE_S_MAX == 1.0,
      f"3.0 s clamped to {gp.BRAKE_S_MAX} s - each release mutes autonomy for "
      f"the window + dimOS's 1.0 s tele_cooldown_sec, at most 2.0 s")
check(gp.resolve_brake_s(None, {"MOSS_BRAKE_S": "-1"}) == 0.0,
      "a negative window clamped to 0.0 s (= no brake at all, but never a "
      "window in the past)")
check(gp.resolve_brake_s(None, {"MOSS_BRAKE_S": "abc"}) == gp.BRAKE_S,
      f"a mistyped MOSS_BRAKE_S gives the default window ({gp.BRAKE_S} s), "
      f"never a random one")
check(GamepadTeleop(rate_hz=50.0).brake_s == gp.BRAKE_S,
      f"a module built with no argument carries {gp.BRAKE_S} s")
check(GamepadTeleop(rate_hz=50.0, brake_s=1.0).brake_s == 1.0,
      "a module built with brake_s=1.0 carries 1.00 s")


def run_release_repress(module, watch_s, repress_after_s=None):
    """Drive a module to the ceiling, release the deadman, optionally press it
    again `repress_after_s` later, and watch for `watch_s` from the release.

    Returns [(dt seconds since the release, vx m/s)] - the zero Twist that
    Module.stop() publishes on shutdown is EXCLUDED, it belongs to the
    teardown, not to the brake.
    """
    js.pad = FakePad("Fake H", [0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
    log = []
    module.tele_cmd_vel.subscribe(lambda t: log.append((time.monotonic(), t.linear.x)))
    module.start()
    time.sleep(0.4)                                   # trust earned at neutral
    js.pad.buttons[gp.DEADMAN_BUTTON] = 1
    js.pad.axes = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0]    # full forward
    wait_for(lambda: bool(log) and close(log[-1][1], LIN), 3.0)
    js.pad.buttons[gp.DEADMAN_BUTTON] = 0             # RELEASE at the ceiling
    t_rel = time.monotonic()
    if repress_after_s is not None:
        time.sleep(repress_after_s)
        js.pad.buttons[gp.DEADMAN_BUTTON] = 1         # RE-PRESS
        time.sleep(max(0.0, watch_s - repress_after_s))
    else:
        time.sleep(watch_s)
    seen = len(log)                                   # cut BEFORE the shutdown zero
    module.stop()
    return [(t - t_rel, vx) for t, vx in log[:seen] if t > t_rel]


# H4. THE ordered case, at the real 50 Hz: released at speed, deadman pressed
# again at t = 0.200 s -> at t = 0.250 s the published command is the STICK'S,
# not a zero. (Physical units: vx in m/s; at 0.250 s the slew has had ~3 ticks
# at 0.6 m/s2 / 50 Hz = 0.012 m/s each, so vx is small but NOT zero.)
after = run_release_repress(GamepadTeleop(rate_hz=50.0), 0.45, repress_after_s=0.200)
at_250 = [(dt, vx) for dt, vx in after if 0.235 <= dt <= 0.275]
check(bool(at_250), f"a message exists around t = 0.250 s ({len(at_250)} of them)")
check(all(vx > 0.0 for _, vx in at_250),
      f"re-press at 0.200 s -> at 0.250 s the published command is the STICK'S, "
      f"not zero: {[f'{dt:.3f}s {vx:+.3f}m/s' for dt, vx in at_250]}")
# The invariant that matters on the bus: once the command flows again, NOTHING
# zero comes behind it. (Not "no zero after t=0.200": a tick reads the button at
# its start and publishes up to one period later, so a zero decided at 0.198 s
# can legitimately land at 0.218 s. That one is ahead of the command, not
# behind it, and no software can un-decide it.)
first_cmd = next((i for i, (dt, vx) in enumerate(after) if not close(vx, 0.0)), None)
check(first_cmd is not None, "the command flow restarts after the press")
behind = [(dt, vx) for dt, vx in after[first_cmd + 1:] if close(vx, 0.0)] if first_cmd is not None else [None]
check(not behind,
      f"... and NOT ONE zero comes BEHIND the restarted command "
      f"({len(behind)} zeros after the first command)")
brake_zeros = [dt for dt, vx in after if dt <= 0.200 and close(vx, 0.0)]
check(bool(brake_zeros) and max(brake_zeros) <= 0.205,
      f"the brake did run during the window: {len(brake_zeros)} zeros between "
      f"the release and the press")

# H5. no re-press at all: zeros for brake_s, then TOTAL silence. Physical
# units: 0.5 s x 50 Hz = 25 messages maximum, then nothing for 0.4 s.
lone = run_release_repress(GamepadTeleop(rate_hz=50.0), 0.9)
zeros = [dt for dt, vx in lone if close(vx, 0.0)]
check(bool(zeros) and all(close(vx, 0.0) for _, vx in lone),
      f"deadman released and never pressed again -> every message after it is a "
      f"zero Twist ({len(lone)} of them)")
check(bool(zeros) and max(zeros) <= gp.BRAKE_S + 0.05,
      f"... the last one lands at {max(zeros, default=0.0):.3f} s <= BRAKE_S "
      f"({gp.BRAKE_S} s), not later")
check(len(zeros) <= int(gp.BRAKE_S * 50) + 2,
      f"... i.e. at most {int(gp.BRAKE_S * 50) + 2} messages at 50 Hz: got {len(zeros)}")
late = [dt for dt in zeros if dt > gp.BRAKE_S + 0.05]
check(not late, f"then SILENCE: nothing published past {gp.BRAKE_S + 0.05:.2f} s "
                f"({len(late)} stragglers)")

# H6. the longer window, measured in seconds: brake_s = 1.0 s keeps the base
# being TOLD to stop for twice as long before the module goes quiet.
long_brake = run_release_repress(GamepadTeleop(rate_hz=50.0, brake_s=1.0), 1.4)
long_zeros = [dt for dt, vx in long_brake if close(vx, 0.0)]
check(bool(long_zeros) and 0.80 <= max(long_zeros) <= 1.05,
      f"brake_s=1.0 -> the zeros run to {max(long_zeros, default=0.0):.3f} s (0.80-1.05 s "
      f"window), twice the default 0.5 s")
check(len(long_zeros) > len(zeros),
      f"... and there are more of them than at 0.5 s ({len(long_zeros)} vs "
      f"{len(zeros)} at 50 Hz)")

# H7. the flag OFF, same scenario: the default behaviour does not depend on it.
# The module already publishes the stick command from the first tick after
# the press without the cancel; what the cancel buys is the right to run H6's
# longer window with no stale zero behind a fresh command.
gp.BRAKE_CANCEL_ON_DEADMAN = False
try:
    before_flag = run_release_repress(GamepadTeleop(rate_hz=50.0), 0.45,
                                      repress_after_s=0.200)
    at_250_off = [(dt, vx) for dt, vx in before_flag if 0.235 <= dt <= 0.275]
    check(bool(at_250_off) and all(vx > 0.0 for _, vx in at_250_off),
          f"BRAKE_CANCEL_ON_DEADMAN=False -> SAME result at 0.250 s "
          f"({[f'{vx:+.3f}m/s' for _, vx in at_250_off]}): the flag does not "
          f"change the default behaviour, it protects the longer window")
finally:
    gp.BRAKE_CANCEL_ON_DEADMAN = True


# H8. why the brake-cancel lines live BELOW the trust gate.
# A single missed joystick read - the pad is physically still there, one poll
# lies - while the pilot is rolling at the ceiling with the deadman held. The
# module drops the pad, re-acquires it, and has to re-earn trust at neutral;
# with the sticks still pushed it never does, so the ONLY thing it may still
# publish is the brake: zeros, until brake_until, then silence. That repetition
# is the whole point of BRAKE_S on a latest-only bus (LCM/zenoh).
# With the cancel running ABOVE the gate, the held deadman would look like a
# rising edge on the first tick after the re-acquisition, cancel the window,
# and the untrusted path would have nothing to publish in its place: 25 zeros
# over 0.486 s would become 1 zero at 0.011 s.
def drop_one_read(then_axes=None):
    """Make exactly ONE joystick poll fail - a single missed read, pad still
    plugged in. If `then_axes` is given, the sticks take that value at the very
    instant of the glitch (deterministic: no tick can slew in between)."""
    real_count = js.get_count
    fired = {"n": 0}

    def once():
        if fired["n"] == 0:
            fired["n"] = 1
            if then_axes is not None:
                js.pad.axes = list(then_axes)
            return 0          # "joystick disappeared" for this tick only
        return real_count()

    js.get_count = once
    return fired


def run_dropout(module, watch_s, then_axes=None):
    """Drive to the ceiling with the deadman HELD, glitch one read, watch.

    Returns [(dt seconds since the glitch, vx m/s)], the shutdown zero excluded.
    """
    js.pad = FakePad("Fake H8", [0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
    log = []
    module.tele_cmd_vel.subscribe(lambda t: log.append((time.monotonic(), t.linear.x)))
    module.start()
    time.sleep(0.4)                                   # trust earned at neutral
    js.pad.buttons[gp.DEADMAN_BUTTON] = 1
    js.pad.axes = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0]    # full forward
    wait_for(lambda: bool(log) and close(log[-1][1], LIN), 3.0)
    drop_one_read(then_axes)                          # ONE missed read
    t_glitch = time.monotonic()
    try:
        time.sleep(watch_s)
        seen = len(log)                               # cut BEFORE the shutdown zero
    finally:
        module.stop()
        try:
            del js.get_count                          # back to the real fake
        except AttributeError:
            pass
    return [(t - t_glitch, vx) for t, vx in log[:seen] if t > t_glitch]


drop = run_dropout(GamepadTeleop(rate_hz=50.0), 0.9)     # sticks stay pushed
drop_zeros = [dt for dt, vx in drop if close(vx, 0.0)]
check(all(close(vx, 0.0) for _, vx in drop),
      f"one missed read at the 0.45 m/s ceiling -> NOTHING but zeros comes out "
      f"(the pad has to re-earn trust at neutral): {len(drop)} messages")
check(len(drop_zeros) >= 15,
      f"... and the stop order is REPEATED, not said once: {len(drop_zeros)} "
      f"zero Twists at 50 Hz (1 if the cancel ran above the trust gate)")
check(bool(drop_zeros) and 0.40 <= max(drop_zeros) <= gp.BRAKE_S + 0.08,
      f"... the last of them lands at {max(drop_zeros, default=0.0):.3f} s, i.e. the full "
      f"BRAKE_S window ({gp.BRAKE_S} s) and not 0.011 s")
check(not [dt for dt in drop_zeros if dt > gp.BRAKE_S + 0.08],
      "... then SILENCE, as before: the window still ends on its own")

# H9. the slew state must not survive a pad loss. Same single missed read,
# but the sticks come back at NEUTRAL - so trust IS re-earned at once and the
# module drives again. Known input -> known output, in m/s: prev_vx must be 0,
# so the command is 0.000 m/s. A ramp that survived the loss would still hold
# 0.45 m/s and the first published command would be +0.438 m/s with nobody
# touching the sticks.
NEUTRAL = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
drop_n = run_dropout(GamepadTeleop(rate_hz=50.0), 0.9, then_axes=NEUTRAL)
peak = max((abs(vx) for _, vx in drop_n), default=0.0)
check(bool(drop_n), f"the module publishes again after the glitch ({len(drop_n)} messages)")
check(close(peak, 0.0),
      f"missed read at the ceiling + sticks at NEUTRAL -> peak |vx| = "
      f"{peak:.3f} m/s (0.438 m/s with a stale ramp; the ramp dies with the pad)")

# H10. the operator's lever on a run, end to end: the ENVIRONMENT of the
# process -> a module built with no brake_s argument, as a blueprint builds it.
os.environ["MOSS_BRAKE_S"] = "1.0"
try:
    check(GamepadTeleop(rate_hz=50.0).brake_s == 1.0,
          "MOSS_BRAKE_S=1.0 in the real environment -> a module built with no "
          "argument at all carries a 1.00 s window")
finally:
    del os.environ["MOSS_BRAKE_S"]
check(GamepadTeleop(rate_hz=50.0).brake_s == gp.BRAKE_S,
      f"... and with the variable gone again: back to {gp.BRAKE_S} s")

# no pygame at all: the module says so, with an install line that works
# (pygame is on PyPI; this package is not, so no "pip install <package>[extra]")
sys.modules["pygame"] = None            # makes `import pygame` raise ImportError
spy.lines.clear()
GamepadTeleop(rate_hz=50.0)._worker()   # returns at once without pygame
check(spy.count("pygame is not installed - gamepad teleop is off (pip install pygame)") == 1,
      f"no pygame -> one error line naming 'pip install pygame' ({spy.lines[-1:] if spy.lines else 'nothing logged'})")

gp.logger = real_logger
del sys.modules["pygame"]


# --- C. the real pygame, headless -----------------------------------------
print("\nC. real pygame with no display")
try:
    import pygame
except ImportError:
    print("  ..  pygame not installed here - headless check skipped")
else:
    logging.disable(logging.CRITICAL)  # keep SDL chatter out of the report
    n_ok, n_fail = pygame.init()
    pygame.joystick.init()
    pygame.event.pump()                      # the call that needs a display
    before = pygame.joystick.get_count()
    pygame.joystick.quit()
    pygame.joystick.init()                   # the hot-plug re-scan
    after_count = pygame.joystick.get_count()
    pygame.quit()
    logging.disable(logging.NOTSET)
    check(n_fail == 0, f"pygame.init() with SDL_VIDEODRIVER=dummy ({n_ok} modules up)")
    check(before == after_count, f"event.pump() and joystick re-scan work headless "
                                 f"({after_count} pad(s) connected)")

print("\nF. the safety nets: trust gate, ceilings, rim envelope")
from moss_dimos.gamepad import (  # noqa: E402
    CLAMP_ANGULAR_RADS, CLAMP_LINEAR_MS, DEFAULT_LEVER_M, DEFAULT_RIM_MS, axes_neutral, clamp_twist,
)

cfg = TeleopConfig()
# axes uninitialised at full deflection: neutral was never observed
check(axes_neutral(1.0, -1.0, 1.0, cfg.deadzone) is False,
      "full deflection = NOT neutral (the trust gate refuses it)")
check(axes_neutral(0.01, -0.02, 0.0, cfg.deadzone) is True,
      "pad at rest = neutral (the gate opens)")
# absolute ceilings: even an insane config never exceeds them
wild = TeleopConfig(linear_speed=5.0, angular_speed=9.0, boost_multiplier=4.0)
vx, vy, wz = axes_to_twist(-1.0, -1.0, 1.0, 1.0, wild)
check(abs(vx) <= CLAMP_LINEAR_MS + 1e-9, f"full stick + boost + insane config -> |vx| <= {CLAMP_LINEAR_MS}")
check(abs(vy) <= CLAMP_LINEAR_MS + 1e-9, f"... and |vy| <= {CLAMP_LINEAR_MS}")
check(abs(wz) <= CLAMP_ANGULAR_RADS + 1e-9, f"... and |wz| <= {CLAMP_ANGULAR_RADS}")
# clamp_twist ends with the rim envelope (translation and rotation ADD at the rim)
vx, vy, wz = clamp_twist(9.0, -9.0, 9.0)
check(abs(abs(vx) + abs(vy) + DEFAULT_LEVER_M * abs(wz) - DEFAULT_RIM_MS) < 1e-9,
      "pure clamp_twist: 9 m/s everywhere -> rim exactly at the envelope")
check(clamp_twist(9.0, 0.0, 0.0) == (0.45, 0.0, 0.0),
      "no envelope given -> the conservative default ceiling, 0.45 m/s")
# a single stick at full travel loses NOTHING (the envelope = the feel of one stick alone)
check(clamp_twist(0.45, 0.0, 0.0) == (0.45, 0.0, 0.0), "pure forward 0.45 -> unchanged")
check(clamp_twist(0.0, 0.0, 0.8) == (0.0, 0.0, 0.8), "pure rotation 0.8 -> unchanged (rim 0.40 < 0.45)")
# full forward + full rotation on the default envelope
vx, vy, wz = clamp_twist(0.45, 0.0, 0.8)
rim = abs(vx) + DEFAULT_LEVER_M * abs(wz)
check(abs(rim - DEFAULT_RIM_MS) < 1e-9,
      f"forward+rotation mix -> rim {rim:.3f} = envelope {DEFAULT_RIM_MS} (0.85 unclamped)")
check(abs(vx / wz - 0.45 / 0.8) < 1e-9,
      "... and the proportions of the gesture are kept (vx/wz constant)")


print("\nG. the teleop ramp")
from moss_dimos.gamepad import slew  # noqa: E402
check(abs(slew(0.0, 0.45, 0.1) - 0.06) < 1e-9,
      "no jump: 0 -> full stick limited to 0.06 m/s per 0.1 s step")
check(abs(slew(0.45, 0.0, 0.1) - 0.39) < 1e-9, "ramp down limited too (0.45 -> 0)")
check(slew(0.05, 0.06, 0.1) == 0.06, "a near target is reached exactly")

print()
print(f"{OK} OK, {KO} KO")
print("TEST " + ("PASSED" if KO == 0 else "FAILED"))
raise SystemExit(0 if KO == 0 else 1)
