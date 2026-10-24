"""Gamepad teleop for MOSS as a dimOS module: pygame.joystick in, Twist out.

dimOS ships keyboard, phone and Quest teleop but no gamepad; this module
fills the gap. It publishes on ``tele_cmd_vel``: wire it through dimOS's
MovementManager so navigation and teleop coexist, or remap it to ``cmd_vel``
for direct drive.

Mapping (standard dual-stick):
    left stick Y  -> vx (forward)     left stick X  -> vy (strafe)
    right stick X -> wz (rotation)    R2 held       -> boost
The tracked base ignores vy (moss_adapter); it is still published, so the
same module drives a holonomic base unchanged.

Publishes ONLY while a stick is out of its deadzone - there is no deadman
by default: a small tracked rover at walking pace does not need a held button,
and MOSS_PAD_DEADMAN=r1 (or an SDL button number) brings one back - plus a
brake of zeros right after the sticks return to centre (or the optional
deadman is released) - 0.5 s by default, up to 1.0 s with MOSS_BRAKE_S - then
silence (see BRAKE_S and resolve_brake_s). A pad lying at rest owns nothing,
so the same stack can explore on its own while the pad is plugged in.
Moving a stick again (or pressing the deadman again) ends that brake window
on the spot (BRAKE_CANCEL_ON_DEADMAN), but only on a tick that is trusted and
about to publish - see the worker.

Wiring in this package's blueprints (moss_dimos.blueprints):
    moss-gamepad, moss-nav  tele_cmd_vel is REMAPPED to cmd_vel: direct
                            drive, no MovementManager in the path
    moss-explore            tele_cmd_vel is NOT remapped: the Twist goes
                            tele_cmd_vel -> MovementManager -> cmd_vel, and
                            any pad message cancels the current nav goal

Runs headless: the robot's computer has no display, so SDL is pinned to its
dummy video/audio drivers below. The pad is hot-pluggable - the module waits
for one instead of giving up at startup, and goes back to waiting if it is
unplugged, so ``dimos run moss-dimos.moss-gamepad`` can be started before the
receiver is in.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
from collections.abc import Mapping
import threading
import time
from typing import Any

# SDL picks its drivers at import time, so this has to run before "import
# pygame" (which happens lazily in the worker thread below). setdefault, not
# assignment: an operator who exports SDL_VIDEODRIVER=x11 keeps their display.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")  # no banner in daemon logs

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Generic defaults. The MOSS blueprints always pass the base's own scale
# (moss_adapter.teleop_speeds), so these are only the fallback.
DEFAULT_LINEAR_SPEED = 0.6    # m/s at full stick
DEFAULT_ANGULAR_SPEED = 1.2   # rad/s at full stick
DEFAULT_DEADZONE = 0.32
DEFAULT_RATE_HZ = 50.0
DEFAULT_BOOST = 2.0
RESCAN_PERIOD_S = 2.0         # how often we re-scan SDL for a pad

# SDL axis indices on a standard dual-stick pad (DS4/DS5, Xbox, 8BitDo).
# The pad's LAYOUT depends on how it shows up: the MOSS pad on its 2.4 GHz dongle in Android mode
# (2563:0526) is the default; the same pad on its USB cable enumerates as an Xbox 360 pad (045e:028e,
# kernel xpad), where R1 is button 5, the right stick X is axis 3 and the triggers are axes 2 and 5.
# MOSS_PAD_LAYOUT=android|xinput picks a set; MOSS_PAD_AXIS_RIGHT_X / MOSS_PAD_AXIS_R2 override one
# value each; MOSS_PAD_DEADMAN adds a held button (none by default: r1 = the layout's R1, or an SDL
# button number); MOSS_PAD_INVERT_Y=1 / MOSS_PAD_INVERT_TURN=1 flip a stick whose sign is the other way
# round (a pad that reports stick UP as positive drives backwards on a forward push); MOSS_PAD_INDEX
# picks the SDL joystick when several are plugged (the mute dongle is usually index 0). All read once
# at import.
PAD_LAYOUTS = {
    "android": {"r1": 7, "axis_right_x": 2, "axis_r2": 5},   # the shanwan receiver in Android mode (2563:0526)
    "xinput": {"r1": 5, "axis_right_x": 3, "axis_r2": 5},    # an Xbox 360 pad to the kernel (xpad): the shanwan pad on its cable
    "ds4": {"r1": 5, "axis_right_x": 2, "axis_r2": 4},       # a DualShock 4 to the kernel (hid-generic): the GameSir 2.4G receiver in its DInput mode
}
DEADMAN_OFF = ("", "none", "off", "no")   # MOSS_PAD_DEADMAN values that mean: no deadman
JS_SYSFS_GLOB = "/sys/class/input/js*/device/name"


def layout_for_name(name: str | None) -> str:
    """The layout a pad's kernel name calls for: "Sony ... Wireless Controller" / DualShock -> ds4,
    "X-Box" / Xbox -> xinput, anything else (the shanwan Android GamePad included) -> android. Pure."""
    n = (name or "").lower()
    if "sony" in n or "dualshock" in n or "wireless controller" in n:
        return "ds4"
    if "x-box" in n or "xbox" in n or "x box" in n:
        return "xinput"
    return "android"


def detect_pad_name() -> str | None:
    """The kernel name of the first /dev/input/js* (sysfs), None without one."""
    import glob
    for path in sorted(glob.glob(JS_SYSFS_GLOB)):
        try:
            with open(path) as f:
                return f.read().strip()
        except OSError:
            continue
    return None


def _on(raw: str | None) -> bool:
    """An on/off switch as the stack reads them: 1 / true / yes / on."""
    return (raw or "").strip().lower() in ("1", "true", "yes", "on")


def pad_layout(env: Mapping[str, str] | None = None, pad_name: str | None = None) -> dict[str, int | None]:
    """The button / axis numbers in force: a named layout, then one-value overrides. Pure when
    the layout is named; MOSS_PAD_LAYOUT=auto picks it from the pad's kernel name (pad_name, else
    the first js device in sysfs), so the same robot.env serves a pad that shows up as a
    DualShock 4 one day and as an Xbox 360 pad the next.
    "deadman" is None by default (the sticks drive); MOSS_PAD_DEADMAN=r1 is the layout's R1,
    an integer is that SDL button, none / off / empty is no deadman."""
    e = os.environ if env is None else env
    name = (e.get("MOSS_PAD_LAYOUT") or "android").strip().lower()
    if name == "auto":
        name = layout_for_name(detect_pad_name() if pad_name is None else pad_name)
    base = PAD_LAYOUTS.get(name, PAD_LAYOUTS["android"])
    out: dict[str, int | None] = {"deadman": None, "axis_right_x": base["axis_right_x"], "axis_r2": base["axis_r2"], "name": name,
                                  "y_sign": -1 if _on(e.get("MOSS_PAD_INVERT_Y")) else 1,
                                  "turn_sign": -1 if _on(e.get("MOSS_PAD_INVERT_TURN")) else 1}
    raw = (e.get("MOSS_PAD_DEADMAN") or "").strip().lower()
    if raw == "r1":
        out["deadman"] = base["r1"]
    elif raw not in DEADMAN_OFF:
        try:
            out["deadman"] = int(raw)
        except ValueError:
            pass
    for key, var in (("axis_right_x", "MOSS_PAD_AXIS_RIGHT_X"), ("axis_r2", "MOSS_PAD_AXIS_R2")):
        raw = (e.get(var) or "").strip()
        if raw:
            try:
                out[key] = int(raw)
            except ValueError:
                pass
    return out


def pad_index(env: Mapping[str, str] | None = None) -> int:
    """MOSS_PAD_INDEX, else 0. Pure."""
    raw = ((os.environ if env is None else env).get("MOSS_PAD_INDEX") or "").strip()
    try:
        return max(0, int(raw)) if raw else 0
    except ValueError:
        return 0


_LAYOUT = pad_layout()
LAYOUT_NAME = _LAYOUT["name"]       # the layout in force (auto resolved at import, from the first js device)
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = _LAYOUT["axis_right_x"]   # android: measured on the 2563:0526 pad, right-stick X sweeps axis 2 over its full travel (axis 3 is its vertical, which leaks a little)
AXIS_R2 = _LAYOUT["axis_r2"]
Y_SIGN = _LAYOUT["y_sign"]          # -1 with MOSS_PAD_INVERT_Y: the pad reports stick up as positive
TURN_SIGN = _LAYOUT["turn_sign"]    # -1 with MOSS_PAD_INVERT_TURN: the pad reports stick right as negative


@dataclass(frozen=True)
class TeleopConfig:
    """Speed envelope for :func:`axes_to_twist` - no pygame, no module."""

    linear_speed: float = DEFAULT_LINEAR_SPEED
    angular_speed: float = DEFAULT_ANGULAR_SPEED
    deadzone: float = DEFAULT_DEADZONE
    boost_multiplier: float = DEFAULT_BOOST
    envelope: Any = None          # TwistEnvelope (defined below); None = DEFAULT_ENVELOPE


def apply_deadzone(value: float, deadzone: float) -> float:
    """Hard deadzone: below the threshold the axis reads zero.

    No rescaling of the remaining travel - a stick just past the deadzone
    gives a small command, which is what you want for fine positioning.
    """
    return 0.0 if abs(value) < deadzone else value


# Absolute ceilings, applied LAST, regardless of config or boost. A userspace
# pad driver that has just started can report uninitialised axes at full
# deflection; without a last gate those go straight onto tele_cmd_vel (a
# priority channel). No teleop output may ever exceed these.
CLAMP_LINEAR_MS = 0.45
SLEW_LINEAR_MS2 = 0.6     # max change of linear command per second: full stick
                          # reaches 0.45 m/s in ~0.75 s instead of one step, so
                          # the base never sees a step command from the pad
CLAMP_ANGULAR_RADS = 0.8
# The rim envelope: translation and rotation ADD at the wheels (rim speed =
# |vx| + |vy| + lever * |wz|), so each axis under its own clamp can still
# drive the wheels near double. The fallback below is conservative: a long
# lever and the rim held to the fastest single-stick feel (0.45 m/s). MOSS
# passes its own envelope, sized to the tracks (moss_adapter.teleop_envelope).
DEFAULT_LEVER_M = 0.50    # m of rim speed per rad/s of rotation
DEFAULT_RIM_MS = 0.45     # max rim speed however the sticks are mixed
DEADMAN_BUTTON = _LAYOUT["deadman"]   # None = no deadman, the sticks drive (default). MOSS_PAD_DEADMAN=r1: SDL button 7 on the 2563:0526 pad in Android mode, 5 on the cable (xinput). Held = commands allowed; released = zeros, always
BRAKE_S = 0.5             # how long the zeros keep flowing after the deadman is
                          # released - then TOTAL SILENCE until it is held again.
                          # dimOS's MovementManager reads ANY tele_cmd_vel message,
                          # zeros included, as "a human is driving": it cancels the
                          # nav goal and mutes nav_cmd_vel for tele_cooldown_sec
                          # (1.0 s). A pad at rest publishing 50 Hz of zeros would
                          # therefore stop exploration for good.
                          # 0.5 s < 1.0 s: the brake is seen, then autonomy gets
                          # the bus back and the two coexist.
# When the pad's radio dies mid-drive (sleep, battery), this module with this
# pad has been seen to stop the base cleanly, so there is no radio watchdog
# here. Revisit only if a radio loss ever leaves a non-zero command running.


BRAKE_CANCEL_ON_DEADMAN = True
# A new deadman press CANCELS whatever is left of the brake window, on its
# RISING EDGE (brake_until = 0.0). From that instant the brake path cannot emit
# one more zero, however long the window was.
#
# At the default BRAKE_S = 0.5 s and 50 Hz, with the deadman re-pressed 0.200 s
# after the release, the module publishes the stick command from the first
# tick after the press either way (+0.213 s -> 0.012 m/s, one slew step) and no
# zero after it: the cancel does not change the default behaviour, and the lag
# felt on a quick re-press is the slew restarting from rest (0.6 m/s2 = 0.75 s
# to the ceiling). What the cancel buys is the right to LENGTHEN the zero
# window (brake_s below) without a stale zero from the release ever chasing a
# fresh command down the bus.
# False = the window always runs to its end.

# The length of that window is a knob. 0.5 s is the default (see BRAKE_S);
# raising it towards 1.0 s keeps the base being TOLD to stop for longer
# instead of falling silent and coasting. dimOS builds this module from a
# blueprint, so the environment of the `dimos run` process is the operator's
# lever (workers inherit it):
#     MOSS_BRAKE_S=1.0 dimos run moss-dimos.moss-gamepad
BRAKE_S_ENV = "MOSS_BRAKE_S"
BRAKE_S_MAX = 1.0
# Why the ceiling is 1.0 and not more - the cost, in seconds.
# dimOS's MovementManager treats ANY tele_cmd_vel message, zeros included, as
# "a human is driving": _on_teleop cancels the nav goal on EVERY message and
# _on_nav refuses to forward nav_cmd_vel until tele_cooldown_sec = 1.0 s has
# passed since the LAST teleop message (movement_manager.py). So on
# moss-explore every release mutes autonomy for brake_s + 1.0 s: 1.5 s at the
# default, 2.0 s at this ceiling. The zeros always stop at brake_s, whatever
# its value; the ceiling only bounds that muted time.


def resolve_brake_s(brake_s: float | None = None,
                    env: dict | None = None) -> float:
    """How long zeros keep flowing after the deadman is released, in seconds.

    Pure, cold-testable. Precedence: explicit argument > MOSS_BRAKE_S >
    BRAKE_S (0.5 s). Out-of-range values are CLAMPED to [0.0, BRAKE_S_MAX]
    and logged; unparseable text falls back to BRAKE_S, loudly - a mistyped
    knob gives the default window, never a random one.
    """
    raw: float | None = None if brake_s is None else float(brake_s)
    if raw is None:
        text = (os.environ if env is None else env).get(BRAKE_S_ENV, "")
        text = text.strip() if isinstance(text, str) else ""
        if text:
            try:
                raw = float(text)
            except ValueError:
                logger.warning("%s=%r is not a number - brake window stays at "
                               "%.2f s", BRAKE_S_ENV, text, BRAKE_S)
                raw = None
    if raw is None:
        return BRAKE_S
    value = max(0.0, min(BRAKE_S_MAX, raw))
    if value != raw:
        logger.warning("brake window %.2f s is outside [0.0, %.2f] s - clamped "
                       "to %.2f s (each release mutes dimOS autonomy for the "
                       "window + 1.0 s; see BRAKE_S_MAX)",
                       raw, BRAKE_S_MAX, value)
    return value


def brake_window_after_press(brake_until: float, deadman: bool,
                             prev_deadman: bool) -> float:
    """The brake deadline this tick keeps - pure, cold-testable.

    RISING edge of the deadman (released -> held) cancels the window by
    returning 0.0, which is always in the past: the brake path can no longer
    publish. Anything else returns the window untouched - in particular a
    deadman simply HELD, because every driving tick re-arms the window itself
    and that one must live its full brake_s after the next release.

    With BRAKE_CANCEL_ON_DEADMAN = False this is the identity function.
    """
    if BRAKE_CANCEL_ON_DEADMAN and deadman and not prev_deadman:
        return 0.0
    return brake_until


def slew(prev: float, target: float, dt: float) -> float:
    """Rate-limit a linear command - pure, cold-testable."""
    step = SLEW_LINEAR_MS2 * dt
    return max(prev - step, min(prev + step, target))


@dataclass(frozen=True)
class TwistEnvelope:
    """The ceilings of clamp_twist. The defaults are the conservative
    fallback above. MOSS passes its own (moss_adapter.teleop_envelope): a
    tracked base's rim is |vx| + |wz| * track/2."""
    linear: float = CLAMP_LINEAR_MS
    angular: float = CLAMP_ANGULAR_RADS
    lever: float = DEFAULT_LEVER_M
    rim: float = DEFAULT_RIM_MS


DEFAULT_ENVELOPE = TwistEnvelope()


def clamp_twist(vx: float, vy: float, wz: float,
                env: TwistEnvelope = DEFAULT_ENVELOPE) -> tuple[float, float, float]:
    """The last gate before the bus - pure, cold-testable. Per-axis clamps,
    then the rim envelope: translation and rotation ADD at the rim, so a
    mixed command is scaled down proportionally (the feel is preserved, the
    top speed is not exceeded)."""
    lim = env.linear
    vx = max(-lim, min(lim, vx))
    vy = max(-lim, min(lim, vy))
    wz = max(-env.angular, min(env.angular, wz))
    rim = abs(vx) + abs(vy) + env.lever * abs(wz)
    if rim > env.rim:
        k = env.rim / rim
        vx, vy, wz = vx * k, vy * k, wz * k
    return (vx, vy, wz)


def axes_neutral(ax0: float, ax1: float, ax2: float, deadzone: float) -> bool:
    """True when every drive axis rests inside the deadzone - the trust gate:
    a pad (or a userspace driver) must be SEEN at neutral once before a single
    non-zero command is believed. Uninitialised axes never pass this."""
    return abs(ax0) < deadzone and abs(ax1) < deadzone and abs(ax2) < deadzone


def sticks_active(ax0: float, ax1: float, ax2: float, deadzone: float) -> bool:
    """True when a drive axis is out of its deadzone. Without a deadman this IS
    the "command present" condition: the exact complement of axes_neutral, so
    an inactive tick is a tick whose twist would have been zero anyway."""
    return not axes_neutral(ax0, ax1, ax2, deadzone)


def axes_to_twist(ax0: float, ax1: float, ax2: float, ax5: float,
                  cfg: TeleopConfig) -> tuple[float, float, float]:
    """Raw SDL axis values -> (vx m/s, vy m/s, wz rad/s).

    Pure function - the whole mapping lives here so it is testable without a
    pad, a display, or pygame.

    Each argument is named after its SDL axis index (AXIS_LEFT_X = 0,
    AXIS_LEFT_Y = 1, AXIS_RIGHT_X = 2, AXIS_R2 = 5).

    SDL reports stick up and stick left as NEGATIVE, while the robot frame is
    x forward / y left / wz counter-clockwise, hence the three sign flips:
        ax0 left stick X  -> vy   (stick left  = -1 -> strafe left,  +y)
        ax1 left stick Y  -> vx   (stick up    = -1 -> forward,      +x)
        ax2 right stick X -> wz   (stick right = +1 -> clockwise,    -wz)
        ax5 R2            -> boost when > 0 (trigger rests at -1)

    Boost multiplies the linear axes only: rotation stays at its capped rate,
    because a spin fast enough to be useful is already fast enough to hurt.
    """
    boost = cfg.boost_multiplier if ax5 > 0.0 else 1.0
    vx = -apply_deadzone(ax1, cfg.deadzone) * cfg.linear_speed * boost
    vy = -apply_deadzone(ax0, cfg.deadzone) * cfg.linear_speed * boost
    wz = -apply_deadzone(ax2, cfg.deadzone) * cfg.angular_speed
    return clamp_twist(vx, vy, wz, cfg.envelope or DEFAULT_ENVELOPE)


class GamepadTeleop(Module):
    """pygame.joystick teleop. Outputs Twist on tele_cmd_vel."""

    dedicated_worker = True

    tele_cmd_vel: Out[Twist]

    def __init__(self, linear_speed: float = DEFAULT_LINEAR_SPEED,
                 angular_speed: float = DEFAULT_ANGULAR_SPEED,
                 deadzone: float = DEFAULT_DEADZONE,
                 rate_hz: float = DEFAULT_RATE_HZ,
                 boost_multiplier: float = DEFAULT_BOOST,
                 joystick_index: int | None = None, brake_s: float | None = None,
                 envelope: Any = None,
                 **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # envelope: a TwistEnvelope, or its dict (blueprints carry plain
        # values), or None = DEFAULT_ENVELOPE. MOSS passes moss_adapter.teleop_envelope().
        if isinstance(envelope, dict):
            envelope = TwistEnvelope(**envelope)
        self.envelope = envelope or DEFAULT_ENVELOPE
        self.cfg = TeleopConfig(linear_speed=linear_speed,
                                angular_speed=angular_speed,
                                deadzone=deadzone,
                                boost_multiplier=boost_multiplier,
                                envelope=self.envelope)
        self.rate_hz = rate_hz
        self.joystick_index = pad_index() if joystick_index is None else joystick_index
        # None -> MOSS_BRAKE_S, else BRAKE_S (0.5 s). Resolved HERE, at
        # construction, so the value is fixed and logged once per run - not
        # re-read per tick.
        self.brake_s = resolve_brake_s(brake_s)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # @rpc is not decoration for its own sake: Module.start/stop carry it,
    # and an override that drops it falls out of the class's rpcs table.
    # dimOS then proxies the call by pickling the module across the worker
    # pipe, which dies on our threading.Event ('cannot pickle _thread.lock').
    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker,
                                        name="moss-gamepad", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        super().stop()

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        self.tele_cmd_vel.publish(Twist(
            linear=Vector3(x=vx, y=vy, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=wz)))

    def _acquire(self, pygame: Any) -> Any:
        """Re-scan SDL for the pad. Returns an initialised joystick or None."""
        try:
            pygame.event.pump()          # let SDL process hotplug events
            pygame.joystick.quit()       # full re-scan: SDL caches the device
            pygame.joystick.init()       # list from the last init()
            if pygame.joystick.get_count() <= self.joystick_index:
                return None
            pad = pygame.joystick.Joystick(self.joystick_index)
            pad.init()
            return pad
        except pygame.error as exc:
            logger.debug("Gamepad scan failed: %s", exc)
            return None

    # Named _worker, NOT _loop: dimOS's Module keeps its asyncio event
    # loop in self._loop, which would shadow the method and make the
    # thread target the event loop object.
    def _worker(self) -> None:
        try:
            import pygame
        except ImportError:
            logger.error("pygame is not installed - gamepad teleop is off "
                         "(pip install pygame)")
            return

        pygame.init()  # dummy video/audio drivers: works with no display
        pygame.joystick.init()
        period = 1.0 / self.rate_hz
        pad = None
        trusted = False            # neutral-first trust gate
        prev_vx = prev_vy = 0.0    # slew-limiter state
        brake_until = 0.0          # publish zeros until this instant, then shut up
        prev_active = False        # to see the RISING edge of a command (stick out, or deadman pressed)
        waiting_logged = False
        logger.info("gamepad layout: %s (right stick X = axis %d, R2 = axis %d)", LAYOUT_NAME, AXIS_RIGHT_X, AXIS_R2)
        if DEADMAN_BUTTON is None:
            logger.info("gamepad: no deadman - a stick out of its deadzone drives, centred sticks stop "
                        "(MOSS_PAD_DEADMAN=r1 brings the held button back)")
        else:
            logger.info("gamepad: deadman = SDL button %d, held to drive", DEADMAN_BUTTON)
        logger.info("gamepad brake window: %.2f s of zeros after the sticks return to centre%s, "
                    "then silence (cancel-on-press: %s)",
                    self.brake_s,
                    "" if DEADMAN_BUTTON is None else " or the deadman is released",
                    BRAKE_CANCEL_ON_DEADMAN)
        try:
            while not self._stop_event.is_set():
                if pad is None:
                    pad = self._acquire(pygame)
                    if pad is None:
                        if not waiting_logged:
                            logger.info("waiting for gamepad (index %d)",
                                        self.joystick_index)
                            waiting_logged = True
                        self._stop_event.wait(RESCAN_PERIOD_S)
                        continue
                    waiting_logged = False
                    trusted = False   # every (re)connection re-earns trust at neutral
                    logger.info("Gamepad connected: %s - waiting to see the sticks at NEUTRAL "
                                "before trusting a single command", pad.get_name())
                try:
                    pygame.event.pump()
                    axes = (_axis(pad, AXIS_LEFT_X), Y_SIGN * _axis(pad, AXIS_LEFT_Y),
                            TURN_SIGN * _axis(pad, AXIS_RIGHT_X), _axis(pad, AXIS_R2))
                    if DEADMAN_BUTTON is None:
                        deadman = True        # no held button required
                    else:
                        deadman = bool(pad.get_button(DEADMAN_BUTTON)) if pad.get_numbuttons() > DEADMAN_BUTTON else False
                    if pygame.joystick.get_count() <= self.joystick_index:
                        raise pygame.error("joystick disappeared")
                except pygame.error as exc:
                    pad = None
                    trusted = False
                    prev_active = False           # a pad that is gone holds nothing
                    # ... and it holds no RAMP either. The slew state must die
                    # with the pad: otherwise a pad that vanished for one read
                    # while driving at the ceiling comes back with 0.45 m/s of
                    # slew state, and the first trusted tick, STICKS AT
                    # NEUTRAL, publishes +0.438 m/s - the rover leaves on its
                    # own for ~0.75 s. The trust gate does not catch it:
                    # neutral axes satisfy the gate, and the danger is inside
                    # this module.
                    prev_vx = prev_vy = 0.0
                    self._publish(0.0, 0.0, 0.0)  # one zero Twist, then wait
                    logger.warning("Gamepad lost (%s) - back to waiting", exc)
                    continue
                # TRUST GATE: until the sticks have been SEEN at neutral once,
                # this pad's words are worth nothing.
                if not trusted:
                    if axes_neutral(axes[0], axes[1], axes[2], self.cfg.deadzone):
                        trusted = True
                        logger.info("Gamepad axes seen at neutral - commands now trusted (%s)",
                                    "move a stick to drive" if DEADMAN_BUTTON is None
                                    else "hold the deadman button to drive")
                    else:
                        if time.monotonic() < brake_until:
                            self._publish(0.0, 0.0, 0.0)
                        time.sleep(period)
                        continue
                # BRAKE CANCEL: the RISING edge of the deadman ends the brake
                # window there and then, so not one zero left over from the
                # release can still go out behind the commands of the new
                # press. Rising edge, not "held": a window re-armed by a tick
                # that DROVE (below) must keep its own life.
                #
                # ORDER MATTERS: these two lines sit BELOW the trust gate. On
                # a pad that has not yet earned trust - after EVERY
                # reconnection, and after every one-tick read glitch, which is
                # exactly when the rover may be rolling with nobody commanding
                # it - a deadman press above the gate would cancel the brake
                # window while the untrusted path can publish NOTHING to
                # replace it: a single missed read at 0.45 m/s would cut the
                # zeros published after the dropout from 25 (to 0.486 s) down
                # to 1 (0.011 s). BRAKE_S exists to REPEAT the stop order on a
                # latest-only bus; the cancel may therefore only run on a tick
                # that is itself about to publish a command - i.e. here.
                # A COMMAND IS PRESENT when the optional deadman is held (always, when there
                # is none) AND, without a deadman, a stick is out of its deadzone: a pad lying
                # at rest publishes nothing either way.
                active = deadman and (DEADMAN_BUTTON is not None
                                      or sticks_active(axes[0], axes[1], axes[2], self.cfg.deadzone))
                brake_until = brake_window_after_press(brake_until, active,
                                                       prev_active)
                prev_active = active
                # NO COMMAND (sticks centred, or the optional deadman released): no motion - ever.
                if not active:
                    # Releasing also resets the slew state: the ramp restarts
                    # from rest, so a re-press with the sticks centred cannot
                    # replay the last driven speed.
                    prev_vx = prev_vy = 0.0
                    # BRAKE THEN SILENCE (see BRAKE_S / self.brake_s): zeros for
                    # brake_s after the last command, then not one message until
                    # the deadman is held again - a released pad must not mute
                    # autonomy.
                    if time.monotonic() < brake_until:
                        self._publish(0.0, 0.0, 0.0)
                    time.sleep(period)
                    continue
                vx, vy, wz = axes_to_twist(*axes, self.cfg)
                vx = slew(prev_vx, vx, period)
                vy = slew(prev_vy, vy, period)
                prev_vx, prev_vy = vx, vy
                # envelope AFTER the slew too: while vx decays and wz is
                # instant, the mix could transiently exceed the rim ceiling
                vx, vy, wz = clamp_twist(vx, vy, wz, self.envelope)
                self._publish(vx, vy, wz)
                brake_until = time.monotonic() + self.brake_s
                time.sleep(period)
        finally:
            try:
                self._publish(0.0, 0.0, 0.0)
            except Exception:  # stream already torn down: nothing to do
                logger.debug("no zero Twist on shutdown", exc_info=True)
            pygame.joystick.quit()
            pygame.quit()


def _axis(pad: Any, index: int) -> float:
    """Axis value, or 0.0 when this pad has no such axis.

    Pads differ (some report 4 axes, not 6). A missing R2 must read as "no
    boost", not throw the loop into the reconnect path on every tick.
    """
    if index >= pad.get_numaxes():
        return 0.0
    return float(pad.get_axis(index))
