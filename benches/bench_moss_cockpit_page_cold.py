"""Cold bench for the cockpit page's decision panel and command arrow (tools/moss_cockpit/).

Known input -> known output: metres on the map and pixels on the canvas for the arrow,
rows, colours and words for the panel. Groups:

  A. the page      - index.html carries #mod-decisions ("What it decides.") above #mod-log in
                     the right column, #now-word, #now-detail and the #decisions list (an <ol>),
                     dimmed offline like the log; before the first answer it says '--', not IDLE;
                     on a phone it stacks after the controls and before the log; every id is
                     unique and every id app.js reads exists
  B. the colours   - the state word: warning for OBSTACLE / BACKING OFF / STUCK, accent for
                     HEADING, dim for the rest; the kinds: contact / push / imu warning,
                     obstacle / replan amber, goal / new_goal / reached accent, waiting /
                     goal_dropped / retry dim; amber is one token (--ui-caution), the depth
                     camera's colour on the map
  C. the wiring    - app.js polls /decisions.json?n=30 every 1000 ms through the same poller
                     as the others, draws the arrow from nav.last_cmd_vel at the robot through
                     pose(), and talks to no other host (relative paths only)
  D. in node       - node --check app.js; the page's own functions run on known input:
                     the arrow in metres (0.45 m/s = 0.6 m, reverse behind the robot, an arc
                     bends left for a left turn, a pivot sweeps the turn's direction, zero and
                     stale commands draw nothing) and on a recording canvas in pixels: it starts
                     at the marker's nose (tail in reverse) and runs at least 22 px past it, so
                     at the fitted zoom (37 px/m) a 0.2 m/s command shows; a pivot at any heading
                     starts at the nose; the feed (newest on top, 30 rows, mm:ss, kind pill and
                     colour, text escaped, only new rows flash); the state word (colour class,
                     detail, time since it was set); and the panel itself: pollDecisions and
                     renderDecisions on a stub document (word, colour, detail, 'for N s', the
                     newest row on top, an error answer keeping the feed, a failed fetch keeping
                     everything)
  E. the server    - when the cockpit serves /decisions.json (demo mode), its answer goes through
                     the page's functions: one row per event, every kind and word known

Nothing here needs dimOS, a bus or hardware; D needs node (skipped with a line without it).

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_moss_cockpit_page_cold.py
"""

import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import urllib.error
import urllib.request
from datetime import datetime, timezone
from html.parser import HTMLParser
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PAGE = ROOT / "tools" / "moss_cockpit"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

OK = 0
KO = 0
_TMPDIR = tempfile.TemporaryDirectory(prefix="moss_cockpit_page_bench_")
os.environ["MOSS_STATE_DIR"] = os.path.join(_TMPDIR.name, "state")
os.environ["XDG_STATE_HOME"] = os.path.join(_TMPDIR.name, "xdg")     # an empty dimOS run registry


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


html = (PAGE / "index.html").read_text()
js = (PAGE / "app.js").read_text()
tokens = (PAGE / "tokens.css").read_text()

# the decision catalogue (kinds) and the state words, as the server sends them
KINDS = {"contact": "warn", "push": "warn", "imu": "warn", "obstacle": "caution", "replan": "caution",
         "goal": "accent", "new_goal": "accent", "reached": "accent",
         "waiting": "dim", "goal_dropped": "dim", "retry": "dim"}
ALL_KINDS = list(KINDS) + ["pivot_fallback", "mission"]
WORDS = {"OBSTACLE": "warn", "BACKING OFF": "warn", "STUCK": "warn", "HEADING": "accent",
         "IDLE": "dim", "READY": "dim", "STOPPED": "dim", "DONE": "dim", "WAITING": "dim"}


class Elements(HTMLParser):
    """Every element with an id: tag, offset in the file, and the ids of its open ancestors."""

    VOID = {"meta", "link", "img", "input", "br", "hr", "source", "path"}

    def __init__(self):
        super().__init__()
        self.stack, self.found, self.ids = [], {}, []

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if "id" in a:
            self.ids.append(a["id"])
            self.found.setdefault(a["id"], {"tag": tag, "pos": self.getpos(), "attrs": a,
                                            "in": [s[1] for s in self.stack if s[1]]})
        if tag not in self.VOID:
            self.stack.append((tag, a.get("id")))

    def handle_startendtag(self, tag, attrs):
        a = dict(attrs)
        if "id" in a:
            self.ids.append(a["id"])
            self.found.setdefault(a["id"], {"tag": tag, "pos": self.getpos(), "attrs": a,
                                            "in": [s[1] for s in self.stack if s[1]]})

    def handle_endtag(self, tag):
        for i in range(len(self.stack) - 1, -1, -1):
            if self.stack[i][0] == tag:
                del self.stack[i:]
                break


# ---------------------------------------------------------------------------
print("A. the page")
el = Elements()
el.feed(html)
E = el.found
need = ["mod-decisions", "dec-title", "now-word", "now-detail", "now-since", "decisions", "mod-log", "col-right"]
check("index.html has #mod-decisions, #now-word, #now-detail, #now-since, #decisions, #mod-log",
      all(i in E for i in need), ", ".join(i for i in need if i not in E))
dups = sorted({i for i in el.ids if el.ids.count(i) > 1})
check("every id in index.html is unique", not dups, ", ".join(dups))
md, ml, mc = E.get("mod-decisions", {}), E.get("mod-log", {}), E.get("cams", {})
check("#mod-decisions sits in the map column (#col-map), under the run banner and BEFORE the cameras (#cams); the log stays in the right column",
      md and ml and mc and "col-map" in md["in"] and md["pos"] < mc["pos"] and "col-right" in ml["in"],
      f"decisions {md.get('pos')} {md.get('in')}, cams {mc.get('pos')}, log {ml.get('pos')} {ml.get('in')}")
check("the feed scrolls inside a bounded height under the map (#decisions max-height + overflow-y)",
      bool(re.search(r"#decisions \{[^}]*max-height: \d+px;[^}]*overflow-y: auto", html)))
title = re.search(r'<h2 class="title" id="dec-title">([^<]*)</h2>', html)
check('the module is titled "What it decides." and is labelled by that title',
      bool(title) and title.group(1) == "What it decides."
      and md.get("attrs", {}).get("aria-labelledby") == "dec-title", title.group(1) if title else "no #dec-title")
check("#now-word, #now-detail and #decisions are inside #mod-decisions; #decisions is an <ol>",
      all("mod-decisions" in E.get(i, {}).get("in", []) for i in ("now-word", "now-detail", "now-since", "decisions"))
      and E.get("decisions", {}).get("tag") == "ol")
check("#now-word announces itself (role status, aria-live polite)",
      E.get("now-word", {}).get("attrs", {}).get("role") == "status"
      and E.get("now-word", {}).get("attrs", {}).get("aria-live") == "polite")
live = re.search(r'<div class="now" data-live>[\s\S]*?</div>', html)
check("offline, the word and the feed keep their last values, dimmed like the log (data-live)",
      bool(live) and 'id="now-word"' in live.group(0) and "data-live" in E.get("decisions", {}).get("attrs", {})
      and "body.offline [data-live]" in html)
nw0 = re.search(r'<p id="now-word" class="(\w*)"[^>]*>([^<]*)</p>', html)
nd0 = re.search(r'<p id="now-detail">([^<]*)</p>', html)
feed0 = re.search(r'<ol id="decisions"[^>]*>([\s\S]*?)</ol>', html)
check("before the first answer the panel says it does not know: '--' dim, 'Waiting for the cockpit.', an empty "
      "feed (not IDLE / 'No stack running.' while a run may be on)",
      bool(nw0 and nd0 and feed0) and nw0.group(1) == "dim" and nw0.group(2) == "--"
      and nd0.group(1) == "Waiting for the cockpit." and feed0.group(1).strip() == "",
      json.dumps([nw0.groups() if nw0 else None, nd0.group(1) if nd0 else None, feed0.group(1) if feed0 else None]))
mob = re.search(r"@media \(max-width: 1099px\) \{([\s\S]*?)\n  \}", html)
orders = dict(re.findall(r"#(mod-[a-z]+) \{ order: (\d+); \}", mob.group(1))) if mob else {}
check("phone layout: the decisions follow the map (no order rule of their own: they are inside #col-map, which comes first); controls before the log",
      "mod-decisions" not in orders and {"mod-controls", "mod-log"} <= set(orders)
      and int(orders["mod-controls"]) < int(orders["mod-log"])
      and len(set(orders.values())) == len(orders), json.dumps(orders))
read_ids = set(re.findall(r'\$\("([\w-]+)"\)', js)) | set(re.findall(r'setText\("([\w-]+)"', js)) \
    | set(re.findall(r'meter\("([\w-]+)"', js))
missing = sorted(i for i in read_ids if i not in E)
check(f"every id app.js reads ($, setText, meter: {len(read_ids)}) exists in index.html", not missing,
      ", ".join(missing))
check("the map caption's subtitle still carries the command (cap-sub, last_cmd_vel)",
      'setText("cap-sub", sub);' in js and 'sub += (sub ? "  \u00b7  " : "") + "cmd "' in js and 'id="cap-sub"' in html)

# ---------------------------------------------------------------------------
print("B. the colours")


def rule(sel):
    m = re.search(re.escape(sel) + r" \{([^}]*)\}", html)
    return m.group(1) if m else ""


check("#now-word is big and uppercase", "text-transform: uppercase" in rule("#now-word")
      and int(re.search(r"font-size: (\d+)px", rule("#now-word")).group(1)) >= 28, rule("#now-word"))
check("#now-word.warn = the page's warning colour (--ui-attention), .accent = mint, .dim = --ui-dim",
      "color: var(--ui-attention)" in rule("#now-word.warn") and "color: var(--moss-mint)" in rule("#now-word.accent")
      and "color: var(--ui-dim)" in rule("#now-word.dim"))
check("feed pills: warn filled with --ui-attention, caution in --ui-caution, accent in mint, dim in --ui-dim",
      "background: var(--ui-attention)" in rule("#decisions .warn .k")
      and "color: var(--ui-caution)" in rule("#decisions .caution .k")
      and "color: var(--moss-mint)" in rule("#decisions .accent .k")
      and "color: var(--ui-dim)" in rule("#decisions .dim .k, #decisions .dim .x"))
caution = re.search(r"--ui-caution: (#[0-9A-Fa-f]{6});", tokens)
depth = re.search(r'var DEPTH_COLOR = "rgba\((\d+),(\d+),(\d+),', js)
rgb = tuple(int(caution.group(1)[i:i + 2], 16) for i in (1, 3, 5)) if caution else None
check("amber is one token: --ui-caution in tokens.css = the depth camera's dots on the map (255, 193, 94); "
      "the HUD's camera line uses it", rgb is not None and depth is not None
      and rgb == tuple(int(g) for g in depth.groups()) and rgb == (255, 193, 94)
      and "color: var(--ui-caution)" in rule("#hud-cam"), f"{rgb} {depth.groups() if depth else None}")

# ---------------------------------------------------------------------------
print("C. the wiring")
poll = re.search(r"var POLL = \{[^}]*\bdecisions: (\d+)", js)
rows = re.search(r"^var DECISION_ROWS = (\d+);", js, re.M)
check("app.js polls /decisions.json?n=30 every 1000 ms (POLL.decisions, poller like the log)",
      poll is not None and int(poll.group(1)) == 1000 and rows is not None and int(rows.group(1)) == 30
      and 'fetchJSON("/decisions.json?n=" + DECISION_ROWS)' in js and "poller(pollDecisions, POLL.decisions);" in js,
      (poll.group(0) if poll else "no POLL.decisions") + (rows.group(0) if rows else " no DECISION_ROWS"))
check("a failed fetch keeps the last panel (as the log); a server without the endpoint says so",
      re.search(r"function pollDecisions\(\) \{[\s\S]*?\}, function \(\) \{\}\);\n\}", js) is not None
      and "d.__missing" in js and "no decision feed" in js)
check("app.js names every kind of the catalogue and every state word",
      all(re.search(r"\b" + k + r": \"", js) for k in ALL_KINDS) and all('"' + w + '"' in js for w in WORDS),
      ", ".join([k for k in ALL_KINDS if not re.search(r"\b" + k + r": \"", js)] + [w for w in WORDS if '"' + w + '"' not in js]))
check("the map draws the arrow at the robot: cmdArrow(nav.last_cmd_vel, pose(), age since the state), "
      "then the robot marker over it",
      re.search(r"var p = pose\(\);\n(?:\s*//.*\n)+\s*var cmd = cmdArrow\(nav && nav\.last_cmd_vel, p, sinceState\(\)\);\n"
                r"\s*if \(cmd\) drawCmd\(cmd\);\n\s*if \(p && typeof p\.x === \"number\"\) \{", js) is not None)
check("the server's command carries what the arrow reads: vx, wz, age_s (live and demo)",
      all(k in (ROOT / "tools" / "moss_cockpit.py").read_text() for k in ('"last_cmd_vel"', '"vx"', '"wz"', '"age_s"')))
urls = re.findall(r'fetchJSON\("([^"]*)"', js) + re.findall(r'src: "([^"]*)"', js) \
    + re.findall(r'\.src = "([^"]*)"', js)
check(f"app.js talks to no other host: no '://', no '//host' literal, every URL relative ({len(urls)})",
      "://" not in js and not re.search(r"""["']//""", js) and urls and all(u.startswith("/") for u in urls)
      and "new WebSocket" not in js and "EventSource" not in js, ", ".join(u for u in urls if not u.startswith("/")))
check("index.html loads nothing from another host", "://" not in re.sub(r"<svg[\s\S]*?</svg>", "", html)
      and all(u.startswith("/") for u in re.findall(r'(?:src|href)="([^"]+)"', html)))

# ---------------------------------------------------------------------------
print("D. in node")


def js_def(name):
    """app.js's top-level `function name(...) {...}` or `var name = ...;` (brace matched), or ""."""
    m = re.search(r"^(function " + re.escape(name) + r"\(|var " + re.escape(name) + r" = )", js, re.M)
    if not m:
        return ""
    i, depth, start = m.end(), 0, m.start()
    line_end = js.find("\n", i)
    if m.group(1).startswith("var") and not js[i:line_end].rstrip().endswith("{"):
        return js[start:line_end]           # a one-line var (a trailing // comment is harmless)
    i = js.find("{", i)
    while i < len(js):
        if js[i] == "{":
            depth += 1
        elif js[i] == "}":
            depth -= 1
            if depth == 0:
                tail = js.find("\n", i)
                return js[start:tail if tail > 0 else len(js)]
        i += 1
    return ""


NAMES = ["LIVE_S", "CMD_ARROW_S", "PIVOT_ARROW_S", "PIVOT_MIN_RAD", "CMD_VX_EPS", "CMD_WZ_EPS", "PIVOT_RADIUS_M",
         "CMD_STEPS",
         "CMD_W", "CMD_HEAD", "CMD_MIN_RUN", "PIVOT_R_PX", "MARK_L", "MINT", "HALO", "DECISION_ROWS",
         "KIND_CLASS", "KIND_LABEL", "WORD_CLASS", "WORD_DETAIL", "pad2", "dur", "esc", "sx", "sy", "cmdArrow",
         "trimEnd", "drawCmd", "tsSeconds", "minSec", "decisionRows", "freshRows", "decisionsHtml", "nowView"]
PANEL = ["$", "setText", "stack", "sinceState", "serverNow", "pollDecisions", "renderDecisions"]
defs = {n: js_def(n) for n in NAMES + PANEL}
node = shutil.which("node")
if node is None:
    print("  -- skipped, no node here: node --check app.js and the page's functions on known input")
else:
    r = subprocess.run([node, "--check", str(PAGE / "app.js")], capture_output=True, text=True, timeout=30)
    check("node --check app.js", r.returncode == 0, r.stderr.strip()[:200])
    check("the page's functions and constants are found in app.js", all(defs.values()),
          ", ".join(n for n, d in defs.items() if not d))

    T0 = 1_000_000_000            # an epoch second the events count from (UTC)
    harness = "\n".join(defs.values()) + "\n" + r"""
var S = { view: { scale: 100, x: 0, y: 3 } };      // 100 px per metre; world (0, 3) at the top-left corner
var ops = [];
var ctx = { save: function () {}, restore: function () {}, closePath: function () { ops.push(["close"]); },
  beginPath: function () { ops.push(["begin"]); },
  moveTo: function (x, y) { ops.push(["move", x, y]); }, lineTo: function (x, y) { ops.push(["line", x, y]); },
  arc: function (x, y, r, a0, a1, acw) { ops.push(["arc", x, y, r, a0, a1, !!acw]); },
  stroke: function () { ops.push(["stroke", this.strokeStyle, this.lineWidth]); },
  fill: function () { ops.push(["fill", this.fillStyle]); } };
function drawn(a) { ops = []; drawCmd(a); return ops; }
var P = { x: 1, y: 2, yaw: 0 }, O = { x: 0, y: 0, yaw: 0 };
var C = function (vx, wz, age) { return { vx: vx, wz: wz, age_s: age === undefined ? 0.1 : age }; };
// the same draw at another map scale (px per metre), the page's fitted zoom on the demo is 37.1
function drawnAt(scale, a) { var s0 = S.view.scale; S.view.scale = scale; var o = drawn(a); S.view.scale = s0; return o; }
var fit = {
  slow: drawnAt(37.1, cmdArrow(C(0.2, 0), P, 0)),
  fast: drawnAt(37.1, cmdArrow(C(0.45, 0), P, 0)),
  metric: drawnAt(50, cmdArrow(C(0.45, 0), P, 0)),
  arc: drawnAt(37.1, cmdArrow(C(0.3, 0.2), P, 0)),
  back: drawnAt(37.1, cmdArrow(C(-0.15, 0), P, 0))
};
var T0 = """ + str(T0) + r""";
var ev = [];
for (var i = 1; i <= 40; i++) ev.push({ ts: T0 + 7 * i, kind: ["goal", "obstacle", "replan", "contact", "waiting"][i % 5],
                                        text: "event " + i, src: "explorer", seq: i });
var rowsA = decisionRows(ev.slice(0, 38), 30), rowsB = decisionRows(ev, 30);
var rowsC = decisionRows([{ ts: T0, kind: "goal", text: "other run", seq: 1 }], 30);
var dupA = decisionRows([{ ts: 62, kind: "replan", text: "Replanning", seq: 1 }], 30);
var dupB = decisionRows([{ ts: 62, kind: "replan", text: "Replanning", seq: 1 },
                         { ts: 62, kind: "replan", text: "Replanning", seq: 2 }], 30);
var out = {
  straight: cmdArrow(C(0.45, 0), P, 0),
  half: cmdArrow(C(0.225, 0), P, 0),
  reverse: cmdArrow(C(-0.45, 0), P, 0),
  north: cmdArrow(C(0.45, 0), { x: 1, y: 2, yaw: Math.PI / 2 }, 0),
  arcLeft: cmdArrow(C(0.3, 0.3), O, 0),
  arcRight: cmdArrow(C(0.3, -0.3), O, 0),
  arcBack: cmdArrow(C(-0.3, 0.3), O, 0),
  pivotLeft: cmdArrow(C(0, 0.385), P, 0),
  pivotRight: cmdArrow(C(0.01, -0.5), P, 0),
  pivotSlow: cmdArrow(C(0, 0.05), P, 0),
  pivotFast: cmdArrow(C(0, 3), P, 0),
  pivotTight: cmdArrow(C(0.03, 0.385), P, 0),
  arcWide: cmdArrow(C(0.05, -0.3), P, 0),
  zero: cmdArrow(C(0, 0), P, 0),
  crawl: cmdArrow(C(0.01, 0.01), P, 0),
  fresh: cmdArrow(C(0.45, 0, 1.9), P, 0.05) !== null,
  staleAtTwo: cmdArrow(C(0.45, 0, 1.5), P, 0.5),
  noAge: cmdArrow({ vx: 0.45, wz: 0 }, P, 0),
  noPose: cmdArrow(C(0.45, 0), null, 0),
  nan: cmdArrow(C(NaN, 0), P, 0),
  drawStraight: drawn(cmdArrow(C(0.45, 0), P, 0)),
  drawReverse: drawn(cmdArrow(C(-0.45, 0), P, 0)),
  drawNorth: drawn(cmdArrow(C(0.45, 0), { x: 1, y: 2, yaw: Math.PI / 2 }, 0)),
  drawPivotLeft: drawn(cmdArrow(C(0, 0.385), P, 0)),
  drawPivotRight: drawn(cmdArrow(C(0, -0.385), P, 0)),
  drawTiny: drawn(cmdArrow(C(0.03, 0), P, 0)),
  drawPivotNorth: drawn(cmdArrow(C(0, 0.385), { x: 1, y: 2, yaw: Math.PI / 2 }, 0)),
  fitSlow: fit.slow, fitFast: fit.fast, fitMetric: fit.metric, fitArc: fit.arc, fitBack: fit.back,
  rowsB: rowsB,
  nRowsB: rowsB.length,
  kinds: ["contact", "push", "imu", "obstacle", "replan", "pivot_fallback", "goal", "new_goal", "reached",
          "waiting", "goal_dropped", "retry", "mission", "foo_bar"].map(function (k) {
    var r = decisionRows([{ ts: T0, kind: k, text: "t", seq: 1 }], 30)[0]; return [k, r.cls, r.label]; }),
  junk: decisionRows([null, 5, "x", { kind: "goal", text: "Goal 1: 2.0 m away (35 cells)", ts: "1970-01-01T00:42:07Z" }], 30),
  notList: [decisionRows(null, 30).length, decisionRows({}, 30).length],
  noSeq: decisionRows([{ kind: "goal", text: "first" }, { kind: "obstacle", text: "second" }], 30).map(function (r) { return r.text; }),
  times: [minSec("1970-01-01T00:42:07Z"), minSec("1970-01-01T00:42:07"), minSec(2527), minSec("12:34:56"), minSec(undefined)],
  escaped: decisionsHtml(decisionRows([{ ts: T0, kind: "contact", text: "<img src=x onerror=alert(1)>", seq: 1 }], 30), null),
  freshFirst: freshRows(rowsA, null).fresh,
  freshEmptyBefore: freshRows(rowsA, []).fresh.filter(Boolean).length,
  freshTwoNew: freshRows(rowsB, freshRows(rowsA, null).keys).fresh,
  freshNewRun: freshRows(rowsC, freshRows(rowsA, null).keys).fresh,
  freshDup: freshRows(dupB, freshRows(dupA, null).keys).fresh,
  htmlFresh: decisionsHtml(rowsB.slice(0, 3), [true, false, false]),
  words: ["OBSTACLE", "BACKING OFF", "STUCK", "HEADING", "IDLE", "READY", "WAITING", "DONE", "STOPPED", "heading", "FOO"]
    .map(function (w) { return [w, nowView({ word: w, detail: "d", since: null }, 100).cls]; }),
  nowObstacle: nowView({ word: "OBSTACLE", detail: "Obstacle ahead: stop", since: T0 - 2 }, T0),
  nowIso: nowView({ word: "WAITING", detail: "Waiting: 3 of 5 frontiers are failed goals", since: "1970-01-01T00:00:10Z" }, 70),
  nowLong: nowView({ word: "HEADING", detail: "Goal 4: 2.1 m away (12 cells)", since: 5 }, 100),
  nowIdle: nowView({ word: "IDLE", detail: "", since: null }, 100),
  nowNone: [nowView(null, 1), nowView({}, 1), nowView("x", 1)]
};
console.log(JSON.stringify(out));
"""
    r = subprocess.run([node, "-e", harness], capture_output=True, text=True, timeout=30, env={**os.environ, "TZ": "UTC"})
    try:
        J = json.loads(r.stdout)
    except ValueError:
        J = {}
        print("  node said:", r.stderr.strip()[:600])

    def near(a, b, tol=1e-6):
        return isinstance(a, (int, float)) and abs(a - b) <= tol

    def tip_at(key, x, y, ux, uy):
        a = J.get(key) or {}
        t = a.get("tip") or {}
        return (a.get("kind") == "path" and near(t.get("x"), x) and near(t.get("y"), y)
                and near(t.get("ux"), ux) and near(t.get("uy"), uy))

    def length(pts):
        return sum(math.dist(pts[i - 1], pts[i]) for i in range(1, len(pts)))

    st = J.get("straight") or {}
    check("arrow: 0.45 m/s straight from (1, 2) facing +x -> a 0.6 m path, (1, 2) -> (1.6, 2), pointing +x",
          tip_at("straight", 1.6, 2.0, 1, 0) and st.get("pts", [[0, 0]])[0] == [1, 2]
          and near(length(st.get("pts", [])), 0.6, 1e-9), json.dumps(st.get("tip")))
    check("arrow: 0.225 m/s -> 0.3 m (length proportional to |vx|): tip (1.3, 2)",
          tip_at("half", 1.3, 2.0, 1, 0), json.dumps((J.get("half") or {}).get("tip")))
    check("arrow: reverse -0.45 m/s -> drawn behind the robot, tip (0.4, 2), pointing -x",
          tip_at("reverse", 0.4, 2.0, -1, 0), json.dumps((J.get("reverse") or {}).get("tip")))
    check("arrow: facing +y (yaw 90 deg), 0.45 m/s -> tip (1, 2.6), pointing +y",
          tip_at("north", 1.0, 2.6, 0, 1), json.dumps((J.get("north") or {}).get("tip")))
    th = 0.3 * 0.6 / 0.45                           # 0.4 rad of turn over the arrow's 4/3 s
    al = J.get("arcLeft") or {}
    check("arrow: arc vx 0.3 m/s, wz +0.3 rad/s -> bends LEFT on a 1 m radius: tip (sin 0.4, 1 - cos 0.4) "
          "= (0.3894, 0.0789), heading 0.4 rad, 0.4 m of path",
          tip_at("arcLeft", math.sin(th), 1 - math.cos(th), math.cos(th), math.sin(th))
          and near(length(al.get("pts", [])), 0.4, 1e-4), json.dumps(al.get("tip")))
    check("arrow: wz -0.3 rad/s -> bends RIGHT: tip (0.3894, -0.0789)",
          tip_at("arcRight", math.sin(th), -(1 - math.cos(th)), math.cos(th), -math.sin(th)),
          json.dumps((J.get("arcRight") or {}).get("tip")))
    check("arrow: reverse arc vx -0.3, wz +0.3 -> behind and to the right: tip (-0.3894, -0.0789), pointing back",
          tip_at("arcBack", -math.sin(th), -(1 - math.cos(th)), -math.cos(th), -math.sin(th)),
          json.dumps((J.get("arcBack") or {}).get("tip")))
    pl, pr = J.get("pivotLeft") or {}, J.get("pivotRight") or {}
    check("arrow: pivot wz +0.385 rad/s (vx 0) -> a left (counterclockwise) arc of 3 s of turn: 1.155 rad",
          pl.get("kind") == "pivot" and pl.get("dir") == 1 and near(pl.get("sweep"), 1.155), json.dumps(pl))
    check("arrow: vx 0.01 m/s (~0), wz -0.5 rad/s -> a right pivot of 1.5 rad",
          pr.get("kind") == "pivot" and pr.get("dir") == -1 and near(pr.get("sweep"), 1.5), json.dumps(pr))
    pt, aw = J.get("pivotTight") or {}, J.get("arcWide") or {}
    check("arrow: a turn about a centre under the robot is a pivot: vx 0.03, wz +0.385 (radius 0.078 m) -> left "
          "pivot; vx 0.05, wz -0.3 (radius 0.167 m) -> a path bending right",
          pt.get("kind") == "pivot" and pt.get("dir") == 1 and aw.get("kind") == "path"
          and (aw.get("tip") or {}).get("y", 2) < 2, json.dumps([pt, aw.get("tip")]))
    check("arrow: a pivot sweeps at least 45 deg (wz 0.05) and at most 270 deg (wz 3)",
          near((J.get("pivotSlow") or {}).get("sweep"), math.pi / 4)
          and near((J.get("pivotFast") or {}).get("sweep"), 1.5 * math.pi))
    check("arrow: nothing for a zero command (0, 0) or a crawl under the thresholds (0.01 m/s, 0.01 rad/s)",
          "zero" in J and J["zero"] is None and "crawl" in J and J["crawl"] is None)
    check("arrow: only fresher than LIVE_S = 2 s: 1.9 + 0.05 s drawn, 1.5 + 0.5 s not, no age not; "
          "no pose / NaN -> nothing", J.get("fresh") is True and all(k in J and J[k] is None
                                                                      for k in ("staleAtTwo", "noAge", "noPose", "nan")))

    def ops_of(key, op):
        return [o for o in J.get(key, []) if o[0] == op]

    def head_tip(key):
        """The arrowhead's tip: the moveTo that starts a closed path (the last one drawn, the mint fill)."""
        ops, tips = J.get(key, []), []
        for i, o in enumerate(ops):
            if o[0] == "move" and i + 3 < len(ops) and ops[i + 3][0] == "close":
                tips.append(o[1:3])
        return tips[-1] if tips else None

    def head_dir(key):
        """The arrowhead's tip and its unit direction (tip minus the middle of its base, over its length)."""
        ops = J.get(key, [])
        for i in range(len(ops) - 4, -1, -1):
            o = ops[i]
            if o[0] == "move" and ops[i + 3][0] == "close":
                b = [(ops[i + 1][1] + ops[i + 2][1]) / 2, (ops[i + 1][2] + ops[i + 2][2]) / 2]
                d = math.dist(o[1:3], b) or 1
                return o[1:3], [(o[1] - b[0]) / d, (o[2] - b[1]) / d]
        return None, None

    def shaft_of(key):
        """The mint line's points (the last stroked move/line run before the mint stroke)."""
        ops, run, last = J.get(key, []), [], []
        for o in ops:
            if o[0] == "begin":
                run = []
            elif o[0] in ("move", "line"):
                run.append(o[1:3])
            elif o[0] == "stroke" and o[1] == "#6CF8C7" and o[2] == 4:
                last = run
        return last

    def at(pt, x, y, tol=1e-6):
        return pt is not None and near(pt[0], x, tol) and near(pt[1], y, tol)

    NOSE, TAIL = 15 * .62, 15 * .38                  # px: the marker's nose ahead of its centre, its tail behind
    ds = J.get("drawStraight", [])
    mint = [o for o in ops_of("drawStraight", "stroke") if o[1] == "#6CF8C7"]
    fills = ops_of("drawStraight", "fill")
    sh = shaft_of("drawStraight")
    check("canvas: 0.45 m/s at (1, 2), 100 px/m -> the line starts at the marker's nose (109.3, 100) px, "
          "head tip 60 px further at (169.3, 100)",
          sh and at(sh[0], 100 + NOSE, 100) and at(head_tip("drawStraight"), 160 + NOSE, 100),
          json.dumps([sh[:1], head_tip("drawStraight")]))
    strokes = ops_of("drawStraight", "stroke")
    check("canvas: mint like the path but thicker (4 px > 2.25 px), inside an opaque navy outline (8 px, "
          "--moss-navy) that sets it apart from the path; the head is filled mint, outlined navy",
          mint and mint[-1][2] == 4 and fills and fills[-1][1] == "#6CF8C7"
          and strokes[0][1] == "#080F1A" and strokes[0][2] == 8 and strokes[1][1] == "#080F1A", json.dumps(strokes))
    shr = shaft_of("drawReverse")
    check("canvas: reverse -> the line starts at the marker's tail (94.3, 100), head tip at (34.3, 100), behind",
          shr and at(shr[0], 100 - TAIL, 100) and at(head_tip("drawReverse"), 40 - TAIL, 100),
          json.dumps([shr[:1], head_tip("drawReverse")]))
    check("canvas: facing +y -> head tip at (100, 30.7) px: up the screen, past the nose",
          at(head_tip("drawNorth"), 100, 40 - NOSE), json.dumps(head_tip("drawNorth")))
    arcs_l, arcs_r = ops_of("drawPivotLeft", "arc"), ops_of("drawPivotRight", "arc")
    tl, tr = head_tip("drawPivotLeft"), head_tip("drawPivotRight")
    ex = (100 + 17 * math.cos(-1.155), 100 + 17 * math.sin(-1.155))
    check("canvas: left pivot -> an arc of 17 px around the robot (100, 100), from its nose (angle 0) "
          "counterclockwise; head tip at the arc's end, up the screen (" + f"{ex[0]:.1f}, {ex[1]:.1f}" + ")",
          arcs_l and arcs_l[-1][1:4] == [100, 100, 17] and near(arcs_l[-1][4], 0) and arcs_l[-1][6] is True
          and arcs_l[-1][5] < 0 and tl is not None and near(tl[0], ex[0], 1e-6) and near(tl[1], ex[1], 1e-6),
          json.dumps({"arc": arcs_l[-1] if arcs_l else None, "tip": tl}))
    check("canvas: right pivot -> clockwise, head tip below the robot (y > 100)",
          arcs_r and arcs_r[-1][6] is False and arcs_r[-1][5] > 0 and tr is not None and tr[1] > 100,
          json.dumps({"arc": arcs_r[-1] if arcs_r else None, "tip": tr}))
    an, tn = ops_of("drawPivotNorth", "arc"), head_tip("drawPivotNorth")
    exn = (100 + 17 * math.cos(-math.pi / 2 - 1.155), 100 + 17 * math.sin(-math.pi / 2 - 1.155))
    check("canvas: left pivot facing +y (yaw 90 deg) -> the arc starts at the nose, screen angle -90 deg (up), "
          "and turns counterclockwise to the left: head tip (" + f"{exn[0]:.1f}, {exn[1]:.1f}" + ")",
          an and near(an[-1][4], -math.pi / 2) and an[-1][6] is True and at(tn, exn[0], exn[1]),
          json.dumps({"arc": an[-1] if an else None, "tip": tn}))
    sht = shaft_of("drawTiny")
    check("canvas: a 0.03 m/s command (4 px of map) -> never under 22 px past the nose: a line and the head, "
          "tip (131.3, 100)",
          at(head_tip("drawTiny"), 100 + NOSE + 22, 100) and sht and at(sht[0], 100 + NOSE, 100),
          json.dumps([head_tip("drawTiny"), sht[:1]]))
    # the fitted zoom of the demo (37.1 px/m): the robot at (1, 2) is at (37.1, 37.1) px
    F0, nose_f = 37.1, 37.1 + NOSE
    ss, tip_s = shaft_of("fitSlow"), head_tip("fitSlow")
    check("canvas at the fitted zoom (37.1 px/m), 0.2 m/s (10 px of map, the run's slowest quarter): the line "
          "leaves the nose, the head tip lies 22 px past it, >= 12 px of line before the head",
          ss and at(ss[0], nose_f, F0) and at(tip_s, nose_f + 22, F0)
          and (ss[-1][0] - ss[0][0]) - 3 >= 12 - 1e-6, json.dumps([ss[:1], ss[-1:], tip_s]))
    check("canvas at 37.1 px/m: 0.45 m/s is 22.3 px of map, past the 22 px floor -> its metres again "
          "(tip 22.26 px past the nose); at 50 px/m 0.45 m/s -> 30 px past the nose",
          at(head_tip("fitFast"), nose_f + 0.6 * 37.1, F0) and at(head_tip("fitMetric"), 50 + NOSE + 30, 50),
          json.dumps([head_tip("fitFast"), head_tip("fitMetric")]))
    th2 = 0.2 * 0.6 / 0.45                          # 0.267 rad of turn over the arrow
    tip_a, dir_a = head_dir("fitArc")
    exa = (nose_f + 22 * math.sin(th2) / th2, F0 - 22 * (1 - math.cos(th2)) / th2)
    check("canvas at 37.1 px/m, arc vx 0.3 wz +0.2: a 22 px arc from the nose that turns 0.267 rad left: tip "
          f"({exa[0]:.2f}, {exa[1]:.2f}), above the axis, the head pointing 0.267 rad up the screen",
          at(tip_a, exa[0], exa[1], 1e-6) and dir_a is not None and near(dir_a[0], math.cos(th2), 1e-6)
          and near(dir_a[1], -math.sin(th2), 1e-6), json.dumps([tip_a, dir_a]))
    sb = shaft_of("fitBack")
    check("canvas at 37.1 px/m, reverse 0.15 m/s: from the tail, head tip 22 px behind it",
          sb and at(sb[0], F0 - TAIL, F0) and at(head_tip("fitBack"), F0 - TAIL - 22, F0),
          json.dumps([sb[:1], head_tip("fitBack")]))

    rb = J.get("rowsB") or []
    t40 = datetime.fromtimestamp(T0 + 7 * 40, timezone.utc).strftime("%M:%S")
    check(f"feed: 40 events -> 30 rows, newest on top (seq 40 ... 11), time mm:ss ({t40} for seq 40)",
          J.get("nRowsB") == 30 and [r.get("seq") for r in rb] == list(range(40, 10, -1))
          and rb[0].get("time") == t40 and rb[0].get("text") == "event 40",
          json.dumps(rb[:2]))
    got = {k: (c, lab) for k, c, lab in J.get("kinds", [])}
    want = {k: (KINDS.get(k, ""), lab) for k, lab in
            [("contact", "CONTACT"), ("push", "PUSH"), ("imu", "IMU"), ("obstacle", "OBSTACLE"),
             ("replan", "REPLAN"), ("goal", "GOAL"), ("new_goal", "NEW GOAL"), ("reached", "REACHED"),
             ("waiting", "WAITING"), ("goal_dropped", "DROPPED"), ("retry", "RETRY"), ("mission", "MISSION"),
             ("foo_bar", "FOO BAR")]}
    want["pivot_fallback"] = ("caution", "PIVOT")
    check("feed pills: contact/push/imu warn, obstacle/replan amber (caution), goal/new_goal/reached accent, "
          "waiting/goal_dropped/retry dim, mission plain; an unknown kind shows its name",
          got == want, json.dumps({k: v for k, v in got.items() if want.get(k) != v}))
    junk = J.get("junk") or []
    check("feed: null, a number and a string are dropped; an ISO 'Z' time -> 42:07",
          len(junk) == 1 and junk[0].get("time") == "42:07" and junk[0].get("cls") == "accent"
          and J.get("notList") == [0, 0], json.dumps(junk))
    check("feed: events without seq keep the server's order, newest (last) on top",
          J.get("noSeq") == ["second", "first"], json.dumps(J.get("noSeq")))
    check("feed times: ISO with and without zone (UTC here), epoch 2527 s, a bare hh:mm:ss, none -> "
          "42:07, 42:07, 42:07, 34:56, --:--",
          J.get("times") == ["42:07", "42:07", "42:07", "34:56", "--:--"], json.dumps(J.get("times")))
    esc_html = J.get("escaped", "")
    check("feed: an event's text is escaped (no markup from the log reaches the page)",
          "&lt;img src=x onerror=alert(1)&gt;" in esc_html and "<img" not in esc_html
          and esc_html.startswith('<li class="warn"><span class="t">'), esc_html[:120])
    check("feed flash: the first answer flashes nothing; after an empty one, every row flashes",
          J.get("freshFirst") == [False] * 30 and J.get("freshEmptyBefore") == 30,
          json.dumps([J.get("freshFirst", [])[:3], J.get("freshEmptyBefore")]))
    check("feed flash: two new events on top -> exactly those two rows flash",
          J.get("freshTwoNew") == [True, True] + [False] * 28, json.dumps(J.get("freshTwoNew", [])[:4]))
    check("feed flash: an answer with nothing in common (a new run) flashes nothing; "
          "a second identical line in the same second flashes once",
          J.get("freshNewRun") == [False] and J.get("freshDup") == [True, False],
          json.dumps([J.get("freshNewRun"), J.get("freshDup")]))
    hf = J.get("htmlFresh", "").split("</li>")
    check("feed html: a fresh row carries its kind class and 'fresh' (goal: accent), the next one only its own (waiting: dim)",
          len(hf) >= 3 and hf[0].startswith('<li class="accent fresh">') and hf[1].startswith('<li class="dim">'),
          " | ".join(hf[:2])[:160])
    words = {w: c for w, c in J.get("words", [])}
    check("state word colour: OBSTACLE / BACKING OFF / STUCK warn, HEADING accent, IDLE / READY / WAITING / "
          "DONE / STOPPED dim; 'heading' -> HEADING; an unknown word plain",
          all(words.get(w) == c for w, c in WORDS.items()) and words.get("heading") == "accent" and words.get("FOO") == "",
          json.dumps(words))
    no = J.get("nowObstacle") or {}
    check("state word: OBSTACLE set 2 s ago on the server's clock -> 'OBSTACLE', warn, its detail, 'for 2 s'",
          no == {"word": "OBSTACLE", "cls": "warn", "detail": "Obstacle ahead: stop", "since": "for 2 s"}, json.dumps(no))
    check("state word: since as ISO (10 s) at 70 s -> 'for 60 s'; since 5 s at 100 s -> 'for 2 min'",
          (J.get("nowIso") or {}).get("since") == "for 60 s" and (J.get("nowLong") or {}).get("since") == "for 2 min",
          json.dumps([J.get("nowIso"), J.get("nowLong")]))
    check("state word: IDLE without detail -> 'No stack running.', no since; nothing -> '--' dim",
          J.get("nowIdle") == {"word": "IDLE", "cls": "dim", "detail": "No stack running.", "since": ""}
          and J.get("nowNone") == [{"word": "--", "cls": "dim", "detail": "", "since": ""}] * 3,
          json.dumps([J.get("nowIdle"), J.get("nowNone")]))

    # The panel itself: pollDecisions and renderDecisions, the page's own code, on a stub document.
    # The server's clock (S.state.ts = T0) is 25 years behind this machine's: a word timed on the
    # browser's clock instead would read in hours, not "for 2 s".
    panel_js = "\n".join(defs[n] for n in ("pad2", "dur", "esc", "DECISION_ROWS", "KIND_CLASS", "KIND_LABEL",
                                           "WORD_CLASS", "WORD_DETAIL", "tsSeconds", "minSec", "decisionRows",
                                           "freshRows", "decisionsHtml", "nowView") + tuple(PANEL)) + r"""
var T0 = """ + str(T0) + r""";
var els = {};
["now-word", "now-detail", "now-since", "decisions"].forEach(function (id) {
  els[id] = { textContent: "", className: "", innerHTML: "" }; });
var document = { getElementById: function (id) { return els[id] || null; } };
var S = { state: { ts: T0, stack: { running: true } }, stateAt: 0, decisions: null, decHtml: null, decKeys: null };
var answers = [], urls = [];
function fetchJSON(url) {
  urls.push(url); var a = answers.shift();
  return a instanceof Error ? Promise.reject(a) : Promise.resolve(a);
}
function snap() {
  return { word: els["now-word"].textContent, cls: els["now-word"].className, detail: els["now-detail"].textContent,
           since: els["now-since"].textContent, feed: els["decisions"].innerHTML, keys: (S.decKeys || []).length };
}
function evs(n) {
  var out = [];
  for (var i = 1; i <= n; i++) out.push({ ts: T0 - 10 + i, kind: ["goal", "obstacle", "replan", "contact"][i % 4],
                                          text: "decision " + i, src: "explorer", seq: i });
  return out;
}
var docA = { now: { word: "OBSTACLE", detail: "Obstacle ahead: stop", since: T0 - 2 }, events: evs(3), run: "r1" };
var docB = { now: { word: "HEADING", detail: "Goal 2: 1.5 m away (20 cells)", since: T0 - 65 }, events: evs(4), run: "r1" };
var out = {};
answers.push(docA);
pollDecisions().then(function () {
  out.polled = snap(); out.urls = urls.slice();
  S.decisions = docB; renderDecisions(); out.next = snap();
  S.decisions = { error: "decisions: main.jsonl unreadable" }; renderDecisions(); out.error = snap();
  S.decisions = docB; renderDecisions(); out.after = snap();
  answers.push(new Error("offline"));
  return pollDecisions();
}).then(function () {
  out.offline = snap();
  S.decisions = { __missing: true }; renderDecisions(); out.missing = snap();
  S.state.stack.running = false; S.decKeys = null;
  S.decisions = { now: { word: "IDLE", detail: "", since: null }, events: [] }; renderDecisions(); out.idle = snap();
  console.log(JSON.stringify(out));
}).catch(function (e) { console.log(JSON.stringify({ crash: String(e && e.stack || e) })); });
"""
    r = subprocess.run([node, "-e", panel_js], capture_output=True, text=True, timeout=30, env={**os.environ, "TZ": "UTC"})
    try:
        PJ = json.loads(r.stdout)
    except ValueError:
        PJ = {}
        print("  node said:", r.stderr.strip()[:600])
    check("the panel's code is found in app.js ($, setText, stack, sinceState, serverNow, pollDecisions, "
          "renderDecisions)", all(defs[n] for n in PANEL), ", ".join(n for n in PANEL if not defs[n]))

    def mmss(t):
        return datetime.fromtimestamp(t, timezone.utc).strftime("%M:%S")

    def first_li(feed):
        return feed.split("</li>")[0] if feed else ""

    pa, nx, er, af = (PJ.get(k) or {} for k in ("polled", "next", "error", "after"))
    check("panel: pollDecisions asks /decisions.json?n=30 and draws the answer: OBSTACLE, class warn, its detail, "
          "'for 2 s' on the server's clock",
          PJ.get("urls") == ["/decisions.json?n=30"] and pa.get("word") == "OBSTACLE" and pa.get("cls") == "warn"
          and pa.get("detail") == "Obstacle ahead: stop" and pa.get("since") == "for 2 s",
          json.dumps({"urls": PJ.get("urls"), "polled": {k: pa.get(k) for k in ("word", "cls", "detail", "since")},
                      "crash": PJ.get("crash")})[:400])
    check("panel: the first answer's feed has its 3 rows, newest on top (decision 3, contact: warn), nothing flashes",
          pa.get("feed", "").count("<li") == 3 and first_li(pa.get("feed")).startswith(
              '<li class="warn"><span class="t">' + mmss(1_000_000_000 - 7) + "</span>")
          and "decision 3" in first_li(pa.get("feed")) and "fresh" not in pa.get("feed", ""), first_li(pa.get("feed"))[:160])
    check("panel: the next answer (one more event) -> HEADING, accent, 'for 65 s'; the new row on top, and only it, "
          "flashes",
          nx.get("word") == "HEADING" and nx.get("cls") == "accent" and nx.get("since") == "for 65 s"
          and first_li(nx.get("feed")).startswith('<li class="accent fresh">') and "decision 4" in first_li(nx.get("feed"))
          and nx.get("feed", "").count("fresh") == 1, json.dumps({k: nx.get(k) for k in ("word", "cls", "since")})
          + " " + first_li(nx.get("feed"))[:100])
    check("panel: an error answer ({error}, no events) -> '--', 'Decision feed error: ...', and the feed and its "
          "keys stay (the decisions still exist)",
          er.get("word") == "--" and er.get("cls") == "dim"
          and er.get("detail") == "Decision feed error: decisions: main.jsonl unreadable"
          and er.get("feed") == nx.get("feed") and er.get("keys") == 4,
          json.dumps({k: er.get(k) for k in ("word", "detail", "keys")}))
    check("panel: the good answer after the error flashes nothing (the keys were kept), and a failed fetch keeps "
          "the word, the detail and the feed",
          af.get("feed") and "fresh" not in af.get("feed") and af.get("word") == "HEADING"
          and PJ.get("offline") == af, json.dumps({"after": first_li(af.get("feed"))[:80], "same": PJ.get("offline") == af}))
    mi, idl = PJ.get("missing") or {}, PJ.get("idle") or {}
    check("panel: no endpoint (404) -> '--', 'no decision feed'; no stack and no event -> IDLE, 'No stack running.', "
          "'No stack, no decisions.'",
          mi.get("word") == "--" and "no decision feed" in mi.get("detail", "") and "No /decisions.json" in mi.get("feed", "")
          and idl.get("word") == "IDLE" and idl.get("cls") == "dim" and idl.get("detail") == "No stack running."
          and "No stack, no decisions." in idl.get("feed", ""), json.dumps([mi, idl])[:300])

# ---------------------------------------------------------------------------
print("E. the server's answer through the page")
try:
    import moss_cockpit as mc  # noqa: E402 - standard library + numpy, no dimOS
    server = mc.make_server("127.0.0.1", 0, demo=True, listen=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urllib.request.urlopen(base + "/decisions.json?n=30", timeout=10) as resp:
            code, body = resp.status, resp.read()
    except urllib.error.HTTPError as e:
        code, body = e.code, e.read()
except Exception as e:  # noqa: BLE001 - the server module itself is broken: a failure, not a skip
    code, body, server = None, repr(e).encode(), None
    check("the cockpit server starts in demo mode", False, body.decode()[:200])
if code == 404:
    print("  -- skipped, this cockpit server has no /decisions.json yet (the page says so: 'no decision feed')")
elif code is not None:
    try:
        d = json.loads(body)
    except ValueError:
        d = None
    ok_shape = (isinstance(d, dict) and isinstance(d.get("now"), dict) and isinstance(d.get("events"), list)
                and "run" in d and isinstance(d["now"].get("word"), str))
    check("demo /decisions.json?n=30 -> 200 {now: {word, detail, since}, events: [...], run}",
          code == 200 and ok_shape, body[:200].decode(errors="replace"))
    if ok_shape and node is not None:
        feed = "\n".join(defs[n] for n in ("pad2", "dur", "esc", "DECISION_ROWS", "KIND_CLASS", "KIND_LABEL",
                                           "WORD_CLASS", "WORD_DETAIL", "tsSeconds", "minSec", "decisionRows",
                                           "decisionsHtml", "nowView")) + "\nvar d = " + json.dumps(d) + r""";
var rows = decisionRows(d.events, DECISION_ROWS);
console.log(JSON.stringify({ n: rows.length, labels: rows.map(function (r) { return r.label; }),
  unknown: d.events.filter(function (e) { return !(e.kind in KIND_LABEL); }).map(function (e) { return e.kind; }),
  times: rows.map(function (r) { return r.time; }), word: nowView(d.now, Date.now() / 1000).word,
  wordKnown: String(d.now.word).toUpperCase() in WORD_CLASS }));
"""
        r = subprocess.run([node, "-e", feed], capture_output=True, text=True, timeout=30)
        try:
            F = json.loads(r.stdout)
        except ValueError:
            F = {}
        check(f"the page renders the demo answer: one row per event (<= 30), every kind and the word known "
              f"({F.get('n')} rows, word {F.get('word')})",
              F.get("n") == min(30, len(d["events"])) and not F.get("unknown") and F.get("wordKnown") is True
              and "--:--" not in F.get("times", ["--:--"]) if d["events"] else F.get("wordKnown") is True,
              json.dumps(F)[:300] + r.stderr[:200])
    elif ok_shape:
        print("  -- skipped, no node here: the demo answer through the page's functions")
if server is not None:
    server.shutdown()

# The stack's own decision lines (one per kind, as a real exploration logs them) through the
# server's parser and the page's functions: every kind the server makes has a pill, the word a colour.
REAL = [("explorer2.py", "goal 1: (-1.27, 1.28) 2.0 m away, 35 frontier cells, 12 clusters, chosen in 114 ms"),
        ("global_planner.py", "Got new goal"),
        ("global_planner.py", "Found safe goal."),
        ("local_planner.py", "Obstacle detected ahead, stopping local planner."),
        ("global_planner.py", "Replanning path due to obstacle found."),
        ("global_planner.py", "Robot is stuck. Replanning."),
        ("global_planner.py", "Travelling to goal 1.2850097467377768m away from requested goal."),
        ("moss_adapter.py", "MOSS base: tracks frozen at L=+56% R=+50% - pushing at 75 % for 1 s before calling it a contact"),
        ("moss_adapter.py", "MOSS base: STALL #1: tracks commanded L=+76% R=+64% but the encoders moved L=+1 R=+0 ticks "
                            "in 1.0 s - stopped (the encoders are the bumper)"),
        ("recovering_planner.py", "BUMP received"),
        ("recovering_planner.py", "Contact escape"),
        ("explorer2.py", "planner gave up on that goal: excluded until the map or the viewpoint changes"),
        ("explorer2.py", "10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, "
                         "not stopping"),
        ("explorer2.py", "starvation escape: the world stayed silent for 12 asks - reopening the oldest failed goal at "
                         "(-1.27, 1.28)"),
        ("local_planner.py", "Reached goal position, starting final rotation"),
        ("wavefront_frontier.py", "Stopped autonomous frontier exploration")]
if not hasattr(mc if "mc" in globals() else None, "decisions_from_records"):
    print("  -- skipped, this cockpit server has no decisions_from_records yet: the real lines through the page")
elif node is None:
    print("  -- skipped, no node here: the real lines through the page's functions")
else:
    recs = [{"ts": T0 + 5.0 * i, "level": "info", "event": e, "logger": lg} for i, (lg, e) in enumerate(REAL)]
    evs = mc.decisions_from_records(recs)
    doc = mc.decisions_doc(evs, 30, True, True, T0 + 5.0 * len(REAL), "bench-run") \
        if hasattr(mc, "decisions_doc") else {"now": mc.now_state(evs, True, True, T0 + 5.0 * len(REAL)), "events": evs}
    feed = "\n".join(defs[n] for n in ("pad2", "dur", "esc", "DECISION_ROWS", "KIND_CLASS", "KIND_LABEL",
                                       "WORD_CLASS", "WORD_DETAIL", "tsSeconds", "minSec", "decisionRows",
                                       "nowView")) + "\nvar d = " + json.dumps(doc) + r""";
var rows = decisionRows(d.events, DECISION_ROWS);
console.log(JSON.stringify({ n: rows.length, kinds: rows.map(function (r) { return r.label; }),
  unknown: d.events.filter(function (e) { return !(e.kind in KIND_LABEL); }).map(function (e) { return e.kind; }),
  newest: rows.length ? rows[0].text : null, word: nowView(d.now, 0).word,
  wordKnown: String(d.now.word).toUpperCase() in WORD_CLASS }));
"""
    r = subprocess.run([node, "-e", feed], capture_output=True, text=True, timeout=30)
    try:
        F = json.loads(r.stdout)
    except ValueError:
        F = {}
    kinds = {e.get("kind") for e in evs}
    check(f"{len(REAL)} real decision lines -> the server's {len(evs)} events -> the page: every kind has its pill, "
          f"newest on top, the word known ({F.get('word')})",
          len(evs) >= 10 and {"goal", "obstacle", "contact", "reached"} <= kinds and F.get("n") == len(evs)
          and not F.get("unknown") and F.get("newest") == evs[-1].get("text") and F.get("wordKnown") is True,
          json.dumps({"kinds": sorted(k for k in kinds if k), "page": F})[:400] + r.stderr[:200])

_TMPDIR.cleanup()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
