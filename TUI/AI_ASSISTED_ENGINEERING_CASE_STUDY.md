# Building a Boat Autopilot HMI with an AI Collaborator: A Case Study

*A worked example of using an AI assistant for real embedded/marine systems engineering — including the mistakes, the recoveries, and the working method that made it productive.*

---

## What this document is

This is the story of building **RemoteRudder**, a heading-hold autopilot HMI for
a V-drive ski/wakeboard boat, over an extended collaboration between a human
engineer and an AI assistant (Claude). It is written to be useful in two ways:

1. **As an engineering log** — what was built, why, and how each decision was
   grounded in real data.
2. **As a demonstrable example of how to work with an AI on non-trivial
   systems code** — what the human did well, where the AI went wrong, and the
   habits that turned a chat interface into a genuine engineering partnership.

The project reached **v0.9.13 with 607 passing tests** across roughly twenty
incremental delivery rounds, most of them driven by real on-water sea trials.
Crucially, this document does **not** hide the AI's errors. Several rounds exist
precisely *because* the AI got something wrong and the human's testing caught it.
That is the honest and more useful version of the story.

---

## The system, in brief

RemoteRudder holds a boat on a heading. On this particular boat that is a hard
problem, because it is a **V-drive with a fixed prop and a small rudder** — it
only steers with forward way on, and barely at that. The HMI has to read the
boat's real sensors, fuse them into a trustworthy heading, run a control loop,
and present all of it to the operator.

**Hardware / signal chain:**

- Raspberry Pi + PEAK PCAN adapter on a socketcan bus at 250 kbit/s
- A J1939 / NMEA 2000 marine bus with Garmin electronics
- A wall-mounted 3-axis compass (the rock-steady heading source)
- A Garmin GPS 24xd and a Zero-Off GPS (course-over-ground sources)
- A steering node (string potentiometer on an intermediate power shaft) and a
  separate rudder node (string potentiometer on the actual rudder)
- A stepper driving the rudder through timing-belt gearing

**Software:**

- Python control and HMI: a `curses` TUI *and* a Tornado web UI, later unified
  into a single process
- A heading priority chain plus an extended Kalman filter (EKF) running
  alongside it
- A filtered CAN recorder for shareable on-water logs
- ~40 source modules, ~600 tests

---

## The working method (this is the actual lesson)

Before the round-by-round history, here is the method that made the
collaboration work. If you take one thing from this document, take this section.

### 1. Ground every number in real data, never in vibes

The single most repeated principle. Thresholds were not guessed; they were
measured from real `candump` logs:

- The **45°/s yaw-rate limit** came from measuring actual yaw rates in a log.
- The **4 mph COG-primary transition** and its hysteresis came from watching
  where COG became trustworthy in real data.
- The **~0.6° compass standard deviation** baseline (used to set stability-test
  thresholds) was measured on a parked boat.

When the AI proposed a threshold from intuition, the correct move — which the
human insisted on — was to go back to the logs and measure it. Every "magic
number" in the codebase can be traced to a measurement.

### 2. Write tests that fail when the bug comes back

A test that passes is nearly worthless if it would *also* pass with the bug
present. Repeatedly in this project, after fixing a bug the AI would:

1. Write a regression test.
2. **Reintroduce the bug** and confirm the test now fails.
3. Restore the fix and confirm the test passes again.

This caught at least one case where the AI's *own test* did not actually
exercise the bug (see Round 10, where the first version of the stability test
passed even with the bug reintroduced, because the test fixture was missing the
messages that triggered it). Verifying the failure is what made the tests real.

### 3. Flag design conflicts *before* building

The clearest example: when the "fishing mode" requirement (hold
course-over-ground so lines trail straight off the transom) was specified, it
revealed that the existing sensor-fusion logic was oriented the wrong way.
Catching that at design time, out loud, saved a large rework. The habit — state
the conflict plainly and get a decision before writing code — is worth more than
any single fix.

### 4. Make the human's testing the ground truth

The human ran the code on the actual boat and brought back screenshots, CAN
logs, and the occasional stack trace. The AI's job was to reproduce those
symptoms *in software* against the real logs before proposing a fix. "It should
work" was never allowed to stand in for "here is the value flowing through the
real pipeline." When the AI drifted into confident static analysis, it was
usually wrong (see the compass-wheel saga, Rounds 13/19).

### 5. Version everything, because stale files are the silent killer

A recurring, genuinely costly failure mode: a fix that was correct in the repo
but **not actually running on the boat**, because a file wasn't copied or the
browser cached an old copy. The eventual fix was to stamp versions everywhere —
the TUI header, a `--version` flag, and finally a console version line in the
browser JavaScript — so "which version is actually running?" became a
one-second question instead of a multi-message misdiagnosis.

### 6. Deliver in small, reversible increments

Work shipped as numbered "rounds," each a small overlay package with an
`INSTALL.md` and a running `CHANGES.md`. Each round was independently testable
and, if wrong, independently revertible. This kept the blast radius of any
single mistake small.

---

## The rounds: what was built, why, and what it taught

The history below is deliberately honest about where things went wrong. The
"correction" rounds are the most instructive.

### Foundations (Rounds 1–2): find the bugs that were already there

The very first work was not new features — it was running the *existing* code
against a real `candump` capture and a live server, and finding bugs. Five real
ones surfaced immediately, each verified failing before the fix.

The most serious was a heading-selection bug:

```python
sig = st.get_signal("fused_heading") or st.get_signal("compass_heading")
```

This reads like a fallback but never is one: both signals always exist as
objects, and an object is always truthy regardless of whether its `.value` is
`None`. So the `or` never reached the second operand, and the autopilot could be
**permanently blocked from engaging even with a perfect compass**. The fix
selected on `is_valid()` (freshness) rather than object truthiness.

**Lesson:** `get_signal(a) or get_signal(b)` is a truthiness trap anywhere it
appears. The first useful thing an AI can do on an existing codebase is read it
critically against real data, not immediately add features.

### Rounds 3–9: build the real capabilities

With the foundation solid, the substantive features landed:

- **A curses TUI** with touch buttons and a live CAN source table.
- **J1939 NAME-bound source identity** — devices are identified by their stable
  address-claim identity (manufacturer, function, serial), not by the momentary
  bus address they happen to hold. This is what lets the table say "Garmin
  Heading Sensor #1039212" instead of just "0xF8."
- **A standalone on-water calibration TUI** for the GPS 24xd.
- **COG-primary "fishing mode"** — holds course-over-ground at speed so fishing
  lines trail straight off the transom in a crosswind. Transitions at 4 mph with
  hysteresis so it doesn't chatter at idle.
- **Yaw-rate limiting** at a measured 45°/s ceiling.
- **A heading EKF** (2-state and 3-state variants) running *alongside* the
  priority chain, never replacing it — a deliberate architectural choice so a
  filter problem could never silently take over steering.
- **Live web graphs** and a **filtered CAN recorder** (~17% of the bus, so an
  on-water capture is small and shareable).
- **Version stamping** — the first defense against the stale-file problem.

### Round 10: the first sea trial, and a test with no teeth

The boat went in the water, and two real bugs came back:

1. **Heading bounced** between ~180° (the real compass) and ~39°. The cause: the
   GPS *vehicle-direction* message (a stale stationary course) was being written
   into the `compass_heading` signal, fighting the real compass. A crossed wire
   in the decoder.
2. **The goal bounced** between the commanded value and 360°, because two goal
   variables normalized differently (`% 360` in one place, `> 360` in another).

Both were fixed. But the more important story is the **stability tests** added
here — and the fact that the AI's *first* version of them was worthless. The
test replayed a parked-boat log and asserted the heading didn't bounce... but it
passed even with the bug reintroduced, because the fixture the AI carved had
accidentally dropped the very messages that caused the bounce. Only by insisting
on "verify the test fails when the bug is present" did this surface. The fixture
was recarved to include the vehicle-direction messages, and *then* the test had
teeth.

**Lesson:** a green test is not evidence until you've watched it go red for the
right reason.

### Rounds 11–12: make the filter honest about uncertainty

The human asked a sharp question: *if the app runs a long time with no CAN, the
EKF's sigma (its self-estimated uncertainty) grows — will enabling CAN later
cause a problem?*

Investigating it surfaced a real bug: with **no** heading reference at all, the
EKF was seeding itself to 0° (north) and publishing it — feeding a **fabricated
north heading into the control loop while blind**. The fixes: don't build the
filter until a real reference exists; read signals with *freshness* so a stale
value can't drive the filter; and stop publishing after coasting too long.

Then a second sea trial (parked facing south) exposed the opposite failure: the
EKF's sigma sat around 12° — too high to engage — *even though a rock-steady
compass was right there*. The human's insight drove the fix: **if the filter
isn't confident, fall back to the priority chain and use the good compass
directly.** The compass was also promoted to an absolute reference for the
filter (weighted by whether COG was available), so a parked boat's sigma
actually converges.

**Lesson:** an uncertainty estimate is only useful if the system *acts* on it —
trusting the filter when it's confident and falling back when it isn't.

### Rounds 13 & 19: the compass wheel, and the value of not trusting yourself

This is the most instructive failure in the whole project, so it gets the most
detail.

The web compass wheel was supposed to rotate so the boat's heading sits at the
bow — parked facing south, "S" at the top. It didn't rotate. Across **two
separate rounds**, the AI claimed to fix it and was wrong the first time.

In Round 13 the AI found a real data bug (the heading feeding the wheel was
stale) and fixed it — but the wheel *still* didn't turn, and the AI had verified
the data and the rotation math "in isolation" and concluded it should work. It
did not.

The breakthrough in Round 19 came only when the AI stopped doing static analysis
and **actually rendered the wheel in a headless browser**, then diffed the image
at heading 0° versus 180°. The images were **identical** — proof the wheel was
static regardless of the math looking correct. The real bug: the cardinal
letters (N/E/S/W) were positioned at fixed ring angles and then each glyph was
counter-rotated by `+heading` to "stay upright" — which *cancelled* the ring's
`-heading` rotation for the letters, pinning "N" to the top forever. And because
the tick marks are rotationally symmetric every 5°, they gave no visual cue that
anything was (or wasn't) moving.

**Lesson:** "I verified the pieces and it should work" is the exact sentence to
distrust. For anything visual or timing-dependent, render it / run it / measure
the actual output. The AI's confident static reasoning was wrong twice in a row
on the same bug; the headless render found it in one shot.

### Rounds 14–18: refinements and one self-inflicted crash

- **Round 14:** the address-claim columns (manufacturer, function, serial) were
  only visible in a very wide terminal; they were brought into the normal
  compact table.
- **Round 15:** a genuine architecture fix. The human was running the web UI and
  the TUI as **two separate processes**, each with its own autopilot and its own
  goal variable, sharing only the CAN bus — so a goal set on the web never
  reached the TUI. They were unified into a single process with one source of
  truth. (The AI had earlier misdiagnosed this as the `% 360` bug; the DOM and
  logs showed it was actually two programs disagreeing.)
- **Round 16:** the PID loop was rewritten. The integral was being reset to zero
  every cycle, so it never accumulated — meaning on a stationary boat chasing an
  unreachable goal, the shaft never "limited out" as it should. It became a true
  accumulating integrator (clamped to bound overshoot), and a live PID tuning
  panel was added to the TUI so gains can be tuned on the water.
- **Round 17:** engaging now seeds the goal to the current heading and pins zero
  initial error, so the boat holds its present course instead of lurching.
- **Round 18:** a crash the AI *caused*. The disengaged goal-tracking logic
  (added in Round 13) used a variable the AI never initialized in `__init__`, so
  the control-loop thread died with `AttributeError` on the first disengaged
  tick. The fix was one line — but the real fix was realizing **the PID tests
  never started the actual control-loop thread**, so the bug was in a code path
  no test exercised. Tests were added that run the real thread.

**Lesson (Round 18 especially):** unit tests that call a function directly can
miss bugs that only appear when the real thread/loop runs. Test the thing that
actually runs.

### Rounds 20–21: the stale-file endgame

The compass-face value labels (Heading / Speed / Rudder / Goal drawn on the
canvas) went missing. The AI relocated them to draw as the top layer (Round 20),
verified on a real served page that they render — and the human *still* didn't
see them.

The human then sent the **actual rendered DOM from the boat**. Running the AI's
current code against that exact DOM proved the labels render correctly. The
conclusion was unavoidable: the boat was loading a **stale compass.js** (an old
file on the Pi, or a cached copy in the browser). Round 21 added a **version
stamp printed to the browser console**, so "which JS is actually loaded?" became
answerable in one second — and wired up the Graphs button, which had never had a
click handler.

**Lesson:** when the code is provably correct but the symptom persists, suspect
the *delivery*, not the logic. The recurring cost of stale files across this
project is why version stamping ended up everywhere: TUI header, CLI flag, and
browser console.

---

## Patterns worth stealing

Distilled from the whole collaboration:

- **Measure, don't guess.** Every threshold traces to a real log.
- **Verify tests fail for the right reason**, by reintroducing the bug.
- **Render/run visual and timing behavior**; never trust static reasoning about
  it. The wheel bug cost two rounds because this wasn't done at first.
- **One source of truth.** Two processes (or two variables) holding "the goal"
  is a bug waiting to happen.
- **Act on uncertainty.** A filter's confidence estimate should change behavior
  (trust vs. fall back), not just be displayed.
- **Fail safe.** The EKF must never publish a fabricated heading; the shaft
  clamps to physical travel; freshness gates everything.
- **Version everything.** Stale files are the silent killer; make the running
  version trivially visible at every layer.
- **Small reversible increments** keep any single mistake cheap.

---

## What the human did that made this work

An AI assistant is only as good as the loop it's placed in. The things the human
did that made this productive:

- **Brought real artifacts** — screenshots, CAN logs, stack traces, and finally
  the rendered DOM — instead of describing symptoms secondhand.
- **Ran the code on the actual boat** and treated that as ground truth over any
  amount of AI reasoning.
- **Corrected the AI's domain misunderstandings** directly (e.g. that the
  "shaft" is a geared intermediate power shaft reported by one string pot, and
  the rudder angle is a *separate* string pot from a different node — the AI had
  conflated them, and the correction reshaped the PID documentation and design).
- **Insisted on data over intuition** for every threshold.
- **Made design decisions** when the AI surfaced a conflict, rather than letting
  the AI guess (e.g. choosing "true accumulating integral, clamp the output"
  over alternatives).
- **Pushed back** when a fix didn't actually work on the boat, which is what
  forced the AI past its incorrect static analysis to the real render-based
  diagnosis.

## What the AI contributed

- **Read the existing code critically** against real data and found latent bugs
  before any features were added.
- **Reproduced symptoms in software** against the real logs before proposing
  fixes.
- **Wrote and maintained the test suite** (~600 tests), including the discipline
  of verifying each test fails when its bug is reintroduced.
- **Flagged design conflicts** early (the fishing-mode/fusion orientation
  clash).
- **Produced the incremental delivery packages** and kept the running changelog.
- **Owned its mistakes** — the crash it introduced, the tests without teeth, the
  compass-wheel misdiagnosis — and fixed both the bug and the process gap that
  let it through.

---

## Honest limitations

For a fair "how to use AI" example, the failure modes matter as much as the wins:

- The AI was **overconfident in static analysis** of visual/timing behavior and
  was wrong twice on the compass wheel before switching to actually rendering
  it.
- It **shipped a test that didn't test the bug** (Round 10) and **left a
  variable uninitialized** that crashed the control loop (Round 18) — both
  caught only by the human's real-world testing and the "verify the failure"
  discipline.
- It repeatedly under-weighted the **stale-file / caching** possibility,
  spending multiple exchanges on logic when the real issue was delivery.

None of these were catastrophic *because* the human kept real-world testing in
the loop and the increments were small and reversible. That is the real
takeaway: **AI is a powerful engineering collaborator when it is embedded in a
tight loop of real data, real tests, and a human who verifies on the actual
hardware — and it is dangerous exactly to the degree that loop is missing.**

---

*RemoteRudder is developed as an AI-assisted engineering case study. The full
per-round detail lives in `CHANGES.md`; each round also shipped with its own
`INSTALL.md`. At the time of writing the project is at v0.9.13 with 607 passing
tests.*
