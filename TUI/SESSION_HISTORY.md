# Session history — the engineering narrative

This is the story of how the work was done: the problems, the decisions, and —
importantly — the places where measuring the real data changed the answer. If
you're keeping this project as a study in working with AI, this is the part
worth reading. The pattern throughout: don't trust assumptions (mine or the
manual's), look at the actual bus data, and let it decide.

## The recurring method

Almost every round followed the same shape:

1. A requirement or a symptom (often with a screenshot or a candump log).
2. A hypothesis about the cause.
3. **Measuring the real logs to test the hypothesis** — and frequently finding
   the hypothesis was wrong, or right for the wrong reason.
4. Building the fix, with tests written against the real captured data.
5. The tests catching a second, subtler bug before it shipped.

That third step is the one that mattered most. Several times the "obvious"
explanation was wrong and only the data revealed it.

## Round 3 — TUI, COG-primary groundwork, test suite

Built the curses TUI (touch buttons, source table), the first COG-primary
fusion, and the pytest suite with real candump-log fixtures. Early bugs found
and fixed: a permanent engage-block, NaN/inf poisoning the goal, a pitch value
decoded 200° off (a J1939 offset error), and a replay that froze freshness. The
web test suite came in here too.

## Round 4 — the compass that wasn't broken

**Symptom:** the heading appeared to bounce 179° → 30° while the boat sat still,
and the source table showed no compass identity.

**Hypothesis:** the compass was failing.

**What the data said:** the compass (source 0xF8) read a rock-steady ~170° the
whole time. It never bounced. The "bounce" was the *fused* value switching
between the good compass and a stale GPS course (source 0x1C) that reads garbage
when stationary. The compass was fine; the fusion was hopping sources.

This reframed everything. The fix wasn't to repair the compass — it was to make
source identity visible. Added J1939 address-claim (NAME) decoding, an 8-column
source table showing manufacturer/function/identity, and a "request addresses"
button to force devices to announce themselves. Validated the decoder against
the real claim: source 0x1C = Garmin, Marine, identity 328706.

**Lesson:** the first explanation ("the sensor broke") was wrong. The data
showed the sensor was the reliable one and the fusion logic was the problem.

## Round 5 — the 375° heading, and NAME binding

You installed the GPS 24xd and captured a log with every device claiming. Five
devices announced their NAMEs; the 24xd came up at source 0x19, sending the full
picture (heading, attitude, magnetic variation, position, COG/SOG).

**Symptom:** the 24xd's heading decoded to a constant 375.5° — impossible.

**What the data said:** the raw field was 0xFFFE, the NMEA "data not available"
sentinel. The 24xd was uncalibrated and honestly reporting "no heading"; the
decoder was scaling the sentinel into a bogus number that then poisoned the
fusion and drove the HEADING_NOISE faults. Fixed by rejecting the sentinel.

The bigger piece: heading now comes from two devices at different addresses.
Built the NAME-bound source registry — sources identified by stable identity, so
they survive address changes — and a standalone on-water calibration tool for
the 24xd. A subtle bug surfaced here: all registry instances were sharing the
same profile objects (a mutable-default trap), so enabling the 24xd in one place
would have enabled it everywhere. The tests caught it before it shipped.

Also fixed the manual-step units: they were labeled "degrees" but the steering
goal is in encoder counts (the screenshots showed 1475 = center). Changed to
increments of 10 counts, as you use them.

## Round 6 — fishing mode, and confronting a design conflict

**Requirement:** in wind and waves, hold a straight *track* so the fishing lines
trail off the transom.

**The conflict I had to flag:** the fusion as built did the opposite of what you
needed. It held magnetic heading at all speeds. But heading and course-over-
ground are different things — in a crosswind the boat crabs, pointing one way
and traveling another. Holding heading would let it drift off track. For fishing
you must hold COG.

So I inverted it: above a speed threshold, COG becomes primary. And I measured
the logs to set the numbers rather than guessing:

- **Yaw rate:** real turns topped out at ~41°/s across every log. So 45°/s is
  just above the real maximum — it clips impossible jumps without touching a
  legitimate sharp turn.
- **COG noise:** even at planing speed, ~12% of COG samples jumped more than
  45°/s (GPS jitter). This proved COG needs *rate-limiting*, not just a speed
  gate — the speed gate alone wouldn't have been enough.

Built the yaw-rate limiter and stable-near-last fallback, with visualizations so
you could confirm the noise behavior made sense before trusting it.

**Lesson:** the requirement exposed that the existing design was backwards, and
measuring the data (not assuming) set every threshold.

## Round 7 — your EKF idea, and where it pays off

**Your idea:** even a biased compass has an accurate *derivative* — a real 10°
yaw still moves the compass ~10°, so the compass yaw *rate* is trustworthy even
when the heading isn't.

**The honest caveat:** correct for declination and hard-iron (constant biases
that differentiate away), but soft-iron *scales* the derivative — a real 10° turn
can read 8° or 13°. So the compass rate is a good input after soft-iron
calibration and with spike-gating (both of which already existed).

You asked to see both a simple and a complex filter and decide from the plots.
The comparison was decisive and, again, not what intuition might suggest:

- 2-state EKF: 2.33° RMS — best with compass derivative alone.
- 3-state EKF (no gyro): 2.88° RMS — *worse*, because the bias state is
  unobservable without an independent reference.
- 3-state EKF (with gyro): 0.77° RMS — best by far.

So the extra complexity only helps once there's an independent gyro. The bridge
picks automatically. The EKF runs *alongside* the priority chain (your choice),
so enabling it can't break the working fusion.

**Lesson:** "more sophisticated" isn't automatically better — the 3-state filter
was worse until the sensor suite justified it. The data drew that line.

## Round 8 — a small change, made robust

**Requirement:** move the COG transition to 4 mph, because the boat at idle
often exceeds 3 mph and you didn't want constant transitions.

Rather than just move the number, I added hysteresis: engage COG at 4 mph, drop
back to magnetic only below 3 mph. A single threshold still chatters when the
boat hovers right at it; the dead-band fixes that. Verified with a simulated
idle-wobble that a single threshold would have flipped on every sample and the
hysteresis reduced to one transition.

## Round 9 — instrumentation for the water

**Requirement:** live graphs and a shareable log, because on the water you can't
upload a full candump for analysis.

Built live web graphs (heading sources, sensor disagreement, rudder command, EKF
state), a status bar with the active source and a log-download link, EKF numbers
in the TUI, and a filtered CAN recorder that logs only the ~17% of bus traffic
the HMI actually uses — small enough to share, exactly what the fusion saw.

## What made this work

- **Real data over assumptions.** The compass "failure," the 375° heading, the
  yaw-rate ceiling, the COG noise, the EKF complexity tradeoff — every one was
  settled by looking at the actual bus, not by reasoning from first principles.
- **Flagging conflicts honestly.** The fishing requirement revealed the fusion
  was backwards; saying so plainly was more useful than quietly building what
  was asked.
- **Tests against real captures.** The fixtures are real candump logs, so the
  tests exercise the actual data shapes — and they repeatedly caught second-order
  bugs (the mutable-default registry, the source-switch-vs-glitch distinction,
  the status-hidden-behind-color rendering) before they reached the water.
- **Incremental, reversible delivery.** Each round is a self-contained overlay
  with its own changelog and tests, so any piece can be reviewed or reverted
  independently — which is exactly what you want going into sea trials.

## Next

More sensor nodes. The architecture was built for this: a new heading source is
one registry profile, and an independent gyro is what promotes the EKF from
2-state to the 0.77° 3-state. Sea trials first — then the data from those trials
tunes the noise parameters against reality instead of synthetic estimates.
