"""Regression suite for the HMI fixes. Run from the HMI directory."""
import math, random, sys, time
from hmi_state import SystemState, F_LINK_DOWN, F_NO_TRAFFIC, F_HEADING_INVALID
from hmi_heading import HeadingMonitor, circular_mean, circular_sigma
from hmi_bridge import Bridge
from hmi_canlink import CANLink, backend_for_platform

fails = []
def ck(name, cond, extra=""):
    print(("  PASS " if cond else "  FAIL ") + name + (("  " + str(extra)) if extra else ""))
    if not cond:
        fails.append(name)

print("\n[1] goal normalization + non-finite rejection")
st = SystemState()
ck("370 -> 10", st.set_goal(370, "t") == 10.0)
ck("-10 -> 350", st.set_goal(-10, "t") == 350.0)
ck("360 -> 0", st.set_goal(360, "t") == 0.0)
ck("1000 -> 280", st.set_goal(1000, "t") == 280.0)
st.set_goal(0, "t")
ck("0 - 1 wraps to 359", st.adjust_goal(-1, "t") == 359.0)
st.set_goal(130, "ok")
st.set_goal(float("nan"), "bad")
ck("NaN rejected", st.heading_goal == 130.0, st.heading_goal)
st.set_goal(float("inf"), "bad")
ck("inf rejected", st.heading_goal == 130.0, st.heading_goal)
st.set_goal("junk", "bad")
ck("str rejected", st.heading_goal == 130.0)
st.set_goal(None, "bad")
ck("None rejected", st.heading_goal == 130.0)

print("\n[2] goal provenance")
st2 = SystemState(); seen = []
st2.subscribe(lambda k, p: seen.append((k, p)))
st2.set_goal(130, "web/192.168.1.47")
ck("notify fired", any(k == "goal" for k, _ in seen))
ck("source recorded", st2.goal_source == "web/192.168.1.47")
n = len(seen); st2.set_goal(130, "tui")
ck("no-op does not re-notify", len(seen) == n)

print("\n[3] circular statistics across the 0/360 seam")
ck("mean(359,1) ~ 0", abs(((circular_mean([359.0, 1.0]) + 180) % 360) - 180) < 1e-6)
ck("sigma tight", circular_sigma([100.0, 100.5, 99.5, 100.2]) < 1.0)
ck("sigma at seam stays small", circular_sigma([359.8, 0.2, 359.5, 0.5]) < 1.0)

print("\n[4] engage interlock (the truthy-Signal bug)")
st3 = SystemState(); mon3 = HeadingMonitor(st3)
random.seed(1); t = time.time()
for i in range(40):
    mon3.add_heading(100.0 + random.gauss(0, 0.4), t + i * 0.1)
    mon3.add_pitch(random.gauss(0, 0.3), t + i * 0.1)
    mon3.add_roll(random.gauss(0, 0.3), t + i * 0.1)
st3.set_signal("compass_heading", 100.0)
ck("compass-only selected when fused is None", mon3.best_heading_signal() is not None)
ok, why = mon3.ok_to_engage()
ck("calm compass -> engage ALLOWED", ok, why)
mon3.update_state()
ck("no spurious HEADING_INVALID", F_HEADING_INVALID not in st3.faults)

print("\n[5] heading noise attribution")
st4 = SystemState(); mon4 = HeadingMonitor(st4); t = time.time()
for i in range(40):
    mon4.add_heading(100.0 + random.gauss(0, 9.0), t + i * 0.1)
    mon4.add_pitch(random.gauss(0, 6.0), t + i * 0.1)
    mon4.add_roll(random.gauss(0, 7.0), t + i * 0.1)
st4.set_signal("compass_heading", 100.0)
q, why = mon4.diagnose()
ck("rough water -> BAD", q == "BAD", q)
ck("blames sea state", "sea state" in why)
ck("engage BLOCKED", not mon4.ok_to_engage()[0])

st5 = SystemState(); mon5 = HeadingMonitor(st5); t = time.time()
for i in range(40):
    mon5.add_heading(100.0 + random.gauss(0, 9.0), t + i * 0.1)
    mon5.add_pitch(random.gauss(0, 0.2), t + i * 0.1)
    mon5.add_roll(random.gauss(0, 0.2), t + i * 0.1)
st5.set_signal("compass_heading", 100.0)
q, why = mon5.diagnose()
ck("noisy + flat -> suspect compass", "compass" in why, why)

st6 = SystemState(); mon6 = HeadingMonitor(st6); t = time.time()
for i in range(40):
    mon6.add_heading(100.0 + random.gauss(0, 12.0), t + i * 0.1)
mon6.set_filter_sigma(0.5)
ck("overconfident filter clamped up", mon6.effective_sigma() > 1.0, mon6.effective_sigma())

st7 = SystemState(); mon7 = HeadingMonitor(st7); t = time.time()
for i in range(40):
    mon7.add_heading((359.5 + random.gauss(0, 0.4)) % 360.0, t + i * 0.1)
    mon7.add_pitch(0.0, t + i * 0.1); mon7.add_roll(0.0, t + i * 0.1)
st7.set_signal("compass_heading", 359.5)
ck("steering north not flagged noisy", mon7.diagnose()[0] == "GOOD")

print("\n[6] turn-rate aliasing guard")
st8 = SystemState(); mon8 = HeadingMonitor(st8); t = time.time()
mon8.add_heading(0.0, t); mon8.add_heading(170.0, t + 1.0)
ck("implausible rate suppressed", mon8.rate_dps() is None, mon8.rate_dps())
st9 = SystemState(); mon9 = HeadingMonitor(st9); t = time.time()
mon9.add_heading(100.0, t); mon9.add_heading(105.0, t + 1.0)
ck("plausible rate reported", abs(mon9.rate_dps() - 5.0) < 1e-6, mon9.rate_dps())

print("\n[7] CAN link faults")
ck("virtual -> vcan0", backend_for_platform("virtual", "can0") == ("virtual", "vcan0"))
ck("pcan honored (Windows path)", backend_for_platform("pcan", "PCAN_USBBUS2") == ("pcan", "PCAN_USBBUS2"))
sta = SystemState()
link = CANLink(sta, channel="can_absent_99", bitrate=250000, backend="socketcan")
ck("missing adapter: no sys.exit", link.open() is False)
ck("missing adapter: LINK_DOWN", F_LINK_DOWN in sta.faults)
ck("missing adapter: actionable detail", "ip link set" in sta.faults[F_LINK_DOWN].detail)
b1 = link.next_backoff(); link.open()
ck("backoff grows", link.next_backoff() > b1)
stb = SystemState()
link2 = CANLink(stb, channel="vcan0", bitrate=250000, backend="virtual")
ck("virtual bus opens", link2.open() is True)
link2._opened_at = time.time() - 10; stb.rx_total = 0; stb.last_rx_any = 0.0
link2.poll_health()
ck("silent bus -> NO_TRAFFIC", F_NO_TRAFFIC in stb.faults)
ck("names bitrate as suspect", "bitrate" in stb.faults[F_NO_TRAFFIC].detail)
stb.rx_total = 5; stb.last_rx_any = time.time(); link2.poll_health()
ck("traffic resumes -> cleared", F_NO_TRAFFIC not in stb.faults)
link2.close()

print("\n[8] stale source detection")
stc = SystemState(); monc = HeadingMonitor(stc); brc = Bridge(stc, monc)
now = time.time()
stc.mark_source(0x09F112F8, now - 10)
stc.mark_source(0x0CF00400, now)
stale = [s.name for s in stc.sources.values() if s.ever_seen and s.age(now) > s.stale_after]
ck("only the quiet source is stale", stale == ["vessel_heading"], stale)

print("\n" + ("ALL PASS" if not fails else f"FAILURES: {fails}"))
sys.exit(1 if fails else 0)
