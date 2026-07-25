# Bringing the PEAK CAN adapter up at 250000 automatically

Goal: whenever the PCAN-USB is plugged in, `can0` comes up at 250 kbit/s with
no manual `ip link` command. Two pieces do this on Ubuntu:

1. a **udev rule** that fires when the adapter appears, and
2. a **systemd-networkd** `.network` file that sets the bitrate and brings the
   link up.

systemd-networkd is the reliable half: it owns the interface and reapplies the
config on every appearance. The udev rule just nudges networkd to reconfigure
the instant the device shows up, so you do not wait for the next poll.

## 1. systemd-networkd

Enable networkd if it is not already running:

```bash
sudo systemctl enable --now systemd-networkd
```

Create `/etc/systemd/network/80-can.network`:

```ini
[Match]
Name=can0

[CAN]
BitRate=250000
# RestartSec bus-off recovery: if the controller goes bus-off (e.g. a wiring
# fault), try to recover automatically instead of staying dead until reboot.
RestartSec=100ms
```

That is the whole steady-state config. On most setups this alone brings `can0`
up at the right bitrate whenever it appears, because networkd reacts to the
kernel's device-add event.

`can0` naming: if you have exactly one PEAK adapter it will almost always be
`can0`. With more than one, the numbering can swap between boots. If that
matters, pin a stable name with a link file (below) and match on that instead.

## 2. udev rule (belt and suspenders)

Create `/etc/udev/rules.d/90-pcan.rules`:

```
# PEAK PCAN-USB / PCAN-USB FD. Vendor 0c72 covers the PEAK USB family.
# On add, ask networkd to (re)configure the interface immediately.
ACTION=="add", SUBSYSTEM=="net", KERNELS=="*", DRIVERS=="peak_usb", \
    RUN+="/usr/bin/systemctl restart systemd-networkd"
```

Reload udev:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The rule matches on the `peak_usb` kernel driver, so it fires for the PCAN
regardless of which `canN` name it gets.

## 3. Optional: pin a stable interface name

If you run more than one adapter, or just want `can0` to always be the PEAK,
create `/etc/systemd/network/80-can.link`:

```ini
[Match]
Driver=peak_usb

[Link]
Name=can0
```

Then match your `.network` file on `Name=can0` as above. With a single adapter
this is not necessary.

## Verify

Unplug and replug the adapter, then:

```bash
ip -details -statistics link show can0
```

You want to see `state UP`, `bitrate 250000`, and `state ERROR-ACTIVE` (not
BUS-OFF). The HMI reads exactly these fields for its link-health panel, so if
the TUI shows `LINK UP  socketcan:can0  250k` you are done.

If it comes up down or at the wrong bitrate:

```bash
# what networkd thinks
networkctl status can0
# force a reconfigure
sudo networkctl reconfigure can0
# manual fallback (what you were doing before)
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 250000
```

## How this maps to the HMI

The HMI does not configure the bitrate itself -- it opens whatever `can0` is
already set to and *reports* the bitrate it finds (`read_socketcan_health`
parses `ip -details link`). So this config is what makes the HMI's
`--bitrate 250000` default actually match the wire. If the netdev were brought
up at a different rate, the HMI would show frames-but-garbage or the
`NO_TRAFFIC / check bitrate` fault, which is the symptom that sends you back
here.

On Windows (PCAN via the PEAK driver) none of this applies: the bitrate is set
when the HMI opens the bus (`--backend pcan --channel PCAN_USBBUS1
--bitrate 250000`), because the Windows PCAN API takes the bitrate at open
time rather than from a persistent netdev.
