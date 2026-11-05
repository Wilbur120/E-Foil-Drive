#!/usr/bin/env python3
"""
Capture trace dumps from the eFoil drive, write them as CSV in engineering
units, and plot them grouped by quantity with the state transitions marked.

    pip install pyserial matplotlib
    py trace_capture.py COM3 run01              one dump, then exit
    py trace_capture.py COM3 run --watch        stay open, number each dump
    py trace_capture.py --list                  show available ports

The script only listens. Arming and triggering happen on the STM32 side:
every throttle-up arms the recorder and the ALIGN to OPEN_LOOP transition
triggers it, so start this first, then pull the throttle. Files are never
overwritten: run01.csv, then run01_02.csv, and so on, unless you pass
--force.

Firmware channel set (trace_ch[] in main.c, 12 channels):
    rpm_est, rpm_ol, id, iq, iq_ref, i_s, vd, vq, idc, vbus, temp, state

Everything is driven by the names in the dump header, so a channel that is
missing is simply skipped and an unknown channel gets its own panel.

Derived columns added to the CSV:
    p_dc   = vbus * idc                    DC link power into the inverter, W
    v_s    = sqrt(vd^2 + vq^2)             stator voltage command magnitude, V
    vmax   = VMAX_FRAC * vbus              voltage circle the PI is clamped to
    p_ac   = 1.5 * (vd*id + vq*iq)         power into the machine, W
             (from the COMMANDED dq voltage, so it excludes the device drop
             and any dead-time residue: treat p_ac/p_dc as an estimate of
             inverter efficiency, not a measurement)
"""

import math
import os
import sys
import time

import serial

# ---- defaults -------------------------------------------------------------
BAUD = 921600
TIMEOUT_S = 60.0

# ---- must match main.c ----------------------------------------------------
RS_OHM = 0.06
LS_H = 40.0e-6
FLUX_WB = 0.020130
POLE_PAIRS = 3.0
VMAX_FRAC = 0.437

STATE_NAMES = {0: "CAL", 1: "IDLE", 2: "ALIGN", 3: "OPEN", 4: "CLOSED",
               5: "FAULT"}
ST_CLOSED = 4

# int16 saturation limit for the clipping warning
CLIP_COUNT = 32767


# ---------------------------------------------------------------- capture --
def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("pyserial tools not available")
        return
    found = list(lp.comports())
    if not found:
        print("no serial ports found")
        return
    for p in found:
        print("  %-10s %s" % (p.device, p.description))

def open_port(port, baud):
    """Keep trying until the port comes back (USB re-enumeration)."""
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1.0)
            ser.reset_input_buffer()
            return ser
        except serial.SerialException:
            time.sleep(0.5)


def capture(ser, timeout_s):
    """Read one complete dump. Returns the raw lines, or None on timeout."""
    lines, started, t0 = [], False, time.time()
    while time.time() - t0 < timeout_s:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", "ignore").strip()
        if line.startswith("#trace"):
            if started:
                print("  warning: new #trace before #end, discarding partial")
            lines, started = [line], True
            print("  capturing: " + line)
            continue
        if not started:
            continue
        if line.startswith("#end"):
            return lines
        if line:
            lines.append(line)
    return None


# ------------------------------------------------------------------ parse --
def parse(lines):
    if len(lines) < 4:
        raise RuntimeError("dump too short to contain a header")

    meta = {}
    for kv in lines[0].lstrip("#").split(",")[1:]:
        if "=" in kv:
            k, v = kv.split("=", 1)
            try:
                meta[k] = int(v)
            except ValueError:
                meta[k] = v

    if not lines[1].startswith("#scale"):
        raise RuntimeError("expected a #scale line, got: " + lines[1][:40])
    scale = [float(x) for x in lines[1].split(",")[1:]]
    names = lines[2].split(",")[1:]
    if len(scale) != len(names):
        raise RuntimeError("scale/name count mismatch (%d vs %d)"
                           % (len(scale), len(names)))

    ts = meta["decim"] / float(meta["fpwm"])   # seconds per stored sample
    trig = meta.get("trig", -1)

    rows, dropped, clipped = [], 0, set()
    for line in lines[3:]:
        f = line.split(",")
        if len(f) != len(names) + 1:
            dropped += 1
            continue
        try:
            n = int(f[0])
            counts = [int(v) for v in f[1:]]
        except ValueError:
            dropped += 1
            continue
        for name, c in zip(names, counts):
            if abs(c) >= CLIP_COUNT - 1:
                clipped.add(name)
        t = (n - trig) * ts if trig >= 0 else n * ts
        rows.append([t] + [c / s for c, s in zip(counts, scale)])

    meta["_dropped"] = dropped
    meta["_clipped"] = sorted(clipped)
    meta["_ts"] = ts
    return ["t_s"] + names, rows, meta


def derive(header, rows):
    """Append p_dc, v_s, vmax and p_ac where their inputs exist."""
    def has(*names):
        return all(n in header for n in names)

    new = []
    if has("vbus", "idc"):
        iv, ii = header.index("vbus"), header.index("idc")
        new.append(("p_dc", lambda r: r[iv] * r[ii]))
    if has("vd", "vq"):
        ivd, ivq = header.index("vd"), header.index("vq")
        new.append(("v_s", lambda r: math.hypot(r[ivd], r[ivq])))
        if has("vbus"):
            iv = header.index("vbus")
            new.append(("vmax", lambda r: VMAX_FRAC * r[iv]))
        if has("id", "iq"):
            iid, iiq = header.index("id"), header.index("iq")
            new.append(("p_ac", lambda r: 1.5 * (r[ivd] * r[iid]
                                                 + r[ivq] * r[iiq])))
    for r in rows:
        r.extend(f(r) for _, f in new)
    return header + [n for n, _ in new], rows


# -------------------------------------------------------------- reporting --
def col(header, name):
    return header.index(name) if name in header else None


def state_events(header, rows):
    """[(t, state_int), ...] at each transition, including the first row."""
    c = col(header, "state")
    if c is None or not rows:
        return []
    ev = [(rows[0][0], int(round(rows[0][c])))]
    last = ev[0][1]
    for r in rows[1:]:
        s = int(round(r[c]))
        if s != last:
            ev.append((r[0], s))
            last = s
    return ev


def integrate(rows, c, ts):
    """Rectangle-rule integral of column c over the rows, in units*s."""
    return sum(r[c] for r in rows) * ts


def summarise(header, rows, meta):
    if not rows:
        print("  no data rows")
        return

    ts = meta["_ts"]
    print("  %d rows at %.1f Hz, t = %.3f to %.3f s, trigger at t = 0"
          % (len(rows), 1.0 / ts, rows[0][0], rows[-1][0]))
    if meta.get("ver", 1) < 2:
        print("  NOTE: ver=%s firmware, speed columns are ELECTRICAL rad/s"
              % meta.get("ver"))
    if meta["_dropped"]:
        print("  WARNING: %d mangled lines dropped (lower TRACE_BAUD?)"
              % meta["_dropped"])
    if meta["_clipped"]:
        print("  WARNING: int16 saturation on: %s"
              % ", ".join(meta["_clipped"]))

    cidc = col(header, "idc")
    if cidc is not None and all(r[cidc] == 0.0 for r in rows):
        print("  WARNING: idc is zero throughout, HASS not calibrated "
              "(idc_sense_ok = 0?), p_dc is meaningless")

    ev = state_events(header, rows)
    if ev:
        print("  states: " + "  ".join(
            "%s@%+.2fs" % (STATE_NAMES.get(s, str(s)), t) for t, s in ev))

    # handoff detail: first sample in CLOSED_LOOP
    cs = col(header, "state")
    cre, cro = col(header, "rpm_est"), col(header, "rpm_ol")
    cvd, cvq = col(header, "vd"), col(header, "vq")
    ho = None
    if cs is not None:
        for r in rows:
            if int(round(r[cs])) == ST_CLOSED:
                ho = r
                break
    if ho is not None:
        bits = ["  handoff at t = %+.3f s" % ho[0]]
        if cre is not None and cro is not None:
            bits.append("rpm_est %.0f vs rpm_ol %.0f" % (ho[cre], ho[cro]))
        if cvd is not None and cvq is not None:
            bits.append("vd %.2f V, vq %.2f V" % (ho[cvd], ho[cvq]))
        print(", ".join(bits))
    elif cs is not None:
        print("  NEVER REACHED CLOSED LOOP: check dbg_ho_best and dbg_ho_fails")

    # steady state over the last 10 percent of the record
    tail = rows[max(0, int(len(rows) * 0.9)):]

    def mean(c):
        return sum(r[c] for r in tail) / float(len(tail))

    parts = []
    if cre is not None:
        parts.append("rpm_est %.0f" % mean(cre))
    for nm, unit in (("id", "A"), ("iq", "A"), ("i_s", "A"),
                     ("vd", "V"), ("vq", "V"), ("vbus", "V"), ("idc", "A")):
        c = col(header, nm)
        if c is not None:
            parts.append("%s %.2f %s" % (nm, mean(c), unit))
    if parts:
        print("  steady state (last 10%): " + ", ".join(parts))

    ci = col(header, "i_s")
    if ci is not None:
        peak = max(abs(r[ci]) for r in rows)
        print("  peak i_s %.1f A (%.1f A rms)" % (peak, peak / math.sqrt(2.0)))

    # voltage headroom
    cvs, cvm = col(header, "v_s"), col(header, "vmax")
    if cvs is not None and cvm is not None:
        on_circle = sum(1 for r in rows if r[cvm] > 0.0
                        and r[cvs] >= 0.98 * r[cvm])
        worst = max(rows, key=lambda r: r[cvs] / r[cvm] if r[cvm] > 0 else 0)
        print("  voltage: peak |v_s| %.2f V against vmax %.2f V at t = %+.2f s,"
              " %d rows within 2%% of the circle"
              % (worst[cvs], worst[cvm], worst[0], on_circle))

    # power and energy
    cpd, cpa = col(header, "p_dc"), col(header, "p_ac")
    if cpd is not None:
        pk = max(rows, key=lambda r: r[cpd])
        e_j = integrate(rows, cpd, ts)
        line = ("  DC power: steady %.0f W, peak %.0f W at t = %+.2f s, "
                "energy over record %.0f J (%.3f Wh)"
                % (mean(cpd), pk[cpd], pk[0], e_j, e_j / 3600.0))
        print(line)
        if cs is not None:
            cl = [r for r in rows if int(round(r[cs])) == ST_CLOSED]
            if cl:
                e_cl = integrate(cl, cpd, ts)
                print("  DC power in CLOSED: mean %.0f W over %.2f s, %.0f J"
                      % (e_cl / (len(cl) * ts), len(cl) * ts, e_cl))
    if cpa is not None:
        line = "  machine power (from commanded vdq): steady %.0f W" % mean(cpa)
        if cpd is not None and mean(cpd) > 20.0:
            line += ", p_ac/p_dc %.2f (estimate only)" % (mean(cpa) / mean(cpd))
        print(line)

    # temperature
    ct = col(header, "temp")
    if ct is not None:
        temps = [r[ct] for r in rows]
        if all(t == 0.0 for t in temps):
            print("  temp: zero throughout, NTC not seeded or faulted")
        else:
            print("  module temp: start %.1f degC, end %.1f degC, max %.1f degC"
                  % (temps[0], temps[-1], max(temps)))

    # independent flux check from vq = Rs*iq + w*Ls*id + w*lambda
    cid, ciq = col(header, "id"), col(header, "iq")
    if cvq is not None and ciq is not None and cre is not None:
        w_e = mean(cre) * 2.0 * math.pi * POLE_PAIRS / 60.0
        if w_e > 60.0:
            id_m = mean(cid) if cid is not None else 0.0
            lam = (mean(cvq) - RS_OHM * mean(ciq) - w_e * LS_H * id_m) / w_e
            print("  lambda from vq: %.5f Wb against FLUX_WB %.5f (%+.1f%%)"
                  % (lam, FLUX_WB, 100.0 * (lam / FLUX_WB - 1.0)))


# ------------------------------------------------------------------ output --
def unique_base(name, force):
    if force or not os.path.exists(name + ".csv"):
        return name
    i = 2
    while os.path.exists("%s_%02d.csv" % (name, i)):
        i += 1
    return "%s_%02d" % (name, i)


def write_csv(base, header, rows):
    path = base + ".csv"
    with open(path, "w") as fh:
        fh.write(",".join(header) + "\n")
        for r in rows:
            fh.write(",".join("%.6g" % v for v in r) + "\n")
    print("  wrote " + path)


# Panel order top to bottom. Any channel not listed gets its own panel.
GROUPS = [
    ("speed (rpm mech)", ("rpm_est", "rpm_ol")),
    ("dq current (A)", ("id", "iq", "iq_ref", "i_s")),
    ("dq voltage (V)", ("vd", "vq", "v_s", "vmax")),
    ("bus voltage (V)", ("vbus",)),
    ("DC current (A)", ("idc",)),
    ("power (W)", ("p_dc",)),
    ("module temp (degC)", ("temp",)),
    # older firmware channel sets, so old dumps still plot sensibly
    ("angle (rad)", ("theta", "theta_est", "theta_ol")),
    ("angle error (deg)", ("theta_err_deg",)),
]

# dashed so the limit reads differently from the signals
DASHED = {"vmax", "iq_ref", "rpm_ol"}

# Written to the CSV but never plotted. p_ac comes from the COMMANDED dq
# voltage, which overstates the applied voltage by the device drop, so on a
# plot beside p_dc it reads as an efficiency above 1.
NO_PLOT = {"p_ac"}


def plot(base, header, rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  matplotlib not installed, skipping plot")
        return
    if not rows:
        return

    chans = [c for c in header[1:] if c not in NO_PLOT]
    panels, used = [], set()
    for label, members in GROUPS:
        present = [m for m in members if m in chans]
        if present:
            panels.append((label, present))
            used.update(present)
    for c in chans:
        if c not in used and c != "state":
            panels.append((c, [c]))
    if "state" in chans:
        panels.append(("state", ["state"]))

    t = [r[0] for r in rows]
    ev = state_events(header, rows)

    fig, ax = plt.subplots(len(panels), 1, sharex=True,
                           figsize=(10, 1.7 * len(panels)))
    if len(panels) == 1:
        ax = [ax]

    for k, (label, members) in enumerate(panels):
        a = ax[k]
        for m in members:
            i = header.index(m)
            y = [r[i] for r in rows]
            if m == "state":
                a.step(t, y, where="post", lw=1.0)
                a.set_yticks(sorted(STATE_NAMES.keys()))
                a.set_yticklabels([STATE_NAMES[s]
                                   for s in sorted(STATE_NAMES.keys())],
                                  fontsize=7)
            else:
                a.plot(t, y, lw=0.9, label=m,
                       ls="--" if m in DASHED else "-")
        if label != "state" and len(members) > 1:
            a.legend(loc="upper right", fontsize=7, framealpha=0.6)
        a.set_ylabel(label, fontsize=8)
        a.tick_params(labelsize=7)
        a.grid(alpha=0.3)
        a.axvline(0.0, color="k", lw=0.6)
        for te, s in ev[1:]:
            a.axvline(te, color="0.5", lw=0.6, ls="--")
            if k == 0:
                a.annotate(STATE_NAMES.get(s, str(s)), xy=(te, 1.02),
                           xycoords=("data", "axes fraction"),
                           fontsize=7, ha="left", color="0.3")

    ax[0].set_title(os.path.basename(base), fontsize=9, loc="left", pad=12)
    ax[-1].set_xlabel("time from trigger (s)")
    fig.tight_layout()
    path = base + ".png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print("  wrote " + path)


# -------------------------------------------------------------------- main --
def handle(lines, name, force, do_plot):
    header, rows, meta = parse(lines)
    header, rows = derive(header, rows)
    base = unique_base(name, force)
    write_csv(base, header, rows)
    summarise(header, rows, meta)
    if do_plot:
        plot(base, header, rows)
    return base


def main():
    args = [a for a in sys.argv[1:]]
    force = "--force" in args
    watch = "--watch" in args
    do_plot = "--no-plot" not in args
    if "--list" in args:
        list_ports()
        return
    baud, timeout = BAUD, TIMEOUT_S
    skip = set()
    for i, a in enumerate(args):
        if a == "--baud" and i + 1 < len(args):
            baud = int(args[i + 1])
            skip.add(i + 1)
        if a == "--timeout" and i + 1 < len(args):
            timeout = float(args[i + 1])
            skip.add(i + 1)
    pos = [a for i, a in enumerate(args)
           if not a.startswith("--") and i not in skip]

    if not pos:
        print(__doc__.strip())
        print("\navailable ports:")
        list_ports()
        return
    port = pos[0]
    name = pos[1] if len(pos) > 1 else "trace"

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print("could not open %s: %s" % (port, e))
        return
    ser.reset_input_buffer()

    try:
        if not watch:
            print("waiting for a dump on %s ..." % port)
            lines = capture(ser, timeout)
            if lines is None:
                print("timed out before #end (partial dump discarded)")
                return
            handle(lines, name, force, do_plot)
            return

        print("watching %s, Ctrl-C to stop" % port)
        n = 0
        while True:
            try:
                lines = capture(ser, timeout)
            except serial.SerialException as e:
                print("  COM port dropped (%s), reopening..." % e)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = open_port(port, baud)
                print("  %s back, still watching" % port)
                continue
            if lines is None:
                continue
            n += 1
            handle(lines, name, force, do_plot)
            print("  (%d captured this session)\n" % n)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        ser.close()


if __name__ == "__main__":
    main()