#!/usr/bin/env python3
"""Analyse the CSV produced by `diag_ctrl_latency.py` (run on the robot).

Answers one question, with two independent estimators:

    Is the controller's path-carrot lagging because the controller is
    processing an old map pose, or because the marker is published/recorded
    late?

Estimators
----------
1. `stamp_gap`  = (newest `/loc/robo/odom` header stamp already received when a
   `/vis/lookahead_goal` message arrives) − (that marker's header stamp).
   The marker carries `header.stamp = controller's robot_odom_time`, so this is
   "how stale was the pose the controller had processed, relative to the
   freshest pose available at that instant".
       ~0 s      -> controller current  -> delay is downstream (publish/record)
       ~1 s      -> controller lagging  -> progress/carrot built on an old pose

2. `marker_age − odom_age`, where each age = host arrival − header stamp,
   measured in the same process. Same interpretation, but includes this
   process's own scheduling delay in `odom_age` (so it is the more conservative
   estimator).

Also reported: message rates and inter-arrival jitter, which show whether the
control loop is keeping its configured 5 Hz.

Usage
-----
    python3 scripts/analyze_ctrl_latency.py ~/latency_123456.csv [--json out.json]
"""

import argparse
import csv
import json
import sys

import numpy as np

LOC = "/loc/robo/odom"
LIO = "/lio/robo/odom"
MARKER = "/vis/lookahead_goal"
CMD = "/cmd_vel"


def load(path):
    cols = {k: [] for k in ("host_time", "topic", "stamp", "age_ms", "x", "y", "lin_x", "ang_z")}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                ht = float(row["host_time"])
                st = float(row["stamp"]) if row["stamp"] else float("nan")
            except (TypeError, ValueError):
                continue
            cols["host_time"].append(ht)
            cols["topic"].append(row["topic"])
            cols["stamp"].append(st)
            try:
                cols["age_ms"].append(float(row["age_ms"]) if row["age_ms"] else float("nan"))
            except ValueError:
                cols["age_ms"].append(float("nan"))
            for k in ("x", "y", "lin_x", "ang_z"):
                try:
                    cols[k].append(float(row[k]))
                except (TypeError, ValueError):
                    cols[k].append(float("nan"))
    n = len(cols["host_time"])
    return {
        "n": n,
        "host": np.array(cols["host_time"]),
        "topic": np.array(cols["topic"], dtype=object),
        "stamp": np.array(cols["stamp"]),
        "age": np.array(cols["age_ms"]) / 1000.0,
        "x": np.array(cols["x"]),
        "y": np.array(cols["y"]),
        "lin_x": np.array(cols["lin_x"]),
        "ang_z": np.array(cols["ang_z"]),
    }


def pct(a, q):
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if len(a) else float("nan")


def topic_series(d, topic):
    m = d["topic"] == topic
    return d["host"][m], d["stamp"][m], d["age"][m]


def rate_and_jitter(host):
    if len(host) < 3:
        return float("nan"), float("nan")
    dt = np.diff(host)
    dt = dt[(dt > 1e-6) & (dt < 5.0)]
    if len(dt) == 0:
        return float("nan"), float("nan")
    span = host[-1] - host[0]
    rate = len(host) / span if span > 0 else float("nan")
    return rate, float(np.std(dt) * 1000.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    d = load(args.csv)
    if d["n"] < 20:
        print(f"only {d['n']} rows — capture looks empty or truncated", file=sys.stderr)
        return 2

    out = {"file": args.csv, "rows": d["n"], "topics": {}}

    print(f"\n== ages (host arrival − header stamp) ==   file: {args.csv}")
    print(f"{'topic':22s} {'n':>7s} {'rate Hz':>8s} {'dt std ms':>10s} "
          f"{'age p50 ms':>11s} {'p95':>8s} {'max':>8s}")
    for t in (LOC, LIO, MARKER, CMD):
        host, stamp, age = topic_series(d, t)
        if len(host) == 0:
            print(f"{t:22s} {'—':>7s}   (no messages)")
            continue
        rate, jitter = rate_and_jitter(host)
        # /cmd_vel is logged with stamp=arrival, so its age is 0 by construction
        a = age if t != CMD else np.array([np.nan])
        print(f"{t:22s} {len(host):7d} {rate:8.2f} {jitter:10.1f} "
              f"{pct(a,50):11.1f} {pct(a,95):8.1f} {pct(a,100):8.1f}")
        out["topics"][t] = dict(
            n=int(len(host)), rate_hz=rate, dt_std_ms=jitter,
            age_p50_ms=pct(a, 50), age_p95_ms=pct(a, 95), age_max_ms=pct(a, 100))

    # ---- estimator 1: stamp gap against the freshest odom already received ----
    odom_host, odom_stamp, odom_age = topic_series(d, LOC)
    mk_host, mk_stamp, mk_age = topic_series(d, MARKER)
    if len(odom_host) < 10 or len(mk_host) < 10:
        print("\n!! need both /loc/robo/odom and /vis/lookahead_goal in the CSV", file=sys.stderr)
        return 2

    order = np.argsort(odom_host)
    odom_host = odom_host[order]
    odom_stamp = odom_stamp[order]
    gaps = []
    for h, s in zip(mk_host, mk_stamp):
        # newest odom message that had *arrived* by the time this marker arrived
        idx = np.searchsorted(odom_host, h, side="right") - 1
        if idx < 0 or not np.isfinite(odom_stamp[idx]) or not np.isfinite(s):
            continue
        gaps.append(odom_stamp[idx] - s)
    gaps = np.array(gaps)
    # drop absurd values (clock jumps, replayed bags)
    gaps = gaps[(gaps > -2.0) & (gaps < 10.0)]

    # ---- estimator 2: marker_age - contemporaneous odom_age ----
    diffs = []
    for h, a in zip(mk_host, mk_age):
        if not np.isfinite(a):
            continue
        near = np.abs(odom_host - h) < 1.0
        if near.sum() < 3:
            continue
        oa = np.nanmedian(odom_age[near])
        if np.isfinite(oa):
            diffs.append(a - oa)
    diffs = np.array(diffs)

    print("\n== controller pose staleness ==")
    print(f"  estimator 1  stamp_gap  (freshest odom stamp − marker stamp)")
    print(f"      n={len(gaps)}  p5={pct(gaps,5)*1000:+.0f} ms  p50={pct(gaps,50)*1000:+.0f} ms  "
          f"p95={pct(gaps,95)*1000:+.0f} ms")
    print(f"  estimator 2  marker_age − odom_age")
    print(f"      n={len(diffs)}  p5={pct(diffs,5)*1000:+.0f} ms  p50={pct(diffs,50)*1000:+.0f} ms  "
          f"p95={pct(diffs,95)*1000:+.0f} ms")

    g50 = pct(gaps, 50)
    g95 = pct(gaps, 95)
    if np.isfinite(g50):
        if g50 < 0.25:
            verdict = ("CONTROLLER CURRENT — the marker carries a fresh pose stamp. "
                       "The 0.35-0.5 m deficit measured in Stage 0 is therefore downstream "
                       "(marker publish/logging path, or the deployed tracker/params), "
                       "not a stale map pose in the control loop.")
        elif g50 > 0.70:
            verdict = ("CONTROLLER LAGGING — the controller builds the carrot from a pose "
                       "~%.1f s old. Fix the pose freshness (executor/queue/lookup) before "
                       "tuning any lookahead parameter." % g50)
        else:
            verdict = ("PARTIAL — ~%.2f s of pose staleness; inconsistent with a purely "
                       "downstream cause. Re-run with RViz closed and nothing else on the "
                       "CPU before concluding." % g50)
    else:
        verdict = "insufficient data"

    print("\n== control loop ==")
    cmd_rate = out["topics"].get(CMD, {}).get("rate_hz", float("nan"))
    print(f"  /cmd_vel rate = {cmd_rate:.2f} Hz (configured 5.0); "
          f"dt std = {out['topics'].get(CMD, {}).get('dt_std_ms', float('nan')):.1f} ms")
    print(f"\n== verdict ==\n  {verdict}\n")

    out.update(
        stamp_gap_p50_ms=pct(gaps, 50) * 1000.0,
        stamp_gap_p95_ms=pct(gaps, 95) * 1000.0,
        age_diff_p50_ms=pct(diffs, 50) * 1000.0,
        age_diff_p95_ms=pct(diffs, 95) * 1000.0,
        verdict=verdict,
    )
    if args.json:
        json.dump(out, open(args.json, "w"), indent=2)
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
