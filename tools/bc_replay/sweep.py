#!/usr/bin/env python3
"""Grid search of PI parameters over all traces (see replay.c).
Prints the Pareto-relevant candidates: bitrate on a good link vs delay/drops."""
import glob, itertools, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

TRACES = sorted(glob.glob("traces/*.csv"))
PAT = re.compile(r"frames\s+(\d+)\s+dropped\s+(\d+).*?>50ms\s+([\d.]+)% >100ms\s+([\d.]+)%\s+mean set\s+(\d+) kbps \(good link\s+(\d+)\)")

def run(mode, params):
    tot = dict(fr=0, dr=0, l50=0, l100=0, mean=0, good=0)
    for t in TRACES:
        out = subprocess.run(["./replay", t, mode] + [f"{k}={v}" for k, v in params], capture_output=True, text=True).stdout
        m = PAT.search(out)
        fr = int(m.group(1))
        tot["fr"] += fr; tot["dr"] += int(m.group(2))
        tot["l50"] += float(m.group(3)) * fr; tot["l100"] += float(m.group(4)) * fr
        tot["mean"] += int(m.group(5)) * fr; tot["good"] += int(m.group(6)) * fr
    f = tot["fr"]
    return dict(drop=100 * tot["dr"] / f, l50=tot["l50"] / f, l100=tot["l100"] / f, mean=tot["mean"] / f, good=tot["good"] / f)

def fmt(r):
    return f"drop {r['drop']:.2f}%  >50ms {r['l50']:.2f}%  >100ms {r['l100']:.2f}%  mean {r['mean']:.0f}  good {r['good']:.0f}"

if __name__ == "__main__":
    base = run("rules", [])
    print("rules       ", fmt(base))
    grid = dict(
        pi_delay_set_ms=[25, 30, 35, 40],
        pi_ki_up=[0.01, 0.03, 0.06],
        pi_kp_up=[0.004, 0.01],
        pi_k_max=[1.1, 1.3],
        pi_slew_up=[1.5, 3.0],
    )
    keys = list(grid)
    combos = [list(zip(keys, v)) for v in itertools.product(*grid.values())]
    with ThreadPoolExecutor(max_workers=12) as ex:
        results = list(ex.map(lambda p: (p, run("pi", p)), combos))
    # candidates: at least as much video on a good link as rules, ranked by delay
    ok = [(p, r) for p, r in results if r["good"] >= base["good"] * 0.97 and r["drop"] <= base["drop"]]
    ok.sort(key=lambda x: (x[1]["l50"] + 2 * x[1]["l100"] + 3 * x[1]["drop"]))
    print(f"{len(ok)} of {len(results)} PI settings keep >=97% of rules' good-link bitrate with fewer drops; best:")
    for p, r in ok[:12]:
        print("  ", fmt(r), " | ", " ".join(f"{k[3:]}={v}" for k, v in p))
