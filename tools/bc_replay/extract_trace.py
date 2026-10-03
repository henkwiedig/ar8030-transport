#!/usr/bin/env python3
"""Turn an ar8030-flightlog session into a replay trace for bc_replay.

  extract_trace.py <session-dir> <out-prefix>

Reads <session-dir>/link.jsonl (ground-side link status, ~1.6 Hz) and
<session-dir>/air_syslog.log (the air's forwarded log) and writes:

  <out-prefix>.csv   t_s,connected,cap_kbps,snr_db,ldpc   -- the link as the
                     air's bitrate controller would have seen it:
                     cap_kbps = BB_GET_MCS throughput for the air's TX MCS
                     (mapping measured from flight logs, see MCS_KBPS),
                     snr_db/ldpc = the ground's receive SNR and LDPC error
                     figure (x10000), used by the plant as a distress proxy.
  <out-prefix>.rec.json  what the controller that actually flew did:
                     its bitrate decisions and the 1/5 forwarded tx stats
                     lines (frame age, ring fill), for calibration.
"""
import datetime
import json
import re
import sys

# BB_GET_MCS throughput (kbps) per air TX MCS -- from correlating the
# controller's "link=N kbps" lines with link.jsonl's peer tx_mcs in the
# 2026-10-02 and 2026-10-03 flights (identical both days).
MCS_KBPS = {-1: 1143, 0: 2286, 1: 2286, 2: 4572, 3: 6858, 4: 6858, 5: 9144, 6: 11486,
            7: 13828, 8: 18288, 9: 23000, 10: 27656, 11: 32000, 12: 36688}


def secs(ts_ms):
    t = datetime.datetime.fromtimestamp(ts_ms / 1000, datetime.UTC)
    return t.hour * 3600 + t.minute * 60 + t.second + t.microsecond / 1e6


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    sdir, out = sys.argv[1], sys.argv[2]
    rows = []
    for line in open(f"{sdir}/link.jsonl"):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        g = r.get("gnd") or {}
        q = g.get("quality") or {}
        conn = g.get("state") == "connected" and bool(q)
        mcs = q.get("peer", {}).get("tx_mcs") if conn else None
        snr = q.get("self", {}).get("snr_db") if conn else None
        ldpc = q.get("self", {}).get("ldpc_err") if conn else None
        rows.append((secs(r["t"]), conn, MCS_KBPS.get(mcs, 0) if conn else 0,
                     snr if snr is not None else 0.0, ldpc or 0))
    if not rows:
        sys.exit("no samples")
    t0 = rows[0][0]
    with open(f"{out}.csv", "w") as fh:
        fh.write("t_s,connected,cap_kbps,snr_db,ldpc\n")
        for t, c, cap, snr, ldpc in rows:
            fh.write(f"{t - t0:.3f},{int(c)},{cap},{snr:.1f},{ldpc}\n")

    rec = {"t0": t0, "decisions": [], "stats": []}
    for line in open(f"{sdir}/air_syslog.log", errors="replace"):
        if "ar8030-tx" not in line:
            continue
        parts = line.split()
        if len(parts) < 2 or parts[1].count(":") != 2:
            continue
        h, m, s = map(int, parts[1].split(":"))
        t = h * 3600 + m * 60 + s - t0
        mb = re.search(r"video0\.bitrate=(\d+)", line)
        if mb and "status=" not in line:
            rec["decisions"].append([round(t, 1), int(mb.group(1))])
        ms = re.search(r"frames ([\d.]+)/s in.*?\| ([\d.]+) Mbit/s \| ring (\d+)% full", line)
        if ms:
            rec["stats"].append([round(t, 1), "frames", float(ms.group(1)), float(ms.group(2)), int(ms.group(3))])
        ma = re.search(r"pts age ([\d.]+) ms \| capture->write start ([\d.]+) ms avg", line)
        if ma:
            rec["stats"].append([round(t, 1), "age", float(ma.group(1)), float(ma.group(2))])
    with open(f"{out}.rec.json", "w") as fh:
        json.dump(rec, fh)
    conn = sum(r[1] for r in rows)
    print(f"{out}: {len(rows)} samples, {rows[-1][0] - t0:.0f} s, connected {conn}, "
          f"{len(rec['decisions'])} recorded decisions")


if __name__ == "__main__":
    main()
