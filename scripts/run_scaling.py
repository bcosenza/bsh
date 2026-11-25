#!/usr/bin/env python3
r"""
run_scaling.py  ->  results/scaling/<tag>/scaling.csv, scaling.md

Agent-count scaling of the long-range group-avoidance scenes on one machine:
scene 4 (two groups) and scene 5 (four groups), Dirac and vMF projections, N from
8192 up to 2M agents (2^21). For every (scene, projection, N) it runs

    boidSimSYCL -scene S -sh SH -n N -neighbours K -goal W -timing STEPS -seed 1 -window w h

and parses the timing report (mean / min / max per phase, in microseconds):
SH projection, grid build and interaction are GPU times from device profiling
events; upload is the host-side device -> GL copy; simulate() is the host time of
the whole step; frame is the wall time per frame including rendering.

Scaling is weak with a bounded neighbourhood: with -neighbours K the start spheres
are sized so every agent has ~K group mates within the cohesion radius, and the
world grows only as needed to fit them. The local flocking work per agent therefore
stays ~constant and the SH long-range term is O(N): ideal scaling is linear in N.
(Without -neighbours, i.e. --neighbours 0, the scene's own density is kept: its
start spheres are smaller than the cohesion radius, so local work per agent grows
with N, up to ~77k neighbours at 2M; measured on an integrated GPU, the interaction
time then grows ~12x per 4x agents instead of ~4x.)
Goal seeking (-goal) keeps the groups on course, as in the paper snapshots.

Runs that fail or time out (e.g. out of device memory) are recorded as failed and
the sweep continues. A run of the 2M case is a few seconds per step on an
integrated GPU and well under a second on a high-end one.

Needs: the built SYCL binary (bsh/build/boidSimSYCL, built on the target machine),
the oneAPI runtime (sourced from /opt/intel/oneapi/setvars.sh if ONEAPI_ROOT is not
set) and an X display, since the app opens a GL window. On a headless node it uses
xvfb-run when DISPLAY is unset and xvfb-run is installed (--xvfb auto); note that
Xvfb renders in software, which slows the 'frame' column but not the GPU phases.

Run:  python3 run_scaling.py                                  # full sweep, tag = host name
      python3 run_scaling.py --scenes 4 --sh vmf --n-max 524288 --steps 50
      python3 run_scaling.py --skip-run --tag mymachine          # re-tabulate a sweep
"""
import argparse
import csv
import os
import re
import shutil
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SETVARS = "/opt/intel/oneapi/setvars.sh"
PHASES = [("SH projection", "project"), ("grid build", "grid"), ("interaction", "interact"),
          ("upload (host)", "upload"), ("simulate()", "step"), ("frame (wall)", "frame")]


def parse_args():
    ap = argparse.ArgumentParser(description="Agent-count scaling of scenes 4 and 5 (Dirac / vMF).")
    ap.add_argument("--scenes", type=int, nargs="+", default=[4, 5], help="scenes to measure")
    ap.add_argument("--sh", nargs="+", default=["dirac", "vmf"], choices=["dirac", "vmf"],
                    help="SH projections to measure")
    ap.add_argument("--n-min", type=int, default=8192, help="smallest agent count")
    ap.add_argument("--n-max", type=int, default=2097152, help="largest agent count (2M = 2097152)")
    ap.add_argument("--ns", type=int, nargs="+", help="explicit agent counts (overrides n-min/n-max)")
    ap.add_argument("--steps", type=int, default=100, help="timed steps per run (after 10 warm-up)")
    ap.add_argument("--neighbours", type=float, default=512,
                    help="target group mates within the cohesion radius (0 = the scene's density)")
    ap.add_argument("--goal", type=float, default=2.0, help="goal-seeking weight (0 = scenes as defined)")
    ap.add_argument("--kappa", type=float, default=2.95, help="vMF concentration k")
    ap.add_argument("--seed", type=int, default=1, help="placement seed")
    ap.add_argument("--window", type=int, nargs=2, default=[640, 480], metavar=("W", "H"),
                    help="window size (rendering cost enters the 'frame' column only)")
    ap.add_argument("--timeout", type=int, default=3600, help="seconds before a run is abandoned")
    ap.add_argument("--xvfb", choices=["auto", "on", "off"], default="auto",
                    help="wrap runs in xvfb-run (auto: when DISPLAY is unset)")
    ap.add_argument("--tag", default=socket.gethostname(), help="name of this machine / sweep")
    ap.add_argument("--build-dir", default=os.path.join(HERE, "..", "bsh", "build"),
                    help="directory holding boidSimSYCL (it must run from there)")
    ap.add_argument("--out-dir", default=os.path.join(HERE, "results", "scaling"),
                    help="results go to OUT_DIR/TAG")
    ap.add_argument("--skip-run", action="store_true", help="only tabulate the saved run logs")
    return ap.parse_args()


def agent_counts(a):
    if a.ns:
        return sorted(set(a.ns))
    ns, n = [], a.n_min
    while n <= a.n_max:
        ns.append(n)
        n *= 2
    return ns


def log_path(out, scene, sh, n):
    return os.path.join(out, "logs", f"scene{scene}_{sh}_n{n}.log")


def run_one(a, binary, build_dir, scene, sh, n, log):
    cmd = [binary, "-scene", str(scene), "-sh", sh, "-kappa", f"{a.kappa:g}", "-n", str(n),
           "-timing", str(a.steps), "-seed", str(a.seed),
           "-window", str(a.window[0]), str(a.window[1])]
    if a.neighbours > 0:
        cmd += ["-neighbours", f"{a.neighbours:g}"]
    if a.goal > 0:
        cmd += ["-goal", f"{a.goal:g}"]
    use_xvfb = a.xvfb == "on" or (a.xvfb == "auto" and not os.environ.get("DISPLAY"))
    if use_xvfb:
        if not shutil.which("xvfb-run"):
            sys.exit("no DISPLAY and xvfb-run not found: install xvfb or run with a display")
        cmd = ["xvfb-run", "-a", "-s", f"-screen 0 {a.window[0]}x{a.window[1]}x24"] + cmd
    if "ONEAPI_ROOT" not in os.environ and os.path.exists(SETVARS):
        cmd = ["bash", "-c", f'source "{SETVARS}" >/dev/null 2>&1; exec "$@"', "bash"] + cmd
    print(f"  scene {scene} {sh:5s} N={n:>8d} ... ", end="", flush=True)
    t0 = time.time()
    try:
        res = subprocess.run(cmd, cwd=build_dir, capture_output=True, text=True, timeout=a.timeout)
        text, code = res.stdout + res.stderr, res.returncode
    except subprocess.TimeoutExpired as e:
        text, code = (e.stdout or b"").decode(errors="replace") + "\nTIMEOUT\n", -1
    with open(log, "w") as f:
        f.write(f"# {' '.join(cmd)}\n# exit {code}, {time.time() - t0:.1f} s\n{text}")
    print(f"{'ok' if parse_log(log) else 'FAILED'} ({time.time() - t0:.0f} s)", flush=True)


def parse_log(log):
    """Timing report of one run -> dict of phase stats, or None if the run failed."""
    if not os.path.exists(log):
        return None
    text = open(log).read()
    m = re.search(r"timing: .*?, (\d+) boids, (.*?), (\d+) steps", text)
    if not m:
        return None
    row = {"n": int(m.group(1)), "projection": m.group(2), "steps": int(m.group(3))}
    d = re.search(r"device: (.*)", text)
    row["device"] = d.group(1).strip() if d else ""
    for label, key in PHASES:
        p = re.search(re.escape(label) + r"\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)", text)
        if not p:
            return None
        row[f"{key}_mean_us"], row[f"{key}_min_us"], row[f"{key}_max_us"] = map(float, p.groups())
    return row


def main():
    a = parse_args()
    build_dir = os.path.abspath(a.build_dir)
    out = os.path.join(os.path.abspath(a.out_dir), a.tag)
    os.makedirs(os.path.join(out, "logs"), exist_ok=True)
    binary = os.path.join(build_dir, "boidSimSYCL")
    ns = agent_counts(a)

    if not a.skip_run:
        if not os.access(binary, os.X_OK):
            sys.exit(f"{binary} not found; build it first (cmake --build build --target boidSimSYCL)")
        print(f"scaling sweep '{a.tag}': scenes {a.scenes}, {a.sh}, N = {ns[0]} .. {ns[-1]}, "
              f"{a.steps} steps, neighbours {a.neighbours:g}, goal {a.goal:g}")
        # small N first, so a failure at large N does not cost the rest of the sweep
        for n in ns:
            for scene in a.scenes:
                for sh in a.sh:
                    run_one(a, binary, build_dir, scene, sh, n, log_path(out, scene, sh, n))

    rows, device = [], ""
    for scene in a.scenes:
        for sh in a.sh:
            for n in ns:
                r = parse_log(log_path(out, scene, sh, n))
                base = {"scene": scene, "sh": sh, "n_requested": n, "status": "ok" if r else "failed"}
                rows.append({**base, **(r or {})})
                if r and r["device"]:
                    device = r["device"]

    fields = ["scene", "sh", "n_requested", "status", "n", "projection", "steps", "device"]
    fields += [f"{k}_{s}_us" for _, k in PHASES for s in ("mean", "min", "max")]
    with open(os.path.join(out, "scaling.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    md = [f"# Agent-count scaling: {a.tag}\n",
          f"Device: {device or 'unknown'}. Scenes 4 (two groups) and 5 (four groups), "
          f"{a.steps} timed steps after 10 warm-up, neighbours {a.neighbours:g}, goal {a.goal:g}, "
          f"k = {a.kappa:g}. Times are means per step in ms; 'ns/agent' is the GPU time of "
          "projection + grid + interaction per agent.\n"]
    for scene in a.scenes:
        for sh in a.sh:
            md += [f"## Scene {scene}, {sh}\n",
                   "| N | projection | grid | interaction | upload | simulate() | frame | GPU ns/agent |",
                   "|---|---|---|---|---|---|---|---|"]
            for r in rows:
                if r["scene"] != scene or r["sh"] != sh:
                    continue
                if r["status"] != "ok":
                    md.append(f"| {r['n_requested']} | failed | | | | | | |")
                    continue
                gpu = r["project_mean_us"] + r["grid_mean_us"] + r["interact_mean_us"]
                md.append(f"| {r['n']} | {r['project_mean_us'] / 1e3:.3f} | {r['grid_mean_us'] / 1e3:.2f} "
                          f"| {r['interact_mean_us'] / 1e3:.2f} | {r['upload_mean_us'] / 1e3:.2f} "
                          f"| {r['step_mean_us'] / 1e3:.2f} | {r['frame_mean_us'] / 1e3:.2f} "
                          f"| {1e3 * gpu / r['n']:.1f} |")
            md.append("")
    text = "\n".join(md) + "\n"
    with open(os.path.join(out, "scaling.md"), "w") as f:
        f.write(text)
    print()
    print(text)
    print(f"results: {out}/scaling.csv, scaling.md, logs/")


if __name__ == "__main__":
    main()
