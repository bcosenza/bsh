#!/usr/bin/env python3
r"""
eval_scene8_accuracy.py  ->  results/scene8_accuracy/scene8_accuracy.md (+ summary CSV)

Accuracy evaluation of behaviour composition with Gaunt products on SYCL scene 8
(four groups composition): Dirac vs von Mises-Fisher (vMF) projection, scored against
the brute-force reference.

In scene 8 the red group steers toward the maximum (over 1024 directions) of the
composite preference field

    P(w) = max(0, w . goal) * F_orange(w) * (1 - F_blue(w)),

with F_g the heading field of group g normalised to [0, 1]. In SH form P is built
with two Gaunt products (order 3 x order 3, truncated to order 3); the reference
evaluates the same product pointwise on the exact (untruncated) vMF factors of the
same k.

Accuracy: for every k (and seed) one run driven by the reference,

    boidSimSYCL 8 -sh exact -kappa k -accuracy STEPS -seed SEED -out raw.csv

in which both SH variants are evaluated on the same states. Metrics, pooled over
all red agents and steps:
  dir. err   mean angle between the direction chosen from the SH field and the
             one chosen from the reference (deg; directions ~6 deg apart)
  field err  RMS (P_sh - P_ref) / RMS P_ref over the 1024 directions
  neg. lobes share of directions where P_sh < -0.05 (P_ref >= 0 everywhere)
  regret     reference preference lost by the SH choice, P_ref(w*_ref) - P_ref(w*_sh),
             absolute and relative to P_ref(w*_ref)

Behaviour: at one k (--behaviour-k) the scene is also driven by each SH variant
(-sh dirac / -sh vmf); reported per driver are the share of steps red heads toward
its goal and red's mean / minimum angle to orange's mean heading.

Needs: the built SYCL binary (bsh/build/boidSimSYCL), a display (the app opens a GL
window) and the oneAPI runtime (sourced from /opt/intel/oneapi/setvars.sh if
ONEAPI_ROOT is not set). One run of 2400 steps takes ~85 s.

Run:  python3 eval_scene8_accuracy.py                       # k = 0.5 1.5 2.95, seed 1
      python3 eval_scene8_accuracy.py --ks 0.5 1 1.5 2.95 6 --seeds 1 2 3
      python3 eval_scene8_accuracy.py --skip-run            # re-tabulate saved runs
"""
import argparse
import csv
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SETVARS = "/opt/intel/oneapi/setvars.sh"


def parse_args():
    ap = argparse.ArgumentParser(description="Scene 8 (composition) accuracy: Dirac vs vMF vs reference.")
    ap.add_argument("--ks", type=float, nargs="+", default=[0.5, 1.5, 2.95],
                    help="vMF concentrations k of the factor fields (the reference uses the same k)")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1],
                    help="placement seeds; results are pooled over them")
    ap.add_argument("--steps", type=int, default=2400, help="fixed-dt steps per run")
    ap.add_argument("--behaviour-k", type=float, default=1.5,
                    help="k of the behaviour comparison (runs driven by Dirac and vMF too)")
    ap.add_argument("--build-dir", default=os.path.join(HERE, "..", "bsh", "build"),
                    help="directory holding boidSimSYCL (it must run from there)")
    ap.add_argument("--out-dir", default=os.path.join(HERE, "results", "scene8_accuracy"),
                    help="where the raw CSVs and the tables are written")
    ap.add_argument("--skip-run", action="store_true",
                    help="do not run the simulator, tabulate the raw CSVs already saved")
    return ap.parse_args()


def raw_path(out_dir, driver, k, seed):
    return os.path.join(out_dir, "raw", f"scene8_{driver}_k{k:g}_seed{seed}.csv")


def run_one(binary, build_dir, driver, k, seed, steps, out_csv):
    if os.path.exists(out_csv):
        os.remove(out_csv)   # the simulator appends
    cmd = [binary, "8", "-sh", driver, "-kappa", f"{k:g}", "-accuracy", str(steps),
           "-seed", str(seed), "-out", out_csv]
    if "ONEAPI_ROOT" not in os.environ and os.path.exists(SETVARS):
        cmd = ["bash", "-c", f'source "{SETVARS}" >/dev/null 2>&1; exec "$@"', "bash"] + cmd
    print(f"  {driver:5s} k={k:g} seed={seed}: running {steps} steps ...", flush=True)
    res = subprocess.run(cmd, cwd=build_dir, capture_output=True, text=True)
    if res.returncode != 0 or not os.path.exists(out_csv):
        sys.exit(f"run failed ({driver}, k={k:g}, seed={seed}, exit {res.returncode}):\n{res.stdout}\n{res.stderr}")


def read_row(path):
    if not os.path.exists(path):
        sys.exit(f"{path} missing: run without --skip-run for this driver / k / seed")
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 1 or "D_field_se" not in rows[0]:
        sys.exit(f"{path}: expected one scene 8 row with field columns (rebuild boidSimSYCL?)")
    return rows[0]


class Pool:
    """Sums over runs (seeds) of one driver / k."""

    def __init__(self):
        self.n = self.ss_ref = self.pref_ref = self.events = self.sum_dist = 0.0
        self.min_dist = float("inf")
        self.v = {x: dict(ang=0.0, ang_n=0.0, se=0.0, neg=0.0, regret=0.0) for x in ("D", "V")}

    def add(self, r):
        n = float(r["agent_steps"])
        self.n += n
        self.ss_ref += float(r["field_ss_ref"])
        self.pref_ref += float(r["aux_ref_mean"]) * n
        self.events += float(r["events"])
        self.sum_dist += float(r["dist_mean"]) * n
        self.min_dist = min(self.min_dist, float(r["min_dist"]))
        for x in ("D", "V"):
            s = self.v[x]
            s["ang"] += float(r[f"{x}_angsum"])
            s["ang_n"] += float(r[f"{x}_angN"])
            s["se"] += float(r[f"{x}_field_se"])
            s["neg"] += float(r[f"{x}_field_neg"]) * n
            s["regret"] += float(r[f"{x}_aux_mae"]) * n

    def accuracy(self, x):
        s = self.v[x]
        pref = self.pref_ref / self.n
        regret = s["regret"] / self.n
        return dict(dir_err=s["ang"] / s["ang_n"] if s["ang_n"] else 0.0,
                    field_err=math.sqrt(s["se"] / self.ss_ref) if self.ss_ref else 0.0,
                    neg_lobes=s["neg"] / self.n, regret=regret,
                    regret_rel=regret / pref if pref else 0.0, pref_ref=pref)

    def behaviour(self):
        return dict(on_course=self.events / self.n, mean_angle_orange=self.sum_dist / self.n,
                    min_angle_orange=self.min_dist)


def main():
    a = parse_args()
    build_dir = os.path.abspath(a.build_dir)
    out_dir = os.path.abspath(a.out_dir)
    binary = os.path.join(build_dir, "boidSimSYCL")
    os.makedirs(os.path.join(out_dir, "raw"), exist_ok=True)

    # reference-driven runs for every k, plus SH-driven runs at the behaviour k
    jobs = [("exact", k) for k in a.ks]
    if a.behaviour_k not in a.ks:
        jobs.append(("exact", a.behaviour_k))
    jobs += [("dirac", a.behaviour_k), ("vmf", a.behaviour_k)]

    if not a.skip_run:
        if not os.access(binary, os.X_OK):
            sys.exit(f"{binary} not found; build it first (cmake --build build --target boidSimSYCL)")
        print(f"scene 8 accuracy: k = {' '.join(f'{k:g}' for k in a.ks)}, behaviour at k = "
              f"{a.behaviour_k:g}, seeds = {' '.join(map(str, a.seeds))}, {a.steps} steps per run")
        for driver, k in jobs:
            for seed in a.seeds:
                run_one(binary, build_dir, driver, k, seed, a.steps, raw_path(out_dir, driver, k, seed))

    pools = {}
    for driver, k in jobs:
        p = Pool()
        for seed in a.seeds:
            p.add(read_row(raw_path(out_dir, driver, k, seed)))
        pools[(driver, k)] = p

    seeds = " ".join(map(str, a.seeds))
    out = ["# Scene 8 (four groups composition): Gaunt-product accuracy, Dirac vs vMF\n",
           f"Red steers toward the maximum of P = max(0, w.goal) * F_orange * (1 - F_blue). "
           f"Reference-driven runs (`-sh exact`), {a.steps} steps, seeds {seeds}; the reference "
           "is the exact pointwise product of the exact vMF factors of the same k.\n",
           "## Accuracy (reference-driven)\n",
           "| k | variant | dir. err (deg) | field err | neg. lobes | regret | regret (rel.) |",
           "|---|---|---|---|---|---|---|"]
    summary = []
    for k in a.ks:
        p = pools[("exact", k)]
        for x, name in (("D", "Dirac"), ("V", "vMF")):
            m = p.accuracy(x)
            out.append(f"| {k:g} | {name} | {m['dir_err']:.1f} | {m['field_err']:.3f} "
                       f"| {100 * m['neg_lobes']:.2f}% | {m['regret']:.3f} | {100 * m['regret_rel']:.1f}% |")
            summary.append(dict(table="accuracy", k=k, variant="dirac" if x == "D" else "vmf", **m))
    out.append("")

    out.append(f"## Behaviour at k = {a.behaviour_k:g} (scene driven by each variant)\n")
    out.append("| driver | on course | mean angle to orange (deg) | min angle to orange (deg) |")
    out.append("|---|---|---|---|")
    for driver, name in (("exact", "reference"), ("dirac", "Dirac"), ("vmf", "vMF")):
        b = pools[(driver, a.behaviour_k)].behaviour()
        out.append(f"| {name} | {100 * b['on_course']:.1f}% | {b['mean_angle_orange']:.1f} "
                   f"| {b['min_angle_orange']:.1f} |")
        summary.append(dict(table="behaviour", k=a.behaviour_k, variant=driver, **b))
    out.append("")
    out.append("dir. err: angle between the directions chosen from the SH and the reference "
               "fields. field err: RMS(P_sh - P_ref) / RMS(P_ref) over 1024 directions. neg. lobes: "
               "share of directions with P_sh < -0.05. regret: P_ref(w*_ref) - P_ref(w*_sh). "
               "on course: share of steps red heads toward its goal (w.goal > 0).")

    text = "\n".join(out) + "\n"
    print()
    print(text)
    md = os.path.join(out_dir, "scene8_accuracy.md")
    with open(md, "w") as f:
        f.write(text)
    sc = os.path.join(out_dir, "scene8_accuracy_summary.csv")
    keys = sorted({k for row in summary for k in row}, key=lambda c: (c not in ("table", "k", "variant"), c))
    with open(sc, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(summary)
    print(f"tables: {md}\nsummary CSV: {sc}\nraw runs: {os.path.join(out_dir, 'raw')}")


if __name__ == "__main__":
    main()
