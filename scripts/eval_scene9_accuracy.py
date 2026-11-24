#!/usr/bin/env python3
r"""
eval_scene9_accuracy.py  ->  results/scene9_accuracy/scene9_accuracy.md (+ summary CSV)

Accuracy evaluation of the obstacle SH term on SYCL scene 9 (plate lanes): Dirac vs
von Mises-Fisher (vMF) projection, scored against the brute-force reference.

For every vMF concentration k (and every seed) it runs

    boidSimSYCL 9 -sh exact -kappa k -accuracy STEPS -seed SEED -out raw.csv

i.e. the scene is driven by the exact reference (the untruncated vMF field with
k_ref = k), and at every step both SH variants are evaluated on the same state:
  - Dirac SH, scored against the reference of concentration k;
  - vMF SH of the same k, which differs from the reference only by the order-3
    truncation.
So each k gives one Dirac row and one vMF row; the Dirac SH itself does not depend on
k, but its reference does.

Metrics (pooled over the 7 lanes and all seeds from the raw sums in the CSV):
  nRMSE       RMS |s - s_ref| / RMS |s_ref|            (s = obstacle steering push)
  fit nRMSE   the same after the best scalar gain g    (shape error a retuned
                                                        strength could not remove)
  gain        g = <s, s_ref> / <s, s>                  (<1: SH pushes too hard)
  dir. err    mean angle between s and s_ref (deg), where both are non-zero
  max err     largest |s - s_ref| of any agent-step
  missed      share of agent-steps where only the reference pushes
  spurious    share of agent-steps where only the SH pushes
  toward      share of agent-steps whose push points toward the plate
The per-lane table gives nRMSE and direction error against the incidence angle; the
behaviour of the reference-driven run (penetrations, minimum clearance) is listed per k.

Needs: the built SYCL binary (bsh/build/boidSimSYCL), a display (the app opens a GL
window) and the oneAPI runtime (sourced from /opt/intel/oneapi/setvars.sh if
ONEAPI_ROOT is not set). One run of 2400 steps takes ~45 s.

Run:  python3 eval_scene9_accuracy.py                       # k = 1.5 2.95 6, seed 1
      python3 eval_scene9_accuracy.py --ks 1 2.95 6 12 --seeds 1 2 3
      python3 eval_scene9_accuracy.py --skip-run            # re-tabulate saved runs
"""
import argparse
import csv
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SETVARS = "/opt/intel/oneapi/setvars.sh"
LANES = 7


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ks", type=float, nargs="+", default=[1.5, 2.95, 6.0],
                    help="vMF concentrations k (the reference uses the same k)")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1],
                    help="placement seeds; results are pooled over them")
    ap.add_argument("--steps", type=int, default=2400,
                    help="fixed-dt steps per run (groups reach the plates after ~1200)")
    ap.add_argument("--build-dir", default=os.path.join(HERE, "..", "bsh", "build"),
                    help="directory holding boidSimSYCL (it must run from there)")
    ap.add_argument("--out-dir", default=os.path.join(HERE, "results", "scene9_accuracy"),
                    help="where the raw CSVs and the tables are written")
    ap.add_argument("--skip-run", action="store_true",
                    help="do not run the simulator, tabulate the raw CSVs already saved")
    return ap.parse_args()


def raw_path(out_dir, k, seed):
    return os.path.join(out_dir, "raw", f"scene9_k{k:g}_seed{seed}.csv")


def run_one(binary, build_dir, k, seed, steps, out_csv):
    """Run one reference-driven accuracy run of scene 9 into a fresh CSV."""
    if os.path.exists(out_csv):
        os.remove(out_csv)   # the simulator appends
    cmd = [binary, "9", "-sh", "exact", "-kappa", f"{k:g}", "-accuracy", str(steps),
           "-seed", str(seed), "-out", out_csv]
    if "ONEAPI_ROOT" not in os.environ and os.path.exists(SETVARS):
        cmd = ["bash", "-c", f'source "{SETVARS}" >/dev/null 2>&1; exec "$@"', "bash"] + cmd
    print(f"  k={k:g} seed={seed}: running {steps} steps ...", flush=True)
    res = subprocess.run(cmd, cwd=build_dir, capture_output=True, text=True)
    if res.returncode != 0 or not os.path.exists(out_csv):
        sys.exit(f"run failed (k={k:g}, seed={seed}, exit {res.returncode}):\n{res.stdout}\n{res.stderr}")


def read_rows(path):
    if not os.path.exists(path):
        sys.exit(f"{path} missing: run without --skip-run for this k / seed")
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != LANES or "ref_ss" not in rows[0]:
        sys.exit(f"{path}: expected {LANES} lane rows with raw sums (rebuild boidSimSYCL?)")
    return rows


class Pool:
    """Sums of one SH variant against the reference, over any set of lanes/runs."""

    def __init__(self):
        self.n = self.ss_ref = self.dot = self.ss = self.se = 0.0
        self.ang = self.ang_n = self.missed = self.spurious = self.toward = 0.0
        self.max_err = 0.0

    def add(self, r, v):
        n = float(r["agent_steps"])
        self.n += n
        self.ss_ref += float(r["ref_ss"])
        self.dot += float(r[f"{v}_dot"])
        self.ss += float(r[f"{v}_ss"])
        self.se += float(r[f"{v}_se"])
        self.ang += float(r[f"{v}_angsum"])
        self.ang_n += float(r[f"{v}_angN"])
        self.missed += float(r[f"{v}_missed"]) * n
        self.spurious += float(r[f"{v}_spurious"]) * n
        self.toward += float(r[f"{v}_neg"]) * n
        self.max_err = max(self.max_err, float(r[f"{v}_maxerr"]))

    def metrics(self):
        ref = math.sqrt(self.ss_ref / self.n) if self.n else 0.0
        den = ref if ref > 0 else 1.0
        gain = self.dot / self.ss if self.ss > 0 else 0.0
        resid = max(0.0, self.ss_ref - self.dot * self.dot / self.ss) if self.ss > 0 else self.ss_ref
        return {
            "ref_rms": ref,
            "nrmse": math.sqrt(self.se / self.n) / den if self.n else 0.0,
            "fit": math.sqrt(resid / self.n) / den if self.n else 0.0,
            "gain": gain,
            "angle": self.ang / self.ang_n if self.ang_n else 0.0,
            "max_err": self.max_err,
            "missed": self.missed / self.n if self.n else 0.0,
            "spurious": self.spurious / self.n if self.n else 0.0,
            "toward": self.toward / self.n if self.n else 0.0,
        }


def main():
    a = parse_args()
    build_dir = os.path.abspath(a.build_dir)
    out_dir = os.path.abspath(a.out_dir)
    binary = os.path.join(build_dir, "boidSimSYCL")
    os.makedirs(os.path.join(out_dir, "raw"), exist_ok=True)

    if not a.skip_run:
        if not os.access(binary, os.X_OK):
            sys.exit(f"{binary} not found; build it first (cmake --build build --target boidSimSYCL)")
        print(f"scene 9 accuracy: k = {' '.join(f'{k:g}' for k in a.ks)}, "
              f"seeds = {' '.join(map(str, a.seeds))}, {a.steps} steps per run")
        for k in a.ks:
            for seed in a.seeds:
                run_one(binary, build_dir, k, seed, a.steps, raw_path(out_dir, k, seed))

    # pool per k: overall and per lane, for Dirac (D) and vMF (V); behaviour per lane
    overall, per_lane, behaviour = {}, {}, {}
    incidence = [None] * LANES
    for k in a.ks:
        overall[k] = {"D": Pool(), "V": Pool()}
        per_lane[k] = [{"D": Pool(), "V": Pool()} for _ in range(LANES)]
        behaviour[k] = [{"pen": 0.0, "clear": float("inf")} for _ in range(LANES)]
        for seed in a.seeds:
            for r in read_rows(raw_path(out_dir, k, seed)):
                lane = int(r["lane"])
                incidence[lane] = float(r["incidence_deg"])
                for v in ("D", "V"):
                    overall[k][v].add(r, v)
                    per_lane[k][lane][v].add(r, v)
                behaviour[k][lane]["pen"] += float(r["events"])
                behaviour[k][lane]["clear"] = min(behaviour[k][lane]["clear"], float(r["min_dist"]))

    seeds = " ".join(map(str, a.seeds))
    out = []
    out.append("# Scene 9 (plate lanes): obstacle SH accuracy, Dirac vs vMF\n")
    out.append(f"Reference-driven runs (`-sh exact`), {a.steps} steps, seeds {seeds}, pooled over "
               f"{LANES} lanes (incidence {', '.join(f'{x:g}' for x in incidence)} deg). "
               "The reference is the exact vMF field with the same k as the vMF row; "
               "Dirac is scored against that same reference.\n")

    out.append("## Overall\n")
    out.append("| variant | k (ref) | ref RMS | nRMSE | fit nRMSE | gain | dir. err (deg) "
               "| max err | missed | spurious | toward plate |")
    out.append("|---|---|---|---|---|---|---|---|---|---|---|")
    summary = []
    for k in a.ks:
        for v, name in (("D", "Dirac"), ("V", f"vMF k={k:g}")):
            m = overall[k][v].metrics()
            out.append(f"| {name} | {k:g} | {m['ref_rms']:.4f} | {m['nrmse']:.3f} | {m['fit']:.3f} "
                       f"| {m['gain']:.3f} | {m['angle']:.2f} | {m['max_err']:.3f} "
                       f"| {m['missed']:.4f} | {m['spurious']:.4f} | {m['toward']:.4f} |")
            summary.append(dict(variant="dirac" if v == "D" else "vmf", k=k, lane="all", **m))
    out.append("")

    out.append("## Per lane: nRMSE / direction error (deg)\n")
    head = "| incidence (deg) | " + " | ".join(f"Dirac @k={k:g} | vMF k={k:g}" for k in a.ks) + " |"
    out.append(head)
    out.append("|---|" + "---|" * (2 * len(a.ks)))
    for lane in range(LANES):
        cells = []
        for k in a.ks:
            for v in ("D", "V"):
                m = per_lane[k][lane][v].metrics()
                cells.append(f"{m['nrmse']:.3f} / {m['angle']:.1f}")
                summary.append(dict(variant="dirac" if v == "D" else "vmf", k=k, lane=lane, **m))
        out.append(f"| {incidence[lane]:g} | " + " | ".join(cells) + " |")
    out.append("")

    out.append("## Reference-driven behaviour: penetrations / minimum clearance\n")
    out.append("| incidence (deg) | " + " | ".join(f"k={k:g}" for k in a.ks) + " |")
    out.append("|---|" + "---|" * len(a.ks))
    for lane in range(LANES):
        cells = [f"{behaviour[k][lane]['pen']:.0f} / {behaviour[k][lane]['clear']:.2f}" for k in a.ks]
        out.append(f"| {incidence[lane]:g} | " + " | ".join(cells) + " |")
    out.append("")
    out.append("nRMSE: RMS error / RMS reference. fit nRMSE: after the best scalar gain. "
               "gain < 1: the SH pushes harder than the reference. Clearance is negative "
               "inside the plate (half thickness 2.5).")

    text = "\n".join(out) + "\n"
    print()
    print(text)
    md = os.path.join(out_dir, "scene9_accuracy.md")
    with open(md, "w") as f:
        f.write(text)
    sc = os.path.join(out_dir, "scene9_accuracy_summary.csv")
    with open(sc, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    print(f"tables: {md}\nsummary CSV: {sc}\nraw runs: {os.path.join(out_dir, 'raw')}")


if __name__ == "__main__":
    main()
