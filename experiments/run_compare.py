"""E2: re-run the paper's experiments (Table VII grid, Table VIII end-to-end) for the
original design and the MIAT 2.0 redesign, plus ablations and fault injection."""
import sys, os, json, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
from par import pmap
import numpy as np

OUT = os.path.join(ROOT, "results")
ANGLES = [45, 60, 75, 90, 105, 120, 135]
DISTS = [2.2, 2.5, 2.8]

ABL = {
    "MIAT2": {},
    "MIAT2-noRecovery": dict(max_retries=0),                         # R-05 cross-subsystem recovery off
    "MIAT2-noBlind": dict(pose_stale_s=0.5, blind_max_s=0.5, near_dist=0.0),   # R-04 off
    "MIAT2-noFOVplan": dict(fov_margin_deg=0.0),                     # R-06 off (FOV edge, empirical)
    "MIAT2-dApp2.2": dict(d_app=2.2),                                # R-05 threshold as in the paper
    "MIAT2-noConfirm": dict(confirm_k=1, width_min=0.0, width_max=99.0),   # R-07 off
}


def factory(name):
    if name == "Original":
        return orig_factory(repair=True)
    if name == "MIAT2+CR":
        return miat2cr_factory()
    return miat2_factory(controls_override=ABL[name] or None)


def job(args):
    name, kind, a, b, seed, faults, dyn = args
    f = factory(name)
    if kind == "grid":
        sc = Scenario("grid", guidance_start(a, b), max_time=120.0, faults=faults or {}, dynamic_obstacle=dyn)
    else:
        rng = np.random.default_rng(seed)
        sc = e2e_scenario(a, rng, faults=faults or {}, dynamic_obstacle=dyn)
    r = run_trial(f, sc, seed)
    r.update(design=name, kind=kind, a=a, b=b, seed=seed, fault=json.dumps(faults or {}), dyn=dyn)
    r.pop("wall", None)
    return r


def grid(names, n=50):
    items = [(nm, "grid", th, d, 10000 + i + 1000 * ANGLES.index(th) + 100000 * DISTS.index(d), None, False)
             for nm in names for th in ANGLES for d in DISTS for i in range(n)]
    return pmap(job, items)


def e2e(names, n=50, faults=None, dyn=False, scenes=("simple", "cluttered")):
    items = [(nm, "e2e", sc, None, 20000 + i + (0 if sc == "simple" else 50000), faults, dyn)
             for nm in names for sc in scenes for i in range(n)]
    return pmap(job, items)


def save(rows, fn):
    with open(os.path.join(OUT, fn), "w") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")


if __name__ == "__main__":
    what = sys.argv[1]
    t0 = time.time()
    if what == "grid":
        rows = grid(["Original", "MIAT2", "MIAT2-noRecovery"])
        save(rows, "grid_main.jsonl")
    elif what == "grid_abl":
        rows = grid(["MIAT2-noBlind", "MIAT2-noFOVplan", "MIAT2-dApp2.2", "MIAT2-noConfirm"], n=20)
        save(rows, "grid_ablation.jsonl")
    elif what == "e2e":
        rows = e2e(["Original", "MIAT2"] + list(k for k in ABL if k != "MIAT2"))
        save(rows, "e2e_main.jsonl")
    elif what == "faults":
        rows = []
        FA = {"F1_tag_occluded": dict(tag_drop=1.0), "F2_camera_freeze": dict(freeze_t0=6.0, freeze_len=3.0),
              "F3_pose_outliers": dict(tag_outlier=0.2), "F4_false_pos_x3": dict(fp_mult=3.0),
              "F5_depth_zero_40": dict(depth_zero=0.4), "F6_odom_slip_x3": dict(slip=3.0)}
        for fname, fa in FA.items():
            rr = e2e(["Original", "MIAT2"], n=30, faults=fa, scenes=("cluttered",) if fname == "F4_false_pos_x3" else ("simple",))
            for r in rr:
                r["fault_name"] = fname
            rows += rr
        rr = e2e(["Original", "MIAT2", "MIAT2+CR"], n=30, dyn=True, scenes=("simple",))
        for r in rr:
            r["fault_name"] = "F7_person_steps_in"
        rows += rr
        save(rows, "faults.jsonl")
    print(what, "done in", round(time.time() - t0), "s")
