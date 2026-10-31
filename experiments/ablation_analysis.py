"""Harness ablation: six arms x two models x 4 independent programs."""
import json, os, sys, collections, math, pickle, copy, importlib.util
import numpy as np
from scipy.stats import mannwhitneyu
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "contracts"))
import test_contract as TC
sys.path.insert(0, os.path.join(ROOT, "experiments"))
from gate3_binding import check as g3check

ARMS = [("R", "Raw req.\nmonolithic"), ("D", "Full req.\nmonolithic"), ("C", "+ contracts\n+ tests,\nown control"),
        ("N", "+ model\n(signatures\nonly)"), ("T", "+ model\n+ contracts,\nno tests"), ("M", "Full\nharness")]
COMP = {  # harness components present per arm
    "R": dict(req="raw", contracts=0, tests=0, grafcet=0), "D": dict(req="full", contracts=0, tests=0, grafcet=0),
    "C": dict(req="full", contracts=1, tests=1, grafcet=0), "N": dict(req="full", contracts=0, tests=0, grafcet=1),
    "T": dict(req="full", contracts=1, tests=0, grafcet=1), "M": dict(req="full", contracts=1, tests=1, grafcet=1)}
rows = [json.loads(l) for l in open(os.path.join(ROOT, "results/gen_v1.jsonl"))]
by = collections.defaultdict(list)
for r in rows:
    by[r["variant"]].append(r)
VEC = pickle.load(open(os.path.join(ROOT, "contracts/test_vectors.pkl"), "rb"))
MODEL = json.load(open(os.path.join(ROOT, "model/miat2_grafcet.json")))


def contract_score(path):
    spec = importlib.util.spec_from_file_location("a_" + str(abs(hash(path))), path)
    A = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(A)
    except Exception:
        return 0, len(VEC)
    ok = 0
    for fn, args, exp in VEC:
        try:
            ok += TC.close(getattr(A, fn)(*copy.deepcopy(args)), exp)
        except Exception:
            pass
    return ok, len(VEC)


out = {}
for arm, _ in ARMS:
    for mdl in ("sonnet", "haiku"):
        progs = []
        for i in range(1, 5):
            v = f"{arm}_{mdl}_{i}"
            rs = by.get(v, [])
            if not rs:
                continue
            core = [r for r in rs if r["fault"] == "{}"]          # grid + e2e simple + cluttered (208)
            flt = [r for r in rs if r["fault"] != "{}"]
            p = dict(variant=v, n=len(core), success=sum(r["success"] for r in core) / len(core),
                     grid=np.mean([r["success"] for r in core if r["kind"] == "grid"]),
                     false_dock=sum(r.get("false_dock", False) for r in rs), collision=sum(r.get("collided", False) for r in rs),
                     hang=sum(r["reason"] == "timeout" for r in rs), exception=sum(r["reason"] in ("exception", "missing") for r in rs),
                     fault_success=np.mean([r["success"] for r in flt]) if flt else None,
                     f1_safe=sum(1 for r in flt if "tag_drop" in r["fault"] and r.get("status") == "ABORTED"))
            ap = os.path.join(ROOT, "generated", v, "actions.py")
            if os.path.exists(ap):
                p["contract_vectors"] = contract_score(ap)[0]
                g = g3check(ap, MODEL)
                p["g3_missing"] = len(g["missing"]) + len(g["arity_mismatch"])
            progs.append(p)
        if progs:
            s = [p["success"] for p in progs]
            out[f"{arm}_{mdl}"] = dict(arm=arm, model=mdl, programs=progs, mean=float(np.mean(s)), min=float(min(s)), max=float(max(s)),
                                       sd=float(np.std(s, ddof=1)) if len(s) > 1 else 0.0,
                                       pooled_k=int(sum(p["success"] * p["n"] for p in progs)), pooled_n=int(sum(p["n"] for p in progs)),
                                       false_dock=sum(p["false_dock"] for p in progs), collision=sum(p["collision"] for p in progs),
                                       hang=sum(p["hang"] for p in progs), exception=sum(p["exception"] for p in progs),
                                       contract_vectors=[p.get("contract_vectors") for p in progs],
                                       g3_missing=[p.get("g3_missing") for p in progs], **COMP[arm])
# pairwise tests on per-program success (unit = program), both models pooled
def arm_scores(a):
    return [p["success"] for m in ("sonnet", "haiku") for p in out.get(f"{a}_{m}", {}).get("programs", [])]
tests = {}
for a, b in [("R", "D"), ("D", "C"), ("D", "N"), ("N", "T"), ("T", "M"), ("C", "M"), ("D", "M"), ("R", "M")]:
    x, y = arm_scores(a), arm_scores(b)
    if x and y:
        u = mannwhitneyu(x, y, alternative="two-sided")
        tests[f"{a}_vs_{b}"] = dict(n=(len(x), len(y)), mean=(float(np.mean(x)), float(np.mean(y))),
                                    sd=(float(np.std(x, ddof=1)), float(np.std(y, ddof=1))), U=float(u.statistic), p=float(u.pvalue))
json.dump(dict(arms=out, tests=tests), open(os.path.join(ROOT, "results/ablation.json"), "w"), indent=1, default=float)

# ---- figure: one dot per program, two rows (models)
C = dict(sonnet="#2a78d6", haiku="#eb6834", ink="#0b0b0b", ink2="#52514e", grid="#e4e3df")
fig, ax = plt.subplots(figsize=(7.4, 3.4), constrained_layout=True)
for ai, (arm, lab) in enumerate(ARMS):
    for mi, mdl in enumerate(("sonnet", "haiku")):
        d = out.get(f"{arm}_{mdl}")
        if not d:
            continue
        vals = [100 * p["success"] for p in d["programs"]]
        xs = [ai + (-0.17 if mi == 0 else 0.17) + (j - 1.5) * 0.045 for j in range(len(vals))]
        ax.scatter(xs, vals, s=34, color=C[mdl], edgecolor="white", linewidth=1.2, zorder=3,
                   label=("Sonnet" if mdl == "sonnet" else "Haiku") if ai == 0 else None)
ax.set_xticks(range(len(ARMS))); ax.set_xticklabels([f"{a}: {l}" for a, l in ARMS], fontsize=7.2)
ax.set_ylabel("Docking success per program [%]", fontsize=8.5); ax.set_ylim(-5, 106)
ax.grid(axis="y", color=C["grid"], lw=0.6); ax.set_axisbelow(True)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
ax.legend(frameon=False, fontsize=8, loc="lower right")
ax.set_title("Harness ablation: each dot is one independently generated program (208 trials each)", fontsize=8.5)
fig.savefig(os.path.join(ROOT, "figures/fig_harness_ablation.png"), dpi=200)
for k, v in out.items():
    print(k, round(v["mean"], 3), round(v["min"], 3), round(v["max"], 3), "FD", v["false_dock"], "coll", v["collision"], "hang", v["hang"], "exc", v["exception"], "cv", v["contract_vectors"], "g3", v["g3_missing"])
for k, v in tests.items():
    print(k, v)
