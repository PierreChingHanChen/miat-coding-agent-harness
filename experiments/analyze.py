"""Aggregate all results into results/summary.json and figures/*.png"""
import json, os, sys, math, collections
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R = os.path.join(ROOT, "results")
FIG = os.path.join(ROOT, "figures")
os.makedirs(FIG, exist_ok=True)
ANGLES = [45, 60, 75, 90, 105, 120, 135]
DISTS = [2.2, 2.5, 2.8]
PAPER_VII = {2.2: [46, 40, 45, 50, 47, 42, 47], 2.5: [0, 44, 49, 50, 48, 43, 0], 2.8: [0, 2, 50, 50, 50, 0, 0]}
C = dict(blue="#2a78d6", orange="#eb6834", aqua="#1baf7a", gray="#8a8986", ink="#0b0b0b", ink2="#52514e", grid="#e4e3df")
plt.rcParams.update({"font.size": 9, "axes.edgecolor": C["ink2"], "axes.labelcolor": C["ink"], "xtick.color": C["ink2"],
                     "ytick.color": C["ink2"], "axes.spines.top": False, "axes.spines.right": False,
                     "font.family": "DejaVu Sans"})


def load(fn):
    p = os.path.join(R, fn)
    return [json.loads(l) for l in open(p)] if os.path.exists(p) else []


def wilson(k, n, z=1.96):
    if n == 0:
        return (0, 0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0, c - h), min(1, c + h))


def rate(rows, key="success"):
    n = len(rows); k = sum(1 for r in rows if r.get(key))
    lo, hi = wilson(k, n)
    return dict(k=k, n=n, p=round(k / n, 4) if n else None, ci=[round(lo, 3), round(hi, 3)])


def outcome(rows):
    n = len(rows)
    hang = sum(1 for r in rows if r.get("reason") == "timeout")
    return dict(n=n, success=rate(rows)["p"], collision=sum(r.get("collided", False) for r in rows),
                false_dock=sum(r.get("false_dock", False) for r in rows), hang=hang,
                safe_abort=sum(1 for r in rows if r.get("status") == "ABORTED" and not r.get("collided")),
                exception=sum(1 for r in rows if r.get("reason") == "exception"),
                t_mean_success=round(float(np.mean([r["t"] for r in rows if r.get("success")])), 1) if any(r.get("success") for r in rows) else None)


S = {}
# ------------------------------------------------------------------ grid
G = load("grid_main.jsonl") + load("grid_ablation.jsonl")
if G:
    S["grid"] = {}
    for d in sorted({r["design"] for r in G}):
        rows = [r for r in G if r["design"] == d]
        tab = {str(dist): [sum(r["success"] for r in rows if r["a"] == th and r["b"] == dist) for th in ANGLES] for dist in DISTS}
        n_cell = len([r for r in rows if r["a"] == 90 and r["b"] == 2.2])
        S["grid"][d] = dict(table=tab, n_per_cell=n_cell, overall=rate(rows), outcome=outcome(rows),
                            row_rates={k: round(sum(v) / (len(v) * n_cell), 3) for k, v in tab.items()},
                            inside_envelope=rate([r for r in rows if r["b"] == 2.2]))
    # heatmaps
    designs = [d for d in ["Original", "MIAT2", "MIAT2-noRecovery"] if d in S["grid"]]
    fig, axs = plt.subplots(1, len(designs) + 1, figsize=(2.6 * (len(designs) + 1), 2.3), constrained_layout=True)
    mats = [("Paper (Gazebo, Table VII)", np.array([PAPER_VII[d] for d in DISTS]) / 50)] + \
           [({"Original":"Original (re-implemented)","MIAT2":"MIAT 2.0","MIAT2-noRecovery":"MIAT 2.0 w/o recovery"}[d], np.array([S["grid"][d]["table"][str(x)] for x in DISTS]) / S["grid"][d]["n_per_cell"]) for d in designs]
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("seq", ["#f4f3f0", "#9cc3ef", C["blue"], "#123f78"])
    for ax, (title, M) in zip(axs, mats):
        ax.imshow(M, cmap=cmap, vmin=0, vmax=1, aspect="auto")
        for i in range(M.shape[0]):
            for j in range(M.shape[1]):
                ax.text(j, i, f"{100*M[i,j]:.0f}", ha="center", va="center", fontsize=7.5,
                        color="white" if M[i, j] > 0.55 else C["ink"])
        ax.set_xticks(range(7)); ax.set_xticklabels([f"{a}°" for a in ANGLES], fontsize=7)
        ax.set_yticks(range(3)); ax.set_yticklabels([f"{d} m" for d in DISTS], fontsize=7)
        ax.set_title(title, fontsize=8.5, color=C["ink"])
        for s in ax.spines.values():
            s.set_visible(False)
    fig.savefig(os.path.join(FIG, "fig_grid_heatmap.png"), dpi=200)
    plt.close(fig)

# ------------------------------------------------------------------ e2e
E = load("e2e_main.jsonl")
if E:
    S["e2e"] = {}
    for d in sorted({r["design"] for r in E}):
        S["e2e"][d] = {sc: dict(rate=rate([r for r in E if r["design"] == d and r["a"] == sc]),
                                outcome=outcome([r for r in E if r["design"] == d and r["a"] == sc])) for sc in ("simple", "cluttered")}

# ------------------------------------------------------------------ faults
F = load("faults.jsonl")
if F:
    S["faults"] = {}
    for fn in sorted({r["fault_name"] for r in F}):
        S["faults"][fn] = {d: outcome([r for r in F if r["fault_name"] == fn and r["design"] == d]) for d in sorted({r["design"] for r in F if r["fault_name"] == fn})}
        if fn == "F7_person_steps_in":
            for d in S["faults"][fn]:
                rr = [r for r in F if r["fault_name"] == fn and r["design"] == d and r.get("dyn_spawned")]
                S["faults"][fn][d]["min_clearance_m"] = round(min([r["min_clear_dyn"] for r in rr]), 3) if rr else None
                S["faults"][fn][d]["person_contacts"] = sum(1 for r in rr if r.get("reason") == "collision_dynamic")

# ------------------------------------------------------------------ generated code
V1, V2 = load("gen_v1.jsonl"), load("gen_v2.jsonl")
ST = json.load(open(os.path.join(R, "gen_static.json"))) if os.path.exists(os.path.join(R, "gen_static.json")) else {}
if V1:
    S["gen"] = {}
    for v in sorted({r["variant"] for r in V1}):
        rows = [r for r in V1 if r["variant"] == v]
        grid = [r for r in rows if r["kind"] == "grid"]
        e2s = [r for r in rows if r["kind"] == "e2e" and r["fault"] == "{}" and r["a"] == "simple"]
        e2c = [r for r in rows if r["kind"] == "e2e" and r["fault"] == "{}" and r["a"] == "cluttered"]
        flt = [r for r in rows if r["fault"] != "{}"]
        f1 = [r for r in flt if "tag_drop" in r["fault"]]
        S["gen"][v] = dict(grid=rate(grid), grid22=rate([r for r in grid if r["b"] == 2.2]), e2e_simple=rate(e2s), e2e_cluttered=rate(e2c),
                           faults=outcome(flt), all=outcome(rows), f1_safe_abort=sum(1 for r in f1 if r.get("status") == "ABORTED"),
                           f1_n=len(f1), crashed_trials=sum(1 for r in rows if r["reason"] in ("exception", "missing")),
                           static=ST.get(v + "_v1"))
    if V2:
        for v in sorted({r["variant"] for r in V2}):
            rows = [r for r in V2 if r["variant"] == v]
            dyn = [r for r in rows if r.get("dyn")]
            reg = [r for r in rows if not r.get("dyn")]
            S["gen"].setdefault(v, {})["cr"] = dict(dyn=outcome(dyn), person_contacts=sum(1 for r in dyn if r.get("reason") == "collision_dynamic"),
                                                    regression=rate(reg), regression_outcome=outcome(reg), diff=ST.get(v + "_diff"),
                                                    static=ST.get(v + "_v2"))

# ------------------------------------------------------------------ mutation
MU = json.load(open(os.path.join(R, "mutation.json"))) if os.path.exists(os.path.join(R, "mutation.json")) else None
if MU:
    base_fail = 0
    out = {}
    for kind in ("model", "action"):
        rows = [m for m in MU if m["kind"] == kind]
        det_static = [m for m in rows if (m["L1"] if kind == "model" else m["L2"])]
        loc = [m for m in det_static if (m["L1info"] or m["L2info"] or {}).get("localized")]
        det_sys = [m for m in rows if m["L3_fail"] > 0]
        either = [m for m in rows if (m["L1"] or m["L2"]) or m["L3_fail"] > 0]
        only_sys = [m for m in rows if not (m["L1"] or m["L2"]) and m["L3_fail"] > 0]
        out[kind] = dict(n=len(rows), detected_by_model_or_contract=len(det_static), localized=len(loc),
                         detected_by_system_tests=len(det_sys), detected_any=len(either), only_system=len(only_sys),
                         undetected=len(rows) - len(either))
    S["mutation"] = out

json.dump(S, open(os.path.join(R, "summary.json"), "w"), indent=1)
print(json.dumps({k: (list(v.keys()) if isinstance(v, dict) else v) for k, v in S.items()}, indent=1))
