"""Generate ACTION_CONTRACTS.md and the contract test vectors from the GRAFCET model,
the hand-written specs and the reference actions (expected values)."""
import json, math, os, sys, pickle, copy
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sim.api import *
from contracts.specs import SPEC, COMMON
import designs.miat2_actions as REF

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def bindings(model):
    out = {}
    for g in model["grafcets"]:
        for a in g.get("continuous", []):
            out.setdefault(a["fn"], (a, g["id"], "continuous", g.get("idef0")))
        for s in g["steps"]:
            for a in s.get("actions", []):
                out.setdefault(a["fn"], (a, g["id"], s["id"], s.get("idef0")))
    return out


def P_of(model):
    return dict(model["controls"])


# ---------------------------------------------------------------- test cases
def cases(P):
    T = lambda x, z, yaw, t=1.0, i=TAG_ID: TagDetection(i, x, z, yaw, t)
    O = lambda x=0.0, y=0.0, yaw=0.0: Odom(x, y, yaw, 0.0, 0.0)
    D = lambda u, wpx, sc, d, t=1.0, zeros=0: PalletDetection(u, wpx, sc, [0.0] * zeros + [d] * (25 - zeros), t)
    est = lambda px, py, yaw, t_meas=1.0, n=5, o=(0.0, 0.0, 0.0): dict(px=px, py=py, yaw=yaw, t_meas=t_meas, n=n, rej=0, o=o)
    C = []
    add = lambda fn, *args: C.append((fn, args))
    add("a332_pallet_distance", [2.0] * 25)
    add("a332_pallet_distance", [0.0] * 10 + [3.0, 3.1, 2.9, 3.0, 3.05] + [6.0] * 2 + [3.02] * 8)
    add("a332_pallet_distance", [0.0] * 22 + [1.0, 1.1, 1.2])
    add("a332_pallet_distance", [1.0, 1.0, 1.0, 1.0, 1.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0] + [0.0] * 14)
    add("a333_pallet_bearing", 320.0)
    add("a333_pallet_bearing", 600.0)
    add("a333_pallet_bearing", 40.0)
    add("a42_tag_to_pose", T(0.0, 1.5, 0.0))
    add("a42_tag_to_pose", T(0.3, 2.0, 0.2))
    add("a42_tag_to_pose", T(-0.4, 1.2, -0.5))
    add("a321_candidates", [D(320, FX * 1.2 / 3.0, 0.9, 3.0)], O(), [], P)
    add("a321_candidates", [D(320, FX * 2.0 / 3.0, 0.8, 3.0), D(200, FX * 1.0 / 2.5, 0.5, 2.5)], O(1, 1, 0.5), [], P)
    add("a321_candidates", [D(400, FX * 1.1 / 2.0, 0.7, 2.0, zeros=5)], O(0, 0, 1.0), [(1.2, 1.4)], P)
    add("a321_candidates", [D(400, FX * 1.1 / 2.0, 0.7, 2.0, zeros=5)], O(0, 0, 1.0), [(5.0, 5.0)], P)
    c1 = dict(ba=0.5, d=3.0, b=0.0, W=1.2, score=0.9, pos=(2.9, 1.7))
    c2 = dict(ba=0.52, d=3.1, b=0.0, W=1.2, score=0.8, pos=(2.9, 1.8))
    h = [dict(t=0.8, c=c1), dict(t=0.9, c=c2)]
    add("a322_confirm", [c1], h, 1.0, P)
    add("a322_confirm", [c1], [dict(t=0.2, c=c1), dict(t=0.9, c=c2)], 1.0, P)
    add("a322_confirm", [], h, 1.0, P)
    tr = dict(ba=0.5, d=3.0, pos=(2.9, 1.7), t=0.5)
    add("a323_track", [c2], tr, 1.0, P)
    add("a323_track", [dict(c2, ba=1.2)], tr, 1.0, P)
    add("a323_track", [], tr, 3.0, P)
    add("a323_track", [], None, 3.0, P)
    add("a34_approach_cmd", dict(ba=0.0, d=3.0, pos=(3.28, 0.0), t=1.0), O(), P)
    add("a34_approach_cmd", dict(ba=0.0, d=3.0, pos=(3.28, 1.0), t=1.0), O(), P)
    add("a34_approach_cmd", dict(ba=0.0, d=1.5, pos=(1.78, 0.02), t=1.0), O(), P)
    add("a34_approach_cmd", dict(ba=0.0, d=1.5, pos=(1.78, -0.5), t=1.0), O(), P)
    add("a34_approach_cmd", None, O(), P)
    add("a40_enter", 0, None, P)
    add("a40_enter", 2, est(0, 2, -1.57), P)
    add("a41_tag_sweep", O(0, 0, 0), dict(ba=0.0, pos=(2.0, 0.0)), None, P)
    add("a41_tag_sweep", O(0, 0, 0), None, dict(targets=[0.5, 1.1, -0.1, 0.5], i=0), P)
    add("a41_tag_sweep", O(0, 0, 0.49), None, dict(targets=[0.5, 1.1, -0.1, 0.5], i=0), P)
    add("a43_update_pose", None, O(), [], 1.0, P)
    add("a43_update_pose", None, O(), [T(0.0, 1.5, 0.0)], 1.0, P)
    e0 = est(0.0, 2.0, -math.pi / 2, t_meas=0.9, n=5, o=(0.0, 0.0, 0.0))
    add("a43_update_pose", e0, O(0.1, 0.0, 0.0), [], 1.0, P)
    add("a43_update_pose", e0, O(0.0, 0.0, 0.1), [], 1.0, P)
    add("a43_update_pose", e0, O(), [T(0.05, 1.72, 0.02)], 1.0, P)
    add("a43_update_pose", e0, O(), [T(0.9, 1.0, 0.6)], 1.0, P)
    add("a43_update_pose", dict(e0, rej=4), O(), [T(0.9, 1.0, 0.6)], 1.0, P)
    add("a43_update_pose", dict(e0, n=1), O(), [T(0.05, 1.72, 0.02)], 1.0, P)
    add("a43_update_pose", e0, O(), [T(0.05, 1.72, 0.02, i=3)], 1.0, P)
    add("a44_pose_status", None, 1.0, P)
    add("a44_pose_status", est(0, 2, -1.57, t_meas=0.9, n=5), 1.0, P)
    add("a44_pose_status", est(0, 2, -1.57, t_meas=0.9, n=2), 1.0, P)
    add("a44_pose_status", est(0, 2, -1.57, t_meas=-3.0, n=5), 1.0, P)
    add("a44_pose_status", est(0, 1.2, -1.57, t_meas=-3.0, n=5), 1.0, P)
    add("a44_pose_status", est(0, 1.2, -1.57, t_meas=-12.0, n=5), 1.0, P)
    add("a4e_blacklist", [(1.0, 1.0)], dict(ba=0.1, pos=(3.0, 0.5)))
    add("a4e_blacklist", [], None)
    add("a51_plan", None, P)
    add("a51_plan", est(0.02, 1.7, -1.57), P)
    add("a51_plan", est(0.3, 2.2, -1.8), P)
    add("a51_plan", est(1.1, 1.9, -2.0), P)
    add("a51_plan", est(0.5, 1.6, -2.0), P)
    add("a52_reposition", est(1.1, 1.9, -2.1), None, P)
    add("a52_reposition", est(1.1, 1.9, -2.1), dict(own="52", ph="drive", W=(0.4, 2.25)), P)
    add("a52_reposition", est(0.4, 2.25, 2.9), dict(own="52", ph="drive", W=(0.4, 2.25)), P)
    add("a52_reposition", est(0.4, 2.25, -1.75), dict(own="52", ph="turn2", W=(0.4, 2.25)), P)
    add("a53_converge", est(0.03, 1.70, -1.57), P)
    add("a53_converge", est(0.4, 1.2, -1.57), P)
    add("a53_converge", est(0.5, 2.2, -1.9), P)
    add("a53_converge", est(-0.3, 2.0, -1.2), P)
    add("a54_reacquire", est(0.5, 2.0, 3.0), None, 1.0, P)
    add("a54_reacquire", est(0.0, 2.0, -1.5708), None, 1.0, P)
    add("a54_reacquire", None, None, 1.0, P)
    add("a55_final_approach", est(0.0, 1.6, -1.5708), P)
    add("a55_final_approach", est(0.1, 1.5, -1.5708), P)
    add("a55_final_approach", est(0.0, 1.13, -1.3), P)
    add("a55_final_approach", est(0.0, 1.13, -1.58), P)
    add("a56_verify", est(0.0, 1.13, -1.57), None, 5.0, P)
    add("a56_verify", est(0.0, 1.13, -1.57), dict(own="56", t0=4.0), 5.0, P)
    add("a56_verify", est(0.08, 1.13, -1.57), dict(own="56", t0=4.0), 5.0, P)
    add("a57_backoff", O(1, 1, 0), None, 0, P)
    add("a57_backoff", O(1.7, 1, 0), dict(own="57", x0=1.0, y0=1.0), 1, P)
    add("a57_backoff", O(1.2, 1, 0), dict(own="57", x0=1.0, y0=1.0), 3, P)
    add("a1_check_sensors", O(), Scan(-math.pi, math.radians(2), [1.0] * 180))
    add("a1_check_sensors", None, None)
    return C


def cr_cases(P):
    sc = lambda near_idx=None, r=0.4: Scan(-math.pi, math.radians(2),
                                           [r if (near_idx is not None and i == near_idx) else 5.0 for i in range(180)])
    C = []
    add = lambda fn, *args: C.append((fn, args))
    add("as1_obstacle_monitor", sc(), None, 1.0, P)
    add("as1_obstacle_monitor", sc(90), None, 1.0, P)
    add("as1_obstacle_monitor", sc(90 + 20), None, 1.0, P)
    add("as1_obstacle_monitor", sc(), dict(last_near=0.5), 1.0, P)
    add("as1_obstacle_monitor", sc(), dict(last_near=-0.5), 1.0, P)
    add("as1_obstacle_monitor", sc(85, 0.6), None, 1.0, P)
    add("as2_inhibit_forward", Command(0.2, 0.3))
    add("as2_inhibit_forward", Command(-0.1, 0.0))
    return C


def build(model_path, out_md, out_vec, cr=False):
    model = json.load(open(model_path))
    P = P_of(model)
    B = bindings(model)
    lines = ["# Action Contracts (IDEF0 ICOM → function signatures)", "",
             "Generated from the frozen GRAFCET model `" + os.path.basename(model_path) + "`. "
             "Implement **every** function below in one module `actions.py`. The controller that calls them is synthesized "
             "from the model by fixed rules (see `controller_synthesized.py`, read-only).", COMMON,
             "## Controls (P)", "", "```json", json.dumps(model["controls"], indent=1), "```", ""]
    fns = list(SPEC.keys()) if cr else [f for f in SPEC if not f.startswith("as")]
    for fn in fns:
        if fn in B:
            a, gid, sid, idef = B[fn]
            sig = f"{fn}({', '.join(a['in'])}) -> ({', '.join(a['out'])})"
            where = f"GRAFCET {gid}, step {sid}" if sid != "continuous" else f"GRAFCET {gid}, continuous action"
            lines += [f"### `{sig}`", f"*IDEF0 {idef} · {where}*", "", SPEC[fn], ""]
        else:   # helper used inside other actions
            sig = {"a331_bbox_center": "a331_bbox_center(det) -> u",
                   "a332_pallet_distance": "a332_pallet_distance(depth_samples) -> (dist, ok)",
                   "a333_pallet_bearing": "a333_pallet_bearing(u) -> bearing",
                   "a42_tag_to_pose": "a42_tag_to_pose(tag) -> (px, py, yaw)"}[fn]
            lines += [f"### `{sig}`", "*atomic sub-function (IDEF0 " + fn[1:4].upper() + "), called by other actions*", "", SPEC[fn], ""]
    lines += ["## Test vectors", "", "`test_contract.py` checks every function against reference input/output vectors "
              "(floats ±1e-3, Commands ±2e-3). All must pass before integration."]
    open(out_md, "w").write("\n".join(lines) + "\n")
    cs = cr_cases(P) if cr else cases(P)
    vec = []
    for fn, args in cs:
        res = getattr(REF, fn)(*copy.deepcopy(args))
        vec.append((fn, args, res))
    pickle.dump(vec, open(out_vec, "wb"))
    print(out_md, len(fns), "functions,", len(vec), "vectors")


if __name__ == "__main__":
    build(os.path.join(ROOT, "model/miat2_grafcet.json"), os.path.join(HERE, "ACTION_CONTRACTS.md"),
          os.path.join(HERE, "test_vectors.pkl"))
    build(os.path.join(ROOT, "model/miat2_cr_grafcet.json"), os.path.join(HERE, "ACTION_CONTRACTS_CR1.md"),
          os.path.join(HERE, "test_vectors_cr1.pkl"), cr=True)
