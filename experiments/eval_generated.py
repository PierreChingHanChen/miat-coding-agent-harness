"""E3/E4: evaluate LLM-generated implementations.
D = Direct prompting (monolithic controller.py)      v2 = controller_v2.py after CR-1
M = MIAT contracts (actions.py + synthesized ctrl)   v2 = actions_v2.py + re-synthesized CR model
"""
import sys, os, json, math, ast, difflib, signal, re, importlib.util, time, traceback
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
from par import pmap
import numpy as np
from designs.runtime import make_factory, load_module_from_file

GEN = os.path.join(ROOT, "generated")
import zlib
# Seeds of the fault trials as used in the reported runs (the original code derived them from
# Python's randomized str hash); new variants fall back to a stable CRC32-based seed.
_FS = os.path.join(ROOT, "results", "fault_seeds.json")
FAULT_SEEDS = json.load(open(_FS)) if os.path.exists(_FS) else {}
OUT = os.path.join(ROOT, "results")
ANGLES = [45, 60, 75, 90, 105, 120, 135]
DISTS = [2.2, 2.5, 2.8]


class TrialTimeout(Exception):
    pass


def _alarm(sig, frm):
    raise TrialTimeout("wall-clock limit (controller blocked or too slow)")


def load_variant(name, ver):
    d = os.path.join(GEN, name)
    if name[0] in "DRC":
        fn = "controller.py" if ver == 1 else "controller_v2.py"
        path = os.path.join(d, fn)
        if not os.path.exists(path):
            return None, f"missing {fn}"
        for f in os.listdir(d):          # drop cached sibling modules of another variant (e.g. actions.py)
            if f.endswith(".py"):
                sys.modules.pop(f[:-3], None)
        sys.path.insert(0, d)
        try:
            mod = load_module_from_file(f"{name}_v{ver}_ctrl", path)
        finally:
            sys.path.remove(d)
        return (lambda: mod.DockingController()), None
    fn = "actions.py" if ver == 1 else "actions_v2.py"
    path = os.path.join(d, fn)
    if not os.path.exists(path):
        return None, f"missing {fn}"
    mod = load_module_from_file(f"{name}_v{ver}_act", path)
    model = os.path.join(ROOT, "model", "miat2_grafcet.json" if ver == 1 else "miat2_cr_grafcet.json")
    return make_factory(model, mod, name=f"{name}_v{ver}"), None


def job(args):
    name, ver, kind, a, b, seed, faults, dyn = args
    signal.signal(signal.SIGALRM, _alarm)
    base = dict(variant=name, ver=ver, kind=kind, a=a, b=b, seed=seed, fault=json.dumps(faults or {}), dyn=dyn)
    try:
        signal.alarm(60)
        f, err = load_variant(name, ver)
        if f is None:
            signal.alarm(0)
            return dict(base, success=False, reason="missing", exception=err, collided=False, false_dock=False, status="NONE", t=0)
        if kind == "grid":
            sc = Scenario("grid", guidance_start(a, b), max_time=120.0, faults=faults or {}, dynamic_obstacle=dyn)
        else:
            sc = e2e_scenario(a, np.random.default_rng(seed), faults=faults or {}, dynamic_obstacle=dyn)
        r = run_trial(f, sc, seed)
        signal.alarm(0)
        r.pop("wall", None)
        return dict(base, **r)
    except Exception as e:
        signal.alarm(0)
        return dict(base, success=False, reason="exception", exception=f"{type(e).__name__}: {e}"[:300],
                    collided=False, false_dock=False, status="EXC", t=0)


def suite(name, ver, cr=False):
    it = []
    if not cr:
        it += [(name, ver, "grid", th, d, 30000 + i + 100 * ANGLES.index(th) + 1000 * DISTS.index(d), None, False)
               for th in ANGLES for d in DISTS for i in range(8)]
        it += [(name, ver, "e2e", sc, None, 40000 + i + (0 if sc == "simple" else 500), None, False)
               for sc in ("simple", "cluttered") for i in range(20)]
        for fn, fa in {"F1": dict(tag_drop=1.0), "F2": dict(freeze_t0=6.0, freeze_len=3.0),
                       "F3": dict(tag_outlier=0.2), "F5": dict(depth_zero=0.4)}.items():
            base = FAULT_SEEDS.get(name, {}).get(fn, 45000 + zlib.crc32(fn.encode()) % 1000)
            it += [(name, ver, "e2e", "simple", None, base + i, fa, False) for i in range(8)]
    else:
        it += [(name, ver, "e2e", "simple", None, 46000 + i, None, True) for i in range(15)]
        it += [(name, ver, "grid", th, d, 47000 + i + 100 * ANGLES.index(th) + 1000 * DISTS.index(d), None, False)
               for th in ANGLES for d in DISTS for i in range(4)]
        it += [(name, ver, "e2e", sc, None, 48000 + i + (0 if sc == "simple" else 500), None, False)
               for sc in ("simple", "cluttered") for i in range(10)]
    return it


# ------------------------------------------------------------------ static analysis
API_ATTRS = {
    "obs": {"t", "odom", "tags", "pallets", "scan"},
    "odom": {"x", "y", "yaw", "v", "w"},
    "scan": {"angle_min", "angle_increment", "ranges", "range_max"},
}
API_NAMES = {"IMG_W", "HFOV", "VFOV", "FX", "CTRL_HZ", "DT", "V_MAX", "W_MAX", "PALLET_WIDTH", "TAG_ID", "CAM_X",
             "DOCK_DIST", "TagDetection", "PalletDetection", "Odom", "Scan", "Observation", "Command", "clamp_cmd"}
TAG_ATTRS = {"id", "x", "z", "yaw", "stamp"}
PAL_ATTRS = {"u", "width_px", "score", "depth_samples", "stamp"}


def cyclomatic(fn):
    c = 1
    for n in ast.walk(fn):
        if isinstance(n, (ast.If, ast.For, ast.While, ast.IfExp, ast.ExceptHandler, ast.With, ast.Assert, ast.comprehension)):
            c += 1
        elif isinstance(n, ast.BoolOp):
            c += len(n.values) - 1
    return c


def static(path):
    src = open(path).read()
    tree = ast.parse(src)
    lines = [l for l in src.splitlines() if l.strip() and not l.strip().startswith("#")]
    fns = [n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
    cc = [cyclomatic(f) for f in fns] or [0]
    flen = [(f.end_lineno - f.lineno + 1) for f in fns] or [0]
    bad_imports, bad_names = [], []
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom):
            if n.module and n.module.startswith("sim"):
                for a in n.names:
                    if a.name not in API_NAMES and a.name != "*":
                        bad_names.append(a.name)
            elif n.module and n.module.split(".")[0] not in ("math", "dataclasses", "typing", "collections", "enum", "copy", "statistics", "__future__", "itertools", "functools"):
                bad_imports.append(n.module)
        elif isinstance(n, ast.Import):
            for a in n.names:
                if a.name.split(".")[0] not in ("math", "copy", "statistics", "collections", "enum", "typing", "dataclasses", "itertools", "functools"):
                    bad_imports.append(a.name)
    # attribute use on API objects (heuristic: names obs/odom/scan/tag/det)
    halluc = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Attribute) and isinstance(n.value.value, ast.Name):
            if n.value.value.id in ("obs", "observation") and n.value.attr == "odom" and n.attr not in API_ATTRS["odom"]:
                halluc.add("odom." + n.attr)
            if n.value.value.id in ("obs", "observation") and n.value.attr == "scan" and n.attr not in API_ATTRS["scan"]:
                halluc.add("scan." + n.attr)
        if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id in ("obs", "observation"):
            if n.attr not in API_ATTRS["obs"]:
                halluc.add("obs." + n.attr)
    sleeps = len(re.findall(r"\btime\.sleep|\bthreading\b|\basyncio\b", src))
    glob = len([n for n in ast.walk(tree) if isinstance(n, ast.Global)])
    modmut = len([n for n in tree.body if isinstance(n, ast.Assign) and isinstance(n.value, (ast.List, ast.Dict, ast.Set, ast.Call))
                  and not all(isinstance(t, ast.Name) and t.id.isupper() for t in n.targets)])
    req_refs = sorted(set(re.findall(r"R-0?\d+", src)))
    return dict(loc=len(lines), n_functions=len(fns), max_cc=max(cc), mean_cc=round(sum(cc) / len(cc), 2),
                max_fn_len=max(flen), bad_imports=bad_imports, bad_api_names=bad_names, api_hallucinations=sorted(halluc),
                blocking_constructs=sleeps, global_stmts=glob, module_mutable_state=modmut, req_ids_in_code=req_refs)


def diff_stats(p1, p2):
    a, b = open(p1).read().splitlines(), open(p2).read().splitlines()
    added = removed = 0
    for l in difflib.unified_diff(a, b, lineterm="", n=0):
        if l.startswith("+") and not l.startswith("+++"):
            added += 1
        elif l.startswith("-") and not l.startswith("---"):
            removed += 1
    def fnmap(p):
        t = ast.parse(open(p).read())
        return {n.name: ast.dump(n) for n in ast.walk(t) if isinstance(n, ast.FunctionDef)}
    f1, f2 = fnmap(p1), fnmap(p2)
    changed = sorted(k for k in f1 if k in f2 and f1[k] != f2[k])
    new = sorted(k for k in f2 if k not in f1)
    gone = sorted(k for k in f1 if k not in f2)
    return dict(lines_added=added, lines_removed=removed, fn_changed=changed, fn_new=new, fn_removed=gone,
                pct_lines_touched=round(100 * (added + removed) / max(1, len(a)), 1))


def similarity(p, ref):
    return round(difflib.SequenceMatcher(None, open(p).read(), open(ref).read()).ratio(), 3)


if __name__ == "__main__":
    what = sys.argv[1]
    names = sorted(n for n in os.listdir(GEN) if n[0] in "DMRCNT" and os.path.isdir(os.path.join(GEN, n)))
    if what == "static":
        res = {}
        for n in names:
            d = os.path.join(GEN, n)
            for ver, fn in ((1, "controller.py" if n[0] in "DRC" else "actions.py"), (2, "controller_v2.py" if n[0] in "DRC" else "actions_v2.py")):
                p = os.path.join(d, fn)
                if os.path.exists(p):
                    try:
                        res[f"{n}_v{ver}"] = static(p)
                    except SyntaxError as e:
                        res[f"{n}_v{ver}"] = dict(syntax_error=str(e))
                    res[f"{n}_v{ver}"]["sim_to_reference"] = similarity(p, os.path.join(ROOT, "designs", "miat2_actions.py"))
            p1 = os.path.join(d, "controller.py" if n[0] in "DRC" else "actions.py")
            p2 = os.path.join(d, "controller_v2.py" if n[0] in "DRC" else "actions_v2.py")
            if os.path.exists(p1) and os.path.exists(p2):
                res[f"{n}_diff"] = diff_stats(p1, p2)
        json.dump(res, open(os.path.join(OUT, "gen_static.json"), "w"), indent=1)
        print(json.dumps(res, indent=1)[:3000])
    elif what in ("v1", "v2"):
        sel = [n for n in names if len(sys.argv) < 3 or n in sys.argv[2].split(",")]
        items = []
        for n in sel:
            items += suite(n, 1, cr=False) if what == "v1" else suite(n, 2, cr=True)
        t0 = time.time()
        rows = pmap(job, items)
        fn = os.path.join(OUT, f"gen_{what}.jsonl")
        mode = "a" if len(sys.argv) >= 3 else "w"
        with open(fn, mode) as f:
            for r in rows:
                f.write(json.dumps(r, default=str) + "\n")
        print(what, len(rows), "trials in", round(time.time() - t0), "s")
