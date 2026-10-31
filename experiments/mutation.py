"""E6: fault localization with controller/datapath separation (mutation analysis).
Model mutants (GRAFCET JSON) and Action mutants (AST) of the MIAT 2.0 design are checked by
  L1 Gate-2 model checker (static, model level)
  L2 contract tests (unit level, per Action)
  L3 system tests in the simulator (black box)
A mutant is *localized* when the first failing layer names the mutated element
(transition/step id for L1, function name for L2)."""
import sys, os, json, ast, copy, random, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
from par import pmap
import numpy as np
from model.checker import check
from designs.runtime import make_factory, load_module_from_source
from model.synth import synthesize
import contracts.test_contract as TC
import pickle

OUT = os.path.join(ROOT, "results")
MODEL = json.load(open(os.path.join(ROOT, "model/miat2_grafcet.json")))
ACT_SRC = open(os.path.join(ROOT, "designs/miat2_actions.py")).read()
VEC = pickle.load(open(os.path.join(ROOT, "contracts/test_vectors.pkl"), "rb"))
rnd = random.Random(7)


# ------------------------------------------------------------ model mutants
def model_mutants(n=30):
    trs = [(gi, ti) for gi, g in enumerate(MODEL["grafcets"]) for ti, t in enumerate(g["transitions"])]
    ops = ["negate", "drop_not", "retarget", "drop_timeout", "true_cond", "drop_transition"]
    out = []
    while len(out) < n:
        op = ops[len(out) % len(ops)]
        m = copy.deepcopy(MODEL)
        gi, ti = rnd.choice(trs)
        g = m["grafcets"][gi]
        t = g["transitions"][ti]
        tag = t["id"]
        if op == "negate":
            t["cond"] = f"not ({t['cond']})"
        elif op == "drop_not":
            if " not " not in " " + t["cond"]:
                continue
            t["cond"] = t["cond"].replace("not ", "", 1)
        elif op == "retarget":
            others = [s["id"] for s in g["steps"] if s["id"] not in t["to"] + t["from"]]
            if not others:
                continue
            t["to"] = [rnd.choice(others)]
        elif op == "drop_timeout":
            st = [s for s in g["steps"] if s.get("timeout")]
            if not st:
                continue
            s = rnd.choice(st); s.pop("timeout"); tag = "X" + s["id"]
            # transitions using TO<id> become unproduced
        elif op == "true_cond":
            t["cond"] = "1"
        elif op == "drop_transition":
            g["transitions"].pop(ti)
        key = (op, tag)
        if key in [(o["op"], o["target"]) for o in out]:
            continue
        out.append(dict(kind="model", op=op, target=tag, grafcet=g["id"], model=m))
    return out


# ------------------------------------------------------------ action mutants
class Mut(ast.NodeTransformer):
    def __init__(self, k, op):
        self.k, self.op, self.i, self.fn, self.hit = k, op, 0, None, None
    def visit_FunctionDef(self, node):
        prev = self.fn; self.fn = node.name
        self.generic_visit(node); self.fn = prev
        return node
    def _maybe(self, node, new):
        if self.fn and self.fn.startswith("a") and not self.fn.startswith("as"):
            if self.i == self.k:
                self.hit = self.fn
                self.i += 1
                return new
            self.i += 1
        return node
    def visit_Compare(self, node):
        self.generic_visit(node)
        if self.op != "cmp":
            return node
        sw = {ast.Lt: ast.Gt, ast.Gt: ast.Lt, ast.LtE: ast.GtE, ast.GtE: ast.LtE}
        if type(node.ops[0]) in sw:
            new = copy.deepcopy(node); new.ops = [sw[type(node.ops[0])]()]
            return self._maybe(node, new)
        return node
    def visit_Constant(self, node):
        if self.op != "const" or not isinstance(node.value, float) or node.value in (0.0,):
            return node
        return self._maybe(node, ast.Constant(node.value * 1.5))
    def visit_BinOp(self, node):
        self.generic_visit(node)
        if self.op != "arith":
            return node
        sw = {ast.Add: ast.Sub, ast.Sub: ast.Add}
        if type(node.op) in sw:
            new = copy.deepcopy(node); new.op = sw[type(node.op)]()
            return self._maybe(node, new)
        return node
    def visit_UnaryOp(self, node):
        self.generic_visit(node)
        if self.op == "neg" and isinstance(node.op, ast.USub):
            return self._maybe(node, node.operand)
        return node


def action_mutants(n=40):
    out, tries = [], 0
    ops = ["cmp", "const", "arith", "neg"]
    while len(out) < n and tries < 2000:
        tries += 1
        op = ops[len(out) % len(ops)]
        tree = ast.parse(ACT_SRC)
        k = rnd.randrange(0, 60)
        mt = Mut(k, op)
        tree = mt.visit(tree)
        if not mt.hit:
            continue
        ast.fix_missing_locations(tree)
        src = ast.unparse(tree)
        if src == ACT_SRC or any(o["src"] == src for o in out):
            continue
        out.append(dict(kind="action", op=op, target=mt.hit, src=src))
    return out


# ------------------------------------------------------------ layers
def L1(mut):
    if mut["kind"] != "model":
        return False, None
    F, _ = check(mut["model"])
    if not F:
        return False, None
    loc = any(mut["target"] in f["where"] or f["where"].endswith(mut["target"]) for f in F) or \
          any(mut["grafcet"] in f["where"] for f in F)
    return True, dict(n=len(F), rules=sorted({f["rule"] for f in F}), localized=loc)


def L2(mut):
    if mut["kind"] != "action":
        return False, None
    A = load_module_from_source("mutA", mut["src"])
    failing = set()
    for fn, args, exp in VEC:
        try:
            got = getattr(A, fn)(*copy.deepcopy(args))
            if not TC.close(got, exp):
                failing.add(fn)
        except Exception:
            failing.add(fn)
    if not failing:
        return False, None
    # direct hit or the caller of a helper (a33x / a42 are used by a321 / a43)
    helpers = {"a331_bbox_center": "a321_candidates", "a332_pallet_distance": "a321_candidates",
               "a333_pallet_bearing": "a321_candidates", "a42_tag_to_pose": "a43_update_pose"}
    loc = mut["target"] in failing or helpers.get(mut["target"]) in failing
    return True, dict(failing=sorted(failing), localized=loc)


SYS_ITEMS = [("grid", th, 2.2, 60000 + i) for th in (45, 60, 75, 90) for i in range(3)] + \
            [("grid", 45, 2.8, 61000 + i) for i in range(3)] + \
            [("e2e", "simple", None, 62000 + i) for i in range(5)] + [("e2e", "cluttered", None, 63000 + i) for i in range(5)]


def sys_job(args):
    mi, kind, a, b, seed = args
    mut = MUTS[mi]
    if mut["kind"] == "model":
        f = lambda: __import__("designs.runtime", fromlist=["MiatController"]).MiatController(mut["model"], MA)
    else:
        A = load_module_from_source(f"mutA{mi}", mut["src"])
        model = MODEL
        f = lambda: __import__("designs.runtime", fromlist=["MiatController"]).MiatController(model, A)
    try:
        if kind == "grid":
            sc = Scenario("g", guidance_start(a, b), max_time=120.0)
        else:
            sc = e2e_scenario(a, np.random.default_rng(seed))
        r = run_trial(f, sc, seed)
        return mi, r["success"], r["reason"]
    except Exception as e:
        return mi, False, "exception"


MUTS = model_mutants(30) + action_mutants(40)

if __name__ == "__main__":
    t0 = time.time()
    rows = []
    for i, m in enumerate(MUTS):
        d1, i1 = L1(m)
        d2, i2 = L2(m)
        rows.append(dict(id=i, kind=m["kind"], op=m["op"], target=m["target"], L1=d1, L1info=i1, L2=d2, L2info=i2))
    # baseline system pass for the unmutated design on the same items
    res = pmap(sys_job, [(i, *it) for i in range(len(MUTS)) for it in SYS_ITEMS])
    import collections
    fails = collections.defaultdict(list)
    for mi, ok, reason in res:
        fails[mi].append((ok, reason))
    for r in rows:
        fl = fails[r["id"]]
        r["L3_fail"] = sum(not ok for ok, _ in fl)
        r["L3_n"] = len(fl)
        r["L3_reasons"] = dict(collections.Counter(reason for ok, reason in fl if not ok))
    json.dump(rows, open(os.path.join(OUT, "mutation.json"), "w"), indent=1)
    print("done", len(rows), "mutants in", round(time.time() - t0), "s")
