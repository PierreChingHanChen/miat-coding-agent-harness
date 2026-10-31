"""Gate-3 binding check: every Action named in the GRAFCET model exists in the actions module
with the declared arity (static, before any simulation)."""
import ast, json, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
def required(model):
    req = {}
    for g in model["grafcets"]:
        for a in g.get("continuous", []):
            req[a["fn"]] = len(a["in"])
        for s in g["steps"]:
            for a in s.get("actions", []):
                req[a["fn"]] = len(a["in"])
    return req
def check(path, model):
    t = ast.parse(open(path).read())
    defs = {n.name: len(n.args.args) for n in t.body if isinstance(n, ast.FunctionDef)}
    req = required(model)
    missing = sorted(f for f in req if f not in defs)
    arity = sorted(f for f in req if f in defs and defs[f] != req[f])
    return dict(required=len(req), missing=missing, arity_mismatch=arity)
if __name__ == "__main__":
    out = {}
    for ver, mfile, afile in [(1, "miat2_grafcet.json", "actions.py"), (2, "miat2_cr_grafcet.json", "actions_v2.py")]:
        m = json.load(open(os.path.join(ROOT, "model", mfile)))
        out[f"reference_v{ver}"] = check(os.path.join(ROOT, "designs/miat2_actions.py"), m)
        for n in sorted(os.listdir(os.path.join(ROOT, "generated"))):
            p = os.path.join(ROOT, "generated", n, afile)
            if n.startswith("M") and os.path.exists(p):
                out[f"{n}_v{ver}"] = check(p, m)
    json.dump(out, open(os.path.join(ROOT, "results/gate3_binding.json"), "w"), indent=1)
    for k, v in out.items(): print(k, v)
