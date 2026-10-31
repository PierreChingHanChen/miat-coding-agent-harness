import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mutation as MU
from model.checker import check
src = {t["id"]: t["from"] for g in MU.MODEL["grafcets"] for t in g["transitions"]}
rows = json.load(open(os.path.join(MU.OUT, "mutation.json")))
out = []
for r, m in zip(rows, MU.MUTS):
    assert r["target"] == m["target"] and r["op"] == m["op"]
    if m["kind"] != "model":
        continue
    F, _ = check(m["model"])
    keys = {m["target"]} | {"X" + s for s in src.get(m["target"], [])}
    if m["target"].startswith("X"):
        keys |= {"TO" + m["target"][1:]}
    loc = any(any(k in f["where"] or k in f["msg"] for k in keys) for f in F)
    out.append(dict(id=r["id"], op=m["op"], target=m["target"], findings=[f"{f['rule']}@{f['where']}" for f in F], localized=loc, L3_fail=r["L3_fail"]))
json.dump(out, open(os.path.join(MU.OUT, "mutation_model_loc.json"), "w"), indent=1)
det = [o for o in out if o["findings"]]
print("model mutants", len(out), "static-detected", len(det), "localized", sum(o["localized"] for o in det),
      "latent (static only, system tests pass)", sum(1 for o in det if o["L3_fail"] == 0))
