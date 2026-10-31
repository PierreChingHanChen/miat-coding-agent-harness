"""Gate M (model checker) reports for the three models; writes results/gate2_*.txt.
Rule ids in the output are the internal ids G1..G13 = C1..C13 in the paper."""
import os, sys, subprocess
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for name, fn in [("original", "original_grafcet.json"), ("miat2_final", "miat2_grafcet.json"), ("miat2cr_final", "miat2_cr_grafcet.json")]:
    out = subprocess.run([sys.executable, os.path.join(ROOT, "model", "checker.py"), os.path.join(ROOT, "model", fn)],
                         capture_output=True, text=True, cwd=ROOT, env=dict(os.environ, PYTHONPATH=ROOT)).stdout
    open(os.path.join(ROOT, "results", f"gate2_{name}.txt"), "w").write(out)
    print(name, out.splitlines()[0] if out else "(no output)")
