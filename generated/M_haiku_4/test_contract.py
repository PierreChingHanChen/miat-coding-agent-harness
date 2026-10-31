"""Contract test: python test_contract.py [actions_module_path] [vectors.pkl]
Checks every Action against the reference input/output vectors of the contract."""
import sys, os, pickle, math, importlib.util, copy, traceback
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.dirname(HERE))
from sim.api import Command

def close(a, b, tol=1e-3):
    if isinstance(b, Command):
        return isinstance(a, Command) and abs(a.v - b.v) <= 2e-3 and abs(a.w - b.w) <= 2e-3
    if isinstance(b, bool) or b is None:
        return a == b or (b is None and a is None)
    if isinstance(b, (int, float)):
        return isinstance(a, (int, float)) and not isinstance(a, bool) and math.isfinite(a) and abs(a - b) <= tol * max(1.0, abs(b))
    if isinstance(b, dict):
        if not isinstance(a, dict):
            return False
        keys = [k for k in b if k not in ("rej",)]
        return all(k in a and close(a[k], b[k], tol) for k in keys)
    if isinstance(b, (list, tuple)):
        return isinstance(a, (list, tuple)) and len(a) == len(b) and all(close(x, y, tol) for x, y in zip(a, b))
    return a == b

def run(path, vec_path):
    spec = importlib.util.spec_from_file_location("actions_under_test", path)
    A = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(A)
    vec = pickle.load(open(vec_path, "rb"))
    res = {}
    for fn, args, exp in vec:
        ok = False
        try:
            got = getattr(A, fn)(*copy.deepcopy(args))
            ok = close(got, exp)
            if not ok:
                print(f"FAIL {fn}{tuple(type(a).__name__ for a in args)}\n   expected {exp}\n   got      {got}")
        except Exception as e:
            print(f"ERROR {fn}: {type(e).__name__}: {e}")
        res.setdefault(fn, []).append(ok)
    n = sum(len(v) for v in res.values()); p = sum(sum(v) for v in res.values())
    fpass = sum(all(v) for v in res.values())
    print(f"vectors passed {p}/{n}; functions fully passing {fpass}/{len(res)}")
    return p, n, fpass, len(res), res

if __name__ == "__main__":
    run(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "actions.py"),
        sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "test_vectors.pkl"))
