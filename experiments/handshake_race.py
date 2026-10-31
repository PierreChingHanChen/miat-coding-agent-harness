"""Dynamic confirmation of the C9 (internal id G9) handshake race in the original model.
Runs the ORIGINAL model WITHOUT the handshake repair on end-to-end simple scenarios and records,
from the synthesized controller's transition trace, whether the parent (G0) catches the first
completion of the tag-localization chart (G4) and of the guidance chart (G5).
  T4_6 : G4 leaves its final step X44 on '=1'      T0_5 : G0 acknowledges G4 completion
  T5_12: G5 leaves its final step X56 (restarts)    T0_6 : G0 acknowledges G5 completion
Writes results/handshake_race.json."""
import os, sys, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import orig_factory, run_trial, e2e_scenario, ROOT

def one(seed):
    holder = {}
    base = orig_factory(repair=False)
    def f():
        c = base(); holder["c"] = c; return c
    sc = e2e_scenario("simple", np.random.default_rng(seed))
    r = run_trial(f, sc, seed)
    tr = holder["c"].trace                      # (t, transition id, from, to)
    ids = [x[1] for x in tr]
    t_of = lambda tid: [x[0] for x in tr if x[1] == tid]
    first46, first05 = (t_of("T4_6") or [None])[0], (t_of("T0_5") or [None])[0]
    g4miss = first46 is not None and (first05 is None or first05 > first46)
    return dict(seed=seed, docked_seen=any("56" in map(str, x[3]) for x in tr),
                T4_6=ids.count("T4_6"), T0_5=ids.count("T0_5"), T5_12=ids.count("T5_12"), T0_6=ids.count("T0_6"),
                g4miss=g4miss, collided=r["collided"], top=holder["c"].active_steps()[:3])

if __name__ == "__main__":
    runs = [one(s) for s in range(int(sys.argv[1]) if len(sys.argv) > 1 else 10)]
    reached = [r for r in runs if r["docked_seen"]]
    summ = dict(runs=len(runs), reached_56=len(reached), parent_ack_T0_6=sum(r["T0_6"] > 0 for r in reached),
                child_restart_T5_12=sum(r["T5_12"] > 0 for r in reached), g4_missed_first=sum(r["g4miss"] for r in runs),
                collided_after=sum(r["collided"] for r in runs))
    print(json.dumps(summ))
    json.dump(dict(runs=runs, summary=summ), open(os.path.join(ROOT, "results", "handshake_race.json"), "w"), indent=1)
