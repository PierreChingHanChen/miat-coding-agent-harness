import sys, os, json, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sim.dock_sim import run_trial, Scenario, guidance_start, e2e_scenario, DockSim
from designs.runtime import make_factory
import designs.original_actions as OA
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
def orig_factory(repair=True, **kw):
    return make_factory(os.path.join(ROOT, "model/original_grafcet.json"), OA, repair=repair, name="orig", **kw)
import designs.miat2_actions as MA
def miat2_factory(**kw):
    return make_factory(os.path.join(ROOT, "model/miat2_grafcet.json"), MA, name="miat2", **kw)
def miat2cr_factory(actions=None, **kw):
    return make_factory(os.path.join(ROOT, "model/miat2_cr_grafcet.json"), actions or MA, name="miat2cr", **kw)
