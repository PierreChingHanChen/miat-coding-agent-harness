"""Runtime wrapper: binds a synthesized controller module + an actions module
to the simulator's controller interface (reset / step)."""
import copy, importlib.util, json, os, sys, types
from sim.api import Command
from model.synth import synthesize

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_module_from_source(name, src):
    mod = types.ModuleType(name)
    exec(compile(src, name, "exec"), mod.__dict__)
    return mod


def load_module_from_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def repair_handshake(model):
    """Orig-R: the child's final step no longer returns to its initial step on '=1';
    it stays final until the parent acknowledges (synthesizer resets the child on re-entry)."""
    m = copy.deepcopy(model)
    parent_of = {}
    for g in m["grafcets"]:
        for s in g["steps"]:
            if s.get("macro"):
                parent_of[s["macro"]] = s["id"]
    for g in m["grafcets"]:
        fin = g.get("final")
        if not fin or g["id"] not in parent_of:
            continue
        for tr in g["transitions"]:
            if tr["from"] == [fin] and tr["cond"].strip() == "1":
                tr["cond"] = f"not X{parent_of[g['id']]}"
    return m


class Ctx(types.SimpleNamespace):
    pass


class MiatController:
    def __init__(self, model, actions, gen=None, name="miat"):
        self.model = model
        self.A = actions
        self.G = gen if gen is not None else load_module_from_source(name + "_gen", synthesize(model))
        self.trace = []

    def reset(self, start_stage="guidance"):
        G, m = self.G, self.model
        self.S = G.State()
        C = Ctx()
        for k, v in m.get("data", {}).items():
            setattr(C, k, copy.deepcopy(v))
        C.P = dict(m.get("controls", {}))
        C.start_stage = start_stage
        C.start_approach = start_stage == "approach"
        C.start_guidance = start_stage == "guidance"
        C.t = 0.0
        C.cmd = Command()
        C.status = "RUNNING"
        self.C = C
        G.reset_all(self.S, 0.0)
        sm = m.get("start_markings", {}).get(start_stage)
        if sm:                                   # forced initial situation (test harness)
            top = [g for g in m["grafcets"] if g["id"] == m["top"]][0]
            for s in top["steps"]:
                self.S.X[s["id"]] = 0
            for sid in sm:
                self.S.activate(sid, 0.0)
                st = [s for s in top["steps"] if s["id"] == sid][0]
                if st.get("macro"):
                    getattr(G, "reset_" + st["macro"])(self.S, 0.0)
        self.trace = self.S.trace

    def step(self, obs):
        C = self.C
        C.t, C.obs = obs.t, obs
        C.tags, C.pallets, C.odom, C.scan = obs.tags, obs.pallets, obs.odom, obs.scan
        if self.model.get("cmd_policy", "zero") == "zero":
            C.cmd = Command()
        self.G.tick(self.S, C, self.A)
        c = C.cmd if C.cmd is not None else Command()
        return Command(c.v, c.w, C.status)

    def active_steps(self):
        return [k for k, v in self.S.X.items() if v]


def make_factory(model_path, actions_module, repair=False, controls_override=None, name="miat"):
    model = json.load(open(model_path))
    if repair:
        model = repair_handshake(model)
    if controls_override:
        model["controls"].update(controls_override)
    gen = load_module_from_source(name + "_gen", synthesize(model, os.path.basename(model_path)))
    return lambda: MiatController(model, actions_module, gen, name)
