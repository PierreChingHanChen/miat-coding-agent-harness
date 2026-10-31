"""Gate-2 model checker for MIAT GRAFCET models (JSON).

Rules (MIAT 2.0 Gate 2 + IEC 60848 well-formedness):
  G1  structure: references valid, non-initial steps have an entry transition
  G2  every transition carries a condition
  G3  a time-consuming action must exit on a completion event, not on '=1'
  G4  OR divergence: conditions mutually exclusive
  G5  AND divergence has a matching AND convergence
  G6  every step reachable
  G7  no unbounded wait: a step that waits on events needs a timeout (or a bounded parent)
  G8  no unbounded loop: every cycle needs a bounded counter or a timed parent
  G9  parent/child handshake must not race (child final step must not auto-return)
  G10 every condition variable has a declared producer (Action output or system input)
  G11 cmd_vel ownership: at most one non-override producer active in any reachable marking
  G12 a safe-abort state exists and is reachable
  G13 traceability: every step maps to an IDEF0 activity; action names match the activity number
"""
import itertools, json, re, sys
from collections import defaultdict

TOK = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\b")
KW = {"and", "or", "not", "True", "False"}


def atoms(cond):
    return [w for w in TOK.findall(cond) if w not in KW]


def ev(cond, env):
    c = cond.strip()
    if c in ("1", "True"):
        return True
    if c in ("0", "False"):
        return False
    expr = TOK.sub(lambda m: m.group(1) if m.group(1) in KW else f"_e[{m.group(1)!r}]", c)
    return bool(eval(expr, {"_e": env}))


def assignments(names, limit=14):
    names = sorted(set(names))
    if len(names) > limit:
        raise ValueError("too many atoms")
    for vals in itertools.product([False, True], repeat=len(names)):
        yield dict(zip(names, vals))


class Model:
    def __init__(self, m):
        self.m = m
        self.G = {g["id"]: g for g in m["grafcets"]}
        self.step = {}
        self.gof = {}
        for g in m["grafcets"]:
            for s in g["steps"]:
                self.step[s["id"]] = s
                self.gof[s["id"]] = g["id"]
        self.parent = {}          # child grafcet -> macro step id
        for s in self.step.values():
            if s.get("macro"):
                self.parent[s["macro"]] = s["id"]
        self.produced = set(m.get("inputs", ["start_approach", "start_guidance"]))
        for g in m["grafcets"]:
            for a in g.get("continuous", []):
                self.produced |= set(a["out"])
            for s in g["steps"]:
                for a in s.get("actions", []):
                    self.produced |= set(a["out"])
        self.bounded = set(m.get("bounded_vars", []))

    def out_tr(self, sid):
        g = self.G[self.gof[sid]]
        return [t for t in g["transitions"] if sid in t["from"]]

    def finals(self, gid):
        g = self.G[gid]
        f = [s["id"] for s in g["steps"] if s.get("final")]
        if g.get("final"):
            f.append(g["final"])
        return f

    def parent_timed(self, gid):
        p = self.parent.get(gid)
        while p is not None:
            if self.step[p].get("timeout"):
                return True
            p = self.parent.get(self.gof[p])
        return False


def check(m):
    M = Model(m)
    F = []
    add = lambda rule, where, msg: F.append(dict(rule=rule, where=where, msg=msg))
    stats = dict(grafcets=len(M.G), steps=len(M.step), transitions=sum(len(g["transitions"]) for g in M.G.values()))
    # G1/G2
    for g in M.G.values():
        ids = {s["id"] for s in g["steps"]}
        entered = set()
        for t in g["transitions"]:
            for s in t["from"] + t["to"]:
                if s not in ids:
                    add("G1", t["id"], f"unknown step {s}")
            entered |= set(t["to"])
            if not t.get("cond", "").strip():
                add("G2", t["id"], "transition without condition")
        for s in g["steps"]:
            if s["id"] not in g["initial"] and s["id"] not in entered:
                add("G1", s["id"], "non-initial step without entry transition")
    # G3
    for sid, s in M.step.items():
        if s.get("timed") or s.get("blocking"):
            for t in M.out_tr(sid):
                if t["cond"].strip() in ("1", "True"):
                    add("G3", f"{t['id']} (X{sid})", "'=1' after a time-consuming action: completion event not modelled (relies on a blocking call)")
    # G4 exclusivity / G7 waits
    for sid, s in M.step.items():
        outs = M.out_tr(sid)
        if not outs:
            continue
        names = set()
        for t in outs:
            names |= set(atoms(t["cond"]))
        taut = False
        try:
            excl_ok = True
            taut = True
            for env in assignments(names):
                fired = [t["id"] for t in outs if ev(t["cond"], env)]
                if len(fired) > 1:
                    excl_ok = False
                    bad = fired
                if not fired:
                    taut = False
            if not excl_ok:
                add("G4", f"X{sid}", f"OR divergence not mutually exclusive ({', '.join(bad)} can fire together)")
        except ValueError:
            add("G4", f"X{sid}", "too many atoms to check")
        if not taut and not s.get("timeout") and not s.get("completes_within") and not s.get("monitor") and not M.parent_timed(M.gof[sid]):
            add("G7", f"X{sid}", "waits for an event without timeout or bounded parent")
    # G5
    for g in M.G.values():
        for t in g["transitions"]:
            if len(t["to"]) > 1:
                if not any(set(t2["from"]) == set(t["to"]) for t2 in g["transitions"]):
                    add("G5", t["id"], "AND divergence without matching convergence")
    # G6 reachability + G8 loops
    for g in M.G.values():
        succ = defaultdict(set)
        for t in g["transitions"]:
            for a in t["from"]:
                for b in t["to"]:
                    succ[a].add((b, t["id"]))
        seen, stack = set(g["initial"]), list(g["initial"])
        while stack:
            a = stack.pop()
            for b, _ in succ[a]:
                if b not in seen:
                    seen.add(b); stack.append(b)
        for s in g["steps"]:
            if s["id"] not in seen:
                add("G6", s["id"], "unreachable step")
        # cycles via SCC (Tarjan)
        idx, low, on, st, sccs, c = {}, {}, set(), [], [], [0]
        def strong(v):
            idx[v] = low[v] = c[0]; c[0] += 1; st.append(v); on.add(v)
            for w, _ in succ[v]:
                if w not in idx:
                    strong(w); low[v] = min(low[v], low[w])
                elif w in on:
                    low[v] = min(low[v], idx[w])
            if low[v] == idx[v]:
                comp = []
                while True:
                    w = st.pop(); on.discard(w); comp.append(w)
                    if w == v:
                        break
                sccs.append(comp)
        for s in g["steps"]:
            if s["id"] not in idx:
                strong(s["id"])
        for comp in sccs:
            if len(comp) < 2:
                continue
            trs = [t for t in g["transitions"] if set(t["from"]) & set(comp) and set(t["to"]) & set(comp)]
            if any(set(atoms(t["cond"])) & M.bounded for t in trs):
                continue
            if g["id"] in M.parent and M.parent_timed(g["id"]):
                continue
            if g.get("supervisor"):
                continue      # supervisor monitor/hold cycle: accepted deviation (escalation via timeout to S9)
            # a pure module cycle start->...->final->start that is only re-entered by the parent is not a loop
            if g["id"] in M.parent and all(t["cond"].strip() in ("1",) or t["from"][0] in M.finals(g["id"]) for t in trs if t["from"][0] in M.finals(g["id"])):
                inner = [x for x in comp if x not in M.finals(g["id"]) and x not in g["initial"]]
                sub = [t for t in trs if not (set(t["from"]) & set(M.finals(g["id"])))]
                # check whether a cycle remains when the final->initial return is removed
                succ2 = defaultdict(set)
                for t in sub:
                    for a in t["from"]:
                        for b in t["to"]:
                            succ2[a].add(b)
                def has_cycle():
                    col = {}
                    def dfs(u):
                        col[u] = 1
                        for v in succ2[u]:
                            if v in comp and (col.get(v) == 1 or (v not in col and dfs(v))):
                                return True
                        col[u] = 2
                        return False
                    return any(dfs(u) for u in comp if u not in col)
                if not has_cycle():
                    continue
            add("G8", f"{g['id']}:{{{','.join(sorted(comp))}}}", "loop without bounded counter or timed parent (possible livelock)")
    # G9 handshake race
    for g in M.G.values():
        for t in g["transitions"]:
            for x in re.findall(r"\bX(\w+)\b", t["cond"]):
                if x in M.step and M.gof[x] != g["id"]:
                    for t2 in M.out_tr(x):
                        if t2["cond"].strip() in ("1", "True"):
                            add("G9", f"{t['id']} ↔ {t2['id']}", f"parent waits for X{x} but X{x} leaves on '=1' in the same scan: completion can be missed unless the flag is produced earlier (binding not specified)")
    # G10 producers
    for g in M.G.values():
        for t in g["transitions"]:
            for a in atoms(t["cond"]):
                if re.fullmatch(r"X\w+", a) and a[1:] in M.step:
                    continue
                if re.fullmatch(r"TO\w+", a) and a[2:] in M.step:
                    if not M.step[a[2:]].get("timeout"):
                        add("G10", t["id"], f"{a} used but step has no timeout")
                    continue
                if a not in M.produced:
                    add("G10", t["id"], f"condition '{a}' has no declared producer")
    # G11 cmd ownership by token game (flattened)
    writers = {sid for sid, s in M.step.items() if any("cmd" in a["out"] for a in s.get("actions", [])) and not s.get("override")}
    top = m["top"]
    conc = m.get("concurrent", [])
    def running(mk):
        r = {top} | set(conc)
        changed = True
        while changed:
            changed = False
            for gid, p in M.parent.items():
                if p in mk and M.gof[p] in r and gid not in r:
                    r.add(gid); changed = True
        return r
    def reset_child(mk, gid):
        g = M.G[gid]
        mk = set(mk) - {s["id"] for s in g["steps"]}
        mk |= set(g["initial"])
        for s in g["steps"]:
            if s["id"] in g["initial"] and s.get("macro"):
                mk = reset_child(mk, s["macro"])
        return mk
    init = set()
    for gid in [top] + conc:
        init = reset_child(init, gid)
    init = frozenset(init)
    seen, stack, n_states, own_viol, dead = {init}, [init], 0, set(), []
    while stack and n_states < 200000:
        mk = stack.pop(); n_states += 1
        act = [s for s in mk if M.gof[s] in running(mk)]
        w = [s for s in act if s in writers]
        if len(w) > 1:
            own_viol.add(tuple(sorted(w)))
        any_succ = False
        for gid in running(mk):
            for t in M.G[gid]["transitions"]:
                if not all(s in mk for s in t["from"]):
                    continue
                names = [a for a in atoms(t["cond"]) if not (re.fullmatch(r"X\w+", a) and a[1:] in M.step)]
                env0 = {f"X{s}": (s in mk) for s in M.step}
                sat = any(ev(t["cond"], {**env0, **e}) for e in assignments(names))
                if not sat:
                    continue
                nm = set(mk) - set(t["from"]) | set(t["to"])
                for d in t["to"]:
                    if M.step[d].get("macro"):
                        nm = reset_child(nm, M.step[d]["macro"])
                nm = frozenset(nm)
                any_succ = True
                if nm not in seen:
                    seen.add(nm); stack.append(nm)
        terminal = any(M.step[s].get("status") for s in mk if M.gof[s] == top)
        if not any_succ and not terminal:
            dead.append(sorted(mk))
    for w in own_viol:
        add("G11", ",".join(w), "several cmd producers active simultaneously")
    for d in dead[:3]:
        add("G11", str(d), "deadlock marking (no transition can ever fire)")
    stats["reachable_markings"] = n_states
    # G12 safe abort
    if not any(s.get("status") == "ABORTED" for s in M.step.values()):
        add("G12", "model", "no safe-abort (ABORTED) state: failures cannot be reported, only waited out")
    # G13 traceability
    n_tr = 0
    for sid, s in M.step.items():
        if not s.get("idef0"):
            add("G13", sid, "step without IDEF0 activity")
            continue
        n_tr += 1
        nums = [re.sub(r"\D", "", x) for x in s["idef0"].split(",")]
        for a in ([] if s.get("status") else s.get("actions", [])):
            fnum = re.match(r"a(\w*?)_", a["fn"])
            if not fnum:
                continue
            fd = re.sub(r"\D", "", fnum.group(1))
            if not any(fd.startswith(n) or n.startswith(fd) for n in nums if n) and not (fnum.group(1).lower().startswith("s") and any(n.startswith("6") for n in nums)):
                add("G13", sid, f"action {a['fn']} does not match activity {s['idef0']}")
    stats["traceable_steps"] = f"{n_tr}/{len(M.step)}"
    return F, stats


if __name__ == "__main__":
    m = json.load(open(sys.argv[1]))
    F, st = check(m)
    print(json.dumps(st))
    by = defaultdict(list)
    for f in F:
        by[f["rule"]].append(f)
    for r in sorted(by, key=lambda x: int(x[1:])):
        print(f"{r}: {len(by[r])}")
        for f in by[r]:
            print(f"   {f['where']}: {f['msg']}")
