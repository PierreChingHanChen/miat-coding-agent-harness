"""CR-1: derive the safety-supervisor model from the MIAT 2.0 model (model change, then re-synthesis)."""
import json, copy, os
H = os.path.dirname(os.path.abspath(__file__))
m = json.load(open(os.path.join(H, "miat2_grafcet.json")))
cr = copy.deepcopy(m)
cr["system"] += " + CR-1 safety supervisor"
cr["concurrent"] = ["GS"]
cr["controls"].update(safe_dist=0.55, safe_sector_deg=30.0, clear_s=1.0)
cr["data"].update(sstate=None, obstacle_near=False, path_clear=True)
for g in cr["grafcets"]:
    if g["id"] == "G0":
        for t in g["transitions"]:
            if t["id"] in ("T0_4", "T0_7", "T0_10", "T0_13"):
                t["cond"] = f"({t['cond']}) or XS9"
            if t["id"] in ("T0_2", "T0_3", "T0_5", "T0_6", "T0_8", "T0_9", "T0_11", "T0_12"):
                t["cond"] = f"({t['cond']}) and not XS9"
for g in cr["grafcets"]:
    if g["id"] == "G0":   # Gate-2 (G11) found deadlock marking {X2, XS9}: add safety abort from X2
        g["transitions"].append({"id": "T0_14", "from": ["2"], "to": ["9"], "cond": "XS9"})
cr["grafcets"].append({"id": "GS", "idef0": "A6", "name": "Supervise path safety (CR-1)", "initial": ["S0"], "supervisor": True,
  "continuous": [{"fn": "as1_obstacle_monitor", "in": ["scan", "sstate", "t", "P"], "out": ["sstate", "obstacle_near", "path_clear"]}],
  "steps": [{"id": "S0", "idef0": "A61", "monitor": True, "actions": []},
            {"id": "S1", "idef0": "A62", "timeout": 20.0, "override": True, "actions": [{"fn": "as2_inhibit_forward", "in": ["cmd"], "out": ["cmd"]}]},
            {"id": "S9", "idef0": "A6", "final": True, "actions": []}],
  "transitions": [{"id": "TS_1", "from": ["S0"], "to": ["S1"], "cond": "obstacle_near"},
                  {"id": "TS_2", "from": ["S1"], "to": ["S0"], "cond": "path_clear"},
                  {"id": "TS_3", "from": ["S1"], "to": ["S9"], "cond": "TOS1 and not path_clear"}]})
json.dump(cr, open(os.path.join(H, "miat2_cr_grafcet.json"), "w"), indent=1)
print("CR model written")
