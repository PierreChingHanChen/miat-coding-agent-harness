import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import *
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from sim.dock_sim import CAM_OFF
C=dict(blue="#2a78d6", orange="#eb6834", ink="#0b0b0b", ink2="#52514e", grid="#e4e3df")
fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.7), constrained_layout=True, sharey=True)
for ax, (name, f) in zip(axs, [("Original design (re-implemented)", orig_factory()), ("MIAT 2.0 redesign", miat2_factory())]):
    ax.add_patch(plt.Rectangle((-0.6, -1.0), 1.2, 1.0, color="#d9d8d3"))
    ax.plot([0, 0], [0, 3.0], ls=":", lw=1, color=C["ink2"])
    for th in (45, 60, 75, 105, 120, 135):
        for s in range(2):
            r = run_trial(f, Scenario("g", guidance_start(th, 2.2), max_time=120), seed=777 + s + th, keep_traj=True)
            xs = [p[1] + CAM_OFF*math.cos(p[3]) for p in r["traj"]]; ys = [p[2] + CAM_OFF*math.sin(p[3]) for p in r["traj"]]
            col = C["blue"] if r["success"] else C["orange"]
            ax.plot(xs, ys, lw=1.2, color=col, alpha=0.9)
            ax.plot(xs[-1], ys[-1], marker="o" if r["success"] else "x", ms=5, color=col)
    ax.set_title(name, fontsize=9); ax.set_aspect("equal"); ax.set_xlim(-2.0, 2.0); ax.set_ylim(-1.05, 2.6)
    ax.set_xlabel("x [m] (pallet frame)")
axs[0].set_ylabel("y [m]")
axs[1].plot([], [], color=C["blue"], label="docked"); axs[1].plot([], [], color=C["orange"], label="failed / not docked")
axs[1].legend(loc="lower right", fontsize=7, frameon=False)
fig.savefig(os.path.join(ROOT, "figures", "fig_trajectories.png"), dpi=200)
print("ok")
