import json, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
S=json.load(open(os.path.join(ROOT,"results/summary.json")))
C=dict(blue="#2a78d6", orange="#eb6834", ink="#0b0b0b", ink2="#52514e", grid="#e4e3df")
groups=[("D_sonnet","Direct\nSonnet"),("D_haiku","Direct\nHaiku"),("M_sonnet","MIAT contract\nSonnet"),("M_haiku","MIAT contract\nHaiku")]
metrics=[("grid","Guidance grid (168 trials)"),("e2e_cluttered","End-to-end, cluttered (20)")]
fig,axs=plt.subplots(1,2,figsize=(7.2,2.9),constrained_layout=True,sharey=True)
for ax,(mk,title) in zip(axs,metrics):
    for gi,(g,lab) in enumerate(groups):
        vals=[v[mk]["p"]*100 for k,v in sorted(S["gen"].items()) if k.startswith(g)]
        col=C["orange"] if g.startswith("D") else C["blue"]
        xs=[gi+(i-1.5)*0.12 for i in range(len(vals))]
        ax.scatter(xs,vals,s=46,color=col,edgecolor="white",linewidth=1.5,zorder=3)
        m=sum(vals)/len(vals); ax.plot([gi-0.3,gi+0.3],[m,m],color=C["ink2"],lw=1.2,zorder=2)
        ax.text(gi+0.33,m,f"{m:.0f}%",va="center",fontsize=7.5,color=C["ink2"])
    ax.set_xticks(range(4)); ax.set_xticklabels([l for _,l in groups],fontsize=7.5)
    ax.set_title(title,fontsize=9); ax.set_ylim(-5,108); ax.grid(axis="y",color=C["grid"],lw=0.6); ax.set_axisbelow(True)
    for s in ("top","right"): ax.spines[s].set_visible(False)
axs[0].set_ylabel("Success per program [%]")
fig.savefig(os.path.join(ROOT,"figures","fig_llm_compare.png"),dpi=200); print("ok")
