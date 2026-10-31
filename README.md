# MIAT as a Harness for LLM Coding Agents — Replication Package

Data, code and generated programs for the manuscript

> C.-H. Chen, W.-T. Lin, et al., **"Model-Driven Harnesses for LLM Coding Agents: Contract-Bounded Generation under Synthesized Controllers,"** submitted to *IEEE Transactions on Software Engineering*, 2026.

The study asks whether a model-driven design method (MIAT: IDEF0 → GRAFCET → controller synthesis with contract-bounded datapath actions) can act as a *harness* for LLM coding agents. The case is a re-engineered ROS 2 AGV pallet-docking system [Chen & Lin, ICRAAI 2026]. Agents built on two models produced 48 independent implementations under six harness configurations. Every implementation was evaluated on 240 trials in a simulator that the agents never saw.

Everything needed to re-run the analysis is here: the harness artifacts, the synthesizer and model checker, the simulator, all 48 generated programs with their change-request revisions, sanitized agent transcripts, raw trial logs, and the analysis scripts. All result files in `results/` regenerate identically with the commands below; the JSONL logs match line for line, though line order can differ. This was checked by running `reproduce.sh` on a clean copy before release.

---

## Repository layout

| Path | Contents |
|---|---|
| `contracts/` | Harness documents given to the agents: requirements (`REQUIREMENTS.md`, raw version `REQUIREMENTS_RAW.md`), action contracts (`ACTION_CONTRACTS*.md`), signature list (`ACTION_LIST.md`), contract-test script and test vectors (`test_contract.py`, `test_vectors*.pkl`), and the scripts that generate them (`make_contracts.py`, `specs.py`). |
| `model/` | GRAFCET models in JSON: `original_grafcet.json` (recovered from the published system), `miat2_grafcet.json` (redesign), `miat2_cr_grafcet.json` (after change request CR-1). Also `synth.py` (rule-based controller synthesizer), `checker.py` (Gate M model checker) and `make_cr.py`. |
| `designs/` | Hand-written reference actions for the original and MIAT 2.0 models (`original_actions.py`, `miat2_actions.py`) and the runtime that binds a synthesized controller to an actions module (`runtime.py`). |
| `sim/` | AGV-DockSim surrogate simulator (`dock_sim.py`) and the platform interface given to agents (`api.py`). |
| `generated/` | The 48 agent-written programs, one folder per run, e.g. `M_haiku_3/`. Each folder holds exactly what the agent's workspace contained at the end, including helper scripts the agent wrote (see *Generated programs*). |
| `agent_logs/` | Sanitized transcripts of all 51 agent runs (`<run>.jsonl`) and a run table (`agent_runs.csv`). |
| `prompts/` | The task instruction(s) each agent received (`<run>.md`) and the CR-1 change-request payload (`cr1/`). |
| `experiments/` | Evaluation and analysis scripts (see *Reproducing the results*). |
| `results/` | Raw trial logs (`*.jsonl`), aggregated results (`summary.json`, `ablation.json`, …), model-checker reports (`gate2_*.txt`), fault-trial seeds (`fault_seeds.json`) and run logs (`logs/`). |
| `figures/` | Figures produced by the analysis scripts. |

---

## Study design in brief

### Arms

| Arm | Requirements | Model + synthesized controller | Contracts | Test vectors | Agent writes |
|---|---|---|---|---|---|
| **R** | raw | – | – | – | monolithic `controller.py` |
| **D** | full | – | – | – | monolithic `controller.py` |
| **C** | full | – | yes (phrased without a model) | yes | `controller.py` + `actions.py` |
| **N** | full | yes | signatures only | – | `actions.py` |
| **T** | full | yes | yes | – | `actions.py` |
| **M** | full | yes | yes | yes | `actions.py` (full harness) |

Arms D and M were run first, together with CR-1. Arms R, C, N and T were added afterwards to explain the D–M difference (see the paper's Threats to Validity).

### Agents

There are four independent runs per arm and model, giving 6 × 2 × 4 = 48 programs.

| Alias | Model identifier (from the transcripts) |
|---|---|
| `sonnet` | `claude-sonnet-5-5` |
| `haiku` | `claude-haiku-5-5` |

Each agent started in a fresh, empty workspace containing only its arm's documents and `sim/api.py`. It ran in the Claude Agent SDK with file and shell tools, and generation ran without human intervention. The evaluation simulator (`sim/dock_sim.py`) was **not** available to any agent.

Run start and end times, tool-call counts and contract-test histories are in `agent_logs/agent_runs.csv`.

### Evaluation suite

There are 240 trials per program, all with fixed seeds:

| Suite | Trials |
|---|---|
| Guidance grid: 7 angles (45°–135°) × 3 distances (2.2/2.5/2.8 m) × 8 seeds | 168 |
| End-to-end, simple scene | 20 |
| End-to-end, cluttered scene | 20 |
| Fault trials: tag occlusion, camera freeze, pose outliers, depth dropout (8 each) | 32 |

The 208 non-fault trials define a program's success rate. CR-1 programs are evaluated on 15 person-intrusion runs plus 104 regression trials.

### Rule-ID mapping

The model checker prints internal rule ids **G1–G13**. These are rules **C1–C13** in the paper (G1 = C1, …, G13 = C13). Stage gates in the companion methodology paper are also called G1–G3; they are unrelated to these rule ids.

---

## Reproducing the results

### Requirements

The scripts were tested with Python 3.13 and NumPy 2.5, SciPy 1.18 and Matplotlib 3.11. Install the dependencies with:

```bash
pip install -r requirements.txt
```

Everything runs on a laptop CPU. Parallelism is set in `experiments/par.py` (default: 2 processes). Run all commands from the repository root.

### Step by step

```bash
# 1. Model checking (Gate M) of the three models  -> results/gate2_*.txt
python experiments/gate2_report.py

# 2. Handshake race in the original model (paper Sec. IV-B) -> results/handshake_race.json
python experiments/handshake_race.py

# 3. Hand-written reference designs (paper Table III, Fig. 2)
python experiments/run_compare.py grid        # results/grid_main.jsonl      (~1 min)
python experiments/run_compare.py grid_abl    # results/grid_ablation.jsonl
python experiments/run_compare.py e2e         # results/e2e_main.jsonl
python experiments/run_compare.py faults      # results/faults.jsonl

# 4. Evaluate the 48 generated programs (paper Tables V-VI, Fig. 3)
python experiments/eval_generated.py v1       # results/gen_v1.jsonl  (11,520 trials, ~10 min)
python experiments/eval_generated.py static   # results/gen_static.json (LoC, complexity, CR-1 diffs)
python experiments/gate3_binding.py           # results/gate3_binding.json (Gate B)

# 5. Change request CR-1 (paper Table VIII) - D and M programs only
python experiments/eval_generated.py v2 D_haiku_1,D_haiku_2,D_haiku_3,D_haiku_4,D_sonnet_1,D_sonnet_2,D_sonnet_3,D_sonnet_4,M_haiku_1,M_haiku_2,M_haiku_3,M_haiku_4,M_sonnet_1,M_sonnet_2,M_sonnet_3,M_sonnet_4

# 6. Mutation analysis (paper Table VII)
python experiments/mutation.py                # results/mutation.json (~1 min)
python experiments/mutation_loc.py            # results/mutation_model_loc.json

# 7. Aggregation, statistics and figures
python experiments/ablation_analysis.py       # results/ablation.json, figures/fig_harness_ablation.png
python experiments/analyze.py                 # results/summary.json,  figures/fig_grid_heatmap.png
```

Or run everything with `bash reproduce.sh`.

Notes:

- `eval_generated.py v1|v2 <list>` **appends** to the result file when a list of programs is given. Without a list, it overwrites the file.
- The fault-trial seeds of the reported runs are pinned in `results/fault_seeds.json`. The original evaluation derived them from Python's randomized string hash, so they cannot be recomputed. New programs fall back to a stable CRC32-based seed.
- Exact Mann–Whitney p-values (full permutation enumeration) and Cliff's δ reported in the paper can be recomputed from the per-program success rates in `results/ablation.json`.

### Where each paper number comes from

| Paper item | File |
|---|---|
| Table II (rule violations in the original model) | `results/gate2_original.txt` |
| Handshake race (Sec. IV-B) | `results/handshake_race.json` |
| Table III, Fig. 2 (reference designs) | `results/summary.json` → `grid`, `e2e`, `faults` |
| Table V (RQ1), Table VI and Fig. 3 (RQ2) | `results/ablation.json`, `results/gen_static.json`, `results/gate3_binding.json` |
| Table VII (RQ3) | `results/mutation.json`, `results/mutation_model_loc.json` |
| Table VIII (RQ4) | `results/summary.json` → `gen.<run>.cr`, `results/gen_v2.jsonl` |
| First-attempt contract-test failures (Sec. VI-A) | `agent_logs/agent_runs.csv` → `contract_test_results` |

---

## Generated programs

Folder names are `<arm>_<model>_<run>`.

- **Delivered program:**
  - Arms R, D and C: `controller.py`. Arm C also has `actions.py`.
  - Arms N, T and M: `actions.py`, run under `controller_synthesized.py`, which is generated from the model and was read-only for the agent.
- **CR-1 revisions** (D and M only): `controller_v2.py` or `actions_v2.py`.
- **Other files:** scripts such as `check_sim.py`, `mytest.py` and `check/` were written by the agents for self-testing. They are kept as delivered.
- **Path sanitization:** absolute paths of the original sandbox, inside agent-written scripts and transcripts, were replaced with `/workspace/`.

The programs were evaluated exactly as delivered. For example, three `M_sonnet_*` programs omit `a5e_fail`. Gate B reports this, and it shows up as exceptions in camera-freeze fault trials.

## Agent transcripts

`agent_logs/<run>.jsonl` holds one JSON object per message:

```json
{"t": "<timestamp>", "role": "user|assistant", "content": [ ... ]}
```

Content blocks have the types `text`, `tool_use` (tool name and input) and `tool_result` (output, truncated at 20,000 characters).

Sanitization:

- Model reasoning ("thinking") blocks are not included.
- Sandbox paths are replaced with `/workspace/`.
- `D_haiku_{1,2,3}_cr1_fresh` are the CR-1 runs that used a fresh agent instance, because the original instance could not be resumed. All other CR-1 messages appear in the program's own transcript.

**Isolation audit.** We checked every tool call in the 51 transcripts for paths outside the agent's own workspace.

- No agent read another agent's folder, the reference implementation, or the simulator.
- Two agents (`R_sonnet_4`, `T_sonnet_3`) wrote throwaway test scripts to a shared temporary folder. They did not read anything from it.

---

## Limitations of this package

- AGV-DockSim is a surrogate calibrated against the detection data of the published system. It is not a replacement for the ROS 2/Gazebo set-up or the physical vehicle.
- The original ROS 2 source code of the published system is not part of this package. The original model in `model/original_grafcet.json` was reconstructed from the paper and the thesis.
- The manuscript itself is not included while it is under review.

## License

- **Code** (`sim/`, `model/`, `designs/`, `experiments/`, `contracts/*.py`, `reproduce.sh`): MIT License, see `LICENSE`.
- **Data and documents** (`results/`, `generated/`, `agent_logs/`, `prompts/`, `contracts/*.md`, `contracts/*.pkl`, `figures/`): Creative Commons Attribution 4.0, see `LICENSE-DATA`.

## Citation

If you use this package, please cite the paper (see `CITATION.cff`).

## Acknowledgment of AI use

The subject programs were generated by Claude models (Anthropic); generating them is the object of the study. Claude also assisted in developing the harness artifacts, the simulator, the checker, the synthesizer and the analysis scripts. The authors reviewed and verified all content.
