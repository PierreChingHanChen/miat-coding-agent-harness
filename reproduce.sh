#!/usr/bin/env bash
# Regenerates every file in results/ and figures/ from scratch (about 15-20 minutes on 2 CPU cores).
set -euo pipefail
cd "$(dirname "$0")"
DM=D_haiku_1,D_haiku_2,D_haiku_3,D_haiku_4,D_sonnet_1,D_sonnet_2,D_sonnet_3,D_sonnet_4,M_haiku_1,M_haiku_2,M_haiku_3,M_haiku_4,M_sonnet_1,M_sonnet_2,M_sonnet_3,M_sonnet_4
python experiments/gate2_report.py
python experiments/handshake_race.py
python experiments/run_compare.py grid
python experiments/run_compare.py grid_abl
python experiments/run_compare.py e2e
python experiments/run_compare.py faults
python experiments/eval_generated.py v1
: > results/gen_v2.jsonl
python experiments/eval_generated.py v2 "$DM"
python experiments/eval_generated.py static
python experiments/gate3_binding.py
python experiments/mutation.py
python experiments/mutation_loc.py
python experiments/ablation_analysis.py
python experiments/analyze.py
echo "All results regenerated."
