## User message 1

You are a robotics software engineer working in a model-driven (MIAT) flow. Your working folder is:
/workspace/M_sonnet_3

The system architect has frozen the requirements (REQUIREMENTS.md), the platform API (sim/api.py), the GRAFCET behaviour model (model/miat2_grafcet.json) and the Action interface contracts (ACTION_CONTRACTS.md). The controller has already been synthesized from the model by fixed rules (controller_synthesized.py, read-only).

Task: implement every Action function specified in ACTION_CONTRACTS.md in ONE module `actions.py` in your folder. Then run `python3 test_contract.py actions.py test_vectors.pkl` inside your folder and fix your code until all vectors pass.

Rules:
- Use only `math` and the types/constants from `sim.api`. Do not modify the model, the synthesized controller, the tests or the vectors.
- Work ONLY inside your folder. Do not open, list or search any other directory or file on this machine (this is an isolated benchmark; reading other folders invalidates it).
- Do the whole job yourself; do not ask questions.

When finished, reply with: the final test_contract.py summary line, how many test runs you needed until everything passed, and a summary of at most 80 words.

## User message 2

The coordinator sent a message while you were working:
A change request has been approved. Read CR1.md in your folder. The architect has already updated the model (model/miat2_cr_grafcet.json: new concurrent Sub-GRAFCET GS / IDEF0 A6, extended abort conditions in G0) and re-synthesized the controller (controller_synthesized_cr1.py, read-only). The updated contracts are in ACTION_CONTRACTS_CR1.md (two new Actions: as1_obstacle_monitor, as2_inhibit_forward).

Task: create `actions_v2.py` in your folder = your actions.py plus the two new Actions (change existing functions only if really needed). It must pass both `python3 test_contract.py actions_v2.py test_vectors.pkl` and `python3 test_contract.py actions_v2.py test_vectors_cr1.pkl`. Same rules as before (only your folder). Reply with both summary lines, the number of test runs, and which existing functions (if any) you changed.

Address this before completing your current task.

