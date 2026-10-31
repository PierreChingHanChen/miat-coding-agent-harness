## User message 1

You are a robotics software engineer working in a model-driven (MIAT) flow. Your working folder is:
/workspace/T_haiku_3

The system architect has frozen the requirements (REQUIREMENTS.md), the platform API (sim/api.py), the GRAFCET behaviour model (model/miat2_grafcet.json) and the Action interface contracts (ACTION_CONTRACTS.md). The controller has already been synthesized from the model by fixed rules (controller_synthesized.py, read-only). No test vectors are available.

Task: implement every Action function specified in ACTION_CONTRACTS.md in ONE module `actions.py` in your folder. You may write and run your own checks with python3.

Rules: only `math` and `sim.api`; do not modify the model or the synthesized controller; work ONLY inside your folder (do not open, list or search any other directory — isolated benchmark); no simulator is available; do not ask questions. Reply with a summary of at most 80 words.

