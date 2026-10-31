## User message 1

You are a robotics software engineer working in a model-driven (MIAT) flow. Your working folder is:
/workspace/N_sonnet_4

The architect has frozen the requirements (REQUIREMENTS.md), the platform API (sim/api.py) and the GRAFCET behaviour model (model/miat2_grafcet.json). The controller has been synthesized from the model by fixed rules (controller_synthesized.py, read-only). ACTION_LIST.md lists the Action functions the controller calls (names, inputs, outputs). There is no further behavioural specification and no test suite.

Task: implement every function listed in ACTION_LIST.md in ONE module `actions.py` in your folder so that the synthesized controller satisfies the requirements. You may write and run your own checks with python3.

Rules: only `math` and `sim.api`; do not modify the model or the synthesized controller; work ONLY inside your folder (do not open, list or search any other directory — isolated benchmark); no simulator is available; do not ask questions. Reply with a summary of at most 80 words.

