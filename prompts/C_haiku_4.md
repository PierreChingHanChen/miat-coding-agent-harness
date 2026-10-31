## User message 1

You are a robotics software engineer. Your working folder is:
/workspace/C_haiku_4

It contains REQUIREMENTS.md, sim/api.py, ACTION_CONTRACTS.md (interface contracts of the datapath Action functions), test_contract.py and test_vectors.pkl.

Task:
1. Implement every Action function specified in ACTION_CONTRACTS.md in ONE module `actions.py`; run `python3 test_contract.py actions.py test_vectors.pkl` inside your folder and fix your code until all vectors pass.
2. Write `controller.py` with `class DockingController` (methods `reset(self, start_stage)` and `step(self, obs) -> Command`) that satisfies REQUIREMENTS.md by calling your Actions. You design the control logic (which Actions run when, transitions, timeouts, status reporting) yourself.

Rules: only `math`/standard library and `sim.api`; work ONLY inside your folder (do not open, list or search any other directory — isolated benchmark); no simulator is available, you may run python3 for checks; do not ask questions. Reply with the final test summary line, the number of test runs, and a design summary of at most 80 words.

