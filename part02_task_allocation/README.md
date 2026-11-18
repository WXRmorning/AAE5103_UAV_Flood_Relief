# AAE5103: Resource Allocation and UAV Dispatch

Source code and Excel workbooks for Modules 03 and 04 of the course project, *AI-assisted UAV Locate-and-Supply Missions After Urban Floods*.

## Contents

- `module03_allocation/process_tables/`: two Excel workbooks implementing initial resource allocation and rolling replanning.
- `module04_dispatch/src/module04_train.py`: Python simulator, dispatch baselines, tabular Q-learning, and exhaustive reference search.
- `module04_dispatch/process_tables/`: three Excel workbooks documenting baseline comparisons, deadline assumptions, and training visualisations.
- `module04_dispatch/requirements.txt`: Python dependency specification.

Module 03 uses spreadsheet formulas. No separate Module 03 Python implementation is included. The workbooks are process records: editing them does not change or retrain the Python model. Python scenario inputs are defined in the script. Workbooks include intermediate comparison stages; they need not all show the final policy.

## Run Module 04

Use Python 3.11 or later with the pinned dependency. From the repository root:

```bash
cd module04_dispatch
python -m pip install -r requirements.txt
python src/module04_train.py
```

The script compares four dispatch references, enumerates 1,260 distinct hub sequences, and trains Q-learning for 30,000 episodes. It creates `module04_dispatch/results/module04_ai_results.json` when run. No API key, external dataset or supplied result file is required. Figures are not automatically regenerated.

## Scenario and expected results

The simulated initial stock is 100 L. Initial commitments reserve 40 L for A and 20 L for B; these are not confirmed deliveries. An urgent request from C arrives at 10:15. Only the remaining 40 L is allocated to the next dispatch episode: A 13.09 L, B 8.73 L, C 18.18 L. With two UAVs and a maximum load of 5 L per trip, this gives 3/2/4 trips. Both UAVs are assumed available at 10:15.

With seed 11, the Q-learning policy has total reward -441, first arrivals A/B/C of 3/25/7 minutes, and completion including return at 50 minutes. First-delivery targets are 15/25/10 minutes. The supplied deadline-aware reference completes in 52 minutes. The exhaustive optimum has the same reward under the same UAV-assignment rule. Tied exhaustive sequences may differ between runs.

## Scope and limitations

All quantities, distances and time targets are simulation assumptions, not measured Zhengzhou rescue performance. Earliest-available-UAV assignment is fixed. Hub B has no first-delivery time buffer. Battery use, weather, loading and turnaround time are not modelled. The package covers Modules 03 and 04 only, not the entire group project.

Prepared with AI assistance. Consult the course disclosure requirements when submitting. No open-source licence is assigned by this package.
