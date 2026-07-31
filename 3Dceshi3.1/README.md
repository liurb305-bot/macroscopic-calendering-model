# 3Dceshi3.1

Local static 3D roll-press equivalent model for a single self-supporting dry
electrode film between rigid roller arc surfaces.

## Model scope

- Single self-supporting film only, no current collector and no three-layer
  coating/current-collector/coating structure.
- Abaqus/Standard Static General analysis.
- Local normal press-down, hold, and unload only.
- No roller rotation, no feeding, no differential speed, and no explicit
  dynamics.
- Units are mm, N, and MPa.

## Included files

- `create_selfsupport_dpc_local_static_press.py`: builds the 10% thickness
  reduction model, saves the CAE, and writes the INP.
- `postprocess_selfsupport_dpc_local_static_press.py`: postprocesses the 10%
  model ODB and writes CSV results.
- `SelfSupport_DPC_LocalStaticPress_Ch3_1.cae`: 10% reduction CAE model.
- `SelfSupport_DPC_LocalStaticPress_Ch3_1.inp`: 10% reduction input file.
- `create_selfsupport_dpc_local_static_press_20pct.py`: builds the 20%
  thickness reduction model.
- `postprocess_selfsupport_dpc_local_static_press_20pct.py`: postprocesses the
  20% model ODB and writes CSV results.
- `SelfSupport_DPC_LocalStaticPress_20pct_Ch3_1.cae`: 20% reduction CAE model.
- `SelfSupport_DPC_LocalStaticPress_20pct_Ch3_1.inp`: 20% reduction input file.
- `Job-1.inp`: input deck corresponding to the completed 10% trial job.
- `Job-1_results.csv`: extracted 10% trial-job summary.

Large Abaqus solver output databases such as `.odb` are intentionally not
included in Git. Regenerate them locally from the CAE/INP files when needed.
