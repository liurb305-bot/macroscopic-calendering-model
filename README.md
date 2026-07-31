# Macroscopic Calendering Model

This repository contains Abaqus files for the macroscopic lithium-ion electrode
calendering models built in `E:\abaqus`.

## Contents

- `3D2/`: wide electrode 1/8 equivalent static rolling model, Abaqus input/CAE
  files, completed ODB, postprocessing scripts, and extracted result figures.
- `3D/`: visible roll + electrode assembly models for later continuous rolling
  setup, including a true-size model and a clear display model.
- `3D3/`: original full-size 3D 10% thickness-reduction roll-press model,
  including the build script, CAE/INP files, smoke input, and rolling-force
  extraction script.
- `3D辊压刚性辊子/`: V3p0 3D double-coat differential-speed roll-press model
  with analytic rigid rollers, including the build script, CAE, INP, and run
  notes.
- `latest_3d_s030_self_support_50mm/`: latest self-supporting film 3D DPC
  calendering model with deformable rollers, `S=0.3` cap hardening, 50 mm sheet
  length, 10 mm rolling stability-check setup, postprocessing scripts, and
  extracted thickness/roller-force summaries.
- `quick_3d_rigid_roller_2mm/`: short 3D rigid-roller verification model for a
  2 mm rolling distance, used to check contact, rebound, and thickness
  extraction quickly.
- `dpc_parameter_fitting_2d_selected/`: selected 2D DPC cap-hardening fitting
  and sensitivity files used to screen candidate parameters before transfer to
  3D models.
- `3Dceshi3.1/`: local static self-supporting film DPC press model with rigid
  local roller arc shell surfaces, including 10% and 20% thickness-reduction
  CAE/INP files, build scripts, postprocessing scripts, and extracted 10% CSV
  results.

## Notes

- Units are mm, N, MPa, and tonne.
- The `3D2` model includes a completed Abaqus/Standard result database
  `wide_roll_1_8.odb`.
- The `3D` visible models are geometry/assembly bases and do not include a
  formal analysis step.
- Newly added model folders intentionally exclude large Abaqus solver output
  databases and scratch files (`.odb`, `.abq`, `.pac`, `.stt`, etc.). Regenerate
  them from the included `.cae`, `.inp`, and build/postprocess scripts.
