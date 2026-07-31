# Macroscopic Calendering Model

This repository contains Abaqus files for the macroscopic lithium-ion electrode
calendering models built in `E:\abaqus`.

## Contents

- `3D2/`: wide electrode 1/8 equivalent static rolling model, Abaqus input/CAE
  files, completed ODB, postprocessing scripts, and extracted result figures.
- `3D/`: visible roll + electrode assembly models for later continuous rolling
  setup, including a true-size model and a clear display model.

## Notes

- Units are mm, N, MPa, and tonne.
- The `3D2` model includes a completed Abaqus/Standard result database
  `wide_roll_1_8.odb`.
- The `3D` visible models are geometry/assembly bases and do not include a
  formal analysis step.
- Abaqus replay, journal, cache, command, and temporary files are intentionally
  ignored.
