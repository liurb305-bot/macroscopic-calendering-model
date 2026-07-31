# Visible Roll + Electrode Abaqus Assembly

This folder contains a geometry-only Abaqus model intended as the starting point
for a later continuous calendering simulation.

## Model Intent

- Show an actual visible upper roll and the layered electrode in Abaqus/CAE.
- Keep the coordinate convention used in the PDF:
  - X: transverse direction / roll axis.
  - Y: normal direction.
  - Z: rolling direction.
- Do not run the model yet and do not create contact, loads, motion, or an
  analysis step.

## Geometry

- Upper roll: analytical rigid cylindrical surface, radius 450 mm, diameter
  900 mm, axis along X, half-width length 675 mm.
- Current collector: X = 0 to 675 mm, Y = 0 to 0.0075 mm,
  Z = -40 to 40 mm.
- Coating: X = 0 to 650 mm, Y = 0.0075 to 0.0825 mm,
  Z = -40 to 40 mm.
- The roll is initially tangent to the coating top surface at the roll-gap
  center and is not pre-penetrated.

## Named Items For Later Simulation

- `RP_ROLL`: roll reference point. Use this to add rotation and vertical
  displacement later.
- `S_ROLL`: analytical roll surface.
- `S_COATING_TOP`: coating top surface.
- `SET_SYM_X0`: transverse symmetry face.
- `SET_SYM_Y0`: bottom symmetry face.
- `SET_ELECTRODE_INLET`: electrode inlet end.
- `SET_ELECTRODE_OUTLET`: electrode outlet end.

## Files

- `build_visible_roll_electrode.py`: Abaqus/CAE script that builds the model.
- `visible_roll_electrode.cae`: generated CAE database.
- `visible_roll_electrode.inp`: generated input deck for inspection/import.
- `visible_roll_electrode_preview.png`: generated preview image if CAE image
  export succeeds.
- `visible_roll_electrode_schematic.png`: PDF-style schematic preview.  This is
  not a simulation result; it only helps identify the roll/electrode layout.

## Notes

The roll is analytical rigid by design, so it is visible as a cylindrical
surface but has no roll mesh. If elastic roll bending is needed later, replace
`P_ROLL_ANALYTIC_RIGID` with a deformable solid roll or a locally meshed roll
surface.

The 20% half-model reduction displacement from the previous model is retained
in the build script as `REDUCTION_20PCT_HALF_DISP = 0.0165 mm`, but it is not
applied in this geometry-only version.

## If the viewport looks empty

Abaqus may open the default empty `Model-1` if it exists in an older database
session.  In the model selector, switch from `Model-1` to
`visible_roll_electrode`, then display `Assembly`.  The real geometry is under:

- `visible_roll_electrode` -> `Assembly` for the full roll/electrode assembly.
- `visible_roll_electrode` -> `Parts` -> `P_ROLL_ANALYTIC_RIGID`,
  `P_COATING`, and `P_COLLECTOR` for individual parts.
