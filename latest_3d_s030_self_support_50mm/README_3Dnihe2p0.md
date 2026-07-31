# 3Dnihe2.0 V2p2 coarse-roller stability-check roll pressing model

## Model files

- Working directory: `E:\abaqus\3Dnihe2.0`
- Build script: `build_roll_press_3d.py`
- Model/job name: `RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2`
- Abaqus files:
  - `RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2.cae`
  - `RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2.inp`

## Key setup

- Sheet: single self-supporting active film, no collector, no tie.
- Sheet size: `50 mm x 100 mm x 0.150 mm`.
- Target reduction: `10%`, target thickness `0.135 mm`, upper roller drop `0.015 mm`.
- Roller type: deformable elastic rollers.
- Upper/lower line speed: `0.8 m/min` and `0.5 m/min`.
- Friction coefficient: `0.1` for both upper and lower roller-film contact.
- Staged motion: `Clamp_Down=0.05 s`, `Hold_Clamp=0.05 s`, `Rolling=0.75 s`.
- Rolling distance: `10 mm`, used as a stability check before extending to longer rolling.
- Mass scaling: whole-model fixed mass scaling target `dt=5.0e-6 s`.

## DPC material

Active film elastic data:

- Density: `2.55e-9 tonne/mm^3`
- Young's modulus: `6500 MPa`
- Poisson's ratio: `0.01`

DPC parameters:

- Cohesion: `4 MPa`
- Friction angle: `65 deg`
- Eccentricity: `0.8`
- Initial cap position: `18 MPa`
- Transition surface radius: `0.02`
- K: `1.0`

Cap hardening uses the original pressure table scaled by `S=0.3`:

```text
18.0, 0.00
18.6, 0.05
19.8, 0.10
21.6, 0.15
24.0, 0.18
27.0, 0.20
33.0, 0.22
42.0, 0.24
57.0, 0.26
81.0, 0.28
117.0, 0.30
144.0, 0.31
180.0, 0.32
222.0, 0.33
```

## Run commands

Generate CAE and INP:

```text
abaqus cae noGUI=build_roll_press_3d.py
```

Data check:

```text
abaqus job=RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2_datacheck input=RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2.inp datacheck cpus=6 domains=6 double=both interactive
```

Full stability-check run on a 16-core machine:

```text
abaqus job=RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2 input=RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2.inp cpus=16 domains=16 double=both interactive
```

Post-process after the ODB is complete:

```text
abaqus python extract_3d_roll_press_results.py RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2
```

## Acceptance checks

- The job should finish without deformation-speed/wave-speed fatal errors.
- Nip thickness should approach `135 um`.
- Downstream unloaded thickness can be evaluated only where `CPRESS` is approximately zero.
- Check `ALLKE/ALLIE`, `ALLPD`, `ALLAE`, `ALLMW`, and mass increase from `.sta`.
- If this 10 mm run is stable, extend rolling distance gradually to `25 mm` and then `50 mm`.

- V2p2 mesh update: roller global seed is `25 mm` and roller face seed is `10 mm` to reduce non-contact/internal roller tetrahedra.
