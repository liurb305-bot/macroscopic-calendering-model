# RollPress 3D V3p0 Analytic Rigid Roller

Working directory:

`E:\abaqus\3D5+3.0`

Main generated files:

- `build_roll_press_3d_v3p0_analytic_rigid.py`
- `RollPress_3D_DoubleCoat_DiffSpeed_DCP_V3p0_AnalyticRigid.cae`
- `RollPress_3D_DoubleCoat_DiffSpeed_DCP_V3p0_AnalyticRigid.inp`

This version uses analytic rigid upper and lower rollers. The roller solid
mesh, roller material section, end-face coupling, roller shoulder mesh, and
roller nip partition mesh are removed.

The active layer mesh keeps 23 elements through thickness. In the feed
direction, `x >= -120 mm` is the refined contact-passing region,
`-160 mm <= x < -120 mm` is the transition region, and `x < -160 mm` remains
coarse.

Reduced fixed global mass scaling is enabled in both explicit steps with target
time increment `1.0e-6 s`. This keeps the full explicit run below the
impractical increment count that occurs with mass scaling disabled.

The roller contact pairs use kinematic normal contact with friction unchanged.
The saved CAE job uses the double precision explicit executable.

Generate CAE/INP:

```powershell
abaqus cae noGUI=build_roll_press_3d_v3p0_analytic_rigid.py
```

Data check:

```powershell
abaqus job=RollPress_3D_DoubleCoat_DiffSpeed_DCP_V3p0_AnalyticRigid input=RollPress_3D_DoubleCoat_DiffSpeed_DCP_V3p0_AnalyticRigid.inp cpus=6 domains=6 double=both datacheck interactive
```

Full submit command:

```powershell
abaqus job=RollPress_3D_DoubleCoat_DiffSpeed_DCP_V3p0_AnalyticRigid input=RollPress_3D_DoubleCoat_DiffSpeed_DCP_V3p0_AnalyticRigid.inp cpus=6 domains=6 double=both interactive
```
