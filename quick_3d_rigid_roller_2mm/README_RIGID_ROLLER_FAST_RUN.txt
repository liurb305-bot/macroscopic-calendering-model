Rigid roller 2 mm fast verification model
=========================================

Use this model on the other workstation:

  RollPress_3D_SelfSupport_DiffSpeed_DPC_S00008B5_50mm_Roll2mm_RigidRoller_Stable.cae
  RollPress_3D_SelfSupport_DiffSpeed_DPC_S00008B5_50mm_Roll2mm_RigidRoller_Stable.inp

Recommended command:

  abaqus job=RollPress_3D_SelfSupport_DiffSpeed_DPC_S00008B5_50mm_Roll2mm_RigidRoller_Stable input=RollPress_3D_SelfSupport_DiffSpeed_DPC_S00008B5_50mm_Roll2mm_RigidRoller_Stable.inp cpus=16 domains=16 double=both interactive

Main settings:

  Sheet size = 50 mm x 100 mm x 0.150 mm
  Target gap = 0.135 mm
  Upper roller drop = 0.015 mm
  Upper roller speed = 0.8 m/min = 13.333 mm/s
  Lower roller speed = 0.5 m/min = 8.333 mm/s
  Rolling distance target = about 2 mm by upper roller surface speed

  Clamp_Down = 0.005 s
  Hold_Clamp = 0.020 s
  Rolling = 0.150 s
  Rolling ramp = 0.010 s
  Total explicit step time = 0.175 s

  Rollers = discrete rigid cylindrical shell surfaces
  Film material = S00008B5 DPC fitted self-supporting active film
  Friction coefficient = 0.1 for all roller-film contact
  Mass scaling target dt = 5.0e-6 s

Local check:

  Data check completed with no fatal error.
  Remaining warning is not fatal:
    - rigid elements are ignored by mass scaling. This is expected because the
      rollers are rigid; the useful mass scaling acts on the deformable film.

This is a fast verification model. Use it to check contact, compaction trend,
stress/strain output, and model stability. It is not intended to evaluate
elastic roller deflection because the rollers are rigid.
