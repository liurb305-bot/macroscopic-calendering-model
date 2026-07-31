# -*- coding: ascii -*-
"""
Build a 2D DPC roll-press fitting variant.

Usage:
    abaqus cae noGUI=build_roll_press_2d_fit_variant.py -- TAG SCALE

SCALE multiplies the original DPC cohesion, initial cap position and all cap
hardening hydrostatic yield pressures.  Geometry, mesh, roller speeds, roll
gap, friction and other model controls are inherited from the latest E6500
2D roll verification template.
"""

import os
import sys
import build_roll_press_2d_fast_long135_E50Nu015 as base


BASE_CAP_HARDENING = (
    (60.0, 0.00),
    (62.0, 0.05),
    (66.0, 0.10),
    (72.0, 0.15),
    (80.0, 0.18),
    (90.0, 0.20),
    (110.0, 0.22),
    (140.0, 0.24),
    (190.0, 0.26),
    (270.0, 0.28),
    (390.0, 0.30),
    (480.0, 0.31),
    (600.0, 0.32),
    (740.0, 0.33),
)


def main():
    args = sys.argv
    if "--" in args:
        args = args[args.index("--") + 1:]
    else:
        args = []
    if len(args) >= 2:
        tag = args[0]
        scale = float(args[1])
    else:
        tag = os.environ.get("FIT_TAG", "")
        scale_text = os.environ.get("FIT_SCALE", "")
        if not tag or not scale_text:
            raise RuntimeError("Expected arguments: TAG SCALE or env FIT_TAG/FIT_SCALE")
        scale = float(scale_text)

    base.MODEL_NAME = "RollPress_2D_DPC_RollFit_E6500_%s" % tag
    base.JOB_NAME = base.MODEL_NAME

    # Elastic constants intended for transfer to the later 3D roll model.
    base.ACTIVE_E = 6500.0
    base.ACTIVE_NU = 0.01

    beta = float(os.environ.get("FIT_BETA", "65.0"))

    # Keep DPC shape parameters unchanged unless FIT_BETA is supplied; the cap
    # pressure level is scaled for the fitting sweep.
    base.DPC_COHESION = 4.0 * scale
    base.DPC_FRICTION_ANGLE = beta
    base.DPC_ECCENTRICITY = 0.8
    base.DPC_INITIAL_CAP_POSITION = 60.0 * scale
    base.DPC_TRANSITION_SURFACE_RADIUS = 0.02
    base.DPC_FLOW_STRESS_RATIO = 1.0
    base.DPC_CAP_HARDENING = tuple((p * scale, ev)
                                   for p, ev in BASE_CAP_HARDENING)

    base.main()


if __name__ == "__main__":
    main()
