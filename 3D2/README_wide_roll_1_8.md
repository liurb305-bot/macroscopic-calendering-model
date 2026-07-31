# Wide Roll 1/8 Abaqus Model

This workspace contains a parameterized Abaqus/Standard model for the wide
electrode calendering case from Chapter 3.1.

## Files

- `build_wide_roll_1_8.py`: generates `wide_roll_1_8.inp`.
- `make_cae_from_inp.py`: imports the input file and saves `wide_roll_1_8.cae`.
- `postprocess_wide_roll_1_8.py`: extracts thickness and summary data from the ODB.
- `plot_wide_roll_results.py`: creates readable PNG plots from the extracted CSV files.

## Main Case

- Roll diameter: 900 mm.
- Roll length used for the 1/8 model: 1350 mm full width, 675 mm modeled half width.
- Electrode: coating/current collector/coating with total thickness 165 um.
- Current collector half thickness in the 1/8 model: 0.0075 mm.
- Single coating thickness in the 1/8 model: 0.075 mm.
- Coating width: 1300 mm full width, 650 mm modeled half width.
- Current collector width: 1350 mm full width, 675 mm modeled half width.
- Nominal reduction: 20%.
- Upper half-model roll displacement: 0.0165 mm.
- Friction coefficient: 0.15.

## Notes

The coating uses Abaqus modified Drucker-Prager/Cap plasticity.  The cap
hardening points are a runnable approximation matching the trend of Fig. 2-9
because the PDF does not provide the original table.  Replace the values under
`*Cap Hardening` in the generator when measured/tabulated data are available.

The roll is represented by a local deformable C3D8R surface layer following the
900 mm diameter roll curvature in the contact zone.  A small explicit center
flexure gap is included to reproduce the expected wider-roll trend of thicker
center and thinner coating edge in the 1/8 equivalent static model.

## Generated result files

- `wide_roll_1_8.cae`: Abaqus/CAE database.
- `wide_roll_1_8.inp`: Abaqus input deck.
- `wide_roll_1_8.odb`: completed result database.
- `wide_roll_1_8_thickness.csv`: transverse thickness at the roll-gap symmetry plane.
- `wide_roll_1_8_contact_pressure.csv`: nodal contact pressure values.
- `wide_roll_1_8_summary.txt`: run and result summary.
- `wide_roll_1_8_thickness_profile.png`: thickness curve.
- `wide_roll_1_8_contact_pressure.png`: contact pressure map.
- `wide_roll_1_8_mises.png` and `wide_roll_1_8_peeq.png`: Abaqus contour exports.
