# Nip thickness check

- Job: `RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2`
- Active instance: `ACTIVELAYER-1`
- Paired top/bottom nodes: 966
- Initial thickness: 150.000 um
- Target thickness: 135.000 um

## Key result

- Minimum actual thickness anywhere during run: 149.981139 um at Rolling t=0.206252 s
- Minimum actual thickness in |x|<=0.500 mm nip window: 149.981139 um (pairs=21, CPRESS max=7.868279e-02 MPa, step=Rolling, t=0.206252 s)
- Minimum actual thickness in |x|<=1.000 mm nip window: 149.981139 um (pairs=42, CPRESS max=7.868279e-02 MPa, step=Rolling, t=0.206252 s)
- Minimum actual thickness in |x|<=2.000 mm nip window: 149.981139 um (pairs=63, CPRESS max=7.868279e-02 MPa, step=Rolling, t=0.206252 s)
- Minimum actual thickness among CPRESS>1.0e-08 contact pairs: 149.981139 um (contact pairs=35, pair CPRESS max=7.868279e-02 MPa, step=Rolling, t=0.206252 s)

## Maximum CPRESS frame

- Global CPRESS max: 9.321902e-02 MPa at Rolling t=0.721876 s
- At that frame, all-pair actual min/avg/max: 149.990534 / 149.999998 / 150.005069 um
- At that frame, |x|<=0.500 mm actual min/avg: 149.994899 / 149.998881 um, pairs=21
- At that frame, |x|<=1.000 mm actual min/avg: 149.994899 / 150.000138 um, pairs=42
- At that frame, |x|<=2.000 mm actual min/avg: 149.994899 / 149.999648 um, pairs=63
- At that frame, contact-pair actual min/avg: 149.990534235388 / 149.99913369768385 um, pairs=17

## Interpretation

- The film was not compressed close to the 135 um target even inside the nip/contact frames.
- Therefore the final 150 um thickness is not mainly caused by severe rebound; it was almost never pressed down.
