"""Build a 1/8 wide electrode calendering Abaqus input model.

The model follows the approved plan for the Chapter 3.1 equivalent static
normal reduction case.  It writes an Abaqus/Standard input file with a
structured C3D8R orphan mesh so the mesh density and node/element sets are
fully explicit and easy to audit.
"""

from __future__ import print_function

import math
import os


JOB_NAME = "wide_roll_1_8"
OUT_INP = JOB_NAME + ".inp"


# Geometry and process parameters, mm, N, MPa, tonne.
ROLL_RADIUS = 450.0
ROLL_DIAMETER = 2.0 * ROLL_RADIUS
ROLL_LENGTH = 1350.0
COLLECTOR_WIDTH = 1350.0
COATING_WIDTH = 1300.0
HALF_COLLECTOR_WIDTH = COLLECTOR_WIDTH / 2.0
HALF_COATING_WIDTH = COATING_WIDTH / 2.0

COLLECTOR_HALF_THICKNESS = 0.0075
COATING_THICKNESS = 0.075
HALF_STACK_THICKNESS = COLLECTOR_HALF_THICKNESS + COATING_THICKNESS
TOTAL_STACK_THICKNESS = 2.0 * HALF_STACK_THICKNESS
REDUCTION = 0.20
HALF_REDUCTION_DISPLACEMENT = TOTAL_STACK_THICKNESS * REDUCTION / 2.0

FRICTION_COEFF = 0.15
CONTACT_HALF_LENGTH = math.sqrt(ROLL_RADIUS * TOTAL_STACK_THICKNESS * REDUCTION)
MODEL_Z_LENGTH = 8.0

# Equivalent roll-flexure gap: center has the largest opening and the free
# coating edge has zero extra gap.  This is kept as an explicit calibration
# value because the paper does not provide the full roll/bearing data.
ROLL_CENTER_FLEXURE_GAP = 0.004
ROLL_LAYER_THICKNESS = 2.0
ROLL_LAYER_DIVISIONS = 2


def graded_axis(segments, ndigits=6):
    """Return a sorted unique axis from (start, end, step) segments."""
    values = []
    for start, end, step in segments:
        n = int(round((end - start) / step))
        for i in range(n):
            values.append(round(start + i * step, ndigits))
        values.append(round(end, ndigits))
    out = []
    for value in values:
        if not out or abs(value - out[-1]) > 10 ** (-ndigits):
            out.append(value)
    return out


def chunks(values, size=16):
    for i in range(0, len(values), size):
        yield values[i : i + size]


def write_label_lines(handle, labels):
    for group in chunks(labels):
        handle.write(", ".join(str(v) for v in group) + "\n")


def face_surface_elset(handle, name, elset_name, face):
    handle.write("*Surface, type=ELEMENT, name=%s\n" % name)
    handle.write("%s, %s\n" % (elset_name, face))


def flexure_gap(x):
    if x >= HALF_COATING_WIDTH:
        return 0.0
    ratio = x / HALF_COATING_WIDTH
    return ROLL_CENTER_FLEXURE_GAP * (1.0 - ratio * ratio)


def roll_arc(z):
    return ROLL_RADIUS - math.sqrt(max(0.0, ROLL_RADIUS * ROLL_RADIUS - z * z))


def build_model():
    x_values = graded_axis(((0.0, HALF_COLLECTOR_WIDTH, 0.5),))
    z_values = graded_axis(((0.0, MODEL_Z_LENGTH, 0.4),))
    y_electrode = [
        0.0,
        COLLECTOR_HALF_THICKNESS,
        COLLECTOR_HALF_THICKNESS + 0.030,
        COLLECTOR_HALF_THICKNESS + 0.060,
        HALF_STACK_THICKNESS,
    ]
    y_roll = [
        i * ROLL_LAYER_THICKNESS / float(ROLL_LAYER_DIVISIONS)
        for i in range(ROLL_LAYER_DIVISIONS + 1)
    ]

    nodes = []
    electrode_nodes = {}
    roll_nodes = {}
    next_node = 1

    def add_node(key, xyz, target):
        nonlocal_next[0] += 1
        label = nonlocal_next[0] - 1
        target[key] = label
        nodes.append((label, xyz[0], xyz[1], xyz[2]))
        return label

    nonlocal_next = [next_node]

    coating_x_limit_index = max(i for i, x in enumerate(x_values) if x <= HALF_COATING_WIDTH)

    for k, z in enumerate(z_values):
        for j, y in enumerate(y_electrode):
            for i, x in enumerate(x_values):
                if j >= 2 and i > coating_x_limit_index:
                    continue
                add_node(("e", i, j, k), (x, y, z), electrode_nodes)

    roll_base_y = HALF_STACK_THICKNESS
    for k, z in enumerate(z_values):
        arc = roll_arc(z)
        for j, yoff in enumerate(y_roll):
            for i, x in enumerate(x_values):
                y = roll_base_y + flexure_gap(x) + arc + yoff
                add_node(("r", i, j, k), (x, y, z), roll_nodes)

    elements = []
    collector_elems = []
    coating_elems = []
    roll_elems = []
    coating_top_elems = []
    roll_bottom_elems = []
    next_elem = 1

    def add_element(nodes8, target_sets):
        nonlocal_elem[0] += 1
        label = nonlocal_elem[0] - 1
        elements.append((label,) + tuple(nodes8))
        for target in target_sets:
            target.append(label)
        return label

    nonlocal_elem = [next_elem]

    nx = len(x_values) - 1
    nz = len(z_values) - 1

    for k in range(nz):
        for i in range(nx):
            nodes8 = [
                electrode_nodes[("e", i, 0, k)],
                electrode_nodes[("e", i + 1, 0, k)],
                electrode_nodes[("e", i + 1, 1, k)],
                electrode_nodes[("e", i, 1, k)],
                electrode_nodes[("e", i, 0, k + 1)],
                electrode_nodes[("e", i + 1, 0, k + 1)],
                electrode_nodes[("e", i + 1, 1, k + 1)],
                electrode_nodes[("e", i, 1, k + 1)],
            ]
            add_element(nodes8, [collector_elems])

    for k in range(nz):
        for j in range(1, len(y_electrode) - 1):
            for i in range(coating_x_limit_index):
                nodes8 = [
                    electrode_nodes[("e", i, j, k)],
                    electrode_nodes[("e", i + 1, j, k)],
                    electrode_nodes[("e", i + 1, j + 1, k)],
                    electrode_nodes[("e", i, j + 1, k)],
                    electrode_nodes[("e", i, j, k + 1)],
                    electrode_nodes[("e", i + 1, j, k + 1)],
                    electrode_nodes[("e", i + 1, j + 1, k + 1)],
                    electrode_nodes[("e", i, j + 1, k + 1)],
                ]
                label = add_element(nodes8, [coating_elems])
                if j == len(y_electrode) - 2:
                    coating_top_elems.append(label)

    for k in range(nz):
        for j in range(ROLL_LAYER_DIVISIONS):
            for i in range(nx):
                nodes8 = [
                    roll_nodes[("r", i, j, k)],
                    roll_nodes[("r", i + 1, j, k)],
                    roll_nodes[("r", i + 1, j + 1, k)],
                    roll_nodes[("r", i, j + 1, k)],
                    roll_nodes[("r", i, j, k + 1)],
                    roll_nodes[("r", i + 1, j, k + 1)],
                    roll_nodes[("r", i + 1, j + 1, k + 1)],
                    roll_nodes[("r", i, j + 1, k + 1)],
                ]
                label = add_element(nodes8, [roll_elems])
                if j == 0:
                    roll_bottom_elems.append(label)

    def node_labels(predicate):
        return [label for label, x, y, z in nodes if predicate(label, x, y, z)]

    tol = 1.0e-7
    elec_x0 = node_labels(lambda label, x, y, z: x <= tol and y <= HALF_STACK_THICKNESS + tol)
    elec_y0 = node_labels(lambda label, x, y, z: y <= tol)
    elec_z0 = node_labels(lambda label, x, y, z: z <= tol and y <= HALF_STACK_THICKNESS + tol)
    roll_top = [
        roll_nodes[("r", i, ROLL_LAYER_DIVISIONS, k)]
        for k in range(len(z_values))
        for i in range(len(x_values))
    ]
    roll_x0 = [roll_nodes[("r", 0, j, k)] for k in range(len(z_values)) for j in range(len(y_roll))]
    roll_z0 = [roll_nodes[("r", i, j, 0)] for j in range(len(y_roll)) for i in range(len(x_values))]
    roll_top_center = [roll_nodes[("r", 0, ROLL_LAYER_DIVISIONS, 0)]]
    thickness_line = [
        electrode_nodes[("e", i, len(y_electrode) - 1, 0)]
        for i in range(coating_x_limit_index + 1)
    ]
    coating_top_nodes = [
        electrode_nodes[("e", i, len(y_electrode) - 1, k)]
        for k in range(len(z_values))
        for i in range(coating_x_limit_index + 1)
    ]

    metadata = {
        "x_nodes": len(x_values),
        "z_nodes": len(z_values),
        "nodes": len(nodes),
        "elements": len(elements),
        "collector_elements": len(collector_elems),
        "coating_elements": len(coating_elems),
        "roll_elements": len(roll_elems),
    }

    model = {
        "nodes": nodes,
        "elements": elements,
        "sets": {
            "E_COLLECTOR": collector_elems,
            "E_COATING": coating_elems,
            "E_ROLL": roll_elems,
            "E_COATING_TOP_SURF": coating_top_elems,
            "E_ROLL_BOTTOM_SURF": roll_bottom_elems,
            "ELEC_X0": elec_x0,
            "ELEC_Y0": elec_y0,
            "ELEC_Z0": elec_z0,
            "ROLL_TOP": roll_top,
            "ROLL_X0": roll_x0,
            "ROLL_Z0": roll_z0,
            "ROLL_TOP_CENTER": roll_top_center,
            "THICKNESS_LINE": thickness_line,
            "COATING_TOP_NODES": coating_top_nodes,
        },
        "metadata": metadata,
    }
    return model


def write_input(path, model):
    inst = "WIDE_ROLL_MESH-1"
    with open(path, "w") as f:
        f.write("*Heading\n")
        f.write("Wide electrode calendering 1/8 equivalent static model\n")
        f.write("** Generated by build_wide_roll_1_8.py\n")
        f.write("** Roll diameter: %.3f mm, coating width: %.3f mm, collector width: %.3f mm\n" % (ROLL_DIAMETER, COATING_WIDTH, COLLECTOR_WIDTH))
        f.write("** Reduction: %.3f, half reduction displacement: %.6f mm\n" % (REDUCTION, HALF_REDUCTION_DISPLACEMENT))
        f.write("** Contact half length estimate: %.6f mm, model z length: %.6f mm\n" % (CONTACT_HALF_LENGTH, MODEL_Z_LENGTH))
        f.write("** Center flexure gap: %.6f mm\n" % ROLL_CENTER_FLEXURE_GAP)
        f.write("*Preprint, echo=NO, model=NO, history=NO, contact=NO\n")
        f.write("**\n** PARTS\n**\n")
        f.write("*Part, name=WIDE_ROLL_MESH\n")
        f.write("*Node\n")
        for label, x, y, z in model["nodes"]:
            f.write("%d, %.8f, %.8f, %.8f\n" % (label, x, y, z))
        f.write("*Element, type=C3D8R\n")
        for elem in model["elements"]:
            f.write("%d, %d, %d, %d, %d, %d, %d, %d, %d\n" % elem)

        for name in ("E_COLLECTOR", "E_COATING", "E_ROLL", "E_COATING_TOP_SURF", "E_ROLL_BOTTOM_SURF"):
            f.write("*Elset, elset=%s\n" % name)
            write_label_lines(f, model["sets"][name])

        f.write("*Solid Section, elset=E_COLLECTOR, material=COLLECTOR\n")
        f.write(",\n")
        f.write("*Solid Section, elset=E_COATING, material=COATING_DPC\n")
        f.write(",\n")
        f.write("*Solid Section, elset=E_ROLL, material=ROLL_STEEL\n")
        f.write(",\n")
        f.write("*End Part\n")

        f.write("**\n** ASSEMBLY\n**\n")
        f.write("*Assembly, name=ASSEMBLY\n")
        f.write("*Instance, name=%s, part=WIDE_ROLL_MESH\n" % inst)
        f.write("*End Instance\n")

        for name in ("E_COLLECTOR", "E_COATING", "E_ROLL"):
            f.write("*Elset, elset=%s, instance=%s\n" % (name, inst))
            write_label_lines(f, model["sets"][name])

        for name in ("ELEC_X0", "ELEC_Y0", "ELEC_Z0", "ROLL_TOP", "ROLL_X0", "ROLL_Z0", "ROLL_TOP_CENTER", "THICKNESS_LINE", "COATING_TOP_NODES"):
            f.write("*Nset, nset=%s, instance=%s\n" % (name, inst))
            write_label_lines(f, model["sets"][name])

        f.write("*Surface, type=ELEMENT, name=S_COATING_TOP\n")
        f.write("%s.E_COATING_TOP_SURF, S5\n" % inst)
        f.write("*Surface, type=ELEMENT, name=S_ROLL_BOTTOM\n")
        f.write("%s.E_ROLL_BOTTOM_SURF, S3\n" % inst)
        f.write("*End Assembly\n")

        f.write("**\n** MATERIALS\n**\n")
        f.write("*Material, name=ROLL_STEEL\n")
        f.write("*Density\n")
        f.write("7.93e-09,\n")
        f.write("*Elastic\n")
        f.write("193000., 0.2\n")
        f.write("*Material, name=COLLECTOR\n")
        f.write("*Density\n")
        f.write("2.7e-09,\n")
        f.write("*Elastic\n")
        f.write("72000., 0.33\n")
        f.write("*Material, name=COATING_DPC\n")
        f.write("*Density\n")
        f.write("2.55e-09,\n")
        f.write("*Elastic\n")
        f.write("6500., 0.01\n")
        f.write("*Cap Plasticity\n")
        f.write("65., 4., 0.8, 0.01\n")
        f.write("*Cap Hardening\n")
        for pressure, epv in (
            (55.0, 0.000),
            (58.0, 0.050),
            (62.0, 0.100),
            (72.0, 0.150),
            (95.0, 0.200),
            (140.0, 0.230),
            (220.0, 0.260),
            (370.0, 0.290),
            (740.0, 0.325),
        ):
            f.write("%.6f, %.6f\n" % (pressure, epv))

        f.write("**\n** INTERACTIONS\n**\n")
        f.write("*Surface Interaction, name=ROLL_COATING_CONTACT\n")
        f.write("*Surface Behavior, pressure-overclosure=HARD\n")
        f.write("*Friction\n")
        f.write("%.6f,\n" % FRICTION_COEFF)
        f.write("*Contact Pair, interaction=ROLL_COATING_CONTACT, type=SURFACE TO SURFACE\n")
        f.write("S_COATING_TOP, S_ROLL_BOTTOM\n")

        f.write("**\n** INITIAL BOUNDARY CONDITIONS\n**\n")
        f.write("*Boundary\n")
        f.write("ELEC_X0, 1, 1\n")
        f.write("ELEC_Y0, 2, 2\n")
        f.write("ELEC_Z0, 3, 3\n")
        f.write("ROLL_X0, 1, 1\n")
        f.write("ROLL_Z0, 3, 3\n")
        f.write("ROLL_TOP, 1, 1\n")
        f.write("ROLL_TOP, 3, 3\n")

        f.write("**\n** STEP\n**\n")
        f.write("*Step, name=ROLL_DOWN_20PCT, nlgeom=YES, inc=300\n")
        f.write("*Static, stabilize=2e-4\n")
        f.write("0.02, 1.0, 1e-08, 0.05\n")
        f.write("*Boundary\n")
        f.write("ROLL_TOP, 2, 2, -%.8f\n" % HALF_REDUCTION_DISPLACEMENT)
        f.write("*Restart, write, frequency=0\n")
        f.write("*Output, field, variable=PRESELECT, number interval=1\n")
        f.write("*Node Output, nset=THICKNESS_LINE\n")
        f.write("U, RF\n")
        f.write("*Node Output, nset=ROLL_TOP\n")
        f.write("U, RF\n")
        f.write("*Element Output, elset=E_COATING, directions=YES\n")
        f.write("S, E, PE, PEEQ\n")
        f.write("*Element Output, elset=E_COLLECTOR, directions=YES\n")
        f.write("S, E\n")
        f.write("*Contact Output, variable=PRESELECT\n")
        f.write("*Output, history, frequency=1\n")
        f.write("*Node Output, nset=ROLL_TOP_CENTER\n")
        f.write("U2, RF2\n")
        f.write("*End Step\n")


def main():
    model = build_model()
    out_path = os.path.abspath(OUT_INP)
    write_input(out_path, model)
    print("Wrote %s" % out_path)
    for key in sorted(model["metadata"]):
        print("%s: %s" % (key, model["metadata"][key]))
    print("nominal_half_reduction_displacement_mm: %.8f" % HALF_REDUCTION_DISPLACEMENT)
    print("contact_half_length_estimate_mm: %.8f" % CONTACT_HALF_LENGTH)
    print("center_flexure_gap_mm: %.8f" % ROLL_CENTER_FLEXURE_GAP)


if __name__ == "__main__":
    main()
