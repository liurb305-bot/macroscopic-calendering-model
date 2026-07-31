"""Build a clear display CAE for the roll + electrode assembly.

This file is meant for visual inspection in Abaqus/CAE.  The electrode
thickness is magnified for display so the roll and layered electrode can be
seen together.  It does not create analysis steps, contact, or loads.
"""

from __future__ import print_function

import os

from abaqus import Mdb, mdb, session
from abaqusConstants import (
    C3D8R,
    CARTESIAN,
    DEFORMABLE_BODY,
    HEX,
    ON,
    PNG,
    S4R,
    STANDARD,
    STRUCTURED,
    THREE_D,
    UNION,
    XAXIS,
    XYPLANE,
    XZPLANE,
)
import mesh


MODEL_NAME = "visible_roll_electrode_clear"
CAE_NAME = MODEL_NAME + ".cae"
INP_NAME = MODEL_NAME + ".inp"
PREVIEW_NAME = MODEL_NAME + "_preview"


# True values from the PDF-based setup.
TRUE_ROLL_RADIUS = 450.0
TRUE_COLLECTOR_HALF_THICKNESS = 0.0075
TRUE_COATING_THICKNESS = 0.075
TRUE_ELECTRODE_TOP_Y = TRUE_COLLECTOR_HALF_THICKNESS + TRUE_COATING_THICKNESS
TRUE_REDUCTION_20PCT_HALF_DISP = 0.0165

# Display values.  X and Z dimensions are unchanged; only Y thickness of the
# electrode is magnified so the layers can be seen beside the 900 mm roll.
DISPLAY_THICKNESS_SCALE = 1000.0
ROLL_RADIUS = TRUE_ROLL_RADIUS
ROLL_LENGTH = 675.0
COLLECTOR_WIDTH = 675.0
COATING_WIDTH = 650.0
ELECTRODE_LENGTH = 80.0
Z_MIN = -ELECTRODE_LENGTH / 2.0
Z_MAX = ELECTRODE_LENGTH / 2.0

COLLECTOR_HALF_THICKNESS = TRUE_COLLECTOR_HALF_THICKNESS * DISPLAY_THICKNESS_SCALE
COATING_THICKNESS = TRUE_COATING_THICKNESS * DISPLAY_THICKNESS_SCALE
ELECTRODE_TOP_Y = COLLECTOR_HALF_THICKNESS + COATING_THICKNESS
ROLL_CENTER_Y = ROLL_RADIUS + ELECTRODE_TOP_Y


def delete_if_exists(container, name):
    if name in container.keys():
        del container[name]


def make_materials(model):
    roll = model.Material(name="ROLL_STEEL_DISPLAY")
    roll.Density(table=((7.93e-9,),))
    roll.Elastic(table=((193000.0, 0.2),))

    collector = model.Material(name="COLLECTOR_AL")
    collector.Density(table=((2.7e-9,),))
    collector.Elastic(table=((72000.0, 0.33),))

    coating = model.Material(name="COATING_DPC")
    coating.Density(table=((2.55e-9,),))
    coating.Elastic(table=((6500.0, 0.01),))
    coating.CapPlasticity(table=((65.0, 4.0, 0.8, 0.01),))
    coating.capPlasticity.CapHardening(
        table=(
            (55.0, 0.000),
            (58.0, 0.050),
            (62.0, 0.100),
            (72.0, 0.150),
            (95.0, 0.200),
            (140.0, 0.230),
            (220.0, 0.260),
            (370.0, 0.290),
            (740.0, 0.325),
        )
    )

    model.HomogeneousSolidSection(name="SEC_COLLECTOR", material="COLLECTOR_AL")
    model.HomogeneousSolidSection(name="SEC_COATING", material="COATING_DPC")
    model.HomogeneousShellSection(name="SEC_ROLL_DISPLAY", material="ROLL_STEEL_DISPLAY", thickness=2.0)


def bbox_faces(obj, x_min, x_max, y_min, y_max, z_min, z_max):
    tol = 1.0e-6
    return obj.faces.getByBoundingBox(
        xMin=x_min - tol,
        xMax=x_max + tol,
        yMin=y_min - tol,
        yMax=y_max + tol,
        zMin=z_min - tol,
        zMax=z_max + tol,
    )


def partition_z(part, z_offsets):
    for z in z_offsets:
        datum = part.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=z)
        part.PartitionCellByDatumPlane(datumPlane=part.datums[datum.id], cells=part.cells)


def partition_y(part, y_offsets):
    for y in y_offsets:
        datum = part.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=y)
        part.PartitionCellByDatumPlane(datumPlane=part.datums[datum.id], cells=part.cells)


def mesh_solid(part, seed_size):
    elem = mesh.ElemType(elemCode=C3D8R, elemLibrary=STANDARD)
    part.setMeshControls(regions=part.cells, elemShape=HEX, technique=STRUCTURED)
    part.setElementType(regions=(part.cells,), elemTypes=(elem,))
    part.seedPart(size=seed_size, deviationFactor=0.1, minSizeFactor=0.1)
    part.generateMesh()


def make_collector(model):
    sketch = model.ConstrainedSketch(name="sk_collector_clear", sheetSize=800.0)
    sketch.rectangle(point1=(0.0, 0.0), point2=(COLLECTOR_WIDTH, COLLECTOR_HALF_THICKNESS))
    part = model.Part(name="P_COLLECTOR_DISPLAY", dimensionality=THREE_D, type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=ELECTRODE_LENGTH)
    delete_if_exists(model.sketches, "sk_collector_clear")
    partition_z(part, (20.0, 30.0, 35.0, 38.0, 40.0, 42.0, 45.0, 50.0, 60.0))
    part.SectionAssignment(region=part.Set(cells=part.cells, name="ALL_COLLECTOR"), sectionName="SEC_COLLECTOR")
    mesh_solid(part, 10.0)
    return part


def make_coating(model):
    sketch = model.ConstrainedSketch(name="sk_coating_clear", sheetSize=800.0)
    sketch.rectangle(point1=(0.0, 0.0), point2=(COATING_WIDTH, COATING_THICKNESS))
    part = model.Part(name="P_COATING_DISPLAY", dimensionality=THREE_D, type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=ELECTRODE_LENGTH)
    delete_if_exists(model.sketches, "sk_coating_clear")
    partition_y(part, (COATING_THICKNESS / 3.0, 2.0 * COATING_THICKNESS / 3.0))
    partition_z(part, (20.0, 30.0, 35.0, 38.0, 40.0, 42.0, 45.0, 50.0, 60.0))
    part.SectionAssignment(region=part.Set(cells=part.cells, name="ALL_COATING"), sectionName="SEC_COATING")
    mesh_solid(part, 10.0)
    return part


def make_roll(model):
    sketch = model.ConstrainedSketch(name="sk_roll_clear", sheetSize=1000.0)
    sketch.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(ROLL_RADIUS, 0.0))
    part = model.Part(name="P_ROLL_DISPLAY_SHELL", dimensionality=THREE_D, type=DEFORMABLE_BODY)
    part.BaseShellExtrude(sketch=sketch, depth=ROLL_LENGTH)
    delete_if_exists(model.sketches, "sk_roll_clear")
    part.SectionAssignment(region=part.Set(faces=part.faces, name="ALL_ROLL_DISPLAY"), sectionName="SEC_ROLL_DISPLAY")
    elem = mesh.ElemType(elemCode=S4R, elemLibrary=STANDARD)
    part.setElementType(regions=(part.faces,), elemTypes=(elem,))
    part.seedPart(size=45.0, deviationFactor=0.1, minSizeFactor=0.1)
    part.generateMesh()
    rp = part.ReferencePoint(point=(0.0, 0.0, ROLL_LENGTH / 2.0))
    part.Set(name="RP_ROLL", referencePoints=(part.referencePoints[rp.id],))
    part.Surface(name="S_ROLL", side1Faces=part.faces)
    return part


def make_face_set(assembly, name, faces):
    if len(faces) == 0:
        raise RuntimeError("No faces found for %s" % name)
    return assembly.Set(name=name, faces=faces)


def make_assembly(model, collector, coating, roll):
    a = model.rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    c = a.Instance(name="COLLECTOR-1", part=collector, dependent=ON)
    coat = a.Instance(name="COATING-1", part=coating, dependent=ON)
    r = a.Instance(name="ROLL-1", part=roll, dependent=ON)

    a.translate(instanceList=("COLLECTOR-1",), vector=(0.0, 0.0, Z_MIN))
    a.translate(instanceList=("COATING-1",), vector=(0.0, COLLECTOR_HALF_THICKNESS, Z_MIN))
    a.rotate(instanceList=("ROLL-1",), axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 1.0, 0.0), angle=90.0)
    a.translate(instanceList=("ROLL-1",), vector=(0.0, ROLL_CENTER_Y, 0.0))

    sx0_c = make_face_set(a, "SET_SYM_X0_COLLECTOR", bbox_faces(c, 0.0, 0.0, 0.0, COLLECTOR_HALF_THICKNESS, Z_MIN, Z_MAX))
    sx0_coat = make_face_set(a, "SET_SYM_X0_COATING", bbox_faces(coat, 0.0, 0.0, COLLECTOR_HALF_THICKNESS, ELECTRODE_TOP_Y, Z_MIN, Z_MAX))
    a.SetByBoolean(name="SET_SYM_X0", sets=(sx0_c, sx0_coat), operation=UNION)
    make_face_set(a, "SET_SYM_Y0", bbox_faces(c, 0.0, COLLECTOR_WIDTH, 0.0, 0.0, Z_MIN, Z_MAX))

    inlet_c = make_face_set(a, "SET_ELECTRODE_INLET_COLLECTOR", bbox_faces(c, 0.0, COLLECTOR_WIDTH, 0.0, COLLECTOR_HALF_THICKNESS, Z_MIN, Z_MIN))
    inlet_coat = make_face_set(a, "SET_ELECTRODE_INLET_COATING", bbox_faces(coat, 0.0, COATING_WIDTH, COLLECTOR_HALF_THICKNESS, ELECTRODE_TOP_Y, Z_MIN, Z_MIN))
    a.SetByBoolean(name="SET_ELECTRODE_INLET", sets=(inlet_c, inlet_coat), operation=UNION)
    outlet_c = make_face_set(a, "SET_ELECTRODE_OUTLET_COLLECTOR", bbox_faces(c, 0.0, COLLECTOR_WIDTH, 0.0, COLLECTOR_HALF_THICKNESS, Z_MAX, Z_MAX))
    outlet_coat = make_face_set(a, "SET_ELECTRODE_OUTLET_COATING", bbox_faces(coat, 0.0, COATING_WIDTH, COLLECTOR_HALF_THICKNESS, ELECTRODE_TOP_Y, Z_MAX, Z_MAX))
    a.SetByBoolean(name="SET_ELECTRODE_OUTLET", sets=(outlet_c, outlet_coat), operation=UNION)

    a.Surface(name="S_COATING_TOP", side1Faces=bbox_faces(coat, 0.0, COATING_WIDTH, ELECTRODE_TOP_Y, ELECTRODE_TOP_Y, Z_MIN, Z_MAX))
    a.Surface(name="S_ROLL", side1Faces=r.faces)
    a.Set(name="RP_ROLL", referencePoints=(list(r.referencePoints.values())[0],))
    a.DatumAxisByPrincipalAxis(principalAxis=XAXIS)
    return a


def make_preview(assembly):
    try:
        viewport = session.viewports["Viewport: 1"]
        viewport.setValues(displayedObject=assembly)
        viewport.assemblyDisplay.setValues(mesh=ON)
        viewport.view.setValues(
            width=1200.0,
            height=980.0,
            cameraPosition=(2000.0, ROLL_CENTER_Y - 45.0, 0.0),
            cameraUpVector=(0.0, 1.0, 0.0),
            cameraTarget=(ROLL_LENGTH / 2.0, ROLL_CENTER_Y - 45.0, 0.0),
        )
        session.printToFile(fileName=PREVIEW_NAME, format=PNG, canvasObjects=(viewport,))
    except Exception as exc:
        print("Preview export skipped: %s" % exc)


def main():
    cwd = os.path.abspath(os.getcwd())
    Mdb()
    try:
        del mdb.models["Model-1"]
    except Exception:
        pass
    model = mdb.Model(name=MODEL_NAME)
    make_materials(model)
    collector = make_collector(model)
    coating = make_coating(model)
    roll = make_roll(model)
    assembly = make_assembly(model, collector, coating, roll)
    make_preview(assembly)
    mdb.saveAs(pathName=os.path.join(cwd, CAE_NAME))
    job = mdb.Job(name=MODEL_NAME, model=MODEL_NAME, description="Clear display model; electrode Y scale is magnified.")
    job.writeInput(consistencyChecking=OFF)
    print("Saved CAE:", os.path.join(cwd, CAE_NAME))
    print("Wrote INP:", os.path.join(cwd, INP_NAME))
    print("Display thickness scale:", DISPLAY_THICKNESS_SCALE)
    print("True electrode half thickness:", TRUE_ELECTRODE_TOP_Y)
    print("Displayed electrode half thickness:", ELECTRODE_TOP_Y)


if __name__ == "__main__":
    main()
