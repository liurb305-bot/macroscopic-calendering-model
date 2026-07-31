"""Build a visible roll + electrode assembly for later calendering simulation.

This script creates a geometry/mesh setup only. It intentionally does not
create contact, loads, boundary conditions, or an analysis step.
"""

from __future__ import print_function

import os

from abaqus import Mdb, mdb, session
from abaqusConstants import (
    ANALYTIC_RIGID_SURFACE,
    C3D8R,
    CARTESIAN,
    DEFORMABLE_BODY,
    HEX,
    ON,
    PNG,
    STANDARD,
    STRUCTURED,
    THREE_D,
    UNION,
    XAXIS,
    XYPLANE,
    XZPLANE,
    YAXIS,
)
import mesh


MODEL_NAME = "visible_roll_electrode"
JOB_NAME = MODEL_NAME
CAE_NAME = MODEL_NAME + ".cae"
INP_NAME = MODEL_NAME + ".inp"
PREVIEW_NAME = MODEL_NAME + "_preview"


# Units: mm, N, MPa, tonne.
ROLL_RADIUS = 450.0
ROLL_LENGTH = 675.0
ROLL_CENTER_Y = ROLL_RADIUS + 0.0825

ELECTRODE_LENGTH = 80.0
Z_MIN = -ELECTRODE_LENGTH / 2.0
Z_MAX = ELECTRODE_LENGTH / 2.0

COLLECTOR_WIDTH = 675.0
COATING_WIDTH = 650.0
COLLECTOR_HALF_THICKNESS = 0.0075
COATING_THICKNESS = 0.075
ELECTRODE_TOP_Y = COLLECTOR_HALF_THICKNESS + COATING_THICKNESS
REDUCTION_20PCT_HALF_DISP = 0.0165


def delete_if_exists(container, name):
    if name in container.keys():
        del container[name]


def make_materials(model):
    roll = model.Material(name="ROLL_STEEL")
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


def partition_z(part, z_offsets):
    for z in z_offsets:
        datum = part.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=z)
        part.PartitionCellByDatumPlane(datumPlane=part.datums[datum.id], cells=part.cells)


def partition_y(part, y_offsets):
    for y in y_offsets:
        datum = part.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=y)
        part.PartitionCellByDatumPlane(datumPlane=part.datums[datum.id], cells=part.cells)


def assign_hex_mesh(part, seed_size):
    elem = mesh.ElemType(elemCode=C3D8R, elemLibrary=STANDARD)
    part.setMeshControls(regions=part.cells, elemShape=HEX, technique=STRUCTURED)
    part.setElementType(regions=(part.cells,), elemTypes=(elem,))
    part.seedPart(size=seed_size, deviationFactor=0.1, minSizeFactor=0.1)
    part.generateMesh()


def bbox_faces(part_or_instance, x_min, x_max, y_min, y_max, z_min, z_max):
    tol = 1.0e-6
    return part_or_instance.faces.getByBoundingBox(
        xMin=x_min - tol,
        xMax=x_max + tol,
        yMin=y_min - tol,
        yMax=y_max + tol,
        zMin=z_min - tol,
        zMax=z_max + tol,
    )


def make_face_set(assembly, name, faces):
    if len(faces) == 0:
        raise RuntimeError("No faces found for %s" % name)
    return assembly.Set(name=name, faces=faces)


def make_collector(model):
    sketch = model.ConstrainedSketch(name="sk_collector", sheetSize=800.0)
    sketch.rectangle(point1=(0.0, 0.0), point2=(COLLECTOR_WIDTH, COLLECTOR_HALF_THICKNESS))
    part = model.Part(name="P_COLLECTOR", dimensionality=THREE_D, type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=ELECTRODE_LENGTH)
    delete_if_exists(model.sketches, "sk_collector")

    partition_z(part, (20.0, 30.0, 35.0, 38.0, 40.0, 42.0, 45.0, 50.0, 60.0))
    part.SectionAssignment(region=part.Set(cells=part.cells, name="ALL_COLLECTOR"), sectionName="SEC_COLLECTOR")
    assign_hex_mesh(part, 25.0)

    part.Set(name="SET_SYM_X0", faces=bbox_faces(part, 0.0, 0.0, 0.0, COLLECTOR_HALF_THICKNESS, 0.0, ELECTRODE_LENGTH))
    part.Set(name="SET_SYM_Y0", faces=bbox_faces(part, 0.0, COLLECTOR_WIDTH, 0.0, 0.0, 0.0, ELECTRODE_LENGTH))
    part.Set(name="SET_ELECTRODE_INLET", faces=bbox_faces(part, 0.0, COLLECTOR_WIDTH, 0.0, COLLECTOR_HALF_THICKNESS, 0.0, 0.0))
    part.Set(name="SET_ELECTRODE_OUTLET", faces=bbox_faces(part, 0.0, COLLECTOR_WIDTH, 0.0, COLLECTOR_HALF_THICKNESS, ELECTRODE_LENGTH, ELECTRODE_LENGTH))
    return part


def make_coating(model):
    sketch = model.ConstrainedSketch(name="sk_coating", sheetSize=800.0)
    sketch.rectangle(point1=(0.0, 0.0), point2=(COATING_WIDTH, COATING_THICKNESS))
    part = model.Part(name="P_COATING", dimensionality=THREE_D, type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=ELECTRODE_LENGTH)
    delete_if_exists(model.sketches, "sk_coating")

    partition_y(part, (COATING_THICKNESS / 3.0, 2.0 * COATING_THICKNESS / 3.0))
    partition_z(part, (20.0, 30.0, 35.0, 38.0, 40.0, 42.0, 45.0, 50.0, 60.0))
    part.SectionAssignment(region=part.Set(cells=part.cells, name="ALL_COATING"), sectionName="SEC_COATING")
    assign_hex_mesh(part, 25.0)

    part.Set(name="SET_SYM_X0", faces=bbox_faces(part, 0.0, 0.0, 0.0, COATING_THICKNESS, 0.0, ELECTRODE_LENGTH))
    part.Set(name="SET_ELECTRODE_INLET", faces=bbox_faces(part, 0.0, COATING_WIDTH, 0.0, COATING_THICKNESS, 0.0, 0.0))
    part.Set(name="SET_ELECTRODE_OUTLET", faces=bbox_faces(part, 0.0, COATING_WIDTH, 0.0, COATING_THICKNESS, ELECTRODE_LENGTH, ELECTRODE_LENGTH))
    part.Surface(name="S_COATING_TOP", side1Faces=bbox_faces(part, 0.0, COATING_WIDTH, COATING_THICKNESS, COATING_THICKNESS, 0.0, ELECTRODE_LENGTH))
    return part


def make_roll(model):
    sketch = model.ConstrainedSketch(name="sk_roll_profile", sheetSize=1000.0)
    sketch.ConstructionLine(point1=(0.0, -ROLL_LENGTH), point2=(0.0, 2.0 * ROLL_LENGTH))
    sketch.Line(point1=(ROLL_RADIUS, 0.0), point2=(ROLL_RADIUS, ROLL_LENGTH))
    part = model.Part(name="P_ROLL_ANALYTIC_RIGID", dimensionality=THREE_D, type=ANALYTIC_RIGID_SURFACE)
    part.AnalyticRigidSurfRevolve(sketch=sketch)
    delete_if_exists(model.sketches, "sk_roll_profile")

    rp = part.ReferencePoint(point=(0.0, ROLL_LENGTH / 2.0, 0.0))
    part.Set(name="RP_ROLL", referencePoints=(part.referencePoints[rp.id],))
    part.Surface(name="S_ROLL", side1Faces=part.faces)
    part.DatumAxisByTwoPoint(point1=(0.0, 0.0, 0.0), point2=(0.0, ROLL_LENGTH, 0.0))
    return part


def make_assembly(model, collector, coating, roll):
    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)

    inst_collector = assembly.Instance(name="COLLECTOR-1", part=collector, dependent=ON)
    inst_coating = assembly.Instance(name="COATING-1", part=coating, dependent=ON)
    inst_roll = assembly.Instance(name="ROLL-1", part=roll, dependent=ON)

    assembly.translate(instanceList=("COLLECTOR-1",), vector=(0.0, 0.0, Z_MIN))
    assembly.translate(instanceList=("COATING-1",), vector=(0.0, COLLECTOR_HALF_THICKNESS, Z_MIN))
    assembly.rotate(instanceList=("ROLL-1",), axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 1.0), angle=-90.0)
    assembly.translate(instanceList=("ROLL-1",), vector=(0.0, ROLL_CENTER_Y, 0.0))

    c_x0 = make_face_set(assembly, "SET_SYM_X0_COLLECTOR", bbox_faces(inst_collector, 0.0, 0.0, 0.0, COLLECTOR_HALF_THICKNESS, Z_MIN, Z_MAX))
    coat_x0 = make_face_set(assembly, "SET_SYM_X0_COATING", bbox_faces(inst_coating, 0.0, 0.0, COLLECTOR_HALF_THICKNESS, ELECTRODE_TOP_Y, Z_MIN, Z_MAX))
    assembly.SetByBoolean(name="SET_SYM_X0", sets=(c_x0, coat_x0), operation=UNION)

    make_face_set(assembly, "SET_SYM_Y0", bbox_faces(inst_collector, 0.0, COLLECTOR_WIDTH, 0.0, 0.0, Z_MIN, Z_MAX))

    c_in = make_face_set(assembly, "SET_ELECTRODE_INLET_COLLECTOR", bbox_faces(inst_collector, 0.0, COLLECTOR_WIDTH, 0.0, COLLECTOR_HALF_THICKNESS, Z_MIN, Z_MIN))
    coat_in = make_face_set(assembly, "SET_ELECTRODE_INLET_COATING", bbox_faces(inst_coating, 0.0, COATING_WIDTH, COLLECTOR_HALF_THICKNESS, ELECTRODE_TOP_Y, Z_MIN, Z_MIN))
    assembly.SetByBoolean(name="SET_ELECTRODE_INLET", sets=(c_in, coat_in), operation=UNION)

    c_out = make_face_set(assembly, "SET_ELECTRODE_OUTLET_COLLECTOR", bbox_faces(inst_collector, 0.0, COLLECTOR_WIDTH, 0.0, COLLECTOR_HALF_THICKNESS, Z_MAX, Z_MAX))
    coat_out = make_face_set(assembly, "SET_ELECTRODE_OUTLET_COATING", bbox_faces(inst_coating, 0.0, COATING_WIDTH, COLLECTOR_HALF_THICKNESS, ELECTRODE_TOP_Y, Z_MAX, Z_MAX))
    assembly.SetByBoolean(name="SET_ELECTRODE_OUTLET", sets=(c_out, coat_out), operation=UNION)

    top_faces = bbox_faces(inst_coating, 0.0, COATING_WIDTH, ELECTRODE_TOP_Y, ELECTRODE_TOP_Y, Z_MIN, Z_MAX)
    assembly.Surface(name="S_COATING_TOP", side1Faces=top_faces)
    assembly.Surface(name="S_ROLL", side1Faces=inst_roll.faces)

    rp_values = list(inst_roll.referencePoints.values())
    assembly.Set(name="RP_ROLL", referencePoints=(rp_values[0],))
    assembly.DatumAxisByPrincipalAxis(principalAxis=XAXIS)
    assembly.DatumAxisByPrincipalAxis(principalAxis=YAXIS)
    return assembly


def set_display_options(assembly):
    try:
        viewport = session.viewports["Viewport: 1"]
        viewport.setValues(displayedObject=assembly)
        viewport.assemblyDisplay.setValues(mesh=ON)
        viewport.view.setValues(
            width=1000.0,
            height=700.0,
            cameraPosition=(1800.0, ROLL_CENTER_Y, 0.0),
            cameraUpVector=(0.0, 1.0, 0.0),
            cameraTarget=(ROLL_LENGTH / 2.0, ROLL_CENTER_Y, 0.0),
        )
        session.printToFile(fileName=PREVIEW_NAME, format=PNG, canvasObjects=(viewport,))
    except Exception as exc:
        print("Preview export skipped: %s" % exc)


def main():
    cwd = os.path.abspath(os.getcwd())
    Mdb()
    delete_if_exists(mdb.models, MODEL_NAME)
    delete_if_exists(mdb.models, "Model-1")
    model = mdb.Model(name=MODEL_NAME)
    make_materials(model)
    collector = make_collector(model)
    coating = make_coating(model)
    roll = make_roll(model)
    assembly = make_assembly(model, collector, coating, roll)
    set_display_options(assembly)

    mdb.saveAs(pathName=os.path.join(cwd, CAE_NAME))
    job = mdb.Job(name=JOB_NAME, model=MODEL_NAME, description="Geometry-only visible roll/electrode assembly")
    job.writeInput(consistencyChecking=OFF)
    print("Saved CAE: %s" % os.path.join(cwd, CAE_NAME))
    print("Wrote INP: %s" % os.path.join(cwd, INP_NAME))
    print("Roll radius: %.3f mm, visible half-width: %.3f mm" % (ROLL_RADIUS, ROLL_LENGTH))
    print("Electrode: x collector 0-%.3f mm, coating 0-%.3f mm, z %.3f to %.3f mm" % (COLLECTOR_WIDTH, COATING_WIDTH, Z_MIN, Z_MAX))
    print("20 percent half-model reduction displacement retained as parameter: %.6f mm" % REDUCTION_20PCT_HALF_DISP)


if __name__ == "__main__":
    main()
