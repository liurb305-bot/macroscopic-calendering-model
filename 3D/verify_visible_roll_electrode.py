from abaqus import openMdb, mdb

openMdb(pathName=r"E:\abaqus\3D\visible_roll_electrode.cae")
print("MODELS", list(mdb.models.keys()))
model = mdb.models["visible_roll_electrode"]
print("PARTS", list(model.parts.keys()))
print("INSTANCES", list(model.rootAssembly.instances.keys()))
print("SETS", list(model.rootAssembly.sets.keys()))
print("SURFACES", list(model.rootAssembly.surfaces.keys()))
print("STEPS", list(model.steps.keys()))
