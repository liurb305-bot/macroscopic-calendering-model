"""Switch the current Abaqus/CAE viewport to the visible roll/electrode model."""

from abaqus import mdb, session

if "visible_roll_electrode" not in mdb.models:
    raise RuntimeError("Model visible_roll_electrode is not loaded in this CAE database.")

if "Model-1" in mdb.models and len(mdb.models["Model-1"].parts) == 0:
    del mdb.models["Model-1"]

model = mdb.models["visible_roll_electrode"]
assembly = model.rootAssembly
viewport = session.viewports[session.currentViewportName]
viewport.setValues(displayedObject=assembly)
viewport.view.fitView()
mdb.save()
print("Displayed visible_roll_electrode assembly and saved the CAE database.")
