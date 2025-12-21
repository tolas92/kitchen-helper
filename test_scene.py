import mujoco
import mujoco.viewer

MODEL_PATH = "franka_pick_place/franka_emika_panda/scene.xml"


model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

print("Model loaded successfully")
print("Number of bodies:", model.nbody)
print("Number of geoms:", model.ngeom)

mujoco.viewer.launch(model, data)
