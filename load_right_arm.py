import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("humanoid.xml")
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)
