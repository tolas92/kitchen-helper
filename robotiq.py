import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("./robotiq_2f85/scene.xml")
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)
