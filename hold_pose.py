import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("franka_pick_place/franka_emika_panda/scene.xml")
data = mujoco.MjData(model)

ACT = 0        # actuator index
target = 0.0   # desired joint position

def controller(model, data):
    global target
    target += 0.001        # slowly change target
    data.ctrl[ACT] = target

mujoco.set_mjcb_control(controller)
mujoco.viewer.launch(model, data)
