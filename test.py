import mujoco
import mujoco.viewer

mjcf = """
<mujoco>
  <option gravity="0 0 -9.81"/>

  <asset>
    <texture name="checker" type="2d" builtin="checker"
             width="512" height="512"
             rgb1="0.8 0.8 0.8"
             rgb2="0.2 0.2 0.2"/>
    <material name="floor_mat" texture="checker"
              texrepeat="10 10"
              specular="0.5"/>
  </asset>

  <worldbody>
    <geom name="floor"
          type="plane"
          size="5 5 0.1"
          material="floor_mat"/>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(mjcf)
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)
