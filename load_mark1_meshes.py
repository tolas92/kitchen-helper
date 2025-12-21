import mujoco
import mujoco.viewer
from pathlib import Path

# Path to one mesh first (base or main body)
mesh_path = Path("mark_1/meshes").resolve()

# Pick ONE STL to start (change filename if needed)
base_mesh = next(mesh_path.glob("*.STL"), None)

if base_mesh is None:
    raise FileNotFoundError("No STL files found in mark_1/meshes")

mjcf = f"""
<mujoco>
  <option gravity="0 0 -9.81"/>

  <asset>
    <mesh name="base" file="{base_mesh.as_posix()}"/>

    <texture name="checker" type="2d" builtin="checker"
             width="512" height="512"
             rgb1="0.8 0.8 0.8"
             rgb2="0.2 0.2 0.2"/>
    <material name="floor_mat" texture="checker" texrepeat="10 10"/>
  </asset>

  <worldbody>
    <geom type="plane" size="5 5 0.1" material="floor_mat"/>

    <body pos="0 0 0.5">
      <joint type="free"/>
      <geom type="mesh" mesh="base"
            rgba="0.7 0.7 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(mjcf)
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)
