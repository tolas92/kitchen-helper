import mujoco
import mujoco.viewer
from pathlib import Path

mesh_dir = Path("mark_1/meshes").resolve()
mesh_files = list(mesh_dir.glob("*.STL"))

if not mesh_files:
    raise FileNotFoundError("No STL files found in mark_1/meshes")

asset_meshes = []
body_geoms = []

for i, mesh in enumerate(mesh_files):
    name = f"mesh_{i}"
    asset_meshes.append(
        f'<mesh name="{name}" file="{mesh.as_posix()}"/>'
    )
    body_geoms.append(
        f'<geom type="mesh" mesh="{name}" rgba="0.7 0.7 0.9 1"/>'
    )

mjcf = f"""
<mujoco>
  <option gravity="0 0 -9.81"/>

  <asset>
    {''.join(asset_meshes)}

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
      {''.join(body_geoms)}
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(mjcf)
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)
