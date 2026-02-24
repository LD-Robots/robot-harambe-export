import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("robot_harambe.urdf")
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
