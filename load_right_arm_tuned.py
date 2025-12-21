import time
import mujoco
import mujoco.viewer
import warnings

# Suppress numpy-to-scalar deprecation warnings that originate from buffer assignments
warnings.filterwarnings("ignore", category=DeprecationWarning)

model = mujoco.MjModel.from_xml_path("right_arm_tuned.xml")
data = mujoco.MjData(model)

# Use passive viewer so we can run a PD torque loop in this thread
handle = mujoco.viewer.launch_passive(model, data)

# Controller targets: hold current joint positions
# Map actuators by names (assume actuator order: shoulder, deltoid, forearm)
shoulder_act = 0
deltoid_act = 1
forearm_act = 2

# Read initial qpos targets
with handle.lock():
        mujoco.mj_forward(model, data)
        q_shoulder = float(data.qpos[model.joint('right_shoulder_joint').qposadr])
        q_deltoid = float(data.qpos[model.joint('right_deltoid_joint').qposadr])
        q_forearm = float(data.qpos[model.joint('right_forearm_joint').qposadr])

# PD gains (tuned)
kp_deltoid = 8.0
kv_deltoid = 30.0
kp_forearm = 6.0
kv_forearm = 25.0
torque_limit = 30.0

try:
    while handle.is_running():
        with handle.lock():
            # position target for shoulder (position actuator)
            data.ctrl[shoulder_act] = q_shoulder

            # deltoid PD torque
            qd = float(data.qpos[model.joint('right_deltoid_joint').qposadr])
            dq = float(data.qvel[model.joint('right_deltoid_joint').qposadr])
            tau_d = float(kp_deltoid * (q_deltoid - qd) - kv_deltoid * dq)
            data.ctrl[deltoid_act] = float(max(min(tau_d, torque_limit), -torque_limit))

            # forearm PD torque
            qf = float(data.qpos[model.joint('right_forearm_joint').qposadr])
            df = float(data.qvel[model.joint('right_forearm_joint').qposadr])
            tau_f = float(kp_forearm * (q_forearm - qf) - kv_forearm * df)
            data.ctrl[forearm_act] = float(max(min(tau_f, torque_limit), -torque_limit))

            # Step physics
            mujoco.mj_step(model, data)

        # Sync state for the viewer
        handle.sync(state_only=False)
        time.sleep(model.opt.timestep)
except KeyboardInterrupt:
    pass
finally:
    handle.close()
