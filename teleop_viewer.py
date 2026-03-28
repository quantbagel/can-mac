"""Bidirectional YAM teleop: real arm mirrors in sim, sim sliders control real arm."""

import logging
import signal
import sys
import time

import mujoco
import numpy as np
import viser

from mjviser import ViserMujocoScene

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import GripperType

# Monkey-patch LINEAR_4310 to skip calibration by providing known limits
_orig_get_limits = GripperType.get_gripper_limits
_orig_get_cal = GripperType.get_gripper_needs_calibration

def _patched_limits(self):
    if self == GripperType.LINEAR_4310:
        return (0.0, 0.0475)  # closed, fully open (from URDF joint range)
    return _orig_get_limits(self)

def _patched_cal(self):
    if self == GripperType.LINEAR_4310:
        return False  # skip calibration
    return _orig_get_cal(self)

GripperType.get_gripper_limits = _patched_limits
GripperType.get_gripper_needs_calibration = _patched_cal

logging.basicConfig(level=logging.WARNING)

# --- Globals ---
robot = None
shutdown_flag = False


REST_POS = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def shutdown(signum=None, frame=None):
    global robot, shutdown_flag
    shutdown_flag = True
    if robot is not None:
        print("\nMoving to rest position...", flush=True)
        robot.move_joints(REST_POS, time_interval_s=3.0)
        time.sleep(0.5)
        robot.close()
        robot = None
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)


def main():
    global robot

    model_path = "yam_with_gripper.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    n_joints = model.njnt
    joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or f"joint_{i}" for i in range(n_joints)]
    print(f"Model has {n_joints} joints: {joint_names}")

    # Connect to real arm
    robot = get_yam_robot(
        channel="can0",
        gripper_type=GripperType.LINEAR_4310,
        zero_gravity_mode=False,
    )

    # Set sim to match real arm's current position
    real_pos = robot.get_joint_pos()
    n_arm = min(len(real_pos), model.nq)
    data.qpos[:n_arm] = real_pos[:n_arm]
    mujoco.mj_forward(model, data)

    # Start viser server
    server = viser.ViserServer()
    scene = ViserMujocoScene(server, model, num_envs=1)
    scene.create_scene_gui()
    scene.create_overlay_gui()

    # Add mode toggle
    with server.gui.add_folder("Teleop"):
        mode_toggle = server.gui.add_dropdown(
            "Mode",
            options=["Mirror Real Arm", "Control from Sim"],
            initial_value="Mirror Real Arm",
        )
        status_text = server.gui.add_text("Status", initial_value="Mirroring real arm", disabled=True)

        # Joint sliders for sim control
        sliders = []
        with server.gui.add_folder("Joint Commands"):
            for i in range(n_arm):
                jnt_range = model.jnt_range[i] if model.jnt_limited[i] else [-3.14, 3.14]
                lo, hi = float(jnt_range[0]), float(jnt_range[1])
                init = float(np.clip(real_pos[i], lo, hi)) if i < len(real_pos) else 0.0
                s = server.gui.add_slider(
                    joint_names[i] if i < len(joint_names) else f"J{i}",
                    min=lo,
                    max=hi,
                    step=0.01,
                    initial_value=init,
                )
                sliders.append(s)

    print("Teleop viewer running at http://localhost:8080", flush=True)
    print("Modes: 'Mirror Real Arm' reads from robot, 'Control from Sim' sends slider values to robot", flush=True)

    try:
        while not shutdown_flag:
            mode = mode_toggle.value

            if mode == "Mirror Real Arm":
                # Read real arm, update sim + sliders
                real_pos = robot.get_joint_pos()
                data.qpos[:n_arm] = real_pos[:n_arm]
                for i in range(n_arm):
                    sliders[i].value = float(real_pos[i])
                status_text.value = f"Mirroring | {[f'{p:.2f}' for p in real_pos]}"

            elif mode == "Control from Sim":
                # Read sliders, send to real arm
                target = np.array([s.value for s in sliders])
                robot.command_joint_pos(target)
                data.qpos[:n_arm] = target[:n_arm]
                status_text.value = f"Controlling | {[f'{p:.2f}' for p in target]}"

            mujoco.mj_forward(model, data)
            scene.update_from_mjdata(data)
            time.sleep(0.02)  # 50Hz update

    except Exception as e:
        print(f"Error: {e}", flush=True)
    finally:
        shutdown()


if __name__ == "__main__":
    main()
