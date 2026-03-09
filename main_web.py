# main_web_controller.py
import time
import threading
import webbrowser
import numpy as np
import robosuite as suite
import logging

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

from robosuite import load_part_controller_config
from robosuite.controllers.composite.composite_controller_factory import refactor_composite_controller_config
from control_state import ControllerState
from controller_ui import create_app
from control_utils.ik_utils import png_control
from control_utils.kinova_gen3 import gen_iris

# ============================================================
# USER CONFIG
# ============================================================

ROBOT_NAME = "Kinova3"
ENV_NAME = "Lift"
CONTROLLER_NAME = "JOINT_VELOCITY" # try to integrate as custom controller later?

PRINT_EVERY = 1
PRINT_ONLY_ON_CMD = True
DT = 1.0 / 30.0

SLEEP_EACH_LOOP = True

# ============================================================

def build_controller_config(robot_name, controller_name):
    arm_cfg = load_part_controller_config(default_controller=controller_name)
    composite_cfg = refactor_composite_controller_config(
        arm_cfg,
        robot_name,
        ["right"] 
    )
    return composite_cfg

def safe_get(arr, n):
    arr = np.asarray(arr).reshape(-1)
    return arr[: min(n, len(arr))]

def print_joint_state(env, robot=None, label="", print_on_cmd=True):
    # note this function does not print on gripper commands
    qpos = safe_get(env.sim.data.qpos, 7)
    qvel = safe_get(env.sim.data.qvel, 7)

    try:
        ctrl = np.array(env.sim.data.ctrl).copy().reshape(-1)
    except Exception:
        ctrl = np.array([])

    # print("sim qpos:", np.round(qpos, 4))
    # print("sim qvel:", np.round(qvel, 4))
    # print("sim ctrl:", np.round(ctrl, 4) if ctrl.size else ctrl)

    if robot is not None:
        pos = getattr(robot, "position", None)
        cmd = getattr(robot, "pending_joint_velocity_cmd", None)

        if print_on_cmd and not np.any(cmd):
            return
        
        print(f"\n--- {label} ---")
        if pos is not None:
            # This formats each float in the array to 3 decimal places
            pos_str = ", ".join([f"{x:.3f}" for x in np.asarray(pos)[:7]])
            print(f"robot pos   : [{pos_str}]")

        if cmd is not None:
            cmd_str = ", ".join([f"{x:.3f}" for x in np.asarray(cmd)[:7]])
            print(f"robot cmd   : [{cmd_str}]")
        print("ctrl axes   :", getattr(robot, "axes_vector", None))
        print("ctrl mode   :", getattr(robot, "mode", None))

def main():
    state = ControllerState()

    app = create_app(state)
    flask_thread = threading.Thread(
        target=lambda: app.run(host="127.0.0.1", port=5001, debug=False, use_reloader=False),
        daemon=True,
    )
    flask_thread.start()
    webbrowser.open("http://127.0.0.1:5001")

    controller_config = build_controller_config(ROBOT_NAME, CONTROLLER_NAME)

    print("ROBOT_NAME       =", ROBOT_NAME)
    print("ENV_NAME         =", ENV_NAME)
    print("CONTROLLER_NAME  =", CONTROLLER_NAME)

    env = suite.make(
        env_name=ENV_NAME,
        robots=ROBOT_NAME,
        controller_configs=controller_config,
        has_renderer=True,
        has_offscreen_renderer=False,
        render_camera="agentview",
        use_camera_obs=False,
        ignore_done=True,
        control_freq=30,
        horizon=100000,
        hard_reset=False,
    )

    env.reset()
    robot = gen_iris(png_control, env, state, DT)
    robot.refresh_state()

    i = 0
    while not state.snapshot()["quit_requested"]:
        robot.step()

        total_action = np.zeros(env.action_dim, dtype=np.float32)
        total_action[-1] = robot.gripper_target_state
        
        env.step(total_action)
        
        robot.apply_pending_command(DT)

        env.sim.forward()
        env.render()

        if i % PRINT_EVERY == 0:
            print_joint_state(env, robot, label=f"loop {i}", print_on_cmd=PRINT_ONLY_ON_CMD)

        i += 1
        if SLEEP_EACH_LOOP:
            time.sleep(DT)

if __name__ == "__main__":
    main()