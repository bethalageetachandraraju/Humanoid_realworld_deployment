import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT

import time
import mujoco.viewer
import mujoco
import numpy as np
import yaml
import os
import select
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import get_gravity_orientation

# Import msvcrt for Windows if available
try:
    import msvcrt
except ImportError:
    msvcrt = None

def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

def get_input_non_blocking():
    """Get input without blocking the main loop"""
    if os.name == 'nt':  # Windows
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8').lower()
    else:  # Unix/Linux/MacOS
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.readline().strip().lower()
    return None

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mujoco_yaml_path = os.path.join(current_dir, "config", "mujoco.yaml")
    with open(mujoco_yaml_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        xml_path = os.path.join(PROJECT_ROOT, config["xml_path"])
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]
        
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    mj_per_step_duration = simulation_dt * control_decimation
    num_joints = m.nu
    policy_output_action = np.zeros(num_joints, dtype=np.float32)
    kps = np.zeros(num_joints, dtype=np.float32)
    kds = np.zeros(num_joints, dtype=np.float32)
    sim_counter = 0
    
    state_cmd = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    FSM_controller = FSM(state_cmd, policy_output)
    
    print("=== Terminal Commands ===")
    print("q: Quit simulation")
    print("p: PASSIVE mode")
    print("r: POS_RESET")
    print("l: LOCO mode")
    print("1: SKILL_1")
    print("2: SKILL_2")
    print("3: SKILL_3")
    print("4: SKILL_4")
    print("5: SKILL_Horse_Stance")
    print("w: Forward movement")
    print("s: Backward movement")
    print("a: Left movement")
    print("d: Right movement")
    print("up: Up movement")
    print("down: Down movement")
    print("========================")
    
    Running = True
    with mujoco.viewer.launch_passive(m, d) as viewer:
        sim_start_time = time.time()
        while viewer.is_running() and Running:
            try:
                # Check for terminal commands
                cmd = get_input_non_blocking()
                if cmd:
                    if cmd == 'q':
                        Running = False
                    elif cmd == 'p':
                        state_cmd.skill_cmd = FSMCommand.PASSIVE
                    elif cmd == 'r':
                        state_cmd.skill_cmd = FSMCommand.POS_RESET
                    elif cmd == 'l':
                        state_cmd.skill_cmd = FSMCommand.LOCO
                    elif cmd == '1':
                        state_cmd.skill_cmd = FSMCommand.SKILL_1
                    elif cmd == '2':
                        state_cmd.skill_cmd = FSMCommand.SKILL_2
                    elif cmd == '3':
                        state_cmd.skill_cmd = FSMCommand.SKILL_3
                    elif cmd == '4':
                        state_cmd.skill_cmd = FSMCommand.SKILL_4
                    elif cmd == '5':
                        state_cmd.skill_cmd = FSMCommand.SKILL_Horse_Stance
                    elif cmd == 'w':
                        state_cmd.vel_cmd[0] = 1.0  # Forward
                    elif cmd == 's':
                        state_cmd.vel_cmd[0] = -1.0  # Backward
                    elif cmd == 'a':
                        state_cmd.vel_cmd[1] = 1.0  # Left
                    elif cmd == 'd':
                        state_cmd.vel_cmd[1] = -1.0  # Right
                    elif cmd == 'up':
                        state_cmd.vel_cmd[2] = 1.0  # Up
                    elif cmd == 'down':
                        state_cmd.vel_cmd[2] = -1.0  # Down
                    elif cmd == 'stop':
                        # Stop all movement
                        state_cmd.vel_cmd = np.zeros(3)
                
                step_start = time.time()
                
                tau_limit = np.array([88, 139, 88, 139, 50, 50,
                    88, 139, 88, 139, 50, 50,
                    88, 50, 50,
                    25, 25, 25, 25, 
                    5, 5, 5,  # not using
                    25, 25, 25, 25,
                    5, 5, 5   # not using
                ], dtype=np.float32)
                tau = pd_control(policy_output_action, d.qpos[7:], kps, np.zeros_like(kps), d.qvel[6:], kds)
                tau = np.clip(tau, -tau_limit, tau_limit)
                # tau[19:22]=0.0
                # tau[26:29]=0.0
                d.ctrl[:] = tau
                mujoco.mj_step(m, d)
                sim_counter += 1
                if sim_counter % control_decimation == 0:
                    
                    qj = d.qpos[7:]
                    dqj = d.qvel[6:]
                    quat = d.qpos[3:7]
                    
                    omega = d.qvel[3:6] 
                    gravity_orientation = get_gravity_orientation(quat)
                    
                    state_cmd.q = qj.copy()
                    state_cmd.dq = dqj.copy()
                    state_cmd.gravity_ori = gravity_orientation.copy()
                    state_cmd.ang_vel = omega.copy()
                    
                    FSM_controller.run()
                    policy_output_action = policy_output.actions.copy()
                    kps = policy_output.kps.copy()
                    kds = policy_output.kds.copy()
            except ValueError as e:
                print(str(e))
            
            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        