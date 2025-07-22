<div align="center">
  <h1 align="center">humanoid_deploy</h1>
  <p align="center">
    <span>🌎English</span> | <a href="README_zh.md">🇨🇳中文</a>
  </p>
</div>

<p align="center">
  <strong>​humanoid_deploy​​ is a multi-policy robot deployment framework based on a state-switching mechanism. Currently, the included policies are designed for the ​​Unitree G1 robot (29-DoF)​​.</strong> 
</p>

---

## Repository

This is the official repository: [https://github.com/bethalageetachandraraju/Humanoid_realworld_deployment.git](https://github.com/bethalageetachandraraju/Humanoid_realworld_deployment.git)

---

## Preface

- **​This deployment framework is only applicable to G1 robots with a 3-DOF waist. If a waist fixing bracket is installed, it must be unlocked according to the official tutorial before this framework can be used normally.​​**

- **It is recommended to remove the hands, as dance movements may cause interference.​**

---

## Installation and Configuration

## 1. Create a Virtual Environment

It is recommended to run training or deployment programs in a virtual environment. We suggest using Conda to create one.

### 1.1 Create a New Environment

Use the following command to create a virtual environment:
```bash
conda create -n robomimic python=3.8
```

### 1.2 Activate the Virtual Environment

```bash
conda activate robomimic
```

---

## 2. Install Dependencies

### 2.1 Install PyTorch
PyTorch is a neural network computation framework used for model training and inference. Install it with the following command:
```bash
conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=12.1 -c pytorch -c nvidia
```

### 2.2 Install RoboMimic_Deploy

#### 2.2.1 Download
Clone the repository via git:

```bash
git clone https://github.com/ccrpRepo/RoboMimic_Deploy.git
```

#### 2.2.2 Install Components

Navigate to the directory and install:
```bash
cd RoboMimic_Deploy
pip install numpy==1.20.0
pip install onnx onnxruntime
```

#### 2.2.3 Install unitree_sdk2_python

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```
---
## Running the Code

### 1. Run Mujoco Simulation (Keyboard Control)

You can run the Mujoco simulation and activate different robot skills using the following keyboard shortcuts directly from the terminal (no joystick required):

```bash
python deploy_mujoco/deploy_mujoco.py
```

#### Keyboard Shortcuts (Current Implementation in Simulation):
| Key/Command | Functionality           |
|-------------|------------------------|
| q           | Quit simulation        |
| p           | PASSIVE mode           |
| r           | POS_RESET              |
| l           | LOCO mode              |
| 1           | SKILL_1                |
| 2           | SKILL_2                |
| 3           | SKILL_3                |
| 4           | SKILL_4                |
| 5           | SKILL_Horse_Stance     |
| w           | Forward movement       |
| s           | Backward movement      |
| a           | Left movement          |
| d           | Right movement         |
| up          | Up movement            |
| down        | Down movement          |
| stop        | Stop all movement      |

- Type the corresponding key/command in the terminal and press Enter to activate the function.
- The simulation will respond to your input in real time.

---

### 2. Policy Descriptions
| Mode Name        | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| **PassiveMode**  | Damping protection mode                                                     |
| **FixedPose**    | Position control reset to default joint values                              |
| **LocoMode**     | Stable walking control mode                                                 |
| **Dance**        | Charleston dance routine                                                    |
| **KungFu**       | Martial arts movement                                                       |
| **KungFu2**      | Failed martial arts training                                                |
| **Kick**         | Bad mimic policy                                                            |
| **SkillCast**    | Lower body + waist stabilization with upper limbs positioned to specific joint angles (typically executed before Mimic strategy) |
| **SkillCooldown**| Lower body + waist continuous balancing with upper limbs reset to default angles (typically executed after Mimic strategy) |
| **HorseStance**  | Horse stance pose (NEW, R1+B on remote or '5' in simulation)                |

---
## 3. Operation Instructions in Simulation

To control the robot in simulation, use the following keyboard shortcuts directly in the terminal after starting the simulation:

```bash
python deploy_mujoco/deploy_mujoco.py
```

| Key/Command | Functionality           |
|-------------|------------------------|
| q           | Quit simulation        |
| p           | PASSIVE mode           |
| r           | POS_RESET              |
| l           | LOCO mode              |
| 1           | SKILL_1                |
| 2           | SKILL_2                |
| 3           | SKILL_3                |
| 4           | SKILL_4                |
| 5           | SKILL_Horse_Stance     |
| w           | Forward movement       |
| s           | Backward movement      |
| a           | Left movement          |
| d           | Right movement         |
| up          | Up movement            |
| down        | Down movement          |
| stop        | Stop all movement      |

- Type the corresponding key/command in the terminal and press Enter to activate the function.
- The simulation will respond to your input in real time.

---
## 4. Real Robot Operation Instructions

1. Power on the robot and suspend it (e.g., with a harness). and then hold L2+R2

2. Run the deploy_real program:
```bash
python deploy_real/deploy_real.py
```
3. Press the ​​Start​​ button to enter position control mode.
4. Subsequent operations are the same as in simulation.

### **Horse Stance (NEW)**
- To activate the Horse Stance policy on the real robot, hold **B + R1** on the remote controller.
- The robot will switch to the Horse Stance policy.

### **Adding/Running Future New Policies**
- To add a new policy:
    1. Implement your policy in a new folder under `policy/` (see `policy/horse_stance/` as an example).
    2. Register your policy in `FSM/FSM.py` by adding it to the FSM class and `get_next_policy`.
    3. Add a new command mapping in `deploy_real/deploy_real.py` (see the Horse Stance example: `KeyMap.B` + `KeyMap.R1`).
    4. Add your policy's configuration and ONNX model as needed.
- To run your new policy, use the corresponding button combination you defined in `deploy_real/deploy_real.py`.

---
## Important Notes
### 1. Framework Compatibility Notice
The current framework does not natively support deployment on G1 robots equipped with Orin NX platforms. Preliminary analysis suggests compatibility issues with the `unitree_python_sdk` on Orin systems. For onboard Orin deployment, we recommend the following alternative solution:

- Replace with [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) (official C++ SDK)
- Implement a dual-node ROS architecture:
  - **C++ Node**: Handles data transmission between robot and controller
  - **Python Node**: Dedicated to policy inference

### 2. Mimic Policy Reliability Warning
The Mimic policy does not guarantee 100% success rate, particularly on slippery/sandy surfaces. In case of robot instability:
- Press `F1` to activate **PassiveMode** (damping protection)
- Press `Select` to immediately terminate the control program

### 3. Charleston Dance (R1+X) - Stable Policy Notes
Currently the only verified stable policy on physical robots:

⚠️ **Important Precautions**:
- **Palm Removal Recommended**: The original training didn't account for palm collisions (author's G1 lacked palms)
- **Initial/Final Stabilization**: Brief manual stabilization may be required when starting/ending the dance
- **Post-Dance Transition**: While switching to **Locomotion/PositionControl/PassiveMode** is possible, we recommend:
  - First transition to **PositionControl** or **PassiveMode**
  - Provide manual stabilization during transition

### 4. Other Movement Advisories
All other movements are currently **not recommended** for physical robot deployment.

### 5. Strong Recommendation
**Always** master operations in simulation before attempting physical robot deployment.