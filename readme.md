# actuator_sdk

This repo contains custom **Bi-Manual Robot SDK**, a Python-based framework for controlling our humanoid robot with dual arms using **Dynamixel motors**, **MuJoCo simulation**, and **inverse kinematics** via the `mink` and `ikpy` libraries.

### 📁 Project Structure

```
.
│   environment.yaml         # Conda environment file
│   readme.md                # This file
│   left_arm.mjb             # Compiled MuJoCo model
│   right_arm.mjb
│
├───src
│       backend.py           # Primary SDK logic for bimanual control
│       fk_manual.py         # Manual forward kinematics tools
│       humanoid_sdk.py      # Interface/wrapper around robot
│       testing.py           # Example/test scripts
│
└───urdf
    │   humanoid_full.xml
    │   left_arm.xml
    │   right_arm.xml
    │   left_arm_URDF.urdf
    │   right_arm_URDF.urdf
    │   right_shoulder_1.STL
    │   v5_URDF.urdf
    │
    └───meshes
            *.STL            # Geometry meshes for arms and torso
```

---

### ⚙️ Conda Environment

To set up the environment:

```bash
conda env create -f environment.yaml
conda activate bmsdk
```

The environment is named `bmsdk` and includes dependencies like:

* `mujoco`
* `dynamixel_sdk`
* `ikpy`
* `mink`
* `numpy`, `scipy`, `pyserial`, etc.

---

### 🚀 Getting Started

1. **Hardware or Simulation**:
   The SDK can be run in simulation-only mode (`simulation_only=True`) or connect to real Dynamixel hardware.

2. **Run Sample Code**:
   Test joint movement or pose generation using:

   ```bash
   python src/testing.py
   ```

3. **Main Class**:
   Use the `DynamixelRobot` class from `backend.py`:

   ```python
   from src.backend import DynamixelRobot
   robot = DynamixelRobot(simulation_only=True)
   ```

---

### 🧠 Features

* Full **forward and inverse kinematics** for both arms.
* Cartesian control using position and orientation (quaternions or Euler).
* MuJoCo-based visualization of joint states.
* Real-time syncing and pose recording/playback.
* Flexible torque control and motion profiling.

---

### 🛠 Requirements

* [MuJoCo](https://mujoco.org/) installed and licensed (place `.mujoco` folder as needed).
* Dynamixel SDK (auto-installed via Conda or available from [ROBOTIS](https://emanual.robotis.com/)).

---

### 📌 Notes

* All `*.urdf` and mesh files are located in `urdf/` and `urdf/meshes/`.
* Predefined poses (`home`, `prepose`, `pose1`, `pose2`) are built-in.
* Use `record_joint_poses()` and `play_saved_sequence()` to program routines.
