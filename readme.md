# Point and Go: Mode Switching for Robotic Teleoperation

This repository contains the `robosuite` implementation of the **Point and Go** mode switching framework. This demo features a Kinova Gen3 robot in a "Lift" task environment, controlled via a custom Web-based joystick GUI. If you're just interested in the PnG source code, most of it is in the two files in ./control_utils. I wrote the DH-parameters transformations myself to learn during my Master's, and have not replaced it. It would certainly work better if we did, but this was the same code that was used for my Thesis, except implemented in Robosuite instead of with Kinova Kortex.

**Paper:** [Point and Go: Intuitive Reference Frame Reallocation in Mode Switching for Assistive Robotics](https://arxiv.org/abs/2510.08753)
**Thesis Website:** [Finding Intuitive Action Spaces for Wheelchair-Mounted Robotic Arms](https://alliewang30.github.io/png/)

---

## 🛠 Installation

Follow these steps to set up the environment and local dependencies.

### 1. Install Python Dependencies
Install the required base packages via pip:

```bash
pip install Flask==3.1.3 numpy==1.26.4 scipy==1.15.3 qpsolvers==4.9.0 quadprog==0.1.13 pynput==1.8.1 opencv-python==4.11.0.86 tqdm==4.67.3 termcolor==3.3.0 mujoco==3.5.0
```

### 2. Setup Third-Party Libraries
We use specific versions of `robosuite` and `robosuite_models`.

```bash
mkdir third_party
cd third_party

# Clone and install robosuite (v1.5.2)
git clone [https://github.com/ARISE-Initiative/robosuite.git](https://github.com/ARISE-Initiative/robosuite.git)
cd robosuite
git checkout tags/v1.5.2
pip install -e .
cd ..

# Clone and install robosuite_models (v1.0.0)
git clone [https://github.com/ARISE-Initiative/robosuite_models.git](https://github.com/ARISE-Initiative/robosuite_models.git)
cd robosuite_models
git checkout tags/v1.0.0
pip install -e .
cd ../..
```

### 3. Required Source Modification
To ensure compatibility with the custom controller logic, you must **comment out** the `torque_compensation` property in the `robosuite` source code.

Open `third_party/robosuite/robosuite/controllers/parts/controller.py` and comment out lines **303 to 311**:

```python
# @property
# def torque_compensation(self):
#     """
#     Gravity compensation for this robot arm
#
#     Returns:
#         np.array: torques
#     """
#     return self.sim.data.qfrc_bias[self.qvel_index]
```

---

## 🚀 Running the Demo

Launch the main controller script:

```bash
python main_web_controller.py
```

- This will automatically open a web browser to `http://127.0.0.1:5001`.
- Use the web interface to toggle modes and control the robot.
- The simulation will render a 3D view of the Kinova robot performing the Lift task.

---