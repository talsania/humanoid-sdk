import mujoco
import mujoco.viewer

MODEL_XML_PATH = "/home/kptal/actuator_sdk/urdf/left_arm_URDF.urdf"

model = mujoco.MjModel.from_xml_path(MODEL_XML_PATH)
data = mujoco.MjData(model)

# Use the passive viewer context
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Viewer launched. Press ESC or close the window to exit.")
    
    # Optional: you can animate or simulate here
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
