
-----
**GuideHand**: Assistive Robotic Handover System

This repository contains the complete ROS 2 Humble software for **GuideHand**, an assistive robotic arm system designed to autonomously detect, grasp, and safely hand common objects to blind and visually impaired individuals.

It enhances user independence by creating a reliable robotic partner that can respond to requests and manage the entire "see-and-retrieve" task. The system uses an AI-powered 3D camera to perceive its environment and a 6-axis robotic arm for precise manipulation.


-----

## 🚀 Key Features

  * **🤖 Autonomous Handover:** Manages the full pipeline from perception to grasp, lift, and handover.
  * **🧠 AI-Powered Perception:** Uses an OAK-D 3D camera with onboard AI (YOLO) to find objects and track the user's hand in real-time.
  * **🛡️ Safe Planning:** Integrates MoveIt2 for collision-aware motion planning.
  * **📦 "Off-the-Shelf" Mode:** Comes with pre-trained models that automatically download, allowing you to run the system immediately for testing.
  * **🔧 Customizable:** Easily swap in your own custom-trained AI models. (See [How to Use Your Own Model](https://www.google.com/search?q=%23-how-to-use-your-own-model))

-----

## 🛠️ Hardware Requirements

  * **Robotic Arm:** Interbotix Viper VX300s (or any arm supported by `interbotix_xsarm_ros`)
  * **3D Camera:** Luxonis OAK-D
  * **Computer:** PC running Ubuntu 22.04 with ROS 2 Humble installed.

-----

## 📦 Installation

This guide assumes you have a **ROS 2 Humble** workspace named `interbotix_ws`.

**1. Install Interbotix Drivers (If not done):**

  * Follow the official [Interbotix ROS 2 Humble Setup Guide](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2.html). This is critical as it installs all robot drivers and MoveIt configurations.

**2. Install OAK-D Dependencies:**

  * Follow the official [Luxonis ROS 2 Driver setup](https://www.google.com/search?q=https://docs.luxonis.com/projects/api/en/latest/install/%23install-on-ubuntu-22-04-humble).
  * Install the `blobconverter` library, which is used to download models:
    ```bash
    pip install blobconverter
    ```

**3. Clone This Repository:**

  * Navigate to your workspace's `src` directory and clone:
    ```bash
    cd ~/interbotix_ws/src
    git clone https://github.com/WalnutEagle/GuideHand.git ./assistive_robotics
    ```
    *(Note: This clones your repo into a folder named `assistive_robotics` which contains the two packages: `assistive_handover` and `assistive_perception`)*

**4. Install Dependencies and Build:**

  * Navigate back to your workspace root and build the new packages:
    ```bash
    cd ~/interbotix_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --packages-select assistive_handover assistive_perception
    ```

-----

## 📍 CRITICAL: Camera Calibration

This system **will not work** until you tell it where the camera is relative to the robot.

You must provide the 6D (X, Y, Z, Roll, Pitch, Yaw) transform from the robot's `base_link` to the camera's `oak_d_camera_optical_frame`.

1.  **Find the Transform:** The best way is using an ArUco marker.

      * Attach an ArUco marker to your robot's gripper.
      * Run the OAK-D's ArUco detection.
      * Move the robot to several known poses (using MoveIt or the `interbotix_xsarm_joy` package).
      * At each pose, record the marker's pose *relative to the camera* and the gripper's pose *relative to the robot base*.
      * Use a tool like the ROS 2 `easy_handeye` package to solve for the static "camera-to-base" transform.

2.  **Update the Launch File:**

      * Open `~/interbotix_ws/src/assistive_robotics/assistive_handover/launch/handover.launch.py`.
      * Find the `static_tf_node` section.
      * Replace the `0.0` **PLACEHOLDER** values with your 6 measured values.

    <!-- end list -->

    ```python
    # Find this section in handover.launch.py
    camera_calibration_args = [
        '-0.1',    # X: REPLACE THIS
        '0.0',     # Y: REPLACE THIS
        '0.3',     # Z: REPLACE THIS
        '0.0',     # Roll: REPLACE THIS
        '0.0',     # Pitch: REPLACE THIS
        '0.0',     # Yaw: REPLACE THIS
        'base_link',
        'oak_d_camera_optical_frame'
    ]
    ```

-----

## ▶️ How to Run

1.  **Source your workspace:**

    ```bash
    cd ~/interbotix_ws
    source install/setup.bash
    ```

2.  **Launch the project:**

    ```bash
    ros2 launch assistive_handover handover.launch.py
    ```

    This single command will:

      * Launch the Interbotix robot drivers and MoveIt2.
      * Launch your calibrated Static Transform.
      * Launch the `perception_node` (it will download the pre-trained models on first run).
      * Launch the `handover_manager` node.
      * Open RViz for visualization.

-----

## 🔬 How It Works: Node Architecture

This runs on three main custom nodes:

  * **`perception_node` (`assistive_perception`):**

      * Initializes the OAK-D camera and its AI pipeline.
      * Downloads or loads the AI models.
      * Continuously runs inference to find objects and hands.
      * Calculates their 3D spatial coordinates (X, Y, Z) in the camera's frame.
      * Publishes the 3D poses to `/handover/object_pose` and `/handover/hand_pose`.

  * **`static_transform_publisher` (`tf2_ros`):**

      * This is the node you configured with your calibration data.
      * It constantly broadcasts the transform between `base_link` and `oak_d_camera_optical_frame`.
      * This allows all other ROS nodes (like MoveIt) to understand where the camera's data is in relation to the robot.

  * **`handover_manager` (`assistive_handover`):**

      * The "brain" of the operation.
      * Subscribes to `/handover/object_pose` and `/handover/hand_pose`.
      * Provides an Action Server called `/request_handover`.
      * When the action is called (e.g., "hand me the bottle"), it uses the object's pose, plans a trajectory with MoveIt2, and executes the full pick-and-place-in-hand motion.

-----

## 🚀 How to Use Your Own Model

Yes, I designed this code for this\! You just need to change a few lines in the `perception_node.py` file.

**See the full instructions in the [CUSTOM\_MODEL\_GUIDE.md](./CUSTOM_MODEL_GUIDE.md) file.**

