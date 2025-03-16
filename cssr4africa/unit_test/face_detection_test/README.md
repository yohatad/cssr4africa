<div align="center">
  <h1>Face and Mutual Gaze Detection Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the **Face and Mutual Gaze Detection and Localization**, a node within the CSSR4Africa project (**cssr_system** package). The node evaluates the performance of face detection and mutual gaze under different conditions such as varying **lighting**, **occlusions**, and **different head poses**. It ensures that the detection algorithm remains robust across single and multiple face scenarios, maintaining accurate localization. The tests also assess the tracking stability of detected faces over multiple frames to measure consistency. Additionally, the module verifies that the mutual gaze detection correctly identifies when a user is engaging with the system, enabling natural human-robot interaction.

---

#  Documentation
The main documentation for this deliverable is found in  **[D4.2.2 Face and Mutual Gaze Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.2.pdf)**, which provides more details regarding how to run the test and gives detail about the face detection and localization module.  

---

#  Running the Node
Before running the node, the configuration file must be set up correctly with the appropriate key-value pairs. The configuration file is typically located in the package folder under the `config` directory.

The test can be performed on a **physical robot using realsense or pepper camera** or using **pre-recorded video saved as a ROS bag file**. The specific test to run can be selected by modifying the configuration.

---

#  Configuration File
The following table provides the key-value pairs used in the configuration file:

## Configuration Parameters

| Parameter           | Description                                   | Possible Values            | Default Value |
|---------------------|-----------------------------------------------|----------------------------|---------------|
| `algorithm`         | Algorithm selected for face detection         | `mediapipe`, `sixdrep`     | `sixdrep`     |
| `save_video`        | Save the output video of the test             | `True`, `False`            | `False`       |
| `save_image`        | Save individual image frames from the test    | `True`, `False`            | `False`       |
| `video_duration`    | Duration (seconds) for saved video            | Positive integer           | `10`          |
| `image_interval`    | Interval (seconds) between captured images    | Positive integer           | `5`           |
| `recording_delay`   | Delay (seconds) before recording starts       | Positive integer           | `5`           |
| `max_frames_buffer` | Maximum number of frames to store in buffer   | Positive integer           | `300`         |
| `speaker`           | Enable speaker announcements for active tests | `True`, `False`            | `True`        |
| `verbose_mode`      | Enable detailed logging and visual output     | `True`, `False`            | `True`        |

---

> **Note:**  
> Enabling **`verbose_mode`** (`True`) will activate detailed logging and **visualize outputs using OpenCV windows**.  

# Run the Face Detection Unit Test

1. **Install the required software components:**

  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf)

2. **Clone and build the project (if not already cloned)**:
  - Move to the source directory of the workspace
    ```bash 
        cd $HOME/workspace/pepper_rob_ws/src
      ```
  - Clone the `CSSR4Africa` software from the GitHub repository
    ```bash 
        git clone https://github.com/cssr4africa/cssr4africa.git
      ```
  - Build the source files
    ```bash 
        cd .. && catkin_make && source devel/setup.bash 
      ```
3. **Update Configuration File**
  Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/face_detection_test/config/face_detection_test_configuration.json` and update the configuration according to the key-value pairs as shown above.

3. **Launch the Physical Robot**
```bash
# The camera in the launch file could be set as 'pepper', 'realsense' or 'video'
roslaunch unit_test face_detection_test_launch_robot.launch camera:=<camera> bag_file:=<bag_file>
```

### Camera Input Options

- **`pepper`**: Use Pepper robot's built-in camera.
- **`realsense`**: Use Intel RealSense camera.
- **`video`**: Use a rosbag video recorded using the intel RealSense camera. 

### Bag File Options (`bag_file`):

- **`singleFace`**: Contains data with a single face for basic detection testing.
- **`multipleFaces`**: Includes multiple faces to test detection robustness in crowded scenes.
- **`faceTracking`**: Designed to evaluate continuous face tracking performance.
- **`mutualGaze`**: Data intended for analyzing gaze direction and mutual eye-contact scenarios.
- **`occlusion`**: Includes scenarios where faces are partially occluded to test resilience.
- **`lighting`**: Features variations in lighting conditions to assess detection stability.

> **Note:**  
> Before running the Test Harness, activate the face detection python environement. 
```bash


4. **Run Face Detection Test Harness**
```bash
# The launch file launches the face_detection_node for the unit test.
roslaunch unit_test face_detection_test_launch_testHarness.launch
```

# Results
When running the face detection system, you will see:

- A window displaying the camera feed with face detection results  
Each detected face will have:

  - A colored bounding box (unique per face ID)  
  - Face ID label  
  - "Engaged" or "Not Engaged" status based on mutual gaze  
  - Depth information showing distance from camera  

# Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:yohanneh@andrew.cmu.edu">yohanneh@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>



# License
  Copyright (C) 2023 CSSR4Africa Consortium  
  Funded by the African Engineering and Technology Network (Afretec)  
  Inclusive Digital Transformation Research Grant Programme 