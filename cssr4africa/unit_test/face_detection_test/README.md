<div align="center">
  <h1>Face and Mutual Gaze Detection Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the **Face and Mutual Gaze Detection and Localization**, a node within the CSSR4Africa project (**cssr_system** package). The node evaluates the performance of face detection and mutual gaze under different conditions such as varying **lighting**, **occlusions**, and **different head poses**. It ensures that the detection algorithm remains robust across single and multiple face scenarios, maintaining accurate localization. The tests also assess the tracking stability of detected faces over multiple frames to measure consistency. Additionally, the module verifies that the mutual gaze detection correctly identifies when a user is engaging with the system, enabling natural human-robot interaction.

---

## 📄 Documentation
The main documentation for this deliverable is found in  **[D4.2.2 Face and Mutual Gaze Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.2.pdf)**, which provides more details regarding how to run the test and gives detail about the face detection and localization module.  

---

## 🚀 Running the Node
Before running the node, the configuration file must be set up correctly with the appropriate key-value pairs. The configuration file is typically located in the package folder under the `config` directory.

The test can be performed on a **physical robot** or using **pre-recorded video saved as a ROS bag file**. The specific test to run can be selected by modifying the configuration.

---

### 🔧 Configuration File
The following table provides the key-value pairs used in the configuration file:

| Parameter                   | Description                                                | Range/Values               | Default Value |
|-----------------------------|------------------------------------------------------------|----------------------------|--------------|
| `algorithm`                 | Algorithm selected for face detection                      | `mediapipe` or `sixdrep`   | `sixdrep`    |
| `bag_file`                  | ROS bag file used as input for testing                     | `singleFace`, `multipleFaces`, `faceTracking`, `mutualGaze`, `occlusion`, `lighting` | `singleFace` |
| `save_video`                | Whether to save the output video of the test               | `True` or `False`          | `False`      |
| `save_image`                | Whether to save individual image frames from the test      | `True` or `False`          | `False`      |
| `video_duration`            | Duration (in seconds) for which the video is saved         | Positive integer           | `10`         |
| `image_interval`            | Time interval (in seconds) at which images are captured    | Positive integer           | `5`          |
| `speaker`                   | Enables the speaker to announce which test is running      | `True` or `False`          | `True`       |
| `verbose_mode`              | Enables detailed logs and diagnostic output                | `True` or `False`          | `False`      |

---

### Steps to Run

## 1️⃣ **Launch the Physical Robot**
```bash
# The camera in the launch file could be set as 'pepper' or 'realsense'
roslaunch unit_test face_detection_test_launch_robot.launch camera:=<camera>
```

## 2️⃣ Run the Face and Mutual Gaze Detection & Localization Unit Test
```bash
roslaunch unit_test face_detection_test_launch_testHarness.launch
```
## 💡Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:yohanneh@andrew.cmu.edu">yohanneh@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>



## 📜 License
  Copyright (C) 2023 CSSR4Africa Consortium  
  Funded by the African Engineering and Technology Network (Afretec)  
  Inclusive Digital Transformation Research Grant Programme 