<div align="center">
  <h1>Sound Detection and Localization Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the **Sound Detection and Localization**, a node within the CSSR4Africa project (**cssr_system** package). The node evaluates the performance of sound detection and direction localization under different conditions. It records both filtered and unfiltered audio to analyze the effectiveness of the audio processing algorithms, and generates visual plots comparing raw and processed signals. The tests assess the accuracy of sound source direction detection, ensuring the system can correctly localize sounds on the horizontal plane. Additionally, the module verifies that the voice activity detection (VAD) correctly identifies when speech is present in the audio stream, enabling reliable human-robot auditory interaction.

---

#  📄 Documentation
The main documentation for this deliverable is found in **[D4.2.3 Sound Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.3.pdf)**, which provides more details regarding how to run the test and gives detail about the sound detection and localization module.

---

#  Configuration File
The following table provides the key-value pairs used in the configuration file:

## 🔧 Configuration Parameters

| Parameter               | Description                                      | Possible Values            | Default Value |
|-------------------------|--------------------------------------------------|----------------------------|---------------|
| `sampleRate`            | Audio sampling rate in Hz                        | Positive integer           | `48000`       |
| `recordFiltered`        | Record filtered audio output                     | `true`, `false`            | `true`        |
| `recordUnfiltered`      | Record unfiltered (raw) audio input              | `true`, `false`            | `true`        |
| `generatePlot`          | Generate comparison plots of signals             | `true`, `false`            | `true`        |
| `recordDuration`        | Duration (seconds) for recorded audio            | Positive integer           | `10`          |
| `plotInterval`          | Interval (seconds) between generated plots       | Positive integer           | `10`          |
| `plotDpi`               | Resolution (DPI) of generated plots              | Positive integer           | `150`         |
| `maxDirectionPoints`    | Maximum data points in direction plots           | Positive integer           | `100`         |
| `directionPlotYlimit`   | Y-axis limit for direction plots (degrees)       | Positive integer           | `90`          |
| `verboseMode`           | Enable detailed logging and diagnostics          | `true`, `false`            | `true`        |

---

> **Note:**  
> Enabling **`verboseMode`** (`true`) will activate detailed logging and provide diagnostic information about buffer sizes, publishing rates, and other metrics useful for debugging.

#  🚀 Run the Sound Detection Unit Test
Before running the node, the configuration file must be set up correctly with the appropriate key-value pairs. The configuration file is typically located in the package folder under the `config` directory.

The test can be performed on a **physical robot using its microphones** or using **pre-recorded audio saved as a ROS bag file**. The specific test to run can be selected by modifying the configuration.

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
   Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/sound_detection_test/config/sound_detection_test_configuration.json` and update the configuration according to the key-value pairs as shown above.

4. **Launch the Physical Robot**
   ```bash
   # Launch the robot with the appropriate IP and network interface
   roslaunch unit_test sound_detection_test_launch.launch robot_ip:=<robot_ip> network_interface:=<network_interface>
   ```

   To use pre-recorded audio instead:
   ```bash
   # Use a pre-recorded audio file
   roslaunch unit_test sound_detection_test_launch.launch use_recorded_audio:=true audio_file:=<audio_file>
   ```

   ### Audio File Options (`audio_file`):
   - **`normal`**: Standard speech at normal volume and distance
   - **`noisy`**: Speech with background noise for robustness testing
   - **`multiple`**: Multiple speakers from different directions
   - **`quiet`**: Soft speech for testing sensitivity

   > **Note:**  
   > Before running the Test Harness, activate the sound detection Python environment:
   ```bash
   source ~/workspace/pepper_rob_ws/cssr4africa_sound_detection_env/bin/activate
   ```
   ```bash
   # Command to make application executable  
   chmod +x ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/sound_detection/src/sound_detection_application.py
   chmod +x ~/workspace/pepper_rob_ws/src/cssr4africa/unit_test/sound_detection_test/src/sound_detection_test_application.py
   ```

5. **Run Sound Detection Test Harness**
   ```bash
   # The launch file launches the sound_detection node for the unit test
   roslaunch unit_test sound_detection_test_launch_test_harness.launch
   ```

# 🖥️ Output Result
When running the sound detection test node, the following outputs are generated:

- **Audio Recordings**:
  - Filtered audio files (`filtered_YYYYMMDD_HHMMSS.wav`)
  - Unfiltered raw audio files (`unfiltered_YYYYMMDD_HHMMSS.wav`)

- **Visualization Plots**:
  - Audio signal comparison plots showing both raw and processed signals (`audio_signals_YYYYMMDD_HHMMSS.png`)
  - Direction data plots showing the detected angles over time (`direction_data_YYYYMMDD_HHMMSS.png`)

- **Console Output**:
  - Detailed logging of buffer fill percentages
  - Signal intensity and publishing rate information
  - Direction detection results

The details of the result are documented in the Sound Detection and Localization Deliverable Report.

# 💡Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:yohanneh@andrew.cmu.edu">yohanneh@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

# 📜 License
Copyright (C) 2023 CSSR4Africa Consortium  
Funded by the African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme 

2025-04-06