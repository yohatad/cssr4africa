/* sensorTestApplication.cpp
*
* <detailed functional description>
* The component test the functionality of the sensor of the robot using the ROS interface.
* The test is performed by subscribing to the sensor topics and checking if the robot sends
* the expected data. The test is performed in two modes: sequential and parallel. In the sequential
* mode, the tests are performed one after the other. In the parallel mode, the tests are performed
* simultaneously.
...

* Libraries
* Standard libraries
- std::string, std::vector, std::thread, std::fstream, std::cout, std::endl, std::fabs, std::time_t, std::tm, std::localtime, std::strftime
* ROS libraries
- ros/ros.h, ros/package.h, image_transport/image_transport.h, sensor_msgs/CameraInfo.h, sensor_msgs/Range.h, sensor_msgs/JointState.h, sensor_msgs/LaserScan.h, cv_bridge/cv_bridge.h, opencv2/highgui/highgui.hpp

...
* Parameters
*
* Command-line Parameters
...
* Configuration File Parameters

* Key | Value 
* --- | ---
* platform        | robot
* simulatorTopics | simulatorTopics.dat
* robotTopics     | pepperTopics.dat
* mode            | sequential

* Key | Value
* --- | ---
* BackSonar     |   True
* FrontSonar    |   True    
* BottomCamera  |   True
* FrontCamera   |   True
* DepthCamera   |   True
* StereoCamera  |   True
* LaserSensor   |   True
* Microphone    |   True
* JointState    |   True
* Odometry      |   True
* IMU           |   True
* Speech        |   True
...
* Subscribed Topics and Message Types
*
* /naoqi_driver/sonar/back                      sensor_msgs/Range                 
* /naoqi_driver/sonar/front                     sensor_msgs/Range
* /naoqi_driver/camera/front/image_raw          sensor_msgs/Image
* /naoqi_driver/camera/bottom/image_raw         sensor_msgs/Image
* /naoqi_driver/camera/depth/image_raw          sensor_msgs/Image
* /naoqi_driver/camera/stereo/image_raw         sensor_msgs/Image
* /pepper/laser_2                               sensor_msgs/LaserScan
* /pepper/microphone/naoqi_microphone           naoqi_driver/AudioCustomMsg
* /pepper/joint_states                          sensor_msgs/JointState
* /pepper/odom                                  nav_msgs/Odometry
* /pepper/imu                                   sensor_msgs/Imu
* /pepper/naoqi_speech/speech                   std_msgs/String

* /pepper/sonar_back                            sensor_msgs/Range
* /pepper/sonar_front                           sensor_msgs/Range
* /pepper/camera/front/image_raw                sensor_msgs/Image
* /pepper/camera/bottom/image_raw               sensor_msgs/Image
* /pepper/camera/depth/image_raw                sensor_msgs/Image
* /pepper/laser_2                               sensor_msgs/LaserScan
...
* Published Topics and Message Types
*
* None
...
* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
...
* Output Data Files
*
* sensorTestOutput.dat
...
* Configuration Files
*
* sensorTestConfiguration.ini
* actuatorTestInput.ini
...
* Example Instantiation of the Module
*
* rosrun pepper_interface_tests sensorTest
...
*
* Author: Yohannes Tadesse Haile, Carnegie Mellon University Africa
* Email: yohanneh@andrew.cmu.edu
* Date: January 11, 2024
* Version: v1.0
*
*/


# include "pepper_interface_tests/sensorTest.h"

/* Main function */
int main(int argc, char **argv){
    // Initialize the ROS node
    ros::init(argc, argv, "sensorTest");
    ros::NodeHandle nh;

    std::vector<std::string> testNames = extractTests("sensor");
    std::string mode = extractMode();

    std::string path = getOutputFilePath();

    std::ofstream out_of;
    initializeOutputFile(out_of, path);
    
    if (mode == "parallel") {
        executeTestsInParallel(testNames, nh);
    } else if (mode == "sequential") {
        executeTestsSequentially(testNames, nh);
    } else {
        std::cerr << "Invalid mode provided. Exiting...\n";
        promptAndExit(1);
    }

    finalizeOutputFile(out_of, path);
    return 0;
}