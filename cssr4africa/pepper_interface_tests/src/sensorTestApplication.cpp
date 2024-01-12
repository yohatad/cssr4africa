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
    // Get the tests to run
    std::vector<std::string> testNames;
    
    char start_buf[50];
    
    std::time_t start_t = std::time(0);
    std::tm* start_now = std::localtime(&start_t);
    strftime(start_buf, sizeof(start_buf), "%Y-%m-%d.%X", start_now);

    testNames = extractTests("sensor");

    string path;
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        path = "..";
    #endif
    
    // complete the path of the output file
    path += "/data/sensorTestOutput.dat";
    
    std::ofstream out_of;
    out_of.open(path.c_str(), ofstream::app);
    if (!out_of.is_open()){
        printf("Unable to open the output file %s\n", path.c_str());
        promptAndExit(1);
    }
    out_of << "[TESTING] ############ SENSORS ############\n\n";
    out_of << "[START TIME] " << start_buf << "\n";
    
    out_of.close();
    
    // Initialize the ROS node
    ros::init(argc, argv, "sensorTest");
    ros::NodeHandle nh;

    // Get the mode
    std::string mode = extractMode();
    
    if (mode == "parallel"){
        // Run each test in a separate thread
        std::vector<std::thread> threads;
        for (auto test : testNames){
            if (test == "backsonar"){
                threads.push_back(std::thread(backSonar, nh));
            }
            else if (test == "frontsonar"){
                threads.push_back(std::thread(frontSonar, nh));
            }
            else if (test == "frontcamera"){
                threads.push_back(std::thread(frontCamera, nh));
            }
            else if (test == "bottomcamera"){
                threads.push_back(std::thread(bottomCamera, nh));
            }
            else if (test == "depthcamera"){
                threads.push_back(std::thread(depthCamera, nh));
            }
            else if (test == "lasersensor"){
                threads.push_back(std::thread(laserSensor, nh));
            }
            else if (test == "stereocamera"){
                threads.push_back(std::thread(stereoCamera, nh));
            }
            else{
                std::cout << "No test provided. Exiting...\n";
                promptAndExit(1);
            }
        }
        // Wait for all threads to finish
        for (auto& th : threads){
            th.join();
        }
    }
    else if (mode == "sequential"){
        // Run each test sequentially
        for (auto test : testNames){
            if (test == "backsonar"){
                backSonar(nh);
            }
            else if (test == "frontsonar"){
                frontSonar(nh);
            }
            else if (test == "frontcamera"){
                frontCamera(nh);
            }
            else if (test == "bottomcamera"){
                bottomCamera(nh);
            }
            else if (test == "depthcamera"){
                depthCamera(nh);
            }
            else if (test == "lasersensor"){
                laserSensor(nh);
            }
            else if (test == "stereocamera"){
                stereoCamera(nh);
            }
            else{
                std::cout << "No test provided. Exiting...\n";
                promptAndExit(1);
            }
        }
    }
    else{
        std::cout << "No mode provided. Exiting...\n";
        promptAndExit(1);
    }

    char end_buf[50];
    
    std::time_t end_t = std::time(0);
    std::tm* end_now = std::localtime(&end_t);
    strftime(end_buf, sizeof(end_buf), "%Y-%m-%d.%X", end_now);

    out_of.open(path.c_str(), ofstream::app);    
    out_of << "[END TIME] " << end_buf << "\n\n";
    out_of.close();

    return 0;
}