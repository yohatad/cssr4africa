#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

# include <ros/ros.h>
# include <ros/package.h>
# include <sensor_msgs/CameraInfo.h>
# include <sensor_msgs/Range.h>
# include <sensor_msgs/JointState.h>
# include <sensor_msgs/LaserScan.h>
# include <image_transport/image_transport.h>
# include <opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>

# include <thread>
# include <fstream>
# include <string>
# include <boost/algorithm/string.hpp>
# include <ctime>

using namespace boost;
using namespace std;

#define ROS

/* Call back functions executed when a sensor data arrived */
void jointStateCallBack(const sensor_msgs::JointState& state); 
void imageCallBack(const sensor_msgs::ImageConstPtr& msg);


void backSonar(ros::NodeHandle nh);
void frontSonar(ros::NodeHandle nh);
void frontCamera(ros::NodeHandle nh);
void bottomCamera(ros::NodeHandle nh);
void depthCamera(ros::NodeHandle nh);
void laserSensor(ros::NodeHandle nh);
void stereoCamera(ros::NodeHandle nh);

void backSonarMessageReceived(const sensor_msgs::Range& msg);
void frontSonarMessageReceived(const sensor_msgs::Range& msg);
void frontCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void bottomCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void depthCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void laserSensorMessageReceived(const sensor_msgs::LaserScan& msg);
void stereoCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);


std::vector<string> extractTests(string key);
string extractTopic(string set);   
std::string extractMode();
extern bool output;
extern int timeDuration;

void promptAndExit(int err);
void promptAndContinue();

#endif // SENSOR_TEST_H