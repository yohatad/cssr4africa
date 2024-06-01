/************************************************************************************************
*  
* This code implement Interaural Time Difference (ITD) algorithm to detect the direction of the sound source
* by reading two audio file one is the original audio file and the other is the delayed audio file. 
* 
*
*
*
*************************************************************************************************/

#include "util.h"
#include "sound.h"
#include <vector>
#include <ros/ros.h>

const float speed_of_sound = 346;
const float distance_between_ears = 0.07;
const float sampling_rate = 48000;
const int bufferSize = 4096;

const int N = 10;
double buffered_ITD[N];
double lastUsedITD = 0.0;
const int UPDATE_FREQUENCY_MS = 500; // Update frequency in milliseconds
std::chrono::steady_clock::time_point last_update = std::chrono::steady_clock::now();
const double THRESHOLD = 0.05; // Threshold to trigger movement

// Print the highest sum_square_leftSound and sum_square_rightSound
// compare to the previous sum_square_leftSound and sum_square_rightSound
double prev_sum_square_leftSound = 0;
double prev_sum_square_rightSound = 0; 

std::string topicName = "/pepper_dcm/Head_controller/follow_joint_trajectory";

double Itd(float data1[], float data2[]){
    
    float ans[2 * bufferSize];
    float sum_square_leftSound = 0;
    float sum_square_rightSound = 0;

    // Apply Energy Thresholding
    for (int i = 0; i < bufferSize; i++) {
        sum_square_leftSound += data1[i] * data1[i];
        sum_square_rightSound += data2[i] * data2[i];
    }   

    float leftThreshold = 2.66608e+07;
    float rightThreshold = 4.77235e+07;

    if (sum_square_leftSound < leftThreshold || sum_square_rightSound < rightThreshold) {
        std::cout << "azimuth: 0 degree (Front)" << std::endl;
        return 0;
    }

    // correl(data1, data2, bufferSize, ans);
    correl(data1 - 1, data2 - 1, bufferSize, ans);

    float max = ans[1];
    int location = 1;
    
    for (int i = 2; i <= bufferSize; i++) {
        if (max < ans[i]) {
            max = ans[i];
            location = i;
        }
    }

    if (location != 1) {
        int num_samples = (location >= bufferSize / 2 + 1) ? (bufferSize + 1) - location : location - 1;
        float ITD = num_samples / sampling_rate;
        float z = ITD * (speed_of_sound / distance_between_ears);
        double azimuth = std::asin(z) * (180.0 / M_PI);

        if (location >= bufferSize / 2 + 1) {
            std::cout << "azimuth: " << azimuth << " degree (Left)" << std::endl;
            return -azimuth;
        }
        else {
            std::cout << "azimuth: " << azimuth << " degree (Right)" << std::endl;
            return azimuth;
        }
    }
    else {
        std::cout << "azimuth: 0 degree (Front)" << std::endl;
        return 0;
    }
}

ControlClientPtr createClient(const std::string& topicName) {
    ControlClientPtr actionClient(new ControlClient(topicName, true));
    int maxIterations = 5;

    for (int iterations = 0; iterations < maxIterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;
        }
        ROS_DEBUG("Waiting for the %s controller to come up", topicName.c_str());
    }

    throw std::runtime_error("Error creating action client for " + topicName + " controller: Server not available");
}

void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames, double duration, 
                    std::vector<double> positions) {
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(duration);

    client->sendGoal(goal);

    // Wait for the action to finish and check the result.
    bool finishedBeforeTimeout = client->waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed

    if (finishedBeforeTimeout) {
        actionlib::SimpleClientGoalState state = client->getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully moved to the position");
        } else {
            ROS_WARN("The action failed to move to the position. State: %s", state.toString().c_str());
        }
    } else {
        ROS_WARN("The action did not finish before the timeout.");
    }
}

void moveHeadToPosition(ros::NodeHandle& nh){
    ControlClientPtr headClient = createClient(topicName);
    std::vector<std::string> jointNames = {"HeadYaw", "HeadPitch"};
    std::vector<double> positions(2, 0.0);

    // Move the head to the center position 
    moveToPosition(headClient, jointNames, 1.0, positions);
}

void audiocallback(const naoqi_driver::AudioCustomMsg& msg) {
    const std::vector<int16_t>* frontLeft   = &msg.frontLeft;
    const std::vector<int16_t>* frontRight  = &msg.frontRight;

    float data1[bufferSize];
    float data2[bufferSize];

    for (int i = 0; i < bufferSize; i++) {
        data1[i] = frontLeft->at(i);
        data2[i] = frontRight->at(i);
    }

    // Perform ITD
    double value = Itd(data1, data2);
    
    // 
      
}


int main (int argc, char *argv[]){
    ros::init (argc, argv, "sound_localization");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber sub1 = nh.subscribe("/naoqi_driver/audio", 1000, audiocallback);

    ros::spin();
    return 0;
}

