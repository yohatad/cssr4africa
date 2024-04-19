#include "ros/ros.h"
#include "gesture_execution/perform_gesture.h"
#include <cstdlib>
#include <string>
#include <vector>
#include <cmath>   
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

ControlClientPtr create_client(const std::string& TopicName) {
    ControlClientPtr actionClient(new ControlClient(TopicName, true));
    int maxIterations = 10;

    for (int iterations = 0; iterations < maxIterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;
        }
        ROS_DEBUG("Waiting for the %s controller to come up", TopicName.c_str());
    }

    throw std::runtime_error("Error creating action client for " + TopicName + " controller: Server not available");
}

void move_to_position(ControlClientPtr& client, const std::vector<std::string>& jointNames, double duration, 
                        const std::string& positionName, std::vector<double> positions){
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(duration);

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed
}

// generate duration by taking the velocity max, min and home position (t = (max - min) / velocity)
std::vector<std::vector<double>> calculateDuration(std::vector<double> homePosition, std::vector<double> maxPosition, std::vector<double> minPosition, std::vector<std::vector<double>> velocity){
    
    // Initialize the duration vector similar to the velocity vector
    std::vector<std::vector<double>> duration(velocity.size(), std::vector<double>(velocity[0].size(), 0.0));
    
    // Calculate the duration for each joint check if the velocity is 0 or not
    for (int i = 0; i < homePosition.size(); ++i){
        // Calculate the duration for the first part of the trajectory
        duration[i][0] = std::fabs(minPosition[i] - homePosition[i]) / velocity[i][0];
        
        // Calculate the duration for the second part of the trajectory
        duration[i][1] = std::fabs(maxPosition[i] - minPosition[i]) / velocity[i][1];
        
        // Calculate the duration for the third part of the trajectory
        duration[i][2] = std::fabs(homePosition[i] - maxPosition[i]) / velocity[i][2];   
    }

    return duration;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "perform_gesture_client");
    if (argc != 9)
    {
        ROS_INFO("Sample usage: rosrun gesture_execution perform_gesture_client deictic 01 1000 0 0 0.0 0.0 0.0");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gesture_execution::perform_gesture>("perform_gesture");
    gesture_execution::perform_gesture srv;

    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
        ROS_FATAL("Timeout waiting for valid time");
        return EXIT_FAILURE; 
    }

    /* Initialize with default values */
    string gesture_type = "iconic";
    int gesture_id = 01;
    int gesture_duration = 1000;
    int bow_angle = 0;
    int nod_angle = 0;
    float location_x = 0.0;
    float location_y = 0.0;
    float location_z = 0.0;

    /* Parse input arguments */
    gesture_type = argv[1];
    gesture_id = atoi(argv[2]);
    gesture_duration = atoll(argv[3]);
    bow_angle = atoi(argv[4]);
    nod_angle = atoi(argv[5]);
    location_x = atof(argv[6]);
    location_y = atof(argv[7]);
    location_z = atof(argv[8]);

    /* Pass input arguments to service request */
    srv.request.gesture_type = gesture_type;
    srv.request.gesture_id = gesture_id;
    srv.request.gesture_duration = gesture_duration;
    srv.request.bow_angle = bow_angle;
    srv.request.nod_angle = nod_angle;
    srv.request.location_x = location_x;
    srv.request.location_y = location_y;
    srv.request.location_z = location_z;


  // if (client.call(srv))
  // {
  //   ROS_INFO("Status of gestureExecution: %ld", (long int)srv.response.gesture_success);
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service perform_gesture");
  //   return 1;
  // }
//   double max_angle = 45.0;
//     double set_angle = double(bow_angle); 
//     double interpolation_factor = max_angle / set_angle;
//     printf("Interpolation factor: %f\n", interpolation_factor);
//   string legTopic = "/pepper/Pelvis_controller/follow_joint_trajectory";

//   // int execution_status = bowing_gesture(bow_angle, gesture_duration, topic, true);
//     ControlClientPtr legClient = create_client(legTopic);
//     std::vector<std::string> jointNames = {"HipPitch", "HipRoll", "KneePitch"};
//     std::vector<double> position(3, 0.0);
    
    
//     // Maximum and minimum positions for each joint
//     std::vector<double> maxPosition = {1.0385,   0.5149,   0.5149};
//     std::vector<double> minPosition = {-1.0385, -0.5149 , -0.5149};
//     std::vector<double> homePosition = {-0.0107, -0.00766, 0.03221};

//     std::vector<std::vector<double>> velocities = {{0.5, 0.5, 0.5},{0.5, 0.5, 0.5},{0.5, 0.5, 0.5}};
//     std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocities);


//     ROS_INFO_STREAM("----------[START LEG CONTROL TEST]-----------");

//     // For each joint, move to the maximum position, then to the minimum position, then to the mid-range position
//     for (int i = 0; i < 1; ++i) {
//         ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

//         ROS_INFO_STREAM("Moving to the Minimum position");
//         // position[i] = minPosition[i];
//         // move_to_position(legClient, jointNames, duration[i][0], "min", position);

//         // ROS_INFO_STREAM("Moving to the Maximum position");
//         // position[i] = maxPosition[i];
//         // move_to_position(legClient, jointNames, duration[i][1], "max", position);

//         // ROS_INFO_STREAM("Moving to the Mid-range position");
//         position[i] = (homePosition[i] + minPosition[i]) / interpolation_factor;
//         move_to_position(legClient, jointNames, duration[i][2], "mid", position);

//         ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
//     }

//     // calc_velocity(homePosition, maxPosition, minPosition, duration);

//     ROS_INFO_STREAM("[PUT DOWN LEG] Moving to the Home position");
//     double homeDuration = 2.0;
//     move_to_position(legClient, jointNames, homeDuration, "home", homePosition);

//     // End of test
//     ROS_INFO_STREAM("----------[END LEG CONTROL TEST]-----------");

  return 0;
}