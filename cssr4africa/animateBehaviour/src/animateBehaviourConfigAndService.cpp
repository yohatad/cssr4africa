
/* 
 * animateBehaviourConfigAndService.cpp
 * Implementation of functionalities for a ROS node that can be configured to work with different platforms (simulators or real robots),
 * and Generate animate behavour.
 * This fmain function for this code is animateBehaviour.cpp
 */

#include "cssr_system/animateBehaviourConfigAndService.h"

// Global variables initialization
bool isActive = false; 
std::map<std::string, ros::Publisher> publishers; 
std::map<std::string, std::string> configParams; 
std::string platform; 
std::map<std::string, std::string> topicData;
double maximumRange;
double selectedRange;

/**List out all the joint name*/
std::vector<std::string> rArmJointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
std::vector<std::string> lArmJointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
std::vector<std::string> rhandJointNames = {"RHand"};
std::vector<std::string> lhandJointNames = {"LHand"};
std::vector<std::string> legJointNames = {"HipPitch", "HipRoll", "KneePitch"};

// Control
ControlClientPtr createClient(const std::string& topicName) {
    ROS_INFO("Creating action client for topic: %s", topicName.c_str());
    ControlClientPtr actionClient(new ControlClient(topicName, true));

    // Exponential backoff parameters
    int maxAttempts = 5;
    double waitTime = 5.0; // Initial wait time in seconds
    double backoffMultiplier = 2.0;

    for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
        ROS_INFO("Waiting for action server to start. Attempt %d/%d", attempt, maxAttempts);
        // Wait for the server to start
        if (actionClient->waitForServer(ros::Duration(waitTime))) {
            ROS_INFO("Action server is available after %d attempt(s)", attempt);
            return actionClient;
        } else {
            ROS_WARN("Could not connect to action server on attempt %d. Retrying...", attempt);
            waitTime *= backoffMultiplier; // Increase the wait time for the next attempt
        }
    }

    ROS_ERROR("Failed to connect to action server '%s' after %d attempts.", topicName.c_str(), maxAttempts);
    throw std::runtime_error("Error creating action client for " + topicName + ": Server not available after multiple attempts.");
}

//Activates or deactivates the animate behavior based on the service request 
bool setActivation(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    isActive = req.data; // Set the activation state based on the request
    res.success = true; // Indicate that the request was successfully processed
    ROS_INFO("Animate behaviour %s.", isActive ? "enabled" : "disabled");
    return true;
}

// Utility to trim leading and trailing whitespaces from a string
void trim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

//Load configuration file
std::string loadConfiguration(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Unable to open the config file: %s", filename.c_str());
        return "";
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream is_line(line);
        std::string key, value;
        // Use std::noskipws to consider leading whitespaces and std::skipws to ignore them after the key is extracted
        if (is_line >> std::noskipws >> key >> std::skipws && std::getline(is_line >> std::ws, value)) {
            // Find the first whitespace in key and erase from it to the end to get the actual key
            auto firstSpace = key.find(' ');
            if (firstSpace != std::string::npos) {
                key.erase(firstSpace);
            }
            trim(key);
            trim(value);
            configParams[key] = value; // Store the configuration parameter
        }
    }
    file.close();
    // Set the platform based on the configuration
    std::string platform = configParams["platform"];
    ROS_INFO("Configuration loaded successfully. Platform: %s", platform.c_str());
    return platform; 
}

//Load the data
void loadDataBasedOnPlatform(const std::string& platform) {
    std::string dataFilePath = ros::package::getPath("cssr_system") + "/data/";
    // Select the appropriate data file based on the platform
    if (platform == "robot") {
        dataFilePath += "pepperTopics.dat";
    } else if (platform == "simulator") {
        dataFilePath += "simulatorTopics.dat";
    } else {
        ROS_ERROR("Unknown platform: %s", platform.c_str());
        return;
    }

    std::ifstream file(dataFilePath);
    if (!file.is_open()) {
        ROS_ERROR("Unable to open the data file: %s", dataFilePath.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream is_line(line);
        std::string key, value;
        // Read the key up to the first whitespace and the rest of the line as value
        if (is_line >> std::noskipws >> key >> std::skipws && std::getline(is_line >> std::ws, value)) {
            // Correct the key by removing any trailing spaces
            auto firstSpace = key.find(' ');
            if (firstSpace != std::string::npos) {
                key.erase(firstSpace);
            }
            trim(key);
            trim(value);
            topicData[key] = value; // Store the data
        }
    }

    // After loading the data and before closing the file
    for (const auto& pair : topicData) {
        ROS_INFO("Key: %s, Value: %s", pair.first.c_str(), pair.second.c_str());
    }

    file.close();
    ROS_INFO("Data loaded successfully for platform: %s", platform.c_str());
}

//Helper function to generate a random double in a range 
double randomDoubleInRange(double min, double max) {
    static std::default_random_engine engine(std::random_device{}());
    std::uniform_real_distribution<double> distribution(min, max);
    return distribution(engine);
}

// seed the random number generator
void initRandomSeed() {
    static bool initialized = false;
    if (!initialized) {
        std::srand(std::time(nullptr));
        initialized = true;
    }
}

/**
 * Moves the robot to a specific position by sending a FollowJointTrajectoryGoal to an action server.
 * The function sends a goal for the specified joint positions and waits up to 30 seconds for completion.
 * Success or failure, along with specific error states like 'rejected' or 'aborted', are logged.
 */
void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames,
                    const std::string& positionName, std::vector<double> positions) {
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;

    // Assign the joint names to the trajectory
    trajectory.joint_names = jointNames;

    // Resize the trajectory points array to 1 and assign positions and duration
    trajectory.points.resize(1);
    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(5.0); // Set the time from the start to 5 seconds

    // print out the trajectory points for debugging
    ROS_INFO("Moving to %s position with the following joint positions:", positionName.c_str());
    
    // Send the goal to the action server
    client->sendGoal(goal);

    // Wait for the action to finish and check the result.
    bool finishedBeforeTimeout = client->waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed

    if (finishedBeforeTimeout) {
        actionlib::SimpleClientGoalState state = client->getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully moved to %s position.", positionName.c_str());
        } else {
            ROS_WARN("The action failed to move to %s position. State: %s", positionName.c_str(), state.toString().c_str());
        }
    } else {
        ROS_WARN("The action did not finish before the timeout.");
    }
}

/**
 * Calculates a target position for robot movement.
 * 
 * Generates a target position within specified ranges.
 * It ensures the target position is within the max and min positions
 * and adjusts based on the 'maximumRange' and 'selectedRange' from configParams.
 * 
 * @param homePosition The current or home position of the robot's joints.
 * @param maxPosition The maximum allowable position for each joint.
 * @param minPosition The minimum allowable position for each joint.
 * @return A vector containing the calculated target positions for each joint.
 */
std::vector<double> calculateTargetPosition(const std::vector<double>& homePosition,
                                            const std::vector<double>& maxPosition,
                                            const std::vector<double>& minPosition) {
    std::vector<double> targetPosition(homePosition.size(), 0.0);
    initRandomSeed(); // Ensure random seed is initialized for reproducible results

    // Retrieve maximum and selected ranges from configuration parameters
    double maxRange = std::stod(configParams["maximumRange"]);
    double selectedRange = std::stod(configParams["selectedRange"]);

    for (size_t i = 0; i < homePosition.size(); ++i) {
        double fullRange = maxPosition[i] - minPosition[i]; // Total possible movement range
        double maximumRangeOffset = fullRange * maxRange; // Adjusted range limit
        double selectedRangeOffset = (maximumRangeOffset * selectedRange) / 2.0; // Selected movement range

        // Calculate maximum and minimum positions within the selected range
        double tempPositionMax = std::min(homePosition[i] + selectedRangeOffset, maxPosition[i]);
        double tempPositionMin = std::max(homePosition[i] - selectedRangeOffset, minPosition[i]);

        // Generate a random position within the allowable range
        double tempPosition = randomDoubleInRange(tempPositionMin, tempPositionMax);
        targetPosition[i] = tempPosition; // Assign calculated position to target
    }
    return targetPosition; // Return the calculated target positions
}

/**
 * Animates the right arm
 *
 * @param nh ROS NodeHandle 
 * @param rightArmTopic The ROS topic name for controlling the right arm's movement.
 * @param start The index of the first joint to animate.
 * @param end The index after the last joint to animate.
 * 
 * Initializes movement control for the right arm, generates target positions within
 * given maximum and minimum ranges, and sequentially moves the specified joints. 
 * Finally, it returns the arm to its home position.
 */
void rArm(ros::NodeHandle& nh, std::string rightArmTopic, int start, int end) {
    ControlClientPtr rightArmClient = createClient(rightArmTopic);
    
    // Define initial, maximum, minimum, and home positions for the arm's joints
    std::vector<double> position = {1.7410, -0.09664, 0, 0, -0.05679};
    std::vector<double> maxPosition = {2.0857, -0.0087, 1.5620, 2.0857, 1.8239};
    std::vector<double> minPosition = {-2.0857, -1.5620, 0.0087, -2.0857, -1.5620};
    std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    
    ROS_INFO_STREAM("----------[START RIGHT ARM ANIMATE MOVEMENT]-----------");
    
    // Animate the specified joints by moving them to random target positions within the allowed range
    for (int i = start; i < end; ++i) {
        std::vector<double> targetPosition = calculateTargetPosition(homePosition, maxPosition, minPosition);
        position[i] = targetPosition[i]; 
        moveToPosition(rightArmClient, rArmJointNames, "random", position);
    }
    
    // Move the arm to the home position after animation
    ROS_INFO_STREAM("[PUT DOWN RIGHT ARM] MOVING TO THE HOME POSITION");
    moveToPosition(rightArmClient, rArmJointNames, "home", homePosition);
    
    ROS_INFO_STREAM("----------[END RIGHT ARM ANIMATE MOVEMENT]-----------");
}

/**
 * Animates the right hand 
 * 
 * @param nh ROS NodeHandle 
 * @param rightHandTopic The ROS topic name for controlling the right hand's movement.
 * @param start The index of the first finger joint to control.
 * @param end The index after the last finger joint to control.
 * 
 * Initializes movement control for the right hand, generates target positions
 * within given maximum and minimum ranges, and moves the specified joints.
 * Finally, it returns the hand to its home position.
 */
void rHand(ros::NodeHandle& nh, std::string rightHandTopic, int start, int end) {
    ControlClientPtr rightHandClient = createClient(rightHandTopic);
    
    // Initialize the position with a single joint for the hand
    std::vector<double> position(1, 0.0);
    // Define maximum and minimum positions for the hand's joint
    std::vector<double> maxPosition = {1.0};
    std::vector<double> minPosition = {0.0};
    std::vector<double> homePosition = {0.66608};

    ROS_INFO_STREAM("----------[START RIGHT HAND ANIMATE MOVEMENT]-----------");
    
    // Move the hand's joint to random target positions within the allowed range
    for (int i = start; i < end; ++i) {
        std::vector<double> targetPosition = calculateTargetPosition(homePosition, maxPosition, minPosition);
        position[i] = targetPosition[i]; 
        moveToPosition(rightHandClient, rhandJointNames, "random", position);
    }
    
    // Move the hand to the home position after control test
    ROS_INFO_STREAM("[PUT DOWN RHAND] MOVING TO THE HOME POSITION");
    moveToPosition(rightHandClient, rhandJointNames, "home", homePosition);
    
    ROS_INFO_STREAM("----------[END RIGHT HAND ANIMATE MOVEMENT]-----------");
}

/**
 * Animates the left arm.
 * 
 * @param nh ROS NodeHandle 
 * @param leftArmTopic The ROS topic name for controlling the left arm's movement.
 * @param start The index of the first joint to control.
 * @param end The index after the last joint to control.
 * 
 * Initializes movement control for the left arm, generates target positions
 * within given maximum and minimum ranges, and moves the specified joints. 
 * Finally, it returns the arm to its home position.
 */
void lArm(ros::NodeHandle& nh, std::string leftArmTopic, int start, int end) {
    ControlClientPtr leftArmClient = createClient(leftArmTopic);
    
    // Initialize the position with default values for the left arm's joints
    std::vector<double> position = {1.7625, 0.09970, -0.1334, -1.7150, 0.06592};
    
    // Define maximum and minimum positions for the left arm's joints
    std::vector<double> maxPosition = {2.0857, 1.5620, -0.0087, 2.0857, 1.8239}; 
    std::vector<double> minPosition = {-2.0857, 0.0087, -1.5620, -2.0857, -1.8239};
    std::vector<double> homePosition = {1.7625, 0.09970, -0.1334, -1.7150, 0.06592};

    ROS_INFO_STREAM("----------[START LEFT ARM ANIMATE MOVEMENT]-----------");
    
    // Move the specified joints to random target positions within the allowed range
    for (int i = start; i < end; ++i) {
        std::vector<double> targetPosition = calculateTargetPosition(homePosition, maxPosition, minPosition);
        position[i] = targetPosition[i];
        
        for (size_t i = 0; i < targetPosition.size(); ++i) {
            ROS_INFO("Calculated target position for joint %s: %.4f", lArmJointNames[i].c_str(), position[i]);
        }
        moveToPosition(leftArmClient, lArmJointNames, "random", position);
    
    }    
    // Move the arm to the home position after control test
    ROS_INFO_STREAM("[PUT DOWN LARM] MOVING TO THE HOME POSITION");
    moveToPosition(leftArmClient, lArmJointNames, "home", homePosition);
    
    ROS_INFO_STREAM("----------[END LEFT ARM ANIMATE MOVEMENT]-----------");
}

/**
 * Animates the left hand.
 *
 * @param nh ROS NodeHandle 
 * @param leftHandTopic The ROS topic name for controlling the left hand's movement.
 * @param start The index of the first finger joint to control.
 * @param end The index after the last finger joint to control.
 * 
 * Initializes movement control for the left hand, generates target positions
 * within given maximum and minimum ranges, and moves the specified joints. 
 * Finally, it returns the hand to its home position. 
 */
void lHand(ros::NodeHandle& nh, std::string leftHandTopic, int start, int end) {
    ControlClientPtr leftHandClient = createClient(leftHandTopic);
    
    // Initialize the position with a single joint for the hand
    std::vector<double> position(1, 0.0);
    // Define maximum and minimum positions for the hand's joint
    std::vector<double> maxPosition = {1.0};
    std::vector<double> minPosition = {0.0};
    std::vector<double> homePosition = {0.6695};

    ROS_INFO_STREAM("----------[START LEFT HAND ANIMATE MOVEMENT]-----------");
    
    // Sequentially test each joint within the specified range
    for (int i = start; i < end; ++i) {
        ROS_INFO_STREAM("[START] " << lhandJointNames[i] << " test."); 
        std::vector<double> targetPosition = calculateTargetPosition(homePosition, maxPosition, minPosition);
        position[i] = targetPosition[i];

        for (size_t i = 0; i < targetPosition.size(); ++i) {
            ROS_INFO("Calculated target position for joint %s: %.4f", lhandJointNames[i].c_str(), position[i]);
        }

        moveToPosition(leftHandClient, lhandJointNames, "random", position); 
        ROS_INFO_STREAM("[END] " << lhandJointNames[i] << " test."); 
    }
    
    // Move the hand to the home position after control test
    ROS_INFO_STREAM("[PUT DOWN LHAND] MOVING TO THE HOME POSITION"); 
    moveToPosition(leftHandClient, lhandJointNames, "home", homePosition);
    
    ROS_INFO_STREAM("----------[END LEFT HAND ANIMATE MOVEMENT]-----------");
}

/**
 * Animates the leg.
 * 
 * @param nh ROS NodeHandle 
 * @param legTopic The ROS topic name for controlling the leg's movement.
 * @param start The index of the first leg joint to control.
 * @param end The index after the last leg joint to control.
 * 
 * Initializes movement control for a leg, generates target positions
 * within given maximum and minimum ranges for specified joints.
 * Finally, it returns the leg to its home position.
 */
void leg(ros::NodeHandle& nh, std::string legTopic, int start, int end) {
    ControlClientPtr legClient = createClient(legTopic);
    
    // Initialize positions for three joints of a leg
    std::vector<double> position(3, 0.0);
    // Define maximum and minimum positions for the leg's joints
    std::vector<double> maxPosition = {1.0385, 0.5149, 0.5149};
    std::vector<double> minPosition = {-1.0385, -0.5149, -0.5149};
    std::vector<double> homePosition = {-0.0107, -0.00766, 0.03221};

    ROS_INFO_STREAM("----------[START LEG ANIMATE MOVEMENT]-----------");
    
    // Sequentially test each joint within the specified range
    for (int i = start; i < end; ++i) {
        ROS_INFO_STREAM("[START] " << legJointNames[i] << " test.");
        std::vector<double> targetPosition = calculateTargetPosition(homePosition, maxPosition, minPosition);
        position[i] = targetPosition[i]; 
        moveToPosition(legClient, legJointNames, "random", position);
        ROS_INFO_STREAM("[END] " << legJointNames[i] << " test.");
    }
    
    // Move the leg to the home position after control test
    ROS_INFO_STREAM("[PUT DOWN LEG] MOVING TO THE HOME POSITION"); 
    moveToPosition(legClient, legJointNames, "home", homePosition);
    
    ROS_INFO_STREAM("----------[END LEG ANIMATE MOVEMENT]-----------");
}

/**
 * Calculates an angular velocity in the Z-axis.
 * Calculates using the maximum angular velocity.
 *
 * @param maxAngularVelocity The maximum allowable angular velocity.
 * @return The calculated angular velocity in Z-axis within the defined range.
 */
double calculateAngularVelocityZ(double maxAngularVelocity) {
    // Retrieve maximum and selected ranges from configuration parameters
    double maxRange = std::stod(configParams["maximumRange"]);
    double selectedRange = std::stod(configParams["selectedRange"]);
    double homeAngularVelocity = 0.0; 

    // Calculate the full possible range of angular velocity from the base to the maximum
    double fullRange = maxAngularVelocity - homeAngularVelocity;
    // Apply the maximum range to narrow down the full range
    double maximumRangeOffset = fullRange * maxRange;
    // Apply the selected range to find the actual target range within the maximum offset
    double selectedRangeOffset = maximumRangeOffset * selectedRange;

    // Determine the maximum and minimum angular velocity within the selected range
    double tempVelocityMax = homeAngularVelocity + selectedRangeOffset;
    double tempVelocityMin = homeAngularVelocity - selectedRangeOffset;

    // Randomly select an angular velocity within this range
    double angularVelocityZ = randomDoubleInRange(tempVelocityMin, tempVelocityMax);

    return angularVelocityZ; // Return the calculated angular velocity
}

/**
 * Initiates a rotation of the robot base using a calculated angular velocity.
 * The angular velocity  calculated using calculateAngularVelocityZ function.
 * 
 * @param nh The ROS NodeHandle 
 */
void rotationBaseShift(ros::NodeHandle& nh) {
    // Define the maximum angular velocity
    double maxAngularVelocity = 0.2; // to be edited (Yohannes)
    // Calculate a random angular velocity in the Z-axis
    double angularVelocityZ = calculateAngularVelocityZ(maxAngularVelocity);
    // Retrieve the command velocity topic name from the topic data configuration
    std::string cmdVelTopic = topicData["Wheels"];
    // Initialize the velocity publisher on the command velocity topic
    ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 10);
    ros::Duration(0.5).sleep(); // Brief pause to ensure the publisher is set up

    geometry_msgs::Twist twist; // Message to command rotation
    double rotationDuration = 5.0; // Duration for the rotation in seconds
    ros::Time startTime = ros::Time::now(); // Record the start time

    ROS_INFO("Starting rotation base shift with calculated angular velocity.");
    // Rotate for the specified duration
    while (ros::ok() && (ros::Time::now() - startTime) < ros::Duration(rotationDuration)) {
        twist.angular.z = angularVelocityZ; // Set the calculated angular velocity
        velPub.publish(twist); // Publish the command
        ros::Duration(0.1).sleep(); // Short pause between commands for smoother rotation
    }
    
    twist.angular.z = 0; // Command to stop the rotation
    velPub.publish(twist); // Publish the stop command
    ROS_INFO("Rotation base shift with calculated angular velocity completed.");
}

/**
 * Executes subtle body movements for the robot and simulates lifelike behavior.
 * For the robot platform, the function controls the left and right arms, hands, and legs.
 * for the simulator platform, the function controls the left and right arms and legs.
 *
 * @param nh The ROS NodeHandle.
 */
void subtleBodyMovement(ros::NodeHandle& nh) {
    ROS_INFO("Subtle body movement function started");

    // Handle simulator platform: Control both arms and legs simultaneously
    if (configParams["platform"] == "simulator") {
        // Retrieve topic names for arms and legs from configuration
        std::string rightArmTopic = topicData["RArm"];
        std::string leftArmTopic = topicData["LArm"];
        std::string legTopic = topicData["Leg"];
        
        // Launch threads for simultaneous control of left and right arms and legs
        std::thread lArmThread(lArm, std::ref(nh), leftArmTopic, 2, static_cast<int>(lArmJointNames.size()) - 1);
        // std::thread rArmThread(rArm, std::ref(nh), rightArmTopic, 2, static_cast<int>(rArmJointNames.size()) - 1);
        // std::thread legThread(leg, std::ref(nh), legTopic, 0, static_cast<int>(legJointNames.size()));

        // Wait for all limb control threads to complete
        lArmThread.join();
        // rArmThread.join();
        // legThread.join();
    }
    // Handle physical robot platform: Control arms, hands, and legs simultaneously
    else if (configParams["platform"] == "robot") {
        // Retrieve topic names for arms, hands, and legs from configuration
        std::string rightArmTopic = topicData["RArm"];
        std::string leftArmTopic = topicData["LArm"];
        std::string rightHandTopic = topicData["RHand"];
        std::string leftHandTopic = topicData["LHand"];
        std::string legTopic = topicData["Leg"];

        // lArm(nh, leftArmTopic, 0, static_cast<int>(lArmJointNames.size()));
        lHand(nh, leftHandTopic, 0, static_cast<int>(lhandJointNames.size()));

        // Launch threads for simultaneous control of left and right arms and hands, and legs
        // std::thread lArmThread(lArm, std::ref(nh), leftArmTopic, 0, static_cast<int>(lArmJointNames.size()));
        // std::thread rArmThread(rArm, std::ref(nh), rightArmTopic, 0, static_cast<int>(rArmJointNames.size()));
        // std::thread lHandThread(lHand, std::ref(nh), leftHandTopic, 0, static_cast<int>(lhandJointNames.size()));
        // std::thread rHandThread(rHand, std::ref(nh), rightHandTopic, 0, static_cast<int>(rhandJointNames.size()));
        // std::thread legThread(leg, std::ref(nh), legTopic, 0, static_cast<int>(legJointNames.size()));
        
        // Wait for all limb control threads to complete
        // lArmThread.join();
        // rArmThread.join();
        // lHandThread.join();
        // rHandThread.join();
        // legThread.join();
    }

    ROS_INFO("Subtle body movement function ended");
}

/**
 * Flexi movement focused on robot's hands and wrists only.
 * Notifies when the function is not implemented for simulators.
 *
 * @param nh The ROS NodeHandle to manage communication with ROS.
 */
void flexiMovement(ros::NodeHandle& nh) {
    ROS_INFO("FlexiMovement function started");

    // Check if the platform configuration is set to "robot"
    if (configParams["platform"] == "robot") {
        // Retrieve topic names for arms and hands from configuration data
        std::string rightArmTopic = topicData["RArm"];
        std::string leftArmTopic = topicData["LArm"];
        std::string rightHandTopic = topicData["RHand"];
        std::string leftHandTopic = topicData["LHand"];

        // Launch threads for controlling the wrist and hand of both left and right sides
        std::thread lArmThread(lArm, std::ref(nh), leftArmTopic, lArmJointNames.size() - 1, lArmJointNames.size());
        std::thread rArmThread(rArm, std::ref(nh), rightArmTopic, rArmJointNames.size() - 1, rArmJointNames.size());
        std::thread lHandThread(lHand, std::ref(nh), leftHandTopic, 0, lhandJointNames.size());
        std::thread rHandThread(rHand, std::ref(nh), rightHandTopic, 0, rhandJointNames.size());

        // Wait for all movement threads to complete
        lArmThread.join();
        rArmThread.join();
        lHandThread.join();
        rHandThread.join();
    } else {
        // Notify that FlexiMovement is not implemented for Simulator
        ROS_INFO("Flexi hand movement is implemented only for the robot, not for the simulator.");
    }

    ROS_INFO("FlexiMovement function ended");
}

/**
 * Animation behaviors based on the given behavior parameter.
 * Activate subtle body movements, hand flex movements, or base rotation, 
 * depending on the input or perform all actions if no specific behavior is requested.
 *
 * @param behaviour A string indicating the desired animation behavior(s).
 * @param nh  ROS NodeHandle 
 */
void animateBehaviour(const std::string& behaviour, ros::NodeHandle& nh) 
{
    // if (!isActive) {
    //     ROS_INFO("Animation behavior is inactive.");
    //     return; // Exit if the animation behavior is not active
    // }

    ROS_INFO("[animateBehaviour] Invoked with behaviour: %s", behaviour.c_str());

    // Check for specific behaviors within the 'behaviour' parameter and execute accordingly
    if (behaviour.find("body") != std::string::npos) {
        ROS_INFO("[animateBehaviour] Executing subtle body movements.");
        subtleBodyMovement(nh);
    }
    if (behaviour.find("hands") != std::string::npos) {
        ROS_INFO("[animateBehaviour] Executing hand flex movements.");
        flexiMovement(nh);
    }
    if (behaviour.find("rotation") != std::string::npos) {
        ROS_INFO("[animateBehaviour] Executing rotation base shift.");
        rotationBaseShift(nh);
    }

    // If no specific behaviour is provided, perform all available movements
    if (behaviour.empty()) {
        ROS_INFO("[animateBehaviour] No specific behaviour requested. Executing all movements.");
        subtleBodyMovement(nh);
        flexiMovement(nh);
        rotationBaseShift(nh);
    }
}

