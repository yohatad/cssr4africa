
/* 
 * animateBehaviourConfigAndService.cpp
 * Implementation of functionalities for a ROS node that can be configured to work with different platforms (simulators or real robots),
 * and Generate animate behavour.
 * This fmain function for this code is animateBehaviour.cpp
 */

#include "animate_behaviour/animateBehaviourInterface.h"

#include "cssr_system/set_activation.h" 

// Global variables initialization
bool verboseMode; // It is used to display diagnostic information
std::map<std::string, std::string> configParams; 
// std::string platform; 
std::map<std::string, std::string> topicData;
double maximumRange;
double selectedRange;


/**List out all the joint name*/
std::vector<std::string> rArmJointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
std::vector<std::string> lArmJointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
std::vector<std::string> rhandJointNames = {"RHand"};
std::vector<std::string> lhandJointNames = {"LHand"};
std::vector<std::string> legJointNames = {"HipPitch", "HipRoll", "KneePitch"};


std::string vectorToString(const std::vector<double>& vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i < vec.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";
    return oss.str();
}


std::string vector2dToString(const std::vector<std::vector<double>>& vec2d) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec2d.size(); ++i) {
        oss << vectorToString(vec2d[i]);
        if (i < vec2d.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";
    return oss.str();
}


std::string vectorOfStringsToString(const std::vector<std::string>& vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        oss << "\"" << vec[i] << "\"";
        if (i < vec.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";
    return oss.str();
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

// Load configuration file
void loadConfiguration(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Unable to open the config file: %s", filename.c_str());
        logToFile("Unable to open the config file: " + filename);
        return;
    }
    ROS_INFO("Configuration file opened successfully: %s", filename.c_str());
    logToFile("Configuration file opened successfully: " + filename);
    
    configParams.clear(); // Clear existing configuration
    
    std::string line;
    while (std::getline(file, line)) {
        // Remove any carriage return characters
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        
        // Trim leading and trailing whitespace
        auto trim = [](std::string& s) {
            s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
                return !std::isspace(ch);
            }));
            s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
                return !std::isspace(ch);
            }).base(), s.end());
        };
        trim(line);
        
        // Skip empty lines
        if (line.empty()) continue;
        
        // Split the line into key and value
        size_t delimiterPos = line.find_first_of("\t ");
        if (delimiterPos != std::string::npos) {
            std::string key = line.substr(0, delimiterPos);
            std::string value = line.substr(delimiterPos + 1);
            
            // Trim key and value
            trim(key);
            trim(value);
            
            // Store in configParams, including empty values
            configParams[key] = value;
            ROS_INFO("Loaded config: %s = \"%s\"", key.c_str(), value.c_str());
        }
    }
    file.close();
    
    // If behaviour is not in the configuration, add it with an empty value
    if (configParams.find("behaviour") == configParams.end()) {
        configParams["behaviour"] = "";
        ROS_INFO("Behaviour not found in configuration. Added with empty value.");
    }
    
    // After loading all parameters
    ROS_INFO("Configuration loading complete. Params loaded: %zu", configParams.size());
    for (const auto& param : configParams) {
        ROS_INFO("Config: %s = \"%s\"", param.first.c_str(), param.second.c_str());
    }
    
    // Set verbose mode
    verboseMode = (configParams.find("verboseMode") != configParams.end() && configParams["verboseMode"] == "true");
    ROS_INFO("Verbose mode set to: %s", verboseMode ? "true" : "false");
    
    // Check for platform
    if (configParams.find("platform") != configParams.end()) {
        ROS_INFO("Loaded platform from config: %s", configParams["platform"].c_str());
    } else {
        ROS_ERROR("Platform not found in configuration file");
    }

    // Log the behaviour value (which is now guaranteed to exist)
    ROS_INFO("Loaded behaviour from config: \"%s\"", configParams["behaviour"].c_str());
    // log all the configuration parameters
    for (const auto& pair : configParams) {
        logToFile("Config: " + pair.first + " = " + pair.second);
    }
}


// Load the data
void loadDataBasedOnPlatform(const std::string& platform) {
    std::string base_path = ros::package::getPath(ROS_PACKAGE_NAME);
    std::string dataFilePath = base_path + "/animateBehaviour/data/";
    
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
        logToFile("Key: " + pair.first + ", Value: " + pair.second);
    }
    file.close();
    if (verboseMode) {
        ROS_INFO("Data loaded successfully for platform: %s", platform.c_str());
        logToFile("Data loaded successfully for platform: " + platform);
    }
}


/**
 * @brief Creates an action client for the given topic and connects to the action server with exponential backoff.
 *
 * This function attempts to create an action client and connect to the action server.
 * If verbose mode is enabled, diagnostic data is printed to the terminal 
 *
 * @param topicName The name of the ROS topic to create the action client for.
 * @return A pointer to the created action client.
 * @throws std::runtime_error if the connection to the action server fails after multiple attempts.
 */
ControlClientPtr createClient(const std::string& topicName) {
    if (verboseMode) {
        ROS_INFO("Creating action client for topic: %s", topicName.c_str());
    }
    ControlClientPtr actionClient(new ControlClient(topicName, true));

    // Exponential backoff parameters
    int maxAttempts = 5;
    double waitTime = 5.0; // Initial wait time in seconds
    double backoffMultiplier = 2.0;
    for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
        if (verboseMode) {
            ROS_INFO("Waiting for action server to start. Attempt %d/%d", attempt, maxAttempts);
                
        }
        // Wait for the server to start
        if (actionClient->waitForServer(ros::Duration(waitTime))) {
            ROS_INFO("Action server is available after %d attempt(s)", attempt);
            return actionClient;
        } 
        else {
            ROS_WARN("Could not connect to action server on attempt %d. Retrying...", attempt);
            waitTime *= backoffMultiplier; // Increase the wait time for the next attempt
        }
    }
    if (verboseMode) {
        ROS_ERROR("Failed to connect to action server '%s' after %d attempts.", topicName.c_str(), maxAttempts);
    }
    throw std::runtime_error("Error creating action client for " + topicName + ": Server not available after multiple attempts.");
}


/**
 * @brief Activates or deactivates the animate behavior based on the service request.
 *
 * This function sets the `isActive` flag based on the request and indicates
 * the success of the operation. If verbose mode is enabled, diagnostic data is
 * printed to the terminal and a diagnostic image is displayed in an OpenCV window.
 *
 * @param req The service request containing the desired activation state.
 * @param res The service response indicating success.
 * @return True if the request was successfully processed.
 */
bool setActivation(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    isActive = req.data; 
    res.success = true; 
    if (verboseMode) {
        
        ROS_INFO("Animate behaviour %s.", isActive ? "enabled" : "disabled");
        logToFile("Animate behaviour " + std::string(isActive ? "enabled" : "disabled"));
    } else {
        ROS_INFO("Animate behaviour %s.", isActive ? "enabled" : "disabled");
        logToFile("Animate behaviour " + std::string(isActive ? "enabled" : "disabled"));
    }
    return true;
}


double generateRandomPosition(double min, double max) {
    std::random_device rd;  // Create a random device
    std::mt19937 engine(rd());  // Initialize a random engine
    std::uniform_real_distribution<> distribution(min, max);  // Create a distribution between min and max
    
    return distribution(engine);  // Generate the random number
}


// seed the random number generator
void initRandomSeed() {
    static bool initialized = false;
    if (!initialized) {
        std::srand(std::time(nullptr));
        initialized = true;
    }
}
// Function to check if it's the first run

bool isFirstRun() {
    std::ifstream infile(FLAG_FILE_PATH);
    bool firstRun = !infile.good();
    infile.close();
    return firstRun;
}
// Function to update the flag after the first run

void updateFirstRunFlag() {
    std::ofstream outfile("first_run_flag.txt");
    outfile << "false";
    outfile.close();
}

/**
 * @brief Moves the robot to a specific position by sending a FollowJointTrajectoryGoal to an action server.
 *
 * This function sends a goal for the specified joint positions and waits up to 10 seconds for completion.
 * If verbose mode is enabled, diagnostic data is printed to the terminal and a diagnostic image is displayed
 * in an OpenCV window.
 *
 * This function is used when biological motion is not required.
 *
 * @param client The action client to send the goal to.
 * @param jointNames The names of the joints to move.
 * @param positionName A string identifier for the position.
 * @param positions The target positions for the joints.
 */
void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames,
                    const std::string& positionName, std::vector<double> positions) {
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;
    // Resize the trajectory points array
    trajectory.points.resize(1);
    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(0.2); 

    // Print out the trajectory points
    if (verboseMode) {
        ROS_INFO("Moving to %s position with the following joint positions:", positionName.c_str());
        }

    // Send the goal to the action server
    client->sendGoal(goal);
    // Wait for the action to finish and check the result.
    bool finishedBeforeTimeout = client->waitForResult(ros::Duration(10.0)); 
    if (finishedBeforeTimeout) {
        actionlib::SimpleClientGoalState state = client->getState();
        if (verboseMode){
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            if (verboseMode){
                ROS_INFO("Successfully moved to %s position.", positionName.c_str());
            }
            } else {
                ROS_WARN("The action failed to move to %s position. State: %s", positionName.c_str(), state.toString().c_str());
            }

    } else {
        ROS_WARN("The action did not finish before the timeout.");
    }
}


/**
 * @brief Moves the robot joints to specified positions using a biologically-inspired trajectory.
 *
 * This function sends a trajectory goal to an action server, using positions, velocities, and accelerations
 * that are calculated to simulate biological motion. If verbose mode is enabled, diagnostic data is printed
 * to the terminal and a diagnostic image is displayed in an OpenCV window.
 *
 * @param client The action client used to send goals.
 * @param jointNames The names of the joints to be moved.
 * @param gesture_duration The duration of the gesture.
 * @param positionName The name of the target position.
 * @param positions The target positions for each joint.
 * @param velocities The target velocities for each joint.
 * @param accelerations The target accelerations for each joint.
 * @param durations The durations for each trajectory point.
 */
void moveToPositionBiological(ControlClientPtr& client, 
                              const std::vector<std::string>& jointNames,
                              const std::string& positionName, 
                              const std::vector<std::vector<double>>& positions, 
                              const std::vector<std::vector<double>>& velocities,
                              const std::vector<std::vector<double>>& accelerations, 
                              const std::vector<double>& durations) {
    
    // Initialize the goal message for controlling Pepper's joints
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;

    // Assign the joint names to the trajectory
    trajectory.joint_names = jointNames;

    // Ensure trajectory points size matches the number of positions
    size_t numPoints = positions.size();
    trajectory.points.resize(numPoints);

    if (verboseMode) {
        ROS_INFO("Number of trajectory points: %zu", numPoints);
    }
    
    // Keep track of cumulative time for each point in the trajectory
    double cumulativeTime = 0.0;

    // Populate the trajectory points with positions, velocities, accelerations, and cumulative time
    for (size_t i = 0; i < numPoints; ++i) {
        if (positions[i].size() != jointNames.size() || 
            velocities[i].size() != jointNames.size() || 
            accelerations[i].size() != jointNames.size()) {
            ROS_ERROR("Mismatch in vector sizes for joint %zu", i);
            return;
        }

        // Assign positions, velocities, and accelerations to the trajectory point
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = positions[i];
        point.velocities = velocities[i];
        point.accelerations = accelerations[i];

        // Increment cumulative time
        cumulativeTime += durations[i];
        point.time_from_start = ros::Duration(cumulativeTime);  // Set cumulative time

        trajectory.points[i] = point;

        // if (verboseMode) {
        //     ROS_INFO("Trajectory point %zu: Cumulative Duration %.2f seconds", i, cumulativeTime);
        // }
    }

    // Send the trajectory goal to the action server (Pepper's controller)
    client->sendGoal(goal);

    // Compute the total gesture duration (cumulative time of the last point)
    double totalGestureDuration = cumulativeTime + 5.0; // Add a 1-second buffer for timeout

    // Wait for the action to complete within the total gesture duration
    bool finishedBeforeTimeout = client->waitForResult(ros::Duration(totalGestureDuration));

    // Check the result and provide feedback
    if (finishedBeforeTimeout) {
        actionlib::SimpleClientGoalState state = client->getState();
        if (verboseMode) {
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }

        // Check if the action succeeded or failed
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            if (verboseMode) {
                ROS_INFO("Successfully moved to %s position.", positionName.c_str());
            }
        } else {
            ROS_WARN("Failed to move to %s position. State: %s", positionName.c_str(), state.toString().c_str());
        }
    } else {
        ROS_WARN("The action did not finish before the timeout.");
    }
}




/**
 * @brief Parses percentage values from a configuration string.
 *
 * This function reads a comma-separated string of percentages and converts them to a vector of doubles.
 * If verbose mode is enabled, it prints the parsed percentages to the terminal and displays a diagnostic
 * image in an OpenCV window.
 *
 * @param percentagesStr The string containing comma-separated percentage values.
 * @return A vector of doubles representing the parsed percentages.
 */
std::vector<double> parsePercentages(const std::string& percentagesStr) {
    std::vector<double> percentages;
    std::istringstream percentagesStream(percentagesStr);
    std::string percentage;
    while (std::getline(percentagesStream, percentage, ',')) {
        try {
            percentages.push_back(std::stod(percentage)); // Directly use the percentage
        } catch (const std::invalid_argument& e) {
            ROS_ERROR("Invalid argument in configuration: %s", percentage.c_str());
        } catch (const std::out_of_range& e) {
            ROS_ERROR("Out of range error in configuration: %s", percentage.c_str());
        }
    }
    return percentages;
}


void logTrajectoryData(const std::string& filename,
                       const std::vector<std::vector<double>>& positions,
                       const std::vector<std::vector<double>>& velocities,
                       const std::vector<std::vector<double>>& accelerations,
                       const std::vector<double>& durations, bool append) {
    std::ofstream file;

    // Open the file in append mode if specified, otherwise in write mode
    if (append) {
        file.open(filename, std::ios_base::app); // Append mode
    } else {
        file.open(filename); // Overwrite mode
    }

    // Write the headers only if not appending
    if (!append) {
        file << "Index,Position_1,Position_2,Position_3,Velocity_1,Velocity_2,Velocity_3,Acceleration_1,Acceleration_2,Acceleration_3,Duration\n";
    }

    // Log the trajectory data
    for (size_t i = 0; i < positions.size(); ++i) {
        file << i; // Index
        for (size_t j = 0; j < positions[i].size(); ++j) {
            file << "," << positions[i][j]; // Log each position component
        }
        // for (size_t j = 0; j < velocities[i].size(); ++j) {
        //     file << "," << velocities[i][j]; // Log each velocity component
        // }
        // for (size_t j = 0; j < accelerations[i].size(); ++j) {
        //     file << "," << accelerations[i][j]; // Log each acceleration component
        // }
        // file << "," << durations[i]; // Log the duration
        file << "\n";
    }

    file.close();
    // ROS_INFO("Appended trajectory data to %s", filename.c_str());
}


/**
 * @brief Calculates a target position for robot movement.
 * Generates a target position within specified ranges.
 * It ensures the target position is within the max and min positions
 * and adjusts based on the 'maximumRange' and 'selectedRange' from configParams.
 * 
 * @param homePosition The current or home position of the robot's joints.
 * @param maxPosition The maximum allowable position for each joint.
 * @param minPosition The minimum allowable position for each joint.
 * @return A vector containing the calculated target positions for each joint.
 */
std::vector<std::vector<double>> calculateTargetPosition(const std::vector<double>& homePosition,
                                                         const std::vector<double>& maxPosition,
                                                         const std::vector<double>& minPosition,
                                                         const std::string& jointType) {
    std::vector<std::vector<double>> targetPositions;  // To store 100 positions for each joint
    int count = 100;

    if (verboseMode) {
        ROS_INFO("Calculating 100 target positions Start");
    }

    try {
        std::vector<double> maximumRange;
        if (jointType == "arm") {
            if (configParams.find("armMaximumRange") == configParams.end()) {
                throw std::runtime_error("armMaximumRange not found in configuration");
            }
            maximumRange = parsePercentages(configParams["armMaximumRange"]);
        } else if (jointType == "leg") {
            if (configParams.find("legMaximumRange") == configParams.end()) {
                throw std::runtime_error("legMaximumRange not found in configuration");
            }
            maximumRange = parsePercentages(configParams["legMaximumRange"]);
        } else if (jointType == "hand") {
            if (configParams.find("handMaximumRange") == configParams.end()) {
                throw std::runtime_error("handMaximumRange not found in configuration");
            }
            maximumRange.push_back(std::stod(configParams["handMaximumRange"]));
        } else {
            throw std::runtime_error("Unknown joint type: " + jointType);
        }

        if (configParams.find("selectedRange") == configParams.end()) {
            throw std::runtime_error("selectedRange not found in configuration");
        }
        double selectedRange = std::stod(configParams["selectedRange"]);

        if (homePosition.size() != maxPosition.size() || homePosition.size() != minPosition.size()) {
            throw std::runtime_error("Mismatch in vector sizes for home, max, and min positions");
        }

        for (int i = 0; i < count; ++i) {
            std::vector<double> singleTargetPosition;
            for (size_t j = 0; j < homePosition.size(); ++j) {
                if (j >= maximumRange.size()) {
                    throw std::runtime_error("Not enough maximum range values for joint type: " + jointType);
                }

                double maxRange = maximumRange[j];
                double fullRange = maxPosition[j] - minPosition[j];
                double maximumRangeOffset = fullRange * maxRange;
                double selectedRangeOffset = (maximumRangeOffset * selectedRange) / 2.0;

                double tempPositionMax = std::min(homePosition[j] + selectedRangeOffset, maxPosition[j]);
                double tempPositionMin = std::max(homePosition[j] - selectedRangeOffset, minPosition[j]);

                double randomPosition = generateRandomPosition(tempPositionMin, tempPositionMax);
                singleTargetPosition.push_back(randomPosition);
            }
            targetPositions.push_back(singleTargetPosition);  
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in calculateTargetPosition: %s", e.what());
        return std::vector<std::vector<double>>(homePosition.size(), std::vector<double>(count, 0.0));  // Return a vector of zeros on error
    }

    if (verboseMode) {
        ROS_INFO("Calculating target positions End");
    }

    return targetPositions;  // Return the calculated 100 target positions
}


double computeDistance(const std::vector<double>& pos1, const std::vector<double>& pos2) {
    double distance = 0.0;
    for (size_t i = 0; i < pos1.size(); ++i) {
        double diff = pos2[i] - pos1[i];
        distance += diff * diff;
    }
    return std::sqrt(distance);
}

void computeDurationsBasedOnDistance(const std::vector<std::vector<double>>& positions, 
                                     std::vector<double>& durations, 
                                     double baseTimeFactor) {
    durations.clear();

    // Iterate through the positions and compute durations for each consecutive pair
    for (size_t i = 0; i < positions.size() - 1; ++i) {
        double distance = computeDistance(positions[i], positions[i + 1]);
        // Compute duration based on the distance and base time factor
        double duration = distance * baseTimeFactor;
        // Store the computed duration for this segment
        durations.push_back(duration);
    }

    // Add a small final duration for the last position to ensure smooth completion
    durations.push_back(0.1);  // Final duration to cap off the movement


    // Log the completion of duration calculation
    ROS_INFO("Computed durations for %zu position transitions.", positions.size() - 1);
}

// As the prst of the project wheere ther is ths kind ofthe  nfgjd asas shwn shown the 
//
void computeTrajectoryCubicSpline(const std::vector<std::vector<double>>& randomPositions,
                                  const std::vector<double>& currentPosition, 
                                  double totalTime,
                                  std::vector<std::vector<double>>& velocities_t, 
                                  std::vector<std::vector<double>>& accelerations_t, 
                                  std::vector<double>& durations) {
    
    size_t number_of_joints = currentPosition.size();
    size_t number_of_positions = randomPositions.size();

    // Clear previous trajectory data
    velocities_t.clear();
    accelerations_t.clear();
    durations.clear();

    // Ensure velocities_t and accelerations_t are resized appropriately
    velocities_t.resize(number_of_positions, std::vector<double>(number_of_joints, 0.0));
    accelerations_t.resize(number_of_positions, std::vector<double>(number_of_joints, 0.0));
    durations.resize(number_of_positions, 0.0);

    // Calculate time intervals based on total time and number of positions
    double time_interval = totalTime / (number_of_positions - 1);

    // Set up vectors for spline coefficients (for each joint)
    std::vector<Eigen::VectorXd> spline_coefficients_b(number_of_joints);
    std::vector<Eigen::VectorXd> spline_coefficients_c(number_of_joints);
    std::vector<Eigen::VectorXd> spline_coefficients_d(number_of_joints);

    for (size_t joint = 0; joint < number_of_joints; ++joint) {
        // Extract the position data for the specific joint from randomPositions
        Eigen::VectorXd positions(number_of_positions);
        for (size_t i = 0; i < number_of_positions; ++i) {
            positions(i) = randomPositions[i][joint];
        }

        // Set up time intervals as uniform
        Eigen::VectorXd time_intervals = Eigen::VectorXd::Constant(number_of_positions - 1, time_interval);

        // Spline coefficients (using natural cubic spline formula)
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(number_of_positions, number_of_positions);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(number_of_positions);

        // Boundary conditions (Natural spline)
        A(0, 0) = 1.0;
        A(number_of_positions - 1, number_of_positions - 1) = 1.0;

        // Fill matrix A and vector b for cubic spline interpolation
        for (size_t i = 1; i < number_of_positions - 1; ++i) {
            A(i, i - 1) = time_intervals(i - 1);
            A(i, i) = 2.0 * (time_intervals(i - 1) + time_intervals(i));
            A(i, i + 1) = time_intervals(i);

            b(i) = 3.0 * ((positions(i + 1) - positions(i)) / time_intervals(i) -
                           (positions(i) - positions(i - 1)) / time_intervals(i - 1));
        }

        // Solve for spline coefficients
        Eigen::VectorXd c = A.fullPivLu().solve(b);

        // Resize b and d to handle positions
        Eigen::VectorXd d(number_of_positions - 1);
        Eigen::VectorXd b_vec(number_of_positions - 1);
        for (size_t i = 0; i < number_of_positions - 1; ++i) {
            d(i) = (c(i + 1) - c(i)) / (3.0 * time_intervals(i));
            b_vec(i) = ((positions(i + 1) - positions(i)) / time_intervals(i)) -
                       (time_intervals(i) * (2.0 * c(i) + c(i + 1)) / 3.0);
        }

        // Store coefficients for each joint
        spline_coefficients_b[joint] = b_vec;
        spline_coefficients_c[joint] = c.segment(0, number_of_positions - 1);
        spline_coefficients_d[joint] = d;
    }

    // Now that we have the spline coefficients, we can generate the velocities and accelerations for each joint
    for (size_t i = 0; i < number_of_positions; ++i) {
        double t = i * time_interval;
        durations[i] = t;

        for (size_t joint = 0; joint < number_of_joints; ++joint) {
            if (i < spline_coefficients_b[joint].size()) { // Ensure index `i` is valid
                double dt = t;

                // Use pre-calculated positions_t directly from randomPositions, compute velocities and accelerations
                velocities_t[i][joint] = spline_coefficients_b[joint](i) + 
                                         2 * spline_coefficients_c[joint](i) * dt + 
                                         3 * spline_coefficients_d[joint](i) * dt * dt;

                accelerations_t[i][joint] = 2 * spline_coefficients_c[joint](i) + 
                                            6 * spline_coefficients_d[joint](i) * dt;
            } else {
                std::cerr << "Index out of bounds in spline_coefficients at joint " << joint << " and position " << i << std::endl;
            }
        }
    }
}



/**
 * @brief Animates the left arm of the robot.
 * 
 * This function calculates target positions for the left arm joints and moves the arm
 * to these positions. It also generates and displays diagnostic information if verbose mode is enabled.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param leftArmTopic The ROS topic name for controlling the left arm's movement.
 * @param resetPosition Boolean flag to indicate whether to reset to the home position.
 */
void lArm(ros::NodeHandle& nh, std::string leftArmTopic, bool resetPosition) {
    ControlClientPtr leftArmClient = createClient(leftArmTopic);

    // Joint position limits and home position
    std::vector<double> minPosition = {2.0857,  0.0087,  -1.5620, -2.0857,  -1.8239};
    std::vector<double> maxPosition = {-2.0857, 1.5620 , -0.0087,  2.0857,   1.8239};
    std::vector<double> homePosition = {1.7625, 0.09970, -0.1334, -1.7150,  0.06592};
    const std::string jointType = "arm";

    // Retrieve the base time factor from config parameters (default to 1.0 if not provided)
    double baseTimeFactor = configParams.find("gestureDuration") != configParams.end() 
                            ? std::stod(configParams["gestureDuration"]) 
                            : 1.0;

    // Declare currentPosition statically to keep it persistent between function calls
    static std::vector<double> currentPosition;

    // Reset position if currentPosition is empty or if explicitly requested
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(leftArmClient, lArmJointNames, "home Position", homePosition);

        if (verboseMode) {
            ROS_INFO("Moved to home position.");
        }
    }

    // Prepare vectors for storing random positions and trajectory data
    std::vector<std::vector<double>> randomPositions;
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    // Generate random positions for the arm within joint limits
    // generateListOfRandomPositions(randomPositions, homePosition, maxPosition, minPosition, jointType);
    randomPositions = calculateTargetPosition(homePosition, maxPosition, minPosition, "arm");
     positions_t = randomPositions;


    // Compute durations for each segment based on the distance between consecutive positions
    computeDurationsBasedOnDistance(positions_t, duration_t, baseTimeFactor);
   

   // Compute the trajectory using cubic spline interpolation
   computeTrajectoryCubicSpline(randomPositions, currentPosition, duration_t.back(), velocities_t, accelerations_t, duration_t);

    // Append the trajectory data to the file
    logTrajectoryData("src/cssr4africa/cssr_system/animateBehaviour/data/trajectory_data.csv", randomPositions, positions_t, accelerations_t, duration_t, true);

    // Move the left arm using the computed biological trajectory
    moveToPositionBiological(leftArmClient, lArmJointNames, "Animate Behavior", positions_t, velocities_t, accelerations_t, duration_t);

    if (verboseMode) {
        ROS_INFO("Arm moved through computed trajectory.");
    }

    // Update currentPosition to the last position for continuous movement
    if (!positions_t.empty()) {
        currentPosition = positions_t.back();  // Update currentPosition to the last position
        if (verboseMode) {
            ROS_INFO("Updated current position for continuous movement.");
        }
    }
}



void subtleBodyMovement(ros::NodeHandle& nh) {
    
    std::string leftArmTopic = topicData["LArm"];

    bool resetPosition = isFirstRun();
    if (resetPosition) {
        updateFirstRunFlag();
    }

    ROS_INFO("Robot platform detected and flexi movement initiated");
    std::thread lArmThread(lArm, std::ref(nh), leftArmTopic, resetPosition);
        
    lArmThread.join();
              
}



/**
 * @brief Executes animation behaviors based on the given behavior parameter.
 * 
 * This function activates subtle body movements, hand flex movements, or base rotation
 * depending on the input. If no specific behavior is requested, it performs all actions.
 * 
 * @param behaviour A string indicating the desired animation behavior(s).
 * @param nh The ROS NodeHandle to manage communication with ROS.
 */
void animateBehaviour(const std::string& behaviour, ros::NodeHandle& nh) 
{
    // if (!isActive) {
    //     if (verboseMode) {
    //         ROS_INFO("Animation behavior is inactive.");
    //     }
    //     return; // Exit if the animation behavior is not active
    // }
    std::string platform = configParams["platform"];
    if (verboseMode) {
        ROS_INFO("[animateBehaviour] Invoked with behaviour: %s", behaviour.c_str());
        ROS_INFO("Verbose mode is enabled.");
        ROS_INFO("Platform: %s", platform.c_str());
    }

    // Check for specific behaviors within the 'behaviour' parameter and execute accordingly
    if (behaviour.find("body") != std::string::npos) {
        if (verboseMode) {
            ROS_INFO("[animateBehaviour] Executing subtle body movements.");
        }
        subtleBodyMovement(nh);
    }
    if (behaviour.find("hands") != std::string::npos) {
        if (verboseMode) {
            ROS_INFO("[animateBehaviour] Executing hand flex movements.");
        }
        // flexiMovement(nh, "flexi");
    }
    if (behaviour.find("rotation") != std::string::npos) {
        if (verboseMode) {
            ROS_INFO("[animateBehaviour] Executing rotation base shift.");
        }
        // rotationBaseShift(nh);
    }

    // If no specific behaviour is provided, perform all available movements
    if (behaviour.empty()) {
        logToFile(" -------------- [START ALL ANIMATION BEHAVIOURS] -------------- ");
        if (verboseMode) {
            ROS_INFO(" -------------- [START ALL ANIMATION BEHAVIOURS] -------------- ");
        }
        boost::thread thread1(boost::bind(&subtleBodyMovement, boost::ref(nh)));
        // boost::thread thread2(boost::bind(&rotationBaseShift, boost::ref(nh)));
        // boost::thread thread3(boost::bind(&flexiMovement, boost::ref(nh), "All"));

        thread1.join();
        // thread2.join();
        // thread3.join();

        if (verboseMode) {
            ROS_INFO(" -------------- [END ALL ANIMATION BEHAVIOURS] -------------- ");
        }
        logToFile(" -------------- [END ALL ANIMATION BEHAVIOURS] -------------- ");
    }

}


