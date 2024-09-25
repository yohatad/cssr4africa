
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
 * @brief Computes the trajectory for joint movements.
 *
 * This function calculates the positions, velocities, and accelerations for each joint over a specified duration.
 * If verbose mode is enabled, diagnostic data is printed to the terminal and a diagnostic image is displayed in an OpenCV window.
 *
 * @param startPosition The starting positions of the joints.
 * @param endPosition The target positions of the joints.
 * @param numberOfJoints The number of joints involved.
 * @param trajectoryDuration The total duration of the trajectory.
 * @param positions A reference to store the calculated positions.
 * @param velocities A reference to store the calculated velocities.
 * @param accelerations A reference to store the calculated accelerations.
 * @param durations A reference to store the time points of the trajectory.
 */

// void compute_trajectory(std::vector<double> startPosition, std::vector<double> endPosition, int numberOfJoints,
//                         double trajectoryDuration, std::vector<std::vector<double>>& positions, 
//                         std::vector<std::vector<double>>& velocities, std::vector<std::vector<double>>& accelerations, 
//                         std::vector<double>& durations) {
//     if (verboseMode){
//         ROS_INFO("Computing trajectory started");
//     }

//     try {
//         // Check if input vectors have correct size
//         if (startPosition.size() != numberOfJoints || endPosition.size() != numberOfJoints) {
//             throw std::runtime_error("Start or end position size does not match number of joints");
//         }

//         // Check for valid trajectory duration
//         if (trajectoryDuration <= 0) {
//             throw std::runtime_error("Invalid trajectory duration");
//         }

//         // Check and convert numPoints from config
//         if (configParams.find("numPoints") == configParams.end()) {
//             throw std::runtime_error("numPoints not found in configuration");
//         }
//         int numPoints = std::stoi(configParams["numPoints"]);
//         if (numPoints <= 1) {
//             throw std::runtime_error("numPoints must be greater than 1");
//         }

//         double time_step = trajectoryDuration / (numPoints - 1);

//         // Clear the existing values
//         positions.clear();
//         velocities.clear();
//         accelerations.clear();
//         durations.clear();

//         // Pre-allocate memory
//         positions.reserve(numPoints);
//         velocities.reserve(numPoints);
//         accelerations.reserve(numPoints);
//         durations.reserve(numPoints);

//         // Calculate the trajectory
//         for (int point = 0; point < numPoints; ++point) {
//             double t_t = point * time_step;
//             std::vector<double> x_t, v_t, a_t;
//             x_t.reserve(numberOfJoints);
//             v_t.reserve(numberOfJoints);
//             a_t.reserve(numberOfJoints);

//             for (int i = 0; i < numberOfJoints; i++) {
//                 // double t_norm = t_t / trajectoryDuration;
//                 double t_norm = t_t ;
//                 double delta = endPosition[i] - startPosition[i];

//                 double x = startPosition[i] + delta * (10 * std::pow(t_norm, 3) - 15 * std::pow(t_norm, 4) + 6 * std::pow(t_norm, 5));
//                 x_t.push_back(x);

//                 double v = (delta / trajectoryDuration) * (30 * std::pow(t_norm, 2) - 60 * std::pow(t_norm, 3) + 30 * std::pow(t_norm, 4));
//                 v_t.push_back(v);

//                 double a = (delta / (trajectoryDuration * trajectoryDuration)) * (60 * t_norm - 180 * std::pow(t_norm, 2) + 120 * std::pow(t_norm, 3));
//                 a_t.push_back(a);
//             }

//             positions.push_back(std::move(x_t));
//             velocities.push_back(std::move(v_t));
//             accelerations.push_back(std::move(a_t));
//             durations.push_back(t_t);
//         }

//         if (verboseMode){
//             ROS_INFO("Number of trajectory velocity, acceleration and position points: %zu", positions.size());
//             ROS_INFO("Computing trajectory completed");
//         }
//     } catch (const std::exception& e) {
//         ROS_ERROR("Error in compute_trajectory: %s", e.what());
//         // Clear all output vectors in case of error
//         positions.clear();
//         velocities.clear();
//         accelerations.clear();
//         durations.clear();
//     }
// }

void compute_trajectory(std::vector<double> startPosition, std::vector<double> endPosition, int numberOfJoints,
                        double trajectoryDuration, std::vector<std::vector<double>>& positions, 
                        std::vector<std::vector<double>>& velocities, std::vector<std::vector<double>>& accelerations, 
                        std::vector<double>& durations) {
  // Declare variables
    double time_t = 0;                      // stores the instantaneous time of the trajectory
    std::vector<double> positions_t;        // vector to store the positions of the trajectory
    std::vector<double> velocities_t;       // vector to store the velocities of the trajectory
    std::vector<double> accelerations_t;    // vector to store the accelerations of the trajectory
    std::vector<double> duration_t;         // vector to store the duration of the trajectory
    double acceleration;                    // stores the acceleration
    double velocity;                        // stores the velocity
    double position;                        // stores the position
    double time_step = 0.1;                   // Time step between each point in the trajectory

    // Clear the existing values
    for(int i = 0; i < positions.size(); i++){
        positions[i].clear();
        velocities[i].clear();
        accelerations[i].clear();
    }
    positions.clear();
    velocities.clear();
    accelerations.clear();

    // Compute the trajectory for each point in time
    while(time_t < trajectoryDuration){
        for(int i = 0; i < numberOfJoints; i++){     // Create a trajectory for each joint (5 joints for the arm)
            position = startPosition[i] + (endPosition[i] - startPosition[i]) * ((10 * (pow(time_t/trajectoryDuration, 3))) - (15 * (pow(time_t/trajectoryDuration, 4))) + (6 * (pow(time_t/trajectoryDuration, 5))));
            positions_t.push_back(position);

            velocity = ((endPosition[i] - startPosition[i])/trajectoryDuration) * ((30 * (pow(time_t/trajectoryDuration, 2))) - (60 * (pow(time_t/trajectoryDuration, 3))) + (30 * (pow(time_t/trajectoryDuration, 4))));
            velocities_t.push_back(velocity);

            acceleration = ((endPosition[i] - startPosition[i])/(trajectoryDuration*trajectoryDuration)) * ((60 * (pow(time_t/trajectoryDuration, 1))) - (180 * (pow(time_t/trajectoryDuration, 2))) + (120 * (pow(time_t/trajectoryDuration, 3))));
            accelerations_t.push_back(acceleration);
        }
        // Store the computed trajectory
        positions.push_back(positions_t);
        velocities.push_back(velocities_t);
        accelerations.push_back(accelerations_t);
        durations.push_back(time_t);

        // Increment the time
        time_t = time_t + time_step;

        // Clear the vectors for the next iteration
        positions_t.clear();
        velocities_t.clear();
        accelerations_t.clear();
    }
    // Compute the trajectory for the last point in time
    time_t = trajectoryDuration;
    for(int i = 0; i < numberOfJoints; i++){          // Create a trajectory for each joint (5 joints for the arm)   
        position = startPosition[i] + (endPosition[i] - startPosition[i]) * ((10 * (pow(time_t/trajectoryDuration, 3))) - (15 * (pow(time_t/trajectoryDuration, 4))) + (6 * (pow(time_t/trajectoryDuration, 5))));
        positions_t.push_back(position);

        velocity = ((endPosition[i] - startPosition[i])/trajectoryDuration) * ((30 * (pow(time_t/trajectoryDuration, 2))) - (60 * (pow(time_t/trajectoryDuration, 3))) + (30 * (pow(time_t/trajectoryDuration, 4))));
        velocities_t.push_back(velocity);

        acceleration = ((endPosition[i] - startPosition[i])/(trajectoryDuration*trajectoryDuration)) * ((60 * (pow(time_t/trajectoryDuration, 1))) - (180 * (pow(time_t/trajectoryDuration, 2))) + (120 * (pow(time_t/trajectoryDuration, 3))));
        accelerations_t.push_back(acceleration);
    }
    // Store the computed trajectory for the last point in time
    positions.push_back(positions_t);
    velocities.push_back(velocities_t);
    accelerations.push_back(accelerations_t);
    durations.push_back(time_t);

    ROS_INFO("Number of trajectory  position points: %zu", positions.size());
   
   return;                          
}

void computeTrajectoryDetails(const std::vector<std::vector<double>>& positions, 
                              std::vector<std::vector<double>>& velocities, 
                              std::vector<std::vector<double>>& accelerations, 
                              std::vector<double>& durations, 
                              double totalDuration) {
    size_t numPositions = positions.size();
    if (numPositions < 2) {
        throw std::runtime_error("Not enough positions to compute trajectory details.");
    }

    double timeStep = totalDuration / (numPositions - 1); // Equal time intervals between positions

    for (size_t i = 0; i < numPositions - 1; ++i) {
        std::vector<double> velocity(positions[i].size(), 0.0);
        std::vector<double> acceleration(positions[i].size(), 0.0);
        
        for (size_t j = 0; j < positions[i].size(); ++j) {
            double positionDiff = positions[i + 1][j] - positions[i][j];
            velocity[j] = positionDiff / timeStep;
            acceleration[j] = velocity[j] / timeStep; // Simplified acceleration calculation
        }
        
        velocities.push_back(velocity);
        accelerations.push_back(acceleration);
        durations.push_back(timeStep);
    }

    // Ensure the last segment has the correct timing
    durations.push_back(timeStep);
}

void compute_trajectory_between_random_positions(std::vector<double>& currentPosition,
                                                 const std::vector<std::vector<double>>& random_positions, 
                                                 int number_of_joints, double trajectory_duration, 
                                                 std::vector<std::vector<double>>& positions, 
                                                 std::vector<std::vector<double>>& velocities, 
                                                 std::vector<std::vector<double>>& accelerations, 
                                                 std::vector<double>& durations) {
    
    // Clear the existing values
    for (int i = 0; i < positions.size(); i++) {
        positions[i].clear();
        velocities[i].clear();
        accelerations[i].clear();
    }
    positions.clear();
    velocities.clear();
    accelerations.clear();
    durations.clear();

    // Starting position for the trajectory
    std::vector<double> start_position = currentPosition;

    // Define step size and calculate number of steps
    double step_size = 0.009;  // 0.009 seconds per step
    int steps = static_cast<int>(trajectory_duration / step_size);  // Calculate number of steps

    // Initialize time
    double time_t = 0.0;

    for (size_t idx = 0; idx < random_positions.size(); ++idx) {
        // Extract the end position from random_positions
        const std::vector<double>& end_position = random_positions[idx];

        // Resize vectors to store the trajectory data at once
        std::vector<double> positions_t(number_of_joints, 0.0);
        std::vector<double> velocities_t(number_of_joints, 0.0);
        std::vector<double> accelerations_t(number_of_joints, 0.0);

        // Compute the trajectory
        for (int step = 0; step <= steps; ++step) {
            // Ensure time_t is strictly increasing by updating it in every iteration
            time_t += step_size;

            for (int i = 0; i < number_of_joints; ++i) {
                // Compute position, velocity, and acceleration for joint i
                double alpha = time_t / trajectory_duration;
                double alpha2 = pow(alpha, 2);
                double alpha3 = pow(alpha, 3);
                double alpha4 = pow(alpha, 4);
                double alpha5 = pow(alpha, 5);

                // Fifth-order polynomial trajectory
                positions_t[i] = start_position[i] + (end_position[i] - start_position[i]) *
                                  (10 * alpha3 - 15 * alpha4 + 6 * alpha5);

                velocities_t[i] = ((end_position[i] - start_position[i]) / trajectory_duration) *
                                  (30 * alpha2 - 60 * alpha3 + 30 * alpha4);

                accelerations_t[i] = ((end_position[i] - start_position[i]) / (trajectory_duration * trajectory_duration)) *
                                     (60 * alpha - 180 * alpha2 + 120 * alpha3);
            }

            // Store the results for this time step
            positions.push_back(positions_t);
            velocities.push_back(velocities_t);
            accelerations.push_back(accelerations_t);
            durations.push_back(time_t);
        }

        // Update the start position for the next segment
        start_position = end_position;
    }

    ROS_INFO("Trajectory computed for all random positions with strictly increasing time.");
}


void moveToPositionWithTrajectory(ControlClientPtr& client, 
                                  const std::vector<std::string>& jointNames, 
                                  const std::vector<std::vector<double>>& positions, 
                                  const std::vector<std::vector<double>>& velocities, 
                                  const std::vector<std::vector<double>>& accelerations, 
                                  const std::vector<double>& durations) {
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;

    ros::Duration timeFromStart(0.0); // Initialize time from start

    for (size_t i = 0; i < positions.size(); ++i) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = positions[i];
        point.velocities = velocities[i];
        point.accelerations = accelerations[i];
        timeFromStart += ros::Duration(durations[i]);
        point.time_from_start = timeFromStart;

        trajectory.points.push_back(point);
    }

    client->sendGoal(goal);

    // Wait for the trajectory to finish executing
    bool finishedBeforeTimeout = client->waitForResult(timeFromStart + ros::Duration(1.0)); 
    if (finishedBeforeTimeout) {
        actionlib::SimpleClientGoalState state = client->getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully moved through the trajectory.");
        } else {
            ROS_WARN("The action failed to move through the trajectory. State: %s", state.toString().c_str());
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
                              double gesture_duration,
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

    // Resize the trajectory points array to match the number of positions
    size_t numPoints = positions.size();
    trajectory.points.resize(numPoints);

    if (verboseMode) {
        ROS_INFO("Number of trajectory points: %zu", numPoints);
    }
    
    // Populate the trajectory points with positions, velocities, accelerations, and times
    for (size_t i = 0; i < numPoints; ++i) {
        if (positions[i].size() != jointNames.size() || 
            velocities[i].size() != jointNames.size() || 
            accelerations[i].size() != jointNames.size()) {
            ROS_ERROR("Mismatch in vector sizes for joint %zu", i);
            return;
        }

        // Assign positions, velocities, and accelerations to the trajectory
        trajectory.points[i].positions = positions[i];
        trajectory.points[i].velocities = velocities[i];
        trajectory.points[i].accelerations = accelerations[i];
        trajectory.points[i].time_from_start = ros::Duration(durations[i]);

        if (verboseMode) {
            ROS_INFO("Trajectory point %zu: Duration %.2f seconds", i, durations[i]);
        }
    }

    // Send the trajectory goal to the action server (Pepper's controller)
    client->sendGoal(goal);

    // Wait for the action to complete within the specified time limit
    bool finishedBeforeTimeout = client->waitForResult(ros::Duration(gesture_duration + 1.0));

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
std::vector<double> calculateTargetPosition(const std::vector<double>& homePosition,
                                            const std::vector<double>& maxPosition,
                                            const std::vector<double>& minPosition,
                                            const std::string& jointType) {
    std::vector<double> targetPosition(homePosition.size(), 0.0);
    if (verboseMode){
        ROS_INFO("Calculating target position Start");
    }
    
    try {
        // Retrieve specific maximum ranges for each joint category
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
            maximumRange.push_back(std::stod(configParams["handMaximumRange"])); // Single value
        } else {
            throw std::runtime_error("Unknown joint type: " + jointType);
        }
        
        // Retrieve selected range from configuration parameters
        if (configParams.find("selectedRange") == configParams.end()) {
            throw std::runtime_error("selectedRange not found in configuration");
        }
        double selectedRange = std::stod(configParams["selectedRange"]);

        if (homePosition.size() != maxPosition.size() || homePosition.size() != minPosition.size()) {
            throw std::runtime_error("Mismatch in vector sizes for home, max, and min positions");
        }

        for (size_t i = 0; i < homePosition.size(); ++i) {
            if (i >= maximumRange.size()) {
                throw std::runtime_error("Not enough maximum range values for joint type: " + jointType);
            }
            
            double maxRange = maximumRange[i];
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
    } catch (const std::exception& e) {
        ROS_ERROR("Error in calculateTargetPosition: %s", e.what());
        return std::vector<double>(homePosition.size(), 0.0); // Return a vector of zeros on error
    }

    if (verboseMode){
        ROS_INFO("Calculating target position End");
    }
    return targetPosition; // Return the calculated target positions
}


void generateListOfRandomPositions(std::vector<std::vector<double>>& positions, 
                                         const std::vector<double>& homePosition, 
                                         const std::vector<double>& maxPosition, 
                                         const std::vector<double>& minPosition, 
                                         const std::string& jointType) {
    // Define the number of random positions to generate
    int numberOfPositions = 100;

    // Loop to generate the specified number of random positions
    for (int i = 0; i < numberOfPositions; ++i) {
        // Calculate the target position
        ROS_INFO("Calculating target position Start");
        std::vector<double> randomPosition = calculateTargetPosition(homePosition, maxPosition, minPosition, jointType);
        
        ROS_INFO("Calculating target position End");
        // Store the generated position
        positions.push_back(randomPosition);

        // Print the current iteration
        ROS_INFO("Generated position %d out of %d", i + 1, numberOfPositions);

        // Add a small delay to allow time to progress if necessary
        ros::Duration(0.01).sleep();
    }

    // After generating all positions, log completion
    ROS_INFO("Completed generating %d random positions.", numberOfPositions);
}



/**
 * @brief Animates the right arm of the robot.
 * 
 * This function calculates target positions for the right arm joints and moves the arm
 * to these positions. It also generates and displays diagnostic information if verbose mode is enabled.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param rightArmTopic The ROS topic name for controlling the right arm's movement.
 * @param resetPosition Boolean flag to indicate whether to reset to the home position.
 */
void rArm(ros::NodeHandle& nh, std::string rightArmTopic, bool resetPosition) {
    // Create a client for controlling the right arm
    ControlClientPtr rightArmClient = createClient(rightArmTopic);
    // Define initial, maximum, minimum, and home positions for the arm's joints
    std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> maxPosition = {2.0857, -0.0087, 1.5620, 2.0857, 1.8239};
    std::vector<double> minPosition = {-2.0857, -1.5620, 0.0087, -2.0857, -1.5620};
    std::vector<double> homePositionT = {1.410, -0.09664, 0.09664, 1.6981, -0.05679};
    const std::string jointType = "arm";
    static std::vector<double> currentPosition;
    double gestureDuration = std::stod(configParams["gestureDuration"]);
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;


    // Calculate the target position for the arm joints
    std::vector<double> targetPosition = calculateTargetPosition(homePositionT, maxPosition, minPosition, jointType);
    logToFile("---------------- [START RIGHT ARM ANIMATE MOVEMENT] ----------------");
    //log the topic name
    logToFile("The topic name for the right arm is " + rightArmTopic);
    if (verboseMode){
    ROS_INFO("---------------- [START RIGHT ARM ANIMATE MOVEMENT] ----------------");
    }

    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(rightArmClient, rArmJointNames, "home Position", currentPosition);
        logToFile("The joint names for the right arm are: " + vectorOfStringsToString(rArmJointNames));
        logToFile(std::string("The home position of the right arm is ") + vectorToString(currentPosition));
    }
   
    if (verboseMode) {
        for (size_t i = 0; i < targetPosition.size(); ++i) {
            ROS_INFO("Calculated target position of the for joint %s: %.4f", rArmJointNames[i].c_str(), targetPosition[i]);
        }
    }
    // Compute the trajectory for the arm movement
    compute_trajectory(currentPosition, targetPosition, rArmJointNames.size(), gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
    // Move the arm to the target position using the calculated trajectory
    moveToPositionBiological(rightArmClient, rArmJointNames, gestureDuration, "random", positions_t, velocities_t, accelerations_t, duration_t);
    currentPosition = targetPosition;

    logToFile(std::string("The random position of the right arm is ") + vectorToString(targetPosition));
    
    logToFile("----------[END RIGHT ARM ANIMATE MOVEMENT]-----------");
    if (verboseMode) {
        ROS_INFO_STREAM("----------[END RIGHT ARM ANIMATE MOVEMENT]-----------");
        
    }
}


/**
 * @brief Animates the right arm of the robot focusing on the wrist(used for flexi hand movement).
 * 
 * This function calculates target positions for the right arm joints and moves the arm
 * to these positions. It also generates and displays diagnostic information if verbose mode is enabled.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param rightArmTopic The ROS topic name for controlling the right arm's movement.
 * @param resetPosition Boolean flag to indicate whether to reset to the home position.
 */
void rArml(ros::NodeHandle& nh, std::string rightArmTopic, bool resetPosition) {
    ROS_INFO("Right Arm Flexi Hand Movement");
    // Create a client for controlling the right arm
    ControlClientPtr rightArmClient = createClient(rightArmTopic);
    logToFile("The topic name for the right arm is " + rightArmTopic);
    
    // Define initial, maximum, minimum, and home positions for the arm's joints
    std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
    std::vector<double> maxPosition = {2.0857, -0.0087, 1.5620, 2.0857, 1.8239};
    std::vector<double> minPosition = {-2.0857, -1.5620, 0.0087, -2.0857, -1.5620};
    std::vector<double> homePositionT = {1.410, -0.09664, 0.09664, 1.6981, -0.05679};
    static std::vector<double> currentPosition;
    const std::string jointType = "arm";

    double gestureDuration = std::stod(configParams["gestureDuration"]);
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;
     // joint name used for flexi hand movement
   std::vector<std::string> rjointNames = {"RElbowYaw", "RShoulderRoll"};
    
    logToFile("----------[START RIGHT ARM For Flexi Hand ANIMATE MOVEMENT]-----------");
    if (verboseMode) {
        ROS_INFO("----------[START RIGHT ARM For Flexi Hand ANIMATE MOVEMENT]-----------");
    }
    // Reset to the home position if required
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(rightArmClient, rArmJointNames, "home Position", currentPosition);
        logToFile("The joint names for the right arm are: " + vectorOfStringsToString(rjointNames));
        logToFile("The home position of the right arm is " + vectorToString(currentPosition));
    }
    ROS_INFO("The target postion is going to be calculated");
    // Calculate the target position for the arm joints
    std::vector<double> targetPosition = calculateTargetPosition(homePositionT, maxPosition, minPosition, jointType);
    // Modify the home position for the last joint to the target position
    std::vector<double> RWristYawposition = {homePositionT[0], homePositionT[1], homePositionT[2], targetPosition[3], targetPosition[4]};
    
    
    if (verboseMode) {
        // Print the calculated target positions for debugging
        for (size_t i = 0; i < targetPosition.size(); ++i) {
            ROS_INFO("Calculated target position for joint %s: %.4f", rArmJointNames[i].c_str(), RWristYawposition[i]);
        }
    }
    
    // Compute the trajectory for the arm movement
    compute_trajectory(currentPosition, RWristYawposition, rArmJointNames.size(), gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
    // Move the arm to the target position using the calculated trajectory
    moveToPositionBiological(rightArmClient, rArmJointNames, gestureDuration, "random", positions_t, velocities_t, accelerations_t, duration_t);
    currentPosition = RWristYawposition;

    logToFile("The random position of the right arm is " + vectorToString(currentPosition));

    logToFile("----------[END RIGHT ARM For Flexi Hand ANIMATE MOVEMENT]-----------");
    if (verboseMode) {
        ROS_INFO_STREAM("----------[END RIGHT ARM For Flexi Hand ANIMATE MOVEMENT]-----------");
    }
}


/**
 * @brief Animates the right hand of the robot.
 * 
 * This function calculates target positions for the right hand joints and moves the hand
 * to these positions. It also generates and displays diagnostic information if verbose mode is enabled.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param rightHandTopic The ROS topic name for controlling the right hand's movement.
 * @param resetPosition Boolean flag to indicate whether to reset to the home position.
 */
void rHand(ros::NodeHandle& nh, std::string rightHandTopic, bool resetPosition) {
    // Create a client for controlling the right hand
    ControlClientPtr rightHandClient = createClient(rightHandTopic);

    std::vector<double> maxPosition = {1.0};// Define initial maximum for the hand's joint
    std::vector<double> minPosition = {0.0};//  Define initial minimum for the hand's joint
    std::vector<double> homePosition = {0.66608}; //Define homePosition for the hand's joint
    static std::vector<double> currentPosition;
    const std::string jointType = "hand";

    double gestureDuration = std::stod(configParams["gestureDuration"]);
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    // Calculate the target position for the hand joint
    std::vector<double> targetPosition = calculateTargetPosition(homePosition, maxPosition, minPosition, jointType);
    if (verboseMode) {
        // Print the calculated target positions for debugging
        for (size_t i = 0; i < targetPosition.size(); ++i) {
            ROS_INFO("Calculated target position for joint %s: %.4f", rhandJointNames[i].c_str(), targetPosition[i]);
        }
    }

    if (verboseMode) {
        ROS_INFO_STREAM("----------[START RIGHT HAND ANIMATE MOVEMENT]-----------");
    }

    logToFile("----------[START RIGHT HAND ANIMATE MOVEMENT]-----------");
    logToFile("The topic name for the right hand is " + rightHandTopic);
    // Reset to the home position if required
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(rightHandClient, rhandJointNames, "home Position", currentPosition);
        logToFile("The joint names for the right hand are: " + vectorOfStringsToString(rhandJointNames));
        logToFile("The home position of the right hand is " + vectorToString(currentPosition));
    }

    // Compute the trajectory for the hand movement
    compute_trajectory(currentPosition, targetPosition, rhandJointNames.size(), gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);

    // Move the hand to the target position using the calculated trajectory
    moveToPositionBiological(rightHandClient, rhandJointNames, gestureDuration, "random", positions_t, velocities_t, accelerations_t, duration_t);
    // Update the current position
    currentPosition = targetPosition;

    logToFile("The random position of the right hand is " + vectorToString(targetPosition));
    logToFile("----------[END RIGHT HAND ANIMATE MOVEMENT]-----------");

    if (verboseMode) {
        ROS_INFO_STREAM("----------[END RIGHT HAND ANIMATE MOVEMENT]-----------");
    }
}


void moveToPositiontest(ControlClientPtr& client, 
                        const std::vector<std::string>& jointNames, 
                        const std::vector<std::vector<double>>& positions, 
                        double duration) {
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;

    ros::Duration timeFromStart(0.0); 
    ros::Duration singleStepDuration = ros::Duration(duration / positions.size());

    for (size_t i = 0; i < positions.size(); ++i) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = positions[i];
        timeFromStart += singleStepDuration; 
        point.time_from_start = timeFromStart;

        trajectory.points.push_back(point);
    }

    client->sendGoal(goal);

    // Wait for the trajectory to finish executing
    bool finishedBeforeTimeout = client->waitForResult(timeFromStart + ros::Duration(1.0)); 
    if (finishedBeforeTimeout) {
        actionlib::SimpleClientGoalState state = client->getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully moved through the trajectory.");
        } else {
            ROS_WARN("The action failed to move through the trajectory. State: %s", state.toString().c_str());
        }
    } else {
        ROS_WARN("The action did not finish before the timeout.");
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
    std::vector<double> maxPosition = {2.0857, 1.5620, -0.0087, 2.0857, 1.8239}; 
    std::vector<double> minPosition = {-2.0857, 0.0087, -1.5620, -2.0857, -1.8239};
    std::vector<double> homePosition = {1.7625, 0.09970, -0.1334, -1.7150, 0.06592};
    const std::string jointType = "arm";

    // Retrieve the gesture duration from config parameters
    double gestureDuration = std::stod(configParams["gestureDuration"]);

    // Declare currentPosition statically to keep it persistent between function calls
    static std::vector<double> currentPosition;

    // Reset position if currentPosition is empty or if explicitly requested
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(leftArmClient, lArmJointNames, "home Position", currentPosition);

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
    generateListOfRandomPositions(randomPositions, currentPosition, maxPosition, minPosition, jointType);

    if (verboseMode) {
        ROS_INFO("Generated %zu random positions.", randomPositions.size());
    }

    // Compute the trajectory starting from the current position
    compute_trajectory_between_random_positions(currentPosition, randomPositions, currentPosition.size(), gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);

    if (verboseMode) {
        ROS_INFO("Computed trajectory for %zu positions.", positions_t.size());
    }

    // Move the left arm using the computed biological trajectory
    moveToPositionBiological(leftArmClient, lArmJointNames, gestureDuration, "Animate Behavior", positions_t, velocities_t, accelerations_t, duration_t);

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



/**
 * @brief Animates the left arm's wrist yaw of robot that used for flexi hand movement.
 * 
 * This function calculates target positions for the left arm joints and moves the arm
 * to these positions. It also generates and displays diagnostic information if verbose mode is enabled.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param leftArmTopic The ROS topic name for controlling the left arm's movement.
 * @param resetPosition Boolean flag to indicate whether to reset to the home position.
 */
void lArml(ros::NodeHandle& nh, std::string leftArmTopic, bool resetPosition) {
    ROS_INFO("----------[START LEFT ARM ANIMATE MOVEMENT FOR FLEXI HAND]-----------");
    // Create a client for controlling the left arm
    ROS_INFO("the topic vaue is %s", leftArmTopic.c_str());
    ControlClientPtr leftArmClient = createClient(leftArmTopic);
    ROS_INFO("the cleient is created");
    
    // Define initial, maximum, minimum, and home positions for the arm's joints
    std::vector<double> maxPosition = {2.0857, 1.5620, -0.0087, 2.0857, 1.8239}; 
    std::vector<double> minPosition = {-2.0857, 0.0087, -1.5620, -2.0857, -1.8239};
    std::vector<double> homePosition = {1.7625, 0.09970, -0.1334, -1.7150, 0.06592};
    std::vector<double> homePositionT = {1.4625, 0.09970, -0.1334, -1.7150, 0.06592};
    const std::string jointType = "arm";
    double gestureDuration = std::stod(configParams["gestureDuration"]);
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    logToFile("----------[START LEFT ARM ANIMATE MOVEMENT FOR FLEXI HAND]-----------");
    logToFile("The topic name for the left arm is " + leftArmTopic);
    if (verboseMode) {
        ROS_INFO_STREAM("----------[LEFT ARM ANIMATE MOVEMENT FOR FLEXI HAND]-----------");
    }
    
    // Calculate the target position for the arm joints
    ROS_INFO("the target position to be calculated");
    std::vector<double> targetPosition = calculateTargetPosition(homePositionT, maxPosition, minPosition, jointType);
    ROS_INFO("the target position is calculated");
    // Modify the home position for the wrist yaw
    std::vector<double> RWristYawposition = {homePositionT[0], homePositionT[1], homePositionT[2], targetPosition[3], targetPosition[4]};
    
    // Print the calculated target positions for debugging if verbose mode is enabled
    if (verboseMode) {
        for (size_t i = 0; i < targetPosition.size(); ++i) {
            ROS_INFO("Calculated target position for joint %s: %.4f", lArmJointNames[i].c_str(), RWristYawposition[i]);
        }
    }
    // joint name used for flexi hand movement
    std::vector<std::string> ljointNames = {"LElbowYaw", "LWristYaw"};

    static std::vector<double> currentPosition;
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(leftArmClient, lArmJointNames, "home Position", currentPosition);
        logToFile("The joint names for the left arm are: " + vectorOfStringsToString(ljointNames));
        logToFile("The home position of the left arm is " + vectorToString(currentPosition));
    }
    
    // Compute the trajectory for the arm movement
    compute_trajectory(currentPosition, RWristYawposition, lArmJointNames.size(), gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
    
    // Move the arm to the target position using the calculated trajectory
    moveToPositionBiological(leftArmClient, lArmJointNames, gestureDuration, "random", positions_t, velocities_t, accelerations_t, duration_t);
    // Update the current position
    currentPosition = RWristYawposition;

    logToFile("The random position of the left arm is " + vectorToString(currentPosition));
    logToFile("----------[END LEFT ARM ANIMATE MOVEMENT FOR FLEXI HAND]-----------");
    if (verboseMode) {
        ROS_INFO_STREAM("----------[END LEFT ARM ANIMATE MOVEMENT FOR FLEXI HAND]-----------");
    }
}


/**
 * @brief Animates the left hand of the robot.
 * 
 * This function calculates target positions for the left hand joint and moves the hand
 * to these positions. It also generates and displays diagnostic information if verbose mode is enabled.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param leftHandTopic The ROS topic name for controlling the left hand's movement.
 * @param resetPosition Boolean flag to indicate whether to reset to the home position.
 */
void lHand(ros::NodeHandle& nh, std::string leftHandTopic, bool resetPosition) {
    // Create a client for controlling the left hand
    ControlClientPtr leftHandClient = createClient(leftHandTopic);
    
    // Define maximum and minimum positions for the hand's joint
    std::vector<double> maxPosition = {1.0};
    std::vector<double> minPosition = {0.0};
    std::vector<double> homePosition = {0.6695};
    const std::string jointType = "hand";
    double gestureDuration = std::stod(configParams["gestureDuration"]);
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;
    static std::vector<double> currentPosition;
    
    // Calculate the target position for the hand joint
    std::vector<double> targetPosition = calculateTargetPosition(homePosition, maxPosition, minPosition, jointType);
    
    logToFile("----------[START LEFT HAND ANIMATE MOVEMENT]-----------");
    logToFile("The topic name for the left hand is " + leftHandTopic);
    if (verboseMode){
    ROS_INFO_STREAM("----------[START LEFT HAND ANIMATE MOVEMENT]-----------");
    }

    // Print the calculated target positions for debugging if verbose mode is enabled
    if (verboseMode) {
        for (size_t i = 0; i < targetPosition.size(); ++i) {
            ROS_INFO("Calculated target position for joint %s: %.4f", lhandJointNames[i].c_str(), targetPosition[i]);
        }
    }

    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(leftHandClient, lhandJointNames, "home Position", currentPosition);
        logToFile("The joint names for the left hand are: " + vectorOfStringsToString(lhandJointNames));
        logToFile("The home position of the left hand is " + vectorToString(currentPosition));
    }

    // Compute the trajectory for the hand movement
    compute_trajectory(currentPosition, targetPosition, lhandJointNames.size(), gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
    
    // Move the hand to the target position using the calculated trajectory
    moveToPositionBiological(leftHandClient, lhandJointNames, gestureDuration, "random", positions_t, velocities_t, accelerations_t, duration_t);
    // Update the current position
    currentPosition = targetPosition;

    logToFile("The random position of the left hand is " + vectorToString(targetPosition));
    logToFile("----------[END LEFT HAND ANIMATE MOVEMENT]-----------");
    if (verboseMode) {
        ROS_INFO_STREAM("----------[END LEFT HAND ANIMATE MOVEMENT]-----------");
    }
}


/**
 * @brief Animates the leg of the robot.
 * 
 * This function calculates target positions for the leg joints and moves the leg
 * to these positions. It also generates and displays diagnostic information if verbose mode is enabled.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param legTopic The ROS topic name for controlling the leg's movement.
 * @param resetPosition Boolean flag to indicate whether to reset to the home position.
 */
void leg(ros::NodeHandle& nh, std::string legTopic, bool resetPosition) {
    // Create a client for controlling the leg
    ControlClientPtr legClient = createClient(legTopic);
    
    // Define maximum and minimum positions for the leg's joints
    std::vector<double> maxPosition = {1.0385, 0.5149, 0.5149};
    std::vector<double> minPosition = {-1.0385, -0.5149, -0.5149};
    std::vector<double> homePosition = {-0.0107, -0.00766, 0.03221};
    std::vector<double> homePositionT = {-0.0207, -0.00766, 0.00221};
    const std::string jointType = "leg";
    static std::vector<double> currentPosition;
    double gestureDuration = std::stod(configParams["gestureDuration"]);
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    // Calculate the target position for the leg joints
    std::vector<double> targetPosition = calculateTargetPosition(homePositionT, maxPosition, minPosition, jointType);
    
    logToFile("----------[START LEG ANIMATE MOVEMENT]-----------");
    if (verboseMode) {
        ROS_INFO_STREAM("----------[START LEG ANIMATE MOVEMENT]-----------");
    }
    // Print the calculated target positions for debugging if verbose mode is enabled
    if (verboseMode) {
        for (size_t i = 0; i < targetPosition.size(); ++i) {
            ROS_INFO("Calculated target position for joint %s: %.4f", legJointNames[i].c_str(), targetPosition[i]);
        }
    }

    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(legClient, legJointNames, "home Position", currentPosition);
        logToFile("The joint names for the leg are: " + vectorOfStringsToString(legJointNames));
        logToFile("The home position of the leg is " + vectorToString(currentPosition));
    }

    // Compute the trajectory for the leg movement
    compute_trajectory(currentPosition, targetPosition, legJointNames.size(), gestureDuration, positions_t, velocities_t, accelerations_t, duration_t);
    
    // Move the leg to the target position using the calculated trajectory
    moveToPositionBiological(legClient, legJointNames, gestureDuration, "random", positions_t, velocities_t, accelerations_t, duration_t);
    // Update the current position
    currentPosition = targetPosition;
    logToFile("The random position of the leg is " + vectorToString(targetPosition));

    logToFile("----------[END LEFT HAND ANIMATE MOVEMENT]-----------");
    if (verboseMode) {
        ROS_INFO_STREAM("----------[END LEG ANIMATE MOVEMENT]-----------");
    }
}


/**
 * @brief Calculates an angular velocity in the Z-axis for the robot's base rotation.
 * 
 * This function calculates a target angular velocity within specified ranges. It ensures the
 * angular velocity is within the max and min positions and adjusts based on the 'rotMaximumRange' 
 * and 'selectedRange' from configParams.
 * 
 * @param maxAngularVelocity The maximum allowable angular velocity.
 * @return The calculated angular velocity in the Z-axis.
 */
double calculateAngularVelocityZ(double maxAngularVelocity) {
    logToFile("---------------- [START CALCULATING ANGULAR VELOCITY Z] ----------------");
    if (verboseMode) {
        ROS_INFO(" -------------- [START CALCULATING ANGULAR VELOCITY Z] -------------- ");
    }
    // Retrieve maximum and selected ranges from configuration parameters
    double maxRange = std::stod(configParams["rotMaximumRange"]);
    double selectedRange = std::stod(configParams["selectedRange"]);
    double homeAngularVelocity = 0.0; 
    double fullRange = maxAngularVelocity - homeAngularVelocity;// Calculate the full possible range of angular velocity from the base to the maximum
    double maximumRangeOffset = fullRange * maxRange;// Apply the maximum range to narrow down the full range
    double selectedRangeOffset = maximumRangeOffset * selectedRange; // Apply the selected range to find the actual target range within the maximum offset
    double tempVelocityMax = homeAngularVelocity + selectedRangeOffset; // Determine the maximum and minimum angular velocity within the selected range
    double tempVelocityMin = homeAngularVelocity - selectedRangeOffset;
    double angularVelocityZ = randomDoubleInRange(tempVelocityMin, tempVelocityMax);// Randomly select an angular velocity within this range

    // Print diagnostic data to the terminal if verbose mode is enabled
    if (verboseMode) {
        ROS_INFO("Full range of angular velocity: %.4f", fullRange);
        ROS_INFO("Maximum range offset: %.4f", maximumRangeOffset);
        ROS_INFO("Selected range offset: %.4f", selectedRangeOffset);
        ROS_INFO("Calculated angular velocity Z: %.4f", angularVelocityZ);
    }

    logToFile("Calculated angular velocity Z: " + std::to_string(angularVelocityZ));

    if (verboseMode) {
        ROS_INFO(" -------------- [END CALCULATING ANGULAR VELOCITY Z] -------------- ");
    }
    logToFile("---------------- [END CALCULATING ANGULAR VELOCITY Z] ----------------");
    return angularVelocityZ; // Return the calculated angular velocity
}


/**
 * @brief Initiates a rotation of the robot base using a calculated angular velocity.
 * 
 * This function performs a sequence of rotations with angular velocities calculated
 * using the calculateAngularVelocityZ function. It alternates between positive and
 * negative velocities with stops in between, completing a specified number of cycles.
 * 
 * @param nh The ROS NodeHandle.
 */
void rotationBaseShift(ros::NodeHandle& nh) {
    double maxAngularVelocity = 2.0; 
    std::string cmdVelTopic = topicData["Wheels"]; 
    std::vector<std::string> baseJointNames = {"Wheels"};
    ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 10);
    ros::Duration(0.5).sleep(); 
    geometry_msgs::Twist twist; 
    double rotationDuration = 2.0; 
    int numberOfCycles = 5; 
    int cycleCount = 0; 
    std::atomic<bool> cycleComplete(false);
    
    logToFile(" ----------------- [START ROTATION BASE SHIFT] ----------------- ");
    if (verboseMode) {
        ROS_INFO(" ----------------- [START ROTATION BASE SHIFT] ----------------- ");
    }
    logToFile("The joint names for the base are: " + vectorOfStringsToString(baseJointNames));
    logToFile("The home position of the base is " + std::to_string(maxAngularVelocity));
    auto rotationThread = std::thread([&]() {
        while (ros::ok() && cycleCount < numberOfCycles) {
            double angularVelocity = calculateAngularVelocityZ(maxAngularVelocity); // Calculate a positive random angular velocity
             
            // Sequence of angular velocities: negative, zero, positive, zero
            double angularVelocities[4] = {
                -angularVelocity, // Negative side
                0, // Stop
                angularVelocity, // Positive side
                0 // Stop
            };

            for (int i = 0; i < 4; ++i) {
                twist.angular.z = angularVelocities[i];
                velPub.publish(twist); // Publish the command
                
                logToFile("The random position of angular velocity is " + std::to_string(twist.angular.z));
                if (verboseMode) {
                    ROS_INFO_STREAM("Setting angular velocity to: " << twist.angular.z);
                }

                ros::Time startTime = ros::Time::now(); // Record the start time for each phase
                while ((ros::Time::now() - startTime) < ros::Duration(rotationDuration)) {
                    ros::spinOnce(); // Handle callbacks
                    ros::Duration(0.1).sleep(); // Maintain timing
                }
            }
            cycleCount++; // Increment the cycle counter after each full rotation
            cycleComplete = true;
        }
    });

    // Wait for rotation to complete
    while (!cycleComplete) {
        ros::spinOnce(); // Handle other callbacks
        ros::Duration(0.1).sleep(); // Avoid high CPU usage
    }

    rotationThread.join();
    
    if (verboseMode) {
        ROS_INFO(" ----------------- [END ROTATION BASE SHIFT] ----------------- ");
    }
    logToFile(" ----------------- [END ROTATION BASE SHIFT] ----------------- ");
}


/**
 * @brief Executes subtle body movements for the robot and simulates lifelike behavior.
 * 
 * This function controls the left and right arms, hands, and legs for the robot platform
 * and the left and right arms and legs for the simulator platform. It handles movement
 * simultaneously by launching threads for each limb.
 * 
 * @param nh The ROS NodeHandle.
 */
void subtleBodyMovement(ros::NodeHandle& nh) {
    logToFile(" ----------------- [START SUBTLE BODY MOVEMENT] ----------------- ");
    if (verboseMode) {
        ROS_INFO(" ----------------- [START SUBTLE BODY MOVEMENT] ----------------- ");
    }
    
    // Retrieve topic names for arms, hands, and legs from configuration
    std::string rightArmTopic = topicData["RArm"];
    std::string leftArmTopic = topicData["LArm"];
    std::string rightHandTopic = topicData["RHand"];
    std::string leftHandTopic = topicData["LHand"];
    std::string legTopic = topicData["Leg"];
        
    bool resetPosition = isFirstRun();
    if (resetPosition) {
        updateFirstRunFlag();
    }
    
    // Handle physical robot platform: Control arms, hands, and legs simultaneously
    if (configParams["platform"] == "robot") {
        ROS_INFO("Robot platform detected and flexi movement initiated");
       // Launch threads for simultaneous control of left and right arms and hands, and legs
        std::thread lArmThread(lArm, std::ref(nh), leftArmTopic, resetPosition);
        // std::thread rArmThread(rArm, std::ref(nh), rightArmTopic, resetPosition);
        // std::thread lHandThread(lHand, std::ref(nh), leftHandTopic, resetPosition);
        // std::thread rHandThread(rHand, std::ref(nh), rightHandTopic, resetPosition);
        // std::thread legThread(leg, std::ref(nh), legTopic, resetPosition);
        
        // Wait for all limb control threads to complete
        lArmThread.join();
        // lHandThread.join();
        // rArmThread.join();
        // rHandThread.join();
        // legThread.join();
       
    } else if (configParams["platform"] == "simulator") {
        // Launch threads for simultaneous control of left and right arms and legs
        std::thread lArmThread(lArm, std::ref(nh), leftArmTopic, resetPosition);
        std::thread rArmThread(rArm, std::ref(nh), rightArmTopic, resetPosition);
        std::thread legThread(leg, std::ref(nh), legTopic, resetPosition);
        
        // Wait for all limb control threads to complete
        lArmThread.join();
        rArmThread.join();
        legThread.join();
        
    }
    
    if (verboseMode) {
        ROS_INFO(" ----------------- [END SUBTLE BODY MOVEMENT] ----------------- ");
    }
    logToFile(" ----------------- [END SUBTLE BODY MOVEMENT] ----------------- ");
}


/**
 * @brief Executes flexible movements for the robot's hands and wrists.
 * 
 * This function controls the left and right arms and hands for the robot platform,
 * and only the arms for the simulator platform. It launches threads for simultaneous
 * control of the limbs based on the specified movement type.
 * 
 * @param nh The ROS NodeHandle to manage communication with ROS.
 * @param movementType A string indicating the type of movement ("flexi" or "All").
 */
void flexiMovement(ros::NodeHandle& nh, const std::string& movementType) {
    if (verboseMode) {
        ROS_INFO(" ----------------- [START FLEXI MOVEMENT] ----------------- ");
    }
    logToFile(" ----------------- [START FLEXI MOVEMENT] ----------------- ");

    std::string rightArmTopic = topicData["RArm"];
    std::string leftArmTopic = topicData["LArm"];
    std::string rightHandTopic = topicData["RHand"];
    std::string leftHandTopic = topicData["LHand"];
    bool resetPosition = isFirstRun();

    if (resetPosition) {
        updateFirstRunFlag();
    }

    // Handle physical robot platform: Control arms, and hands simultaneously
    if (configParams["platform"] == "robot") {
        ROS_INFO("Robot platform detected and flexi movement initiated");
        if (movementType == "flexi") {
            // Launch threads for controlling the wrist and hand of both left and right sides
            std::thread lArmlThread(lArml, std::ref(nh), leftArmTopic, resetPosition);
            std::thread lHandThread(lHand, std::ref(nh), leftHandTopic, resetPosition);
            std::thread rHandThread(rHand, std::ref(nh), rightHandTopic, resetPosition);
            std::thread rArmlThread(rArml, std::ref(nh), rightArmTopic, resetPosition);
            
            // Wait for all movement threads to complete
            lArmlThread.join();
            lHandThread.join();
            rHandThread.join();
            rArmlThread.join();
        } else if (movementType == "All") {
            std::thread lHandThread(lHand, std::ref(nh), leftHandTopic, resetPosition);
            std::thread rHandThread(rHand, std::ref(nh), rightHandTopic, resetPosition);
            
            // Wait for all movement threads to complete
            lHandThread.join();
            rHandThread.join();
        }
    }
    // Handle simulator platform: Control arms only
    else if (configParams["platform"] == "simulator") {
        if (movementType == "flexi") {
            // Launch threads for controlling the wrist and hand of both left and right sides
            std::thread lArmlThread(lArml, std::ref(nh), leftArmTopic, resetPosition);
            std::thread rArmlThread(rArml, std::ref(nh), rightArmTopic, resetPosition);
            
            // Wait for all movement threads to complete
            lArmlThread.join();
            rArmlThread.join();
        } else if (movementType == "All") {
            if(verboseMode) {
                ROS_INFO("FlexiMovement function not used for body movement in simulator");
            }
        }
    }

    if (verboseMode) {
        ROS_INFO("F ----------------- [END FLEXI MOVEMENT] ----------------- ");
    }
    logToFile(" ----------------- [END FLEXI MOVEMENT] ----------------- ");
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
        flexiMovement(nh, "flexi");
    }
    if (behaviour.find("rotation") != std::string::npos) {
        if (verboseMode) {
            ROS_INFO("[animateBehaviour] Executing rotation base shift.");
        }
        rotationBaseShift(nh);
    }

    // If no specific behaviour is provided, perform all available movements
    if (behaviour.empty()) {
        logToFile(" -------------- [START ALL ANIMATION BEHAVIOURS] -------------- ");
        if (verboseMode) {
            ROS_INFO(" -------------- [START ALL ANIMATION BEHAVIOURS] -------------- ");
        }
        boost::thread thread1(boost::bind(&subtleBodyMovement, boost::ref(nh)));
        boost::thread thread2(boost::bind(&rotationBaseShift, boost::ref(nh)));
        boost::thread thread3(boost::bind(&flexiMovement, boost::ref(nh), "All"));

        thread1.join();
        thread2.join();
        thread3.join();

        if (verboseMode) {
            ROS_INFO(" -------------- [END ALL ANIMATION BEHAVIOURS] -------------- ");
        }
        logToFile(" -------------- [END ALL ANIMATION BEHAVIOURS] -------------- ");
    }

}


