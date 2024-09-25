#include "animate_behaviour/animateBehaviourInterface.h"

std::ofstream logFile;
bool isActive = true;  // Definition of the global variable

// Function to log messages to the file
void logToFile(const std::string &message) {
    if (logFile.is_open()) {
        logFile << message << std::endl;
    }
}

// Function to close the log file
void closeLogFile() {
    if (logFile.is_open()) {
        logFile.close();
    }
}

// Define the Service Function
bool setActivation(cssr_system::SetActivation::Request &req, cssr_system::SetActivation::Response &res) {
    if (req.state == "enabled") {
        isActive = true;
        res.success = true;
        ROS_INFO("Animate behaviour enabled.");
        logToFile("Animate behaviour enabled.");
    } else if (req.state == "disabled") {
        isActive = false;
        res.success = true;
        ROS_INFO("Animate behaviour disabled.");
        logToFile("Animate behaviour disabled.");
    } else {
        res.success = false;
        ROS_WARN("Invalid state requested: %s. Use 'enabled' or 'disabled'.", req.state.c_str());
        logToFile("Invalid state requested: " + req.state + ". Use 'enabled' or 'disabled'.");
    }
    return true;
}

void resetAnimateBehaviour() {
    // implement clear  isFirstRun function
    if (remove(FLAG_FILE_PATH.c_str()) != 0) {
        std::cerr << "Error deleting file" << std::endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "animateBehaviour");
    ros::NodeHandle nh;

    // Define the log file path dynamically using ROS_PACKAGE_NAME
    std::string logFilePath = ros::package::getPath(ROS_PACKAGE_NAME) + "/animateBehaviour/data/logFile.log";
    std::remove(logFilePath.c_str());
    logFile.open(logFilePath, std::ios::out | std::ios::app);
    if (!logFile.is_open()) {
        ROS_ERROR("Failed to open log file: %s", logFilePath.c_str());
        return -1;
    }

    // Start a thread to close and delete the log file after 5 minutes
    std::thread closeLogFileThread([]() {
        std::this_thread::sleep_for(std::chrono::minutes(10));
        closeLogFile();
    });
    closeLogFileThread.detach();

    // Define the configuration filename
    std::string config_filename = "animateBehaviourConfiguration.ini";
    // Get the base path dynamically using ROS_PACKAGE_NAME and construct the full path
    std::string configPath = ros::package::getPath(ROS_PACKAGE_NAME) + "/animateBehaviour/config/" + config_filename;
    // Log the constructed path
    ROS_INFO("Configuration file path: %s", configPath.c_str());

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("animateBehaviour/set_activation", setActivation);
    ROS_INFO("animateBehaviour node started. Waiting for activation...");
    logToFile("animateBehaviour node started. Waiting for activation...");
    ros::Rate loop_rate(10);

    std::string platform;
    std::string behaviour;
    bool wasActive = false;  // Track the previous active state

    while (ros::ok()) {
        try {
            if (isActive) {
                if (!wasActive) {
                    ROS_INFO("animateBehaviour node is active. Ready to animate.");
                    logToFile("animateBehaviour node is active. Ready to animate.");

                    try {
                        loadConfiguration(configPath);
                    } catch (const std::exception &e) {
                        ROS_ERROR("Exception caught during configuration load: %s", e.what());
                        logToFile("Exception caught during configuration load: " + std::string(e.what()));
                        isActive = false;
                        continue;
                    }

                    platform = configParams["platform"];
                    if (platform.empty()) {
                        ROS_ERROR("Failed to load platform configuration.");
                        logToFile("Failed to load platform configuration.");
                        isActive = false;
                        continue;
                    } else {
                        ROS_INFO("Platform configuration loaded successfully: %s", platform.c_str());
                        logToFile("Platform configuration loaded successfully: " + platform);
                    }

                    behaviour = configParams["behaviour"];
                    try {
                        loadDataBasedOnPlatform(platform);
                    } catch (const std::exception &e) {
                        ROS_ERROR("Exception caught during data load: %s", e.what());
                        logToFile("Exception caught during data load: " + std::string(e.what()));
                        isActive = false;
                        continue;
                    }

                    ROS_INFO("Configuration loaded and data initialized.");
                    logToFile("Configuration loaded and data initialized.");
                }

                try {
                    animateBehaviour(behaviour, nh);
                } catch (const std::exception &e) {
                    ROS_ERROR("Exception caught during animate behaviour: %s", e.what());
                    logToFile("Exception caught during animate behaviour: " + std::string(e.what()));
                    isActive = false;
                    continue;
                }

                wasActive = true;
            } else {
                if (wasActive) {
                    ROS_INFO("animateBehaviour node is inactive. Activate to start animating.");
                    logToFile("animateBehaviour node is inactive. Activate to start animating.");
                    resetAnimateBehaviour();
                }
                wasActive = false;
            }

            ros::spinOnce();
            loop_rate.sleep();
        } catch (const std::exception &e) {
            ROS_ERROR("Exception caught in main loop: %s", e.what());
            logToFile("Exception caught in main loop: " + std::string(e.what()));
        }
    }

    ROS_INFO("Shutting down animateBehaviour node...");
    logToFile("Shutting down animateBehaviour node...");

    return 0;
}
