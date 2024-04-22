#include "cssr_system/animateBehaviourConfigAndService.h" // Adjust this include path as needed

int main(int argc, char **argv) {
    ros::init(argc, argv, "animateBehaviour");
    ros::NodeHandle nh;

    std::string configPath = ros::package::getPath("cssr_system") + "/config/animateBehaviourConfiguration.ini";

    // Load configuration and retrieve the platform and behaviour values
    std::string platform = loadConfiguration(configPath);
    if (platform.empty()) {
        ROS_ERROR("Failed to load platform configuration.");
        return -1; // Exit if configuration loading fails
    }
    else {
        // Print the platform values
        ROS_INFO("Platform configuration loaded successfully: %s", platform.c_str());
    }

    // Assuming `behaviour` is also loaded by `loadConfiguration`
    std::string behaviour = configParams["behaviour"];
    //ROS_INFO("Behaviour loaded successfully: %s", behaviour.c_str());

    // Load data based on the platform
    loadDataBasedOnPlatform(platform);

    // ros::ServiceServer service = nh.advertiseService("animateBehaviour/set_activation", setActivation);

    // ROS_INFO("animateBehaviour node started. Waiting for activation...");

    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
        // if (isActive) {
        //     ROS_INFO_THROTTLE(60, "animateBehaviour is active.");
        animateBehaviour(behaviour, nh); // Call with the loaded behaviour
        // } else {
        //     ROS_INFO_ONCE("animateBehaviour node is inactive. Activate to start animating.");
        // }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Shutting down animateBehaviour node...");

    return 0;
}
