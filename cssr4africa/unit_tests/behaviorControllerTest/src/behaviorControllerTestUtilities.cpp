/* behaviorControllerTestUtilities.cpp - implementation of shared methods and delcaration of shared variables between the driver, stub and application.
*
* Author: Tsegazeab Taye Tefferi
* Date: April 20, 2025
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*/
#include <behaviorControllerTest/behaviorControllerTestUtilitiesInterface.h>

bool testMode;
float failureRate;

/* Returns true if failureRate is lessthan a random evaluated number */
bool hasSucceeded() {
    return (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) > failureRate;
}

/* Returns true if ch isn't an empty space character*/
static bool isNotSpace(unsigned char ch) {
    return !std::isspace(ch);
}

/* Trims whitespaces inplace */
static inline void trim(std::string &s) {
    // Trim leading spaces
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), isNotSpace));
    
    // Trim trailing spaces
    s.erase(std::find_if(s.rbegin(), s.rend(), isNotSpace).base(), s.end());
}

/* Returns the value of a key from the configuration file. */
std::string getValueFromConfig(const std::string &key) {
    std::string value; 
    std::ifstream configFile(ros::package::getPath(ROS_PACKAGE_NAME) + "/behaviorControllerTest/config/behaviorControllerTestConfiguration.ini");
    if (!configFile.is_open()) {
        throw std::runtime_error("Failed to open configuration file.");
    }

    std::string configLineRead;

    while (std::getline(configFile, configLineRead)) {
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        
        trim(paramKey);
        std::getline(iss, paramValue);
        trim(paramValue);
        
        if (paramKey == key) { 
            value = paramValue;
            break;
        }
    }
    configFile.close();
    if (value.empty()) {
        throw std::runtime_error("Failed to retrieve value for key: " + key);
    }
    return value;
}
