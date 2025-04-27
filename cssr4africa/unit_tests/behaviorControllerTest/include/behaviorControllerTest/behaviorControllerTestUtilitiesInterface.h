/* behaviorControllerTestUtilitiesInterface.h - interface file for the behaviorControllerTestUtilities.cpp 
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

#ifndef BEHAVIOR_CONTROLLER_TEST_UTILITIES_INTERFACE_H
#define BEHAVIOR_CONTROLLER_TEST_UTILITIES_INTERFACE_H

#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdexcept>
#include "ros/ros.h"
#include <ros/package.h>

extern bool testMode;
extern float failureRate;

/* Returns the value of a key from the configuration file. */
std::string getValueFromConfig(const std::string &key);

/* Returns true if failureRate is lessthan a random evaluated number */
bool hasSucceeded();

#endif