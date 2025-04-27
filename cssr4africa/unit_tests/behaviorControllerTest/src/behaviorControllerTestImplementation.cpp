/* behaviorControllerTestImplementation.cpp   Source code for the methods used by behaviiorControllerTestApplication
 *
 * Author: Tsegazeab Taye Tefferi
 * Date: April 25, 2025
 * Version: 1.0
 *
 * Copyright (C) 2023 CSSR4Africa Consortium
 *
 * This project is funded by the African Engineering and Technology Network (Afretec)
 * Inclusive Digital Transformation Research Grant Programme.
 *
 * Website: www.cssr4africa.org
 *
 * This program comes with ABSOLUTELY NO WARRANTY.
 *
 */

#include <behaviorControllerTest/behaviorControllerTestInterface.h>

/*
Logs the string (args) to the terminal based on the (type).
Wrapper around the default ROS logging functions
*/
void printMsg(int type,std::string args){
    if(!verboseMode){
        return;
    }

    std::string msg = "[" + nodeName + "]: " + args;
    switch (type){
        case INFO_MSG:
            ROS_INFO_STREAM(msg);
            break;
        case WARNING_MSG:
            ROS_WARN_STREAM(msg);
            break;
        case ERROR_MSG:
            ROS_ERROR_STREAM(msg);
            break;
        default:
            ROS_ERROR_STREAM("UNDEFINED MSG TYPE");
    }
}

