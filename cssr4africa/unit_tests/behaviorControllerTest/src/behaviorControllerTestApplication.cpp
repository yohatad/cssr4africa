/* behaviorControllerTestApplication.cpp
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
* <detailed functional description>
* This module is responsible for running the tests on the behaviorController ROS node
* The tests will check if all the action and condition nodes of the behaviorController are communicating
* and processing the data they receive as expected
*
*
*
...
* Libraries
* Standard libraries
- std::string, std::vector, std::fstream
* ROS libraries
- ros/ros.h, ros/package.h
*
*
...
* Parameters
*
* Command-line Parameters
*
* None
...
* Configuration File Parameters

* Key                   |     Value 
* --------------------- |     -------------------
* verboseMode  `        |      <true/false - enables/disables the display of diagnostic messages>
* failureRate           |      the rate at which service calls and topics will provide a successful response
* arrivalRate           |      the rate at wich an event occurs( valid only for the driver functions)
* testMode              |      <true/false> - true - stubs & drivers will alwaus return successfully


...
* Subscribed Topics and Message Types
*
* None
...
* Published Topics and Message Types
* 
* None

...
* Advertised Services
* 
* None

...
* Services Invoked
* 
* None                                    
...
* Input Data Files
*
* None
...
* Output Data Files
*
* behaviorControllerTestOutput.dat
...
* Configuration Files
*
* behaviorControllerTestConfiguration.ini
...
* Example Instantiation of the Module
*
* roslaunch unit_tests behaviorControllerLaunchTestHarness
...
*
...
*
* Author: Tsegazeab Taye Tefferi, Carnegie Mellon University Africa
* Email: ttefferi@andrew.cmu.edu
* Date: April 20, 2025
* Version: v1.0
*
*/


#include <behaviorControllerTest/behaviorControllerTestInterface.h>

bool verboseMode;
std::string nodeName;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "behaviorControllerTest");
    ros::NodeHandle nh("/behaviorControllerTest");

    nodeName = ros::this_node::getName();
    std::string softwareVersion = "1.0";
    std::string leftIndent = "\n\t\t";
    ROS_INFO_STREAM("\n"
         "**************************************************************************************************\n"
         + leftIndent + nodeName + "\t" + softwareVersion
         + leftIndent + "Copyright (C) 2023 CSSR4Africa Consortium"
         + leftIndent + "This project is funded by the African Engineering and Technology Network (Afretec)"
         + leftIndent + "Inclusive Digital Transformation Research Grant Program.\n"
         + leftIndent + "Website: www.cssr4africa.org\n"
         + leftIndent + "This program comes with ABSOLUTELY NO WARRANTY."
         + "\n\n**************************************************************************************************\n\n"
    );
    
    /* Retrieve the values from the configuration file       */
    /* Display the error and exit, if the file is unreadable */
    try
    {
        verboseMode = (getValueFromConfig("verboseMode") == "true");
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Fatal Error: "<<e.what());
        ros::shutdown();
        return 0;
    }

    /* MAIN TEST SEQUENCE */
    ros::Rate rate(10);
    printMsg(INFO_MSG,"Waiting for main node to start test sequence ...");
    while(ros::ok()){
        if(nh.hasParam("TestStarted")){
            printMsg(INFO_MSG,"Test Sequence Started");
            break;
        }
        rate.sleep();
    }
    printMsg(INFO_MSG,"Waiting for main node to finish test sequence ...");
    std::ofstream resultsFile;
    resultsFile.open(ros::package::getPath(ROS_PACKAGE_NAME) + "/behaviorControllerTest/data/behaviorControllerTestOutput.dat", std::ios::out);

    if (resultsFile.is_open()) {
        resultsFile << "Test Results\n";
        
        // Get current date
        std::time_t now = std::time(nullptr);
        char buf[100];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        resultsFile << "Date: " << buf << "\n\n";
        while(ros::ok()){
            if(nh.hasParam("TestEnded")){
                ros::V_string testParams;  
                nh.getParamNames(testParams);

                for (const auto& testParam : testParams){
                    int result = -1;
                    if(nh.getParam(testParam,result)){
                        std::string testName = testParam.substr(testParam.find_last_of('/') + 1);
                        std::string status = (result ==1) ? "Passed!": "Failed!";
                        std::string logMsg = testName + " -> " + status;
                        if(result==0){
                            printMsg(WARNING_MSG,logMsg);
                        }else{
                            printMsg(INFO_MSG,logMsg);
                        }
                        resultsFile << logMsg << "\n";
                        nh.deleteParam(testParam);
                    }
                }    
                break;
            }
            rate.sleep();

        }
    }
    ros::shutdown(); 
    return 0;
}