#include "gesture_execution/gestureExecution.h"

// gesture types
// 01 - point/deictic




/* Main function */
int main(int argc, char **argv){
    printf("Gesture Execution\n");
    printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n\n");
    bool verbose_mode = false;
    int executionStatus = 0;

    // Initialize ROS
    ros::init(argc, argv, "gestureExecution");
    ros::NodeHandle nh;

    string implementationPlatform;
    string interpolationType;
    string gestureDescriptorsFileName;
    string simulatorTopicsFileName;
    string robotTopicsFileName;
    string verboseModeInput;

    // string topic_name = extract_topic("RArm");
    // printf("Topic name is %s\n", topic_name.c_str());

    /* Read gesture execution configuration */
    read_gesture_execution_configuration(&implementationPlatform, &interpolationType, &gestureDescriptorsFileName, &simulatorTopicsFileName, &robotTopicsFileName, &verboseModeInput);

    // read_gesture_execution_input(&gesture_type, &gesture_id, &robot_location_input, &debug_value);
    if (verboseModeInput == "true"){
        verbose_mode = true;
    }
    else if (verboseModeInput == "false"){
        verbose_mode = false;
    }
    else{
        printf("Verbose value in input file not supported. Supported verboseMode values are: true and false\n");
        prompt_and_exit(1);
    }

    if (verbose_mode){
        printf("Gesture execution node configuration:\n");
        printf("\tImplementation platform is %s\n", implementationPlatform.c_str());
        printf("\tInterpolation type is %s\n", interpolationType.c_str());
        printf("\tGestures Descriptors file is %s\n", gestureDescriptorsFileName.c_str());
        printf("\tSimulator Topics file is %s\n", simulatorTopicsFileName.c_str());
        printf("\tRobot Topics file is %s\n", robotTopicsFileName.c_str());
    }

    string gestureType;
    string gestureID;
    int numberOfWaypoints;
    string jointAngles;

    /* Read gesture descriptors */
    read_gesture_descriptors(gestureDescriptorsFileName, &gestureType, &gestureID, &numberOfWaypoints, &jointAngles);

    if (verbose_mode){
        printf("Gesture descriptors:\n");
        printf("\tGesture type is %s\n", gestureType.c_str());
        printf("\tGesture ID is %s\n", gestureID.c_str());
        printf("\tNumber of waypoints is %d\n", numberOfWaypoints);
        printf("\tJoint angles are %s\n", jointAngles.c_str());
    }

    /* Deictic Gestures Execution */
    if((gestureType == DEICTIC_GESTURES) || (gestureType == DIECTIC_GESTURES)){
        if (verbose_mode) {
            printf("Deictic gesture ");
        }

        if (gestureID == "01"){
            if(verbose_mode){
                printf("ID is 01; Pointing gesture to be executed\n");
            }

            // executionStatus = execute_pointing_gesture(robot_location_input, verbose_mode);
            if(verbose_mode){
                printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
                if (executionStatus == 0){
                    printf("Pointing gesture executed successfully\n");
                }
                else{
                    printf("Pointing gesture execution failed\n");
                }
            }

            prompt_and_exit(executionStatus);
        }
        else{
            printf("ID not supported. Supported deictic gesture IDs are: 01\n");
            prompt_and_exit(1);
        }
    }

    // /* Iconic and Symbolic Gestures Execution */
    // else if((gesture_type == ICONIC_GESTURES) || (gesture_type == SYMBOLIC_GESTURES)){
    //     if (verbose_mode) {
    //         if (gesture_type == ICONIC_GESTURES){
    //             printf("Iconic gesture ");
    //         }
    //         else{
    //             printf("Symbolic gesture ");
    //         }
    //     }

    //     if (gesture_id == "02"){
    //         if(verbose_mode){
    //             printf("ID is 02; Bowing gesture to be executed\n");
    //         }

    //         executionStatus = execute_bowing_gesture(robot_location_input, verbose_mode);
    //         if(verbose_mode){
    //             printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
    //             if (executionStatus == 0){
    //                 printf("Bowing gesture executed successfully\n");
    //             }
    //             else{
    //                 printf("Bowing gesture execution failed\n");
    //             }
    //         }

    //         prompt_and_exit(executionStatus);

    //     }

    //     else if  (gesture_id == "03"){
    //         if(verbose_mode){
    //             printf("ID is 03; Nodding gesture to be executed\n");
    //         }

    //         executionStatus = execute_nodding_gesture(robot_location_input, verbose_mode);
    //         if(verbose_mode){
    //             printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
    //             if (executionStatus == 0){
    //                 printf("Nodding gesture executed successfully\n");
    //             }
    //             else{
    //                 printf("Nodding gesture execution failed\n");
    //             }
    //         }

    //         prompt_and_exit(executionStatus);
    //     }

    //     else if  (gesture_id == "04"){
    //         if(verbose_mode){
    //             printf("ID is 04; Waving gesture to be executed\n");
    //         }

    //         executionStatus = execute_waving_gesture(robot_location_input, verbose_mode);
    //         if(verbose_mode){
    //             if (executionStatus == 0){
    //                 printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
    //                 printf("Waving gesture executed successfully\n");
    //             }
    //             else{
    //                 printf("Waving gesture execution failed\n");
    //             }
    //         }

    //         prompt_and_exit(executionStatus);
    //     }

    //     else{
    //         printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
    //         printf("ID not supported. Supported iconic gesture IDs are: 02 - Bow, 03 - Nod, 04 - Wave.\n");
    //         prompt_and_exit(1);
    //     }
    // }
    
    // /* Gesture type not supported */
    // else{
    //     printf("--------------------------------------------------------------------------------------------------------------------------------------------------\n");
    //     printf("Gesture type not supported. Supported gesture types are: deictic, iconic, and symbolic\n");
    //     prompt_and_exit(1);
    // }

    prompt_and_exit(0);
}