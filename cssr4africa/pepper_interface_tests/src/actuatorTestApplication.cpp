# include "pepper_interface_tests/actuatorTest.h"

int main(int argc, char** argv) {
    std::vector<std::string> test_name = extract_tests("actuator");

    // Initialize ROS
    ros::init(argc, argv, "actuatorTest");
    ros::NodeHandle nh;

    // Extract the mode to run the tests
    std::string mode = extract_mode();
    std::cout << "Mode: " << mode << std::endl;

    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
        ROS_FATAL("Timeout waiting for valid time");
        return EXIT_FAILURE; 
    }
    
    // Run the tests in the mode specified in the configuration file
    if (mode == "sequential"){
        for (int i = 0; i < test_name.size(); ++i){
            if (test_name[i] == "Head"){
                std::string head_topic = extract_topic("Head");
                head(nh, head_topic);
            }
            else if (test_name[i] == "RArm"){
                std::string rightArm_topic = extract_topic("RArm");
                rArm(nh, rightArm_topic);
            }
            else if (test_name[i] == "LArm"){
                std::string leftArm_topic = extract_topic("LArm");
                lArm(nh, leftArm_topic);
            }
            else if (test_name[i] == "RHand"){
                std::string rightHand_topic = extract_topic("RHand");
                rHand(nh, rightHand_topic);
            }
            else if (test_name[i] == "LHand"){
                std::string leftHand_topic = extract_topic("LHand");
                lHand(nh, leftHand_topic);
            }
            else if (test_name[i] == "Leg"){
                std::string leg_topic = extract_topic("Leg");
                leg(nh, leg_topic);
            }
            else if (test_name[i] == "Wheels"){
                std::string wheels_controller = extract_topic("Wheels");
                wheels(nh);
            }
        }
    }

    else if (mode == "parallel"){
        std::vector<std::thread> threads;
        for (int i = 0; i < test_name.size(); ++i){
            if (test_name[i] == "Head"){
                std::string head_topic = extract_topic("Head");
                threads.push_back(std::thread(head, std::ref(nh), head_topic));
            }
            else if (test_name[i] == "RArm"){
                std::string rightArm_topic = extract_topic("RArm");
                threads.push_back(std::thread(rArm, std::ref(nh), rightArm_topic));
            }
            else if (test_name[i] == "LArm"){
                std::string leftArm_topic = extract_topic("LArm");
                threads.push_back(std::thread(lArm, std::ref(nh), leftArm_topic));
            }
            else if (test_name[i] == "RHand"){
                std::string rightHand_topic = extract_topic("RHand");
                threads.push_back(std::thread(rHand, std::ref(nh), rightHand_topic));
            }
            else if (test_name[i] == "LHand"){
                std::string leftHand_topic = extract_topic("LHand");
                threads.push_back(std::thread(lHand, std::ref(nh), leftHand_topic));
            }
            else if (test_name[i] == "Leg"){
                std::string leg_topic = extract_topic("Leg");
                threads.push_back(std::thread(leg, std::ref(nh), leg_topic));
            }
            else if (test_name[i] == "Wheels"){
                std::string wheels_controller = extract_topic("Wheels");
                threads.push_back(std::thread(wheels, std::ref(nh)));
            }
        }
        for (auto& th : threads) th.join();
    }
    

    else{
        printf("Invalid mode. Please check the mode in the configuration file.\n");
        prompt_and_exit(1);
    }
   
       
    return 0;
}
