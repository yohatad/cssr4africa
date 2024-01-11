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
                std::string head_topic = extractTopic("Head");
                head(nh, head_topic);
            }
            else if (test_name[i] == "RArm"){
                std::string rightArm_topic = extractTopic("RArm");
                rArm(nh, rightArm_topic);
            }
            else if (test_name[i] == "LArm"){
                std::string leftArm_topic = extractTopic("LArm");
                lArm(nh, leftArm_topic);
            }
            else if (test_name[i] == "RHand"){
                std::string rightHand_topic = extractTopic("RHand");
                rHand(nh, rightHand_topic);
            }
            else if (test_name[i] == "LHand"){
                std::string leftHand_topic = extractTopic("LHand");
                lHand(nh, leftHand_topic);
            }
            else if (test_name[i] == "Leg"){
                std::string leg_topic = extractTopic("Leg");
                leg(nh, leg_topic);
            }
            else if (test_name[i] == "Wheels"){
                std::string wheels_controller = extractTopic("Wheels");
                wheels(nh);
            }
        }
    }

    else if (mode == "parallel"){
        std::vector<std::thread> threads;
        for (int i = 0; i < test_name.size(); ++i){
            if (test_name[i] == "Head"){
                std::string head_topic = extractTopic("Head");
                threads.push_back(std::thread(head, std::ref(nh), head_topic));
            }
            else if (test_name[i] == "RArm"){
                std::string rightArm_topic = extractTopic("RArm");
                threads.push_back(std::thread(rArm, std::ref(nh), rightArm_topic));
            }
            else if (test_name[i] == "LArm"){
                std::string leftArm_topic = extractTopic("LArm");
                threads.push_back(std::thread(lArm, std::ref(nh), leftArm_topic));
            }
            else if (test_name[i] == "RHand"){
                std::string rightHand_topic = extractTopic("RHand");
                threads.push_back(std::thread(rHand, std::ref(nh), rightHand_topic));
            }
            else if (test_name[i] == "LHand"){
                std::string leftHand_topic = extractTopic("LHand");
                threads.push_back(std::thread(lHand, std::ref(nh), leftHand_topic));
            }
            else if (test_name[i] == "Leg"){
                std::string leg_topic = extractTopic("Leg");
                threads.push_back(std::thread(leg, std::ref(nh), leg_topic));
            }
            else if (test_name[i] == "Wheels"){
                std::string wheels_controller = extractTopic("Wheels");
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
