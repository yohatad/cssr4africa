/************************************************************************************************
*  
* This code implements the Interaural Time Difference (ITD) algorithm to detect the direction of the sound source
* by reading audio from four microphones arranged in a square pattern.
* 
*************************************************************************************************/

#include "util.h"
#include "sound.h"
#include <vector>
#include <ros/ros.h>
#include <map>
#include <cmath>
#include <algorithm>
#include <numeric>  // For std::inner_product
#include <fstream>

const float SPEED_OF_SOUND = 346.0f;
const float DISTANCE_BETWEEN_EARS = 0.07f;
const float SAMPLING_RATE = 48000.0f;
const int BUFFER_SIZE = 4096;
const int WINDOW_SIZE = 3;  // Number of values to consider for the mode
const float INTENSITY_THRESHOLD = 250;

std::vector<double> angle_values;  // Vector to store ITD angle values
bool value_received = false;

double calculateItd(const float* data1, const float* data2, int size);
float calculateRms(const std::vector<int16_t>& data);
void audioCallback(const naoqi_driver::AudioCustomMsg& msg);
double calculateMode(const std::vector<double>& values);

double calculateItd(const float* data1, const float* data2, int size) {
    std::vector<float> ans(2 * size, 0.0f);
    correl(data1 - 1, data2 - 1, size, ans.data());

    auto max_it = std::max_element(ans.begin() + 1, ans.begin() + size + 1);
    int location = std::distance(ans.begin(), max_it);

    if (location != 1) {
        int num_samples = (location >= size / 2 + 1) ? (size + 1 - location) : (location - 1);
        float ITD = num_samples / SAMPLING_RATE;
        float z = ITD * (SPEED_OF_SOUND / DISTANCE_BETWEEN_EARS);
        double angle = std::asin(z) * (180.0 / M_PI);

        return (location >= size / 2 + 1) ? -angle : angle;
    }

    return 0.0;
}

float calculateRms(const std::vector<int16_t>& data) {
    float sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0f);
    return std::sqrt(sum / data.size());
}

void audioCallback(const naoqi_driver::AudioCustomMsg& msg) {
    const std::vector<int16_t>& frontLeft = msg.frontLeft;
    const std::vector<int16_t>& frontRight = msg.frontRight;
    const std::vector<int16_t>& rearLeft = msg.rearLeft;
    const std::vector<int16_t>& rearRight = msg.rearRight;

    int bufferSize = frontLeft.size();

    std::vector<float> data1(frontLeft.begin(), frontLeft.end());
    std::vector<float> data2(frontRight.begin(), frontRight.end());
    std::vector<float> data3(rearLeft.begin(), rearLeft.end());
    std::vector<float> data4(rearRight.begin(), rearRight.end());

    float rmsFrontLeft = calculateRms(frontLeft);
    float rmsFrontRight = calculateRms(frontRight);
    float rmsRearLeft = calculateRms(rearLeft);
    float rmsRearRight = calculateRms(rearRight);

    // Determine the pair of microphones with the highest combined intensity
    float combinedFront = rmsFrontLeft + rmsFrontRight;
    float combinedRear = rmsRearLeft + rmsRearRight;
    float combinedLeft = rmsFrontLeft + rmsRearLeft;
    float combinedRight = rmsFrontRight + rmsRearRight;

    float maxCombinedIntensity = std::max({combinedFront, combinedRear, combinedLeft, combinedRight});
    // std::cout<<"Max combined intensity: "<<maxCombinedIntensity<<std::endl;

    // print the intesnity for each 
    std::cout<<"Front: "<<combinedFront<< " rmsFrontLeft: "<<rmsFrontLeft<<" rmsFrontRight: "<<rmsFrontRight<<std::endl;
    std::cout<<"Rear: "<<combinedRear<< " rmsRearLeft: "<<rmsRearLeft<<" rmsRearRight: "<<rmsRearRight<<std::endl;
    std::cout<<"Left: "<<combinedLeft<< " rmsFrontLeft: "<<rmsFrontLeft<<" rmsRearLeft: "<<rmsRearLeft<<std::endl;
    std::cout<<"Right: "<<combinedRight<< " rmsFrontRight: "<<rmsFrontRight<<" rmsRearRight: "<<rmsRearRight<<std::endl;

    // write the audio data to a file
    writeToFile(frontLeft, "frontLeft.txt");

   

    // compare the four microphones and print out which one has the highest intensity
    // if (rmsFrontLeft > rmsFrontRight && rmsFrontLeft > rmsRearLeft && rmsFrontLeft > rmsRearRight && rmsFrontLeft > INTENSITY_THRESHOLD) {
    //     std::cout<<"Front Left " <<rmsFrontLeft<<std::endl;
    // } else if (rmsFrontRight > rmsFrontLeft && rmsFrontRight > rmsRearLeft && rmsFrontRight > rmsRearRight && rmsFrontRight > INTENSITY_THRESHOLD) {
    //     std::cout<<"Front Right "<<rmsFrontRight<<std::endl;
    // } else if (rmsRearLeft > rmsFrontLeft && rmsRearLeft > rmsFrontRight && rmsRearLeft > rmsRearRight && rmsRearLeft > INTENSITY_THRESHOLD) {
    //     std::cout<<"Rear Left " <<rmsRearLeft<<std::endl; 
    // } else if (rmsRearRight > rmsFrontLeft && rmsRearRight > rmsFrontRight && rmsRearRight > rmsRearLeft && rmsRearRight > INTENSITY_THRESHOLD) {
    //     std::cout<<"Rear Right "<<rmsRearRight<<std::endl;
    // }
    

    if (maxCombinedIntensity < INTENSITY_THRESHOLD) {
        return;
    }

    if (maxCombinedIntensity == combinedFront) {
        // std::cout<<"front"<<std::endl; 
        double angle = calculateItd(data1.data(), data2.data(), bufferSize);
        angle_values.push_back(angle * (rmsFrontLeft > rmsFrontRight ? 1 : -1));
        value_received = true;
    } else if (maxCombinedIntensity == combinedRear) {
        // std::cout<<"back"<<std::endl;
        double angle = calculateItd(data3.data(), data4.data(), bufferSize);
        angle_values.push_back(angle * (rmsRearLeft > rmsRearRight ? 1 : -1));
        value_received = true;
    } else if (maxCombinedIntensity == combinedLeft) {
        // std::cout<<"left"<<std::endl;
        double angle = calculateItd(data1.data(), data3.data(), bufferSize);
        angle_values.push_back(angle * (rmsFrontLeft > rmsRearLeft ? 1 : -1));
        value_received = true;
    } else if (maxCombinedIntensity == combinedRight) {
        // std::cout<<"right"<<std::endl;
        double angle = calculateItd(data2.data(), data4.data(), bufferSize);
        angle_values.push_back(angle * (rmsFrontRight > rmsRearRight ? 1 : -1));
        value_received = true;
    } else {
        ROS_INFO("Sound intensity is too low to determine direction");
    }
}

double calculateMode(const std::vector<double>& values) {
    std::map<double, int> frequency_map;
    for (double value : values) {
        frequency_map[value]++;
    }

    return std::max_element(frequency_map.begin(), frequency_map.end(),
                            [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
                                return a.second < b.second;
                            })->first;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "sound_localization");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/naoqi_driver/audio", 1000, audioCallback);


    ros::Rate rate(10);  // 10 Hz loop rate
    while (ros::ok()) {
        ros::spinOnce();

        if (value_received && angle_values.size() >= WINDOW_SIZE) {
            std::vector<double> last_values(angle_values.end() - WINDOW_SIZE, angle_values.end());
            double mode = calculateMode(last_values);
            // ROS_INFO("Mode of last %d ITD angle values: %f", WINDOW_SIZE, mode);

            angle_values.clear();
            value_received = false;
        }

        rate.sleep();
    }

    return 0;
}
