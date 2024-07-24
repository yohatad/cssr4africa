/************************************************************************************************
*  
* This code implements the Interaural Time Difference (ITD) algorithm to detect the direction of the 
* sound source by reading audio from four microphones arranged in a square pattern.
* 
*************************************************************************************************/

#include "util.h"
#include "sound.h"
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>  // Include for publishing Float64 messages
#include <map>
#include <cmath>
#include <algorithm>
#include <numeric>  // For std::inner_product
#include <fstream>

const float SPEED_OF_SOUND = 346.0f;
const float DISTANCE_BETWEEN_EARS = 0.0686f;
const float SAMPLING_RATE = 48000.0f;
const int TARGET_BUFFER_SIZE = 8192;  // New target buffer size for accumulation
const int WINDOW_SIZE = 1;  // Number of values to consider for the mode
const float INTENSITY_THRESHOLD = 400;

std::vector<double> angle_values;            // Vector to store ITD angle values
std::vector<int16_t> accumulated_frontLeft;  // Accumulated data for front left mic
std::vector<int16_t> accumulated_frontRight; // Accumulated data for front right mic
std::vector<int16_t> accumulated_rearLeft;   // Accumulated data for back left mic
std::vector<int16_t> accumulated_rearRight;  // Accumulated data for back right mic

double calculateItd(const float* data1, const float* data2, int size);
float calculateRms(const std::vector<int16_t>& data);
void audioCallback(const naoqi_driver::AudioCustomMsg& msg);
double calculateMode(const std::vector<double>& values);
double calculateSoundDirection(const std::vector<float>& data1, const std::vector<float>& data2,
                               const std::vector<float>& data3, const std::vector<float>& data4);

// Declare the publisher as a global variable
ros::Publisher sound_direction_pub;

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

        return (location >= size / 2 + 1) ? angle : -angle;
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

    accumulated_frontLeft.insert(accumulated_frontLeft.end(), frontLeft.begin(), frontLeft.end());
    accumulated_frontRight.insert(accumulated_frontRight.end(), frontRight.begin(), frontRight.end());
    accumulated_rearLeft.insert(accumulated_rearLeft.end(), rearLeft.begin(), rearLeft.end());
    accumulated_rearRight.insert(accumulated_rearRight.end(), rearRight.begin(), rearRight.end());
    

    if (accumulated_frontLeft.size() >= TARGET_BUFFER_SIZE) {
        std::vector<float> data1(accumulated_frontLeft.begin(), accumulated_frontLeft.end());
        std::vector<float> data2(accumulated_frontRight.begin(), accumulated_frontRight.end());
        std::vector<float> data3(accumulated_rearLeft.begin(), accumulated_rearLeft.end());
        std::vector<float> data4(accumulated_rearRight.begin(), accumulated_rearRight.end());

        std::vector<float> flfr(2 * TARGET_BUFFER_SIZE, 0.0f);
        std::vector<float> flbl(2 * TARGET_BUFFER_SIZE, 0.0f);
        std::vector<float> frbr(2 * TARGET_BUFFER_SIZE, 0.0f);

        correl(data1.data(), data2.data(), TARGET_BUFFER_SIZE, flfr.data());
        correl(data1.data(), data3.data(), TARGET_BUFFER_SIZE, flbl.data());
        correl(data2.data(), data4.data(), TARGET_BUFFER_SIZE, frbr.data());

        auto max_itFLFR = std::max_element(flfr.begin() + 1, flfr.begin() + TARGET_BUFFER_SIZE + 1);
        int locationFLFR = std::distance(flfr.begin(), max_itFLFR);
        
        if (locationFLFR != 1) {
            int num_samples = (locationFLFR >= TARGET_BUFFER_SIZE / 2 + 1) ? (TARGET_BUFFER_SIZE + 1 - locationFLFR) : (locationFLFR - 1);
            std::cout<<"num_samples FLFR: "<<num_samples<<std::endl;
        }

        auto max_itFLBL = std::max_element(flbl.begin() + 1, flbl.begin() + TARGET_BUFFER_SIZE + 1);
        int locationFLBL = std::distance(flbl.begin(), max_itFLBL);
        
        if (locationFLFR != 1) {
            int num_samples = (locationFLBL >= TARGET_BUFFER_SIZE / 2 + 1) ? (TARGET_BUFFER_SIZE + 1 - locationFLBL) : (locationFLBL - 1);
            std::cout<<"num_samples FLBL: "<<num_samples<<std::endl;
        }

        auto max_itFRBR = std::max_element(frbr.begin() + 1, frbr.begin() + TARGET_BUFFER_SIZE + 1);
        int locationFRBR = std::distance(frbr.begin(), max_itFRBR);

        if (locationFLFR != 1) {
            int num_samples = (locationFRBR >= TARGET_BUFFER_SIZE / 2 + 1) ? (TARGET_BUFFER_SIZE + 1 - locationFRBR) : (locationFRBR - 1);
            std::cout<<"num_samples FRBR: "<<num_samples<<std::endl;
        }

        float rmsFrontLeft = calculateRms(accumulated_frontLeft);
        float rmsFrontRight = calculateRms(accumulated_frontRight);

        float combinedIntensity = rmsFrontLeft + rmsFrontRight;

        if (combinedIntensity > INTENSITY_THRESHOLD){
            double itd = calculateItd(data1.data(), data2.data(), TARGET_BUFFER_SIZE);
            angle_values.push_back(itd);

            // Publish the sound direction
            std_msgs::Float64 sound_direction_msg;
            sound_direction_msg.data = itd;
            sound_direction_pub.publish(sound_direction_msg);
        }

        // Clear the accumulated buffers after processing
        accumulated_frontLeft.clear();
        accumulated_frontRight.clear();
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

    // Initialize the publisher
    sound_direction_pub = nh.advertise<std_msgs::Float64>("/soundDetection/direction", 1000);

    ros::Rate rate(10);  // 10 Hz loop rate
    while (ros::ok()) {
        ros::spinOnce();

        if (angle_values.size() >= WINDOW_SIZE) {
            std::vector<double> last_values(angle_values.end() - WINDOW_SIZE, angle_values.end());
            double mode = calculateMode(last_values);
            ROS_INFO("Mode of last %d ITD angle values: %f", WINDOW_SIZE, mode);
            angle_values.clear();
        }
        rate.sleep();
    }
    return 0;
}
