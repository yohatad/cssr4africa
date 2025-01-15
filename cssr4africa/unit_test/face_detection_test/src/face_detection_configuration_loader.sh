#!/bin/bash

# Check if jq is installed
if ! command -v jq &> /dev/null; then
    echo "Error: jq is not installed. Please install jq to continue."
    exit 1
fi

# Set the ROS package path dynamically
PACKAGE_PATH=$(rospack find face_detection_test)

# Define the path to the configuration file
CONFIG_FILE="$PACKAGE_PATH/config/face_detection_test_configuration.json"

# Check if the configuration file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file not found: $CONFIG_FILE"
    exit 1
fi

# Read and sanitize values from the configuration file using jq
CAMERA_TYPE=$(jq -r '.camera' "$CONFIG_FILE")
BAG_NUMBER=$(jq -r '.bag_number' "$CONFIG_FILE")

# Validate the camera type
if [ "$CAMERA_TYPE" != "realsense" ] && [ "$CAMERA_TYPE" != "pepper" ]; then
    echo "Error: Invalid camera type specified in config file: $CAMERA_TYPE"
    echo "Expected 'realsense' or 'pepper'."
    exit 1
fi

# Construct the path to the .bag file
BAG_PATH="$PACKAGE_PATH/data/face_detection_test_input_${CAMERA_TYPE}_${BAG_NUMBER}.bag"

# Check if the .bag file exists
if [ ! -f "$BAG_PATH" ]; then
    echo "Error: Bag file not found: $BAG_PATH"
    exit 1
fi

# Play the .bag file using rosbag
echo "Playing bag file: $BAG_PATH"
rosbag play "$BAG_PATH"
