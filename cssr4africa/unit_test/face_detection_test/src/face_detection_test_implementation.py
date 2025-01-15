import rospkg
import json
import os
import rospy

class FaceDetectionTest:
    def __init__(self):
        # Initialize the ROS package paths
        self._rospack = rospkg.RosPack()
        self._face_detection_test_package_path = self._rospack.get_path('face_detection_test')
        self._face_detection_package_path = self._rospack.get_path('face_detection')
        
        # Read the configuration file
        self.config = self.read_json_file()
        if not self.config:
            rospy.logerr("Failed to load configuration. Exiting.")
            raise RuntimeError("Configuration file could not be loaded.")

    def read_json_file(self):
        """Read the configuration file and return the parsed JSON object."""
        try:
            config_path = os.path.join(self._face_detection_test_package_path, 'config', 'face_detection_test_configuration.json')
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    config = json.load(file)
                    return config
            else:
                rospy.logerr(f"read_json_file: Configuration file not found at {config_path}")
                return None
        except json.JSONDecodeError as e:
            rospy.logerr(f"read_json_file: Error decoding JSON file: {e}")
            return None
        
    def update_face_detection_configuration(self):
        """Update the face_detection_configuration.json file for verbose_mode, camera, and algorithm."""
        try:
            # Define paths for the test configuration and main configuration
            test_config_path = os.path.join(self._face_detection_test_package_path, 'config', 'face_detection_test_config.json')
            face_detection_config_path = os.path.join(self._face_detection_package_path, 'config', 'face_detection_configuration.json')
            
            # Read the test configuration file
            if not os.path.exists(test_config_path):
                rospy.logerr(f"Test configuration file not found: {test_config_path}")
                return

            with open(test_config_path, 'r') as test_config_file:
                test_config = json.load(test_config_file)
            
            # Read the current face_detection_configuration file
            if not os.path.exists(face_detection_config_path):
                rospy.logwarn(f"Main configuration file not found. A new one will be created: {face_detection_config_path}")
                face_detection_config = {}
            else:
                with open(face_detection_config_path, 'r') as detection_config_file:
                    face_detection_config = json.load(detection_config_file)
            
            # Update only verbose_mode, camera, and algorithm
            face_detection_config["verbose_mode"] = test_config.get("verbose_mode", face_detection_config.get("verbose_mode", False))
            face_detection_config["camera"] = test_config.get("camera", face_detection_config.get("camera", "realsense"))
            face_detection_config["algorithm"] = test_config.get("algorithm", face_detection_config.get("algorithm", "sixdrep"))
            
            # Save the updated face_detection_configuration file
            with open(face_detection_config_path, 'w') as detection_config_file:
                json.dump(face_detection_config, detection_config_file, indent=4)
            
            rospy.loginfo(f"Updated face_detection_configuration.json file at: {face_detection_config_path}")
        
        except Exception as e:
            rospy.logerr(f"An error occurred while updating the configuration: {e}")
   

    def test_media_pipe(self):
        """Perform MediaPipe face detection test."""
        rospy.loginfo("Starting MediaPipe face detection test...")
        # Add your test logic here
        # Example: rospy.loginfo(f"Using configuration: {self.config['mediapipe']}")

    def test_sixdrep(self):
        """Perform Sexdrep face detection test."""
        rospy.loginfo("Starting Sexdrep face detection test...")
        # Add your test logic here
        # Example: rospy.loginfo(f"Using configuration: {self.config['sexdrep']}")

    def run_tests(self):
        """Run both tests in sequence."""
        self.test_media_pipe()
        self.test_sixdrep()