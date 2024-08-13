# Pepper Interface Tests

<img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">

Pepper interface tests is a ROS package to test the sensors and actuators of the Pepper robot on a physical and simulator platform. After setting up the development environment using the software installation document as setout  [D3.3 Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf), the package can be installed and run on the Pepper robot. 

The package is designed to test the sensors and actuators of the Pepper robot. The package is divided into two parts: sensor test and actuator test. The sensor test is designed to test the sensors of the Pepper robot, while the actuator test is designed to test the actuators of the Pepper robot. The sensor test is designed to test the following sensors: sonar, laser, microphone, and camera. The actuator test is designed to test the following actuators: Head, arm, hand, leg, and wheels.

## Documentation
Accompnaying this code, there is a deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report is available at [D4.1 Sensor test](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.1.pdf) and [5.1 Actuator test](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.1.pdf)

## Running Tests
To run the test on the physical platform, change the first line of actuatorTestConfiguration.ini file in the config folder
to “platform robot”. On the other hand, to run the test on the simulator platform, change the first line of simulatorTestConfiguration.ini file to “platform simulator”.

## Physical Robot
This command launches actuator test
```sh
roslaunch pepper_interface_tests actuatorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
```

This command launches sensor test 
```sh
roslaunch roslaunch pepper_interface_tests sensorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
```

## Simulator Robot
This command launches pepper interface test for the simulator robot.
```sh
roslaunch pepper_interface_tests interfaceTestLaunchSimulator.launch
```


## Sensor and actuator Test
actuatorTestInput.dat and sensorTestInput.dat are provided that container key-value pair to test the different actuator and sensor found in Pepper robot.

This command test the sensor for the physical robot
```sh
rosrun pepper_interface_tests sensorTest
```

This command test the actuator for the physical robot
```sh
rosrun pepper_interface_tests actuatorTest
```
