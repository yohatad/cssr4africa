### Culturally Sensitive Social Robotics for Africa (CSS4RAfrica)

## Installation of ROS-Noetic

# Setup your source.list

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

# Set up your keys

``` bash
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

# Installation 
``` bash
sudo apt update
```

Desktop-Full install
``` bash
sudo apt install ros-noetic-desktop-full
```
Source the script every time 
``` bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

To install this tool and other dependencies for building ROS packages
``` bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

Initialize rosdep
``` bash
sudo apt install python3-rosdep
```

``` bash
sudo rosdep init
rosdep update
```


## Installing NAOqi Driver and ROS Packages 
# Install the NAOqi driver

``` bash
sudo apt-get install ros-.*-naoqi-driver
```

Create ROS workspace
``` bash
mkdir -p $HOME/workspace/pepper_rob_ws/src
```

Move to the workspace directory
``` bash
cd $HOME/workspace/pepper_rob_ws/src
```

Clone NAOqi DCM driver repository
``` bash
git clone https://github.com/cssr4africa/naoqi_dcm_driver.git
```

Clone NAOqi driver repository
``` bash
git clone https://github.com/cssr4africa/naoqi_driver.git
```

Clone Pepper DCM driver repository
``` bash
git clone https://github.com/cssr4africa/pepper_dcm_robot.git
```

Clone Pepper virtual repository
``` bash
git clone https://github.com/ros-naoqi/pepper_virtual.git
```

Clone Pepper robot repository
``` bash
git clone https://github.com/ros-naoqi/pepper_robot.git
```

Clone Pepper moveit config repository
``` bash
git clone https://github.com/ros-naoqi/pepper_moveit_config.git
```

Build the repository
``` bash
cd .. && catkin_make
```

Source the setup.bash file
``` bash
source devel/setup.bash
```

Add the setup to your .bashrc file so that you don't have to source it every time
``` bash
echo "source $HOME/workspace/pepper_rob_ws/devel/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc
```

# Install additional packages 
``` bash
sudo apt-get install ros-noetic-joint-trajectory-controller
```

``` bash
sudo apt-get install ros-noetic-ros-controllers
```

``` bash
sudo apt-get install ros-noetic-pepper-meshes
```

When the configuring window opens up, you may agree to the license
terms using the right/left arrow key select <ok> hit enter, and then
select <Yes> to accept the terms and press enter.

Install rosdep
``` bash
sudo pip install -U rosdep
```

``` bash
sudo rosdep init
```

``` bash
rosdep update
```

``` bash
sudo rosdep init
```
