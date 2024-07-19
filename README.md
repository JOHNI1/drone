# Setup dependencies:

<h3>

${\color{orange}OS}$

</h3>

</div>
<div style="margin-left: 40px;">
Download Ubuntu 22.04
</div>

<h3>

${\color{orange}Setup \space Ros2 \space humble}$

</h3>

<div style="margin-left: 40px;">

https://ardupilot.org/dev/docs/building-setup-linux.html

#### ros2 install:
<div style="margin-left: 40px;">

    sudo apt update

    sudo apt upgrade

    sudo apt update && sudo apt install locales

    sudo locale-gen en_US en_US.UTF-8
    
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

    export LANG=en_US.UTF-8

    sudo apt install software-properties-common
    
    sudo add-apt-repository universe

    sudo apt update && sudo apt install curl -y

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update && sudo apt upgrade -y

    sudo apt install ros-humble-desktop

    sudo apt install ros-dev-tools
    
    sudo apt install ros-humble-gazebo-ros-pkgs

    sudo apt install ros-humble-xacro

    sudo apt install ros-humble-visualization-tools

    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

    sudo apt install ros-humble-twist-mux

    source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

    pip install --user -U empy==3.3.4 pyros-genmsg setuptools
</div>

#### Go to .bashrc:
<div style="margin-left: 40px;">

    gedit ~/.bashrc

or

    nano ~/.bashrc
</div>

#### In ~/.bashrc end add this lines:
<div style="margin-left: 40px;">

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=1
    export ROS_DISTRO="humble"
</div>

#### Test:
<div style="margin-left: 40px;">

    ros2 run turtlesim turtlesim_node
</div>

</div>

<h3>

${\color{orange}Setup \space Gazebo \space classic}$

</h3>
<div style="margin-left: 40px;">

https://classic.gazebosim.org/tutorials?tut=install_ubuntu

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    
    cat /etc/apt/sources.list.d/gazebo-stable.list
    
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    
    sudo apt-get update
    
    sudo apt-get install gazebo
    
    sudo apt-get install libgazebo-dev

#### To verify installation enter the command:

    gazebo --verbose

</div>
<h3>

${\color{orange}Setup \space ardupilot(for \space sitl)}$

</h3>

<div style="margin-left: 40px;">

https://ardupilot.org/dev/docs/building-setup-linux.html

#### ardupilot install:
<div style="margin-left: 40px;">

    sudo apt-get update

    sudo apt-get install git

    sudo apt-get install gitk git-gui

    cd ~

    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

    cd ardupilot

    Tools/environment_install/install-prereqs-ubuntu.sh -y
    
    . ~/.profile

</div>

*If there have been updates to some git submodules you may need to do a full clean build. To do that use:*
<div style="margin-left: 40px;">

    ~/ardupilot/waf clean
</div>
</div>


<h3>

${\color{orange}Setup \space ardupilot \space plugin(for \space gazebo)}$

</h3>

<div style="margin-left: 40px;">

https://ardupilot.org/dev/docs/sitl-with-gazebo-legacy.html

#### ardupilot plugin install:
<div style="margin-left: 40px;">

    cd ~

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

    sudo apt update

    git clone https://github.com/khancyr/ardupilot_gazebo

    cd ardupilot_gazebo

    mkdir build

    cd build

    cmake ..

    make -j4

    sudo make install

</div>

#### Go to .bashrc:
<div style="margin-left: 40px;">
    
    gedit ~/.bashrc

or

    nano ~/.bashrc
</div>

#### In ~/.bashrc end add this lines:
<div style="margin-left: 40px;">

    source /usr/share/gazebo/setup.sh
    export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
    export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
    export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
    export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
</div>
<div style="color: red;">

**now RESTART UBUNTU!**

</div>


#### Test:
<div style="margin-left: 40px;">
<div style="color: lightblue;">
<h4>On 1nd Terminal(Launch Gazebo with demo 3DR Iris model)</h4>
</div>
<div style="margin-left: 40px;">

    gazebo --verbose worlds/iris_arducopter_runway.world
</div>
<div style="color: lightblue;">
<h4>On 2st Terminal(Launch ArduCopter SITL)</h4>
</div>
<div style="margin-left: 40px;">

    sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console

Wait untill you see:
<div style="color: green;">AP: EKF3 IMU1 is using GPS</div>
<div style="color: green;">AP: EKF3 IMU0 is using GPS</div>
<div style="color: white;">Flight battery 100 percent</div>

<br>

Then enter:

    mode guided
    arm throttle
    takeoff 30
    guided 30 30 30
</div>
</div>
</div>




# Setup The Drone! <!--# Setup The Drone!# Setup The Drone!# Setup The Drone!# Setup The Drone!# Setup The Drone!# Setup The Drone!# Setup The Drone!-->


<h3>

${\color{orange}Getting \space the \space drone \space package}$

</h3>

<div style="margin-left: 40px;">

https://github.com/JOHNI1/drone

#### Create the work space and import the drone package from github and build:
<div style="margin-left: 40px;">

    cd ~
    mkdir -p drone_ws/src
    cd drone_ws/src
    git clone https://github.com/JOHNI1/drone
    cd ~/drone_ws
    colcon build --symlink-install

</div>
<div style="color: red;">MAKE SURE TO ALWAYS redo the colcon build --symlink-install if you make changes to the src/drone folder like adding file</div>

#### Go to .bashrc:
<div style="margin-left: 40px;">

    gedit ~/.bashrc

or

    nano ~/.bashrc
    
</div>

#### In ~/.bashrc end add this line:
<div style="margin-left: 40px;">

    source ~/drone_ws/install/setup.bash
</div>
<div style="color: red;">MAKE SURE TO ALWAYS source or open new terminal that redoes source, after doing colcon build --symlink-install</div>

</div>

<h3>

${\color{orange}Sitl \space configuration}$

</h3>


<div style="margin-left: 40px;">

#### Make a new sitl parm file for hexa(lets call it gazebo-hexa.parm):
<div style="margin-left: 40px;">

    gedit ~/ardupilot/Tools/autotest/default_params/gazebo-hexa.parm

or

    nano ~/ardupilot/Tools/autotest/default_params/gazebo-hexa.parm

copy paste this into gazebo-hexa.parm:

    # Hexa is X frame
    FRAME_CLASS	2
    FRAME_TYPE	1

    # IRLOCK FEATURE
    RC8_OPTION 39
    PLND_ENABLED    1
    PLND_TYPE       3

    # SONAR FOR IRLOCK
    SIM_SONAR_SCALE 10
    RNGFND1_TYPE 1
    RNGFND1_SCALING 10
    RNGFND1_PIN 0
    RNGFND1_MAX_CM 5000
    
    # Servo configuration
    SERVO7_FUNCTION 1  # Setting SERVO5 to RC passthrough <-- for the gun servo trigger!
    SERVO7_MIN 1000
    SERVO7_MAX 2000
    SERVO7_TRIM 1500
    SERVO7_REVERSED 0
    #    ↑ use SERVO 7 for channel 6 cuz ardupilot channels count from 1 but ardupilot plugin for gazebo counts from 0!!!!



<div style="color: pink;">↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑THE FRAME_CLASS 2 IS FOR HEXA and FRAME_TYPE 1 IS FOR X FRAME!↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑</div><br>

(If you want to define different frame and class type for the drone, follow these links and in general change the parameters.)
- https://ardupilot.org/copter/docs/parameters.html#frame-class   <br>
- https://ardupilot.org/copter/docs/parameters.html#frame-type    <br>

To save press:
<div style="color: white;">

control+o   <br>
control+m   <br>
control+x
</div>
</div>









#### Add the new gazebo-hexa.parm file to vehicleinfo:
<div style="margin-left: 40px;">

    gedit ~/ardupilot/Tools/autotest/pysim/vehicleinfo.py

or

    nano ~/ardupilot/Tools/autotest/pysim/vehicleinfo.py

In the vehicleinfo.py search for:
<div style="color: white;">
<div style="margin-left: 40px;">
gazebo-iris
</div></div>
<br>

Find this:
<div style="color: white;">
<div style="margin-left: 40px;">
<div style="margin-left: 0px;">"gazebo-iris": {</div>
<div style="margin-left: 40px;">    "waf_target": "bin/arducopter",</div>
<div style="margin-left: 40px;">    "default_params_filename": ["default_params/copter.parm",</div>
<div style="margin-left: 225px;">                                "default_params/gazebo-iris.parm"],</div>
<div style="margin-left: 40px;">    "external": True,</div>
<div style="margin-left: px;">},</div>
</div></div>
<br>

Under it add this:

    "gazebo-hexa": {
        "waf_target": "bin/arducopter",
        "default_params_filename": ["default_params/copter.parm",
                                    "default_params/gazebo-hexa.parm"],
        "external": True,
    },

To save press:
<div style="color: white;">

control+o   <br>
control+m   <br>
control+x
</div>

<div style="color: pink;">
basically the sitl parameters are defined by conbining the two files:

*default_params/copter.parm* and *default_params/gazebo-hexa.parm* <- which is the file you created
</div>
</div>
</div>
</div>




<h3>

${\color{orange}Launching \space the \space simulation}$

</h3>

The **model:=** argument can allow you to choose the name of the folder of the model you want to launch, as it uses the robot.urdf.xaco file located inside that folder
to create the urdf of the robot. This project already comes with copterPIX, iris, simple_box. <br>
When model is not specified, **ros2 launch drone sim.launch.py** command is set to launch copterPIX drone as the default drone. <br>
You can also add the **world:=** argument to specify the gazebo world in which you want your model to spawn. For this, you need to enter the path to the world file.  <br> 
(do not start the path with ~/ because it won't work!)

### For copterPIX:
<div style="margin-left: 40px;">

#### In one terminal:
<div style="margin-left: 40px;">

    cd ~
    ros2 launch drone sim.launch.py model:=copterPIX world:=./drone_ws/install/drone/share/drone/worlds/default.world 
</div>


#### In another terminal:
<div style="margin-left: 40px;">

    sim_vehicle.py -v ArduCopter -f gazebo-hexa --console --map

Wait untill you see:
<div style="color: green;">AP: EKF3 IMU1 is using GPS</div>
<div style="color: green;">AP: EKF3 IMU0 is using GPS</div>
<div style="color: white;">Flight battery 100 percent</div>

<br>

commands for the sitl:

    mode guided
    arm throttle
    takeoff 30
    guided 30 30 30

command to control the servo for firing!

    servo set 7 1500

<div style="color: pink;">

*7 is the channel and 1500 is the pwm*

</div>

</div>
</div>


### For iris:
<div style="margin-left: 40px;">

#### In one terminal:
<div style="margin-left: 40px;">
    cd ~
    ros2 launch drone sim.launch.py model:=iris world:=./drone_ws/install/drone/share/drone/worlds/default.world 
</div>


#### In another terminal:
<div style="margin-left: 40px;">

    sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

Wait untill you see:
<div style="color: green;">AP: EKF3 IMU1 is using GPS</div>
<div style="color: green;">AP: EKF3 IMU0 is using GPS</div>
<div style="color: white;">Flight battery 100 percent</div>

<br>

commands for the sitl:

    mode guided
    arm throttle
    takeoff 30
    guided 30 30 30


</div>
</div>



<h3>

${\color{orange}Making \space bash \space sript \space that \space can \space launch \space everything \space (extra)}$

</h3>


<div style="margin-left: 40px;">

#### Make a file called run_sim.sh:
<div style="margin-left: 40px;">

    mkdir ~/drone_ws/sim_launch
    gedit ~/drone_ws/sim_launch/run_sim.sh

or

    nano ~/drone_ws/sim_launch/run_sim.sh

    
</div>

#### And copy paste this into run_sim.sh:
<div style="margin-left: 40px;">

    #!/bin/bash

    # Default values for parameters
    MODEL="iris"
    FRAME="gazebo-hexa"
    WORLD="./drone_ws/install/drone/share/drone/worlds/default.world"

    # Parse command-line arguments
    while getopts m:p:w: flag
    do
        case "${flag}" in
            m) MODEL=${OPTARG};;
            p) FRAME=${OPTARG};;
            w) WORLD=${OPTARG};;
        esac
    done

    # Construct the ROS simulation command
    ROS_COMMAND="ros2 launch drone sim.launch.py model:=$MODEL"
    if [ -n "$WORLD" ]; then
        ROS_COMMAND="$ROS_COMMAND world:=$WORLD"
    fi

    # Open the first terminal for ROS simulation
    gnome-terminal --tab --title="ROS Simulation" -e "bash -c '$ROS_COMMAND; exec bash'" &

    # Open the second terminal for ArduPilot vehicle simulation
    gnome-terminal --tab --title="ArduPilot Vehicle Sim" -e "bash -c 'sim_vehicle.py -v ArduCopter -f $FRAME --console --map; exec bash'" &

    # Wait for both terminals to close
    wait


</div>

#### To run it, enter:
<div style="margin-left: 40px;">

    cd ~
    drone_ws/sim_launch./run_sim.sh -m coterPIX -p gazebo-hexa -w ./drone_ws/install/drone/share/drone/worlds/default.world 

</div>

</div>


<h3>

${\color{orange}Making \space your \space own \space model! \space \color{red} (advanced)}$

</h3>


<div style="margin-left: 40px;">

#### package structure:

This project is a ros project that integrates gazebo and ardupilot. It follows the standard ros2 package structure. The models(drones) are defined using **xacro** which is a xml language that allows to create macros(basically function, parameters, even logics like if statement!).
the xacros folder contains the foundational xacro files that really simplify the creation of the drone.
- The link_macros.xacro defines base macros for the link generation.
- The arm_maker.xacro defines the arm, prop, liftdrag plugin
- The drone_maker.xacro defines the drone. imu, ardupilot plugin and defines all the arms in loop.



#### making new model directory:
<div style="margin-left: 40px;">

    mkdir ~/drone_ws/sim_lanch
    gedit ~/drone_ws/sim_lanch/run_sim.sh

or

    nano ~/drone_ws/sim_lanch/run_sim.sh

    
</div>


</div>
