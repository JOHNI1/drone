########################################################################################################

## setup:

you must first go to work space folder where src is the sub directory which contains the drone folder.
there enter the command:

# colcon build --symlink-install

then make sure in ~/.bashrc you have(in this example, the work space is called drone_ws located in ~/):
# source ~/drone_ws/install/setup.bash



GO TO:
    cd ~/ardupilot/Tools/autotest/default_params

DO:
    nano gazebo-hexa.parm
AND COPY PASTE:

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

TO SAVE PRESS:
    control+o
    control+m
    control+x


OPEN in vscode:
    code ~/ardupilot/Tools/autotest/pysim/vehicleinfo.py
AND SEARCH FOR: 
    gazebo-iris
UNDER:
            "gazebo-iris": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-iris.parm"],
                "external": True,
            },
ADD:
            "gazebo-hexa": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-hexa.parm"],
                "external": True,
            },
SAVE IT!




########################################################################################################




# to launch the simulation:


in one terminal:
if you cloned drone to ~/drone_ws/src, do:
# cd ~/drone_ws
# ros2 launch drone sim.launch.py model:=simple_box world:=./src/drone/config/default.world 



in another terminal:
# sim_vehicle.py -v ArduCopter -f gazebo-hexa --console --map


in the terminal, wait till it prints out all of the following logs:

    online system 1
    Mode STABILIZE
    AP_IRLock_SITL::init()
    AP: ArduCopter V4.6.0-dev (24250233)
    AP: 29180a5b5ad9472691c27cb85cbd850a
    AP: Frame: HEXA/X
    AP: Barometer 1 calibration complete
    AP: Barometer 2 calibration complete
    Init Gyro*******
    AP: ArduPilot Ready
    AP: AHRS: DCM active
    fence present
    AP: RC7: SaveWaypoint LOW
    Flight battery 100 percent
    AP: RC8: PrecisionLoiter MIDDLE
    AP: PreArm: Accels inconsistent
    AP: PreArm: EKF attitude is bad
    AP: PreArm: AHRS: not using configured AHRS type
    AP: EKF3 IMU0 initialised
    AP: EKF3 IMU1 initialised
    AP: AHRS: EKF3 active
    AP: EKF3 IMU0 tilt alignment complete
    AP: EKF3 IMU1 tilt alignment complete
    AP: EKF3 IMU0 MAG0 initial yaw alignment complete
    AP: EKF3 IMU1 MAG0 initial yaw alignment complete
    AP: GPS 1: probing for u-blox at 230400 baud
    AP: GPS 1: detected u-blox
    AP: EKF3 IMU1 origin set
    AP: EKF3 IMU0 origin set
    AP: Field Elevation Set: 584m
    pre-arm good
    AP: EKF3 IMU0 is using GPS
    AP: EKF3 IMU1 is using GPS
    Flight battery 100 percent


commands:
    mode guided
    arm throttle
    takeoff 30
    guided 30 30 30

