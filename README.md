the sim.launch.py launches everything!


# to launch the simulation:


in one terminal:
************ change later to the right one!!!!!! ************
# sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console


in another terminal:
# ros2 launch drone sim.launch.py model:=copterPIX




↑↑↑↑↑↑
for this to work:
you must first go to work space folder where src is the sub directory which contains the drone folder.
there enter the command:

# colcon build --symlink-install

then make sure in ~/.bashrc you have(in this example, the work space is called drone_ws located in ~/):
# source ~/drone_ws/install/setup.bash

