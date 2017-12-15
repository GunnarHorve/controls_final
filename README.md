Read Me:

1. Install ubuntu 16.04
	http://releases.ubuntu.com/16.04/
2. Install ROS Kinetic Kame
	http://wiki.ros.org/kinetic/Installation
3. Install Baxter SDK
	http://sdk.rethinkrobotics.com/wiki/Workstation_Setup
4. Install Baxter Simulator Fork
	http://sdk.rethinkrobotics.com/wiki/Simulator_Installation
5. Install Python 3.5
	https://www.python.org/downloads/release/python-350/
6. Downlaod custom Python Scripts
	https://github.com/GunnarHorve/controls_final

Note: Before running the baxter simulator make sure your IP in the baxter.sh file is set correctly to your current local IP address. Your local IP can be found by running ifconfic in the terminal.

After creating your ROS workspace and importing the appropriate src files (see http://wiki.ros.org/catkin/Tutorials) make sure your roscore is currently running before executing the following steps

TERMINAL 1
$ cd ~/ros_ws
$ ./baxter.sh sim
$ roslaunch baxter_gazebo baxter_world.launch

TERMINAL 2
$ cd ~/ros_ws
$ ./baxter.sh sim
$ rosrun controls_final pd_traj_control.py

once pickleing is complete and robot control stops run following command to attain graphs

TERMINAL 3
$ cd ~/ros_ws
$ ./baxter.sh sim
$ rosrun controls_final make_plots.py
