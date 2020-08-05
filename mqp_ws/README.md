# MQP Scrap Burning

## Compiling
To compile the project, run 'catkin_make', 'source devel/setup.bash', then execute the main launch file: 'roslaunch scrap_burning main.launch'.

## Packages
* franka_description: URDF/Mesh files for the franka robot and camera setup
* panda_moveit_config: Moveit configuration file for panda
* panda_moveit_controller: Controllers for the panda robot, moveit uses these
* scrap_burning: Main package containing all logic, right now only contains launch file for setting up environment
* scrap_descs: Package containing description and meshes for all scrap pieces, for now only a cylinder

## Other
The rgbd camera is setup on the robot, although it is invisible (the camera has an empty <visual> tag attached), that can be fixed if a visual representation of the camera is needed by modifying the depth.xacro file in franka_description/robots.

Moveit is also setup and running, and its API should be usable from C++/Python
