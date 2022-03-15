# ROS Workspace
This folder is the ROS workspace, which should be used to build the OpenMower ROS software.

It contains a reference to other repositories needed to build the software. This way, we can track the exact version of the packages used in each release to ensure package compatibility.



Currently, the following repositories are included:

- **open_mower_ros**: The main OpenMower ROS implementation
- **slic3r_coverage_planner**: A coverage planner based on the Slic3r software for 3d printers. This is used to plan the mowing path.
- **teb_local_planner**: The local planner which allows the robot to avoid obstacles and follow the global path using kinematic constraints.
- **xesc_ros**: The ROS interface for the xESC motor controllers.



# How to Build

Just build as any other ROS workspace: `catkin make`

