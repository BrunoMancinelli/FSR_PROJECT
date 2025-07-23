# FSR_PROJECT

For optimal compatibility and ease of use, it is strongly advised to utilize the Dockerfile contained in this repository(ros2_docker_scripts-fortress folder) when running the different files of this project.

To start, open a termina and build the packages by running:

    $ colcon build

Then, source the environment:

    $ source install/setup.bash

WARNING: To correctly load the URDF model of the arm, in the same terminal, you must set the following environment variable:

    export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/user/ros2_ws/install/ur_description/share:/home/user/ros2_ws/install/rbvogui_description/share

To start the planner, follow these steps in order:

    pip3 install pandas scipy
    pip3 install --upgrade numpy scipy

Next, to run the planner for the **cubic trajectory**, use the following commands:

    chmod +x ~/ros2_ws/src/planner/src/planner_cart.py
    $ cd ~/ros2_ws/src/planner/src
    $ python3 planner_cart.py

To run the planner for the **circular trajectory**, use:

    chmod +x ~/ros2_ws/src/planner/src/planner_circ.py
    $ cd ~/ros2_ws/src/planner/src
    $ python3 planner_circ.py

Then, return to the workspace "home" directory

    $ cd ../../../

So, build the packages again by running:

    $ colcon build

Then, source the environment:

    $ source install/setup.bash

After that, you can launch the controllers.    
The default controller is based on **approximate linearization**.
The default SLAM mode is **mapping**, and the default trajectory is **cubic**.

WARNING: The table containing the data for the MATLAB plots will be saved in the folder from which the launch is executed. The MATLAB plotting script is provided as an attachment.

To launch the system with default parameters:

    $ ros2 launch rbvogui_description gazebo_rbvogui.launch.py
    
WARNING: The simulation will not begin automatically. Please click the "Play" button in the Gazebo window to start the simulation.    

To use SLAM in **localization** mode, add the `slam_mode` argument:

    $ ros2 launch rbvogui_description gazebo_rbvogui.launch.py slam_mode:="localization"

To use the controller based on **I/O linearization**, run:

    $ ros2 launch rbvogui_description gazebo_rbvogui.launch.py command_interface:="IO_linearization"

To use SLAM in **localization** mode with I/O linearization:

    $ ros2 launch rbvogui_description gazebo_rbvogui.launch.py command_interface:="IO_linearization" slam_mode:="localization"

To use the controller based on **full-state feedback linearization**, run:

    $ ros2 launch rbvogui_description gazebo_rbvogui.launch.py command_interface:="full_state_feedback_linearization"

To use SLAM in **localization** mode with full state feedback linearization:

    $ ros2 launch rbvogui_description gazebo_rbvogui.launch.py command_interface:="full_state_feedback_linearization" slam_mode:="localization"

To use the **circular trajectory** with full state feedback linearization:

    $ ros2 launch rbvogui_description gazebo_rbvogui.launch.py command_interface:="full_state_feedback_linearization" traj:="circular"

To combine both (SLAM in localization + circular trajectory):

    $ ros2 launch rbvogui_description gazebo_rbvogui.launch.py command_interface:="full_state_feedback_linearization" traj:="circular" slam_mode:="localization"
     
If you want to test the **look-at-point task**, when you start the simulation, in a second terminal run:

    $ ros2 run aruco_ros single --ros-args \
        -r /image:=/videocamera \
        -r /camera_info:=/camera_info \
        -p marker_id:=115 \
        -p marker_size:=0.1 \
        -p reference_frame:=robot_camera_link_optical \
        -p marker_frame:=aruco_marker_frame \
        -p camera_frame:=robot_camera_link_optical

It is also recommended to open a **third terminal** to visualize the camera image using:

    $ ros2 run rqt_image_view rqt_image_view

To view the real image, select the `/videocamera` topic.  
To view the image with ArUco tag detection, select the `aruco_single/result` topic.
