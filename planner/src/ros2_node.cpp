#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "aruco/aruco.h"
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <string>

#include "controller_manager_msgs/srv/switch_controller.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace KDL;

class BicycleTracker : public rclcpp::Node
{
public:
    BicycleTracker()
    : Node("bicycle_tracker_node"), time_(0.0)
    {
    	// Create service client to switch controllers
	    switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
        
        // Declare and retrieve parameters
        this->declare_parameter("wheels_distance", 0.736);
        wheels_distance_ = this->get_parameter("wheels_distance").as_double();

        declare_parameter("command_interface", "approximate_linearization"); 
        get_parameter("command_interface", command_interface_);
        
        declare_parameter("traj", "cartesian"); 
        get_parameter("traj", traj_);
        
        RCLCPP_INFO(this->get_logger(), "cmd_interface = '%s'", command_interface_.c_str());
        
        RCLCPP_INFO(this->get_logger(), "trajectory type = '%s'", traj_.c_str());

        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {  // CHECK ROBOT_DESCRIPTION
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});                
        
        // Creation of the manipulator sub-tree starting from the full tree 
        KDL::Tree robot_fulltree; 
	    if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_fulltree)) {
	        std::cerr << "Failed to parse URDF into KDL tree" << std::endl;
	        return;
	    }
	    KDL::Chain manipulator_chain;
	    if (!robot_fulltree.getChain("robot_chassis_link", "robot_camera_link_optical", manipulator_chain)) {
	        std::cerr << "Failed to extract manipulator chain" << std::endl;
	        return;
	    }
        
	    std::cout << "[DEBUG] JOINT NAMES";
	    for (unsigned int i = 0; i < manipulator_chain.getNrOfSegments(); ++i) {
	        const auto& joint = manipulator_chain.getSegment(i).getJoint();
	        if (joint.getType() != KDL::Joint::None)
		        std::cout << joint.getName() << " ";
	    }
	    std::cout << std::endl;
     
        KDL::Tree manipulator_tree("robot_chassis_link");
	    if (!manipulator_tree.addChain(manipulator_chain, "robot_chassis_link")) {
    		std::cerr << "Failed to add the chain to the manipulator_tree!" << std::endl;
	    }

	    // Create KDLRobot from the manipulator tree
        robot_ = std::make_shared<KDLRobot>(manipulator_tree);
        

        // Set joint limits
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -6.28,-6.28,-3.14,-6.28,-6.28,-6.28;
        q_max.data << 6.28,6.28,3.14,6.28,6.28,6.28;           
        robot_->setJntLimits(q_min,q_max);
                    
        // Initialize joint state arrays            
        joint_positions_.resize(nj); 
        joint_velocities_.resize(nj);  
        joint_pos.resize(nj);
        joint_vel.resize(nj);

	    //Initialization of the desired trajectory
        load_trajectory();

        // Publishers' and subscribers' definition
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/steering_controller/reference_unstamped", 10);
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
               "/joint_states", 10, std::bind(&BicycleTracker::joint_callback, this, _1));  
        
        while(!joint_state_available_){
    		rclcpp::spin_some(this->get_node_base_interface());
	    }

        aruco_mark_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, std::bind(&BicycleTracker::compute_pose_subscriber, this, std::placeholders::_1));                             
        arm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ur5_arm_controller/commands", 10);
        arm_look_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ur5_arm_velocity_controller/commands", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "steering_controller/odometry", 10, std::bind(&BicycleTracker::odom_callback, this, _1));              
        err_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tracking_errors", 10);
        angle_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/angles", 10);  
        position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position", 10); 

        // Update KDLrobot object
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

        joint_pos=joint_positions_; 
        joint_vel=joint_velocities_;               
        
        // Compute EE frame
        init_cart_pose_ = robot_->getEEFrame();
       
        // Compute IK
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);     

        // Initialize controller
        controller_ = std::make_shared<KDLController>(*robot_);
                    
        // Initialize buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        last_aruco_time_ = this->now();
	    aruco_lost_ = false;

	    // Timers' definition 
	    aruco_watchdog_timer_ = this->create_wall_timer(
	    	100ms, std::bind(&BicycleTracker::aruco_watchdog_callback, this));

        arm_wakeup_timer_ = this->create_wall_timer(
            20ms, std::bind(&BicycleTracker::arm_wakeup_step, this));
        
        timer_ = this->create_wall_timer(10ms, std::bind(&BicycleTracker::tracking_step, this));
            
	    
        // Logging setup based on control interface    
        if (command_interface_ == "approximate_linearization")  
            log_file_.open("tracking_log_AL.csv", std::ios::out);
        else if (command_interface_ == "IO_linearization")
            log_file_.open("tracking_log_IO.csv", std::ios::out);
        else if (command_interface_ == "full_state_feedback_linearization")
            log_file_.open("tracking_log_FSFL.csv", std::ios::out);
                
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open tracking_log.csv");
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "Successfully opened tracking_log.csv for writing.");
        }

        log_file_ << "time,x,y,theta,phi,x_ref,y_ref,theta_ref,v,omega,v_ref,omega_ref,ex,ey,etheta,ephi,phir,phil,\n";
            
        arm_log_file_.open("tracking_log_arm.csv", std::ios::out);
        
        if (!arm_log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open arm_tracking_log.csv");
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully opened arm_tracking_log.csv for writing.");
        }	
        
        arm_log_file_ << "time,errore,q1,q2,q3,q4,q5,q6\n";

        RCLCPP_INFO(this->get_logger(), "Tracking node started.");
    }

private:


    void load_trajectory()  //=== loading of the desired trajectory from trajectory.csv ===
    { 

        trajectory_available_ = true;
        std::string package_path = ament_index_cpp::get_package_share_directory("planner");
        if (traj_ == "circular")
        	file_path = package_path + "/trajectory_circ.csv";
        else if (traj_ == "linear")
        	file_path = package_path + "/trajectory_cart_straight.csv";
        else
        	file_path = package_path + "/trajectory_cart.csv";	
        	
	    std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open %s", file_path.c_str());
            return;
        }
        std::string line;
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::vector<double> row;
            while (std::getline(ss, value, ',')) {
                row.push_back(std::stod(value));
            }
            if (row.size() == 13) {
                t_d.push_back(row[0]);
                x_d.push_back(row[1]);
                y_d.push_back(row[2]);
                theta_d.push_back(row[3]);
                v_d.push_back(row[4]);
                omega_d.push_back(row[5]);
                phi_d.push_back(row[6]);
                dot_xd.push_back(row[7]);
                ddot_xd.push_back(row[8]);
                dddot_xd.push_back(row[9]);
                dot_yd.push_back(row[10]);
                ddot_yd.push_back(row[11]);
                dddot_yd.push_back(row[12]);                                
            }
        }
        RCLCPP_INFO(this->get_logger(), "Trajectory loaded from CSV with %ld points.", t_d.size());
      
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) //=== extraction of joints' info and computation of phi ===
    {
        // Mark that joint states have been received at least once
        joint_state_available_ = true;

        // List of the manipulator's joint names (for KDL extraction)
        std::vector<std::string> kdl_joint_names = {
            "arm_shoulder_pan_joint",
            "arm_shoulder_lift_joint",
            "arm_elbow_joint",
            "arm_wrist_1_joint",
            "arm_wrist_2_joint",
            "arm_wrist_3_joint"
        };

        // Ensure the arrays for joint positions and velocities have correct size
        joint_positions_.resize(6);
        joint_velocities_.resize(6);

        // Extract front wheel steering angles from the JointState message 
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const std::string& name = msg->name[i];
            if (name == "robot_front_left_motor_wheel_joint") {
                tmp_phi_left = msg->position[i];
                found_left = true;
            }
            else if (name == "robot_front_right_motor_wheel_joint") {
                tmp_phi_right = msg->position[i];
                found_right = true;
            }
        }

        // Extract arm joint positions and velocities for the KDL robot model
        for (size_t kdl_idx = 0; kdl_idx < kdl_joint_names.size(); ++kdl_idx) {
            joint_positions_(kdl_idx) = 0.0;
            joint_velocities_(kdl_idx) = 0.0;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == kdl_joint_names[kdl_idx]) {
                    joint_positions_(kdl_idx) = msg->position[i];
                    if (msg->velocity.size() > i)
                        joint_velocities_(kdl_idx) = msg->velocity[i];
                    break;
                }
            }
        }

        //Update wheel angles only if both are found and non-zero 
        if (found_left && found_right) {
            if ( tmp_phi_right != 0 && tmp_phi_left != 0) { 
                phi_left = tmp_phi_left;
                phi_right = tmp_phi_right;   
            }
            // If at least one steering angle is close to zero (within ~5°), use simple average to avoid instability 
            if (std::abs(phi_left)<0.087 || std::abs(phi_right)<0.087)
                phi_ = (phi_right+phi_left)/2;
            else
                // Otherwise, use the nonlinear formula for effective steering angle (Ackermann geometry)
                phi_ = std::atan2(2 * std::tan(phi_left) * std::tan(phi_right), std::tan(phi_left) + std::tan(phi_right));
                
        } else {
            RCLCPP_WARN(rclcpp::get_logger("joint_callback"),
                "Wheel joint(s) missing in JointState! left: %d right: %d", found_left, found_right);
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)  //=== estimation of car-like robot pose ===
    {        
        try {
        
            // Check if the transform from 'map' to 'robot_base_footprint' is available
            if (tf_buffer_->canTransform("map", "robot_base_footprint", tf2::TimePointZero, std::chrono::milliseconds(200))) {
            
            // Lookup the latest available transform between map and robot base    
            tf_map_base = tf_buffer_->lookupTransform(
                "map",                      // target frame 
                "robot_base_footprint",     // initial frame 
                tf2::TimePointZero,         // time
                std::chrono::milliseconds(100) // timeout
            );

            // Extract translation (x, y, z)
            x_map = tf_map_base.transform.translation.x;
            y_map = tf_map_base.transform.translation.y;
            z_map = tf_map_base.transform.translation.z;
            
            // Extract rotation quaternion (qx, qy, qz, qw)
            qx = tf_map_base.transform.rotation.x;
            qy = tf_map_base.transform.rotation.y;
            qz = tf_map_base.transform.rotation.z;
            qw = tf_map_base.transform.rotation.w;

            x_ = x_map;
            y_ = y_map;

            // Convert quaternion to yaw angle theta (heading)
            tf2::Quaternion q(qx, qy, qz, qw);
            theta_ = std::atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz));
                                
            // Mark odometry data as available                    
            odom_available_ = true;
            } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Transform map->robot_base_footprint not available!");
            
            }
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "LookupTransform Error: %s", ex.what());
        }            
        return;                   
    }

    std::vector<double> interpolate_joints(double t, double T,
                                           const std::vector<double>& q0,
                                           const std::vector<double>& qf,
                                           const std::string& profile) {  //=== arm trajectory creation ===
       
        std::vector<double> q(q0.size(), 0.0);
        double s = std::clamp(t/T, 0.0, 1.0);
        double alpha;
        // cubic trajectory profile
        if (profile == "cubic") {              
            alpha = 3*pow(s,2) - 2*pow(s,3);
        } else {  
            alpha = s; // linear trajectory profile
        }
        for (size_t i = 0; i < q.size(); ++i)
            q[i] = (1.0 - alpha)*q0[i] + alpha*qf[i];
        return q;
    }

    void arm_wakeup_step() {      //=== arm wake-up ===
        // If the wake-up trajectory has already completed, exit immediately
        if (arm_wakeup_complete_) return;
        arm_wakeup_time_ += 0.02; // 20ms
        
        // Compute the current target joint positions along the trajectory using interpolation
        std::vector<double> cmd = interpolate_joints(
            arm_wakeup_time_, arm_wakeup_duration_, initial_arm_position_, final_arm_position_, trajectory_profile_
        );
        
        // Create and publish the joint position command message
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = cmd;
        arm_pub_->publish(cmd_msg);
        
        if (arm_wakeup_time_ >= arm_wakeup_duration_) {
            arm_wakeup_complete_ = true;
            // Start a timer to periodically hold the final arm position
            hold_arm_timer_ = this->create_wall_timer(
                100ms, std::bind(&BicycleTracker::hold_arm_position, this));
            // Cancel the wake-up trajectory timer      
            arm_wakeup_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Wakeup arm completed!");
        }
    }

    void hold_arm_position() {  // === position holding ===
        // Create a message containing the desired final arm joint positions
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = final_arm_position_;
        // Publish the message to command the arm to remain at the final position
        arm_pub_->publish(cmd_msg);      
    }

	void switch_controllers_async(const std::vector<std::string>& controller_to_activate,const std::vector<std::string>& controller_to_deactivate) { //=== controllers' switch ===
	    // Wait for the switch_controller service to be available (timeout after 1 second)
        if (!switch_client_->wait_for_service(1s)) {
		    RCLCPP_WARN(this->get_logger(), "Service /controller_manager/switch_controller not available!");
		    return;
	    }
	    RCLCPP_INFO(this->get_logger(), "I am trying to switch controllers...");

        // Create a new request for the SwitchController service
	    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
	    
        // Specify which controllers to activate and which to deactivate
        request->activate_controllers = controller_to_activate;
	    request->deactivate_controllers = controller_to_deactivate;
	    
        // Set the strictness level: STRICT requires all operations to succeed, otherwise the request fails
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
	    // Set a timeout for the switching operation (in seconds)
        request->timeout.sec = 2;
	    
        // Send the request asynchronously, specifying the callback for the response
        auto future = switch_client_->async_send_request(
		request,
		std::bind(&BicycleTracker::switch_response_callback, this, std::placeholders::_1)
	    );
	}

	void switch_response_callback(rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {  //=== switching configuration ===
	    if (future.get()->ok) {
		    RCLCPP_INFO(this->get_logger(), "Switch succesfully completed!");
	    } else {
		    RCLCPP_ERROR(this->get_logger(), "Error in the switching operation!");
	    }
	}


    void tracking_step()  //=== base control and look-at-point behavior
    {

        // Check availability of odometry, joint states and trajectory before proceeding
        if (!(odom_available_ && joint_state_available_ && trajectory_available_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for data: odom=%d, joint=%d, traj=%d",
                                 odom_available_, joint_state_available_, trajectory_available_);
            return;
        }

        // Compute current index in trajectory vectors proportional to time
        int idx = static_cast<int>((time_ / t_d.back()) * t_d.size());
        idx = std::min(idx, (int)t_d.size() - 1);

        // Extract reference trajectory variables at current index
        x_ref = x_d[idx];
        y_ref = y_d[idx];
        theta_ref = theta_d[idx];
        v_ref = v_d[idx];  
        phi_ref = phi_d[idx];
        omega_ref = omega_d[idx];
        dot_x_ref= dot_xd[idx];
        ddot_x_ref= ddot_xd[idx];
        dddot_x_ref= dddot_xd[idx];
        dot_y_ref= dot_yd[idx];    
        ddot_y_ref= ddot_yd[idx]; 
        dddot_y_ref= dddot_yd[idx];
        
        double angles[2] = {theta_,phi_}; 
        
        // Publish current bicycle wheel angles
        cmd_msg_angle.data = std::vector<double>(std::begin(angles), std::end(angles));
        angle_pub_->publish(cmd_msg_angle);  
            
        double position[2] = {x_,y_};
            
        // Publish the position of the bicycle wheels
        cmd_msg_position.data = std::vector<double>(std::begin(position), std::end(position));
        position_pub_->publish(cmd_msg_position);            
        
        // Compute pose errors between reference and actual states
        ex = x_ref-x_;
        ey = y_ref-y_;
        etheta = theta_ref-theta_;
        ephi = phi_ref-phi_;
            
        double err[4]={ex,ey,etheta,ephi};

        for (long int i = 0; i < 4; ++i) {
            errors_[i] = err[i];
        }

        // Publish pose errors
        cmd_msg_err.data = errors_;
        err_pub_->publish(cmd_msg_err);  
            
        if (log_file_.is_open()) {
            
		    log_file_ << time_ << "," 
          	    << x_ << "," << y_ << "," << theta_ << "," << phi_ << ","  
          	    << x_ref << "," << y_ref << "," << theta_ref << ","
          	    << v << "," << omega << "," << v_ref << "," << omega_ref << ","
          	    << ex << "," << ey << "," << etheta << "," << ephi << ","
          	    << phi_right << "," << phi_left << "\n";

        }

        // Stop the base motion when the trajectory is completed       
        if (time_ >= t_d.back()) {
            RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
            geometry_msgs::msg::Twist stop;
            stop.linear.x = 0.0;
            stop.linear.y = 0.0;
            stop.linear.z = 0.0;
            stop.angular.x = 0.0;
            stop.angular.y = 0.0;
            stop.angular.z = 0.0;
            cmd_pub_->publish(stop);            
            if (log_file_.is_open()) {            		
    		    log_file_.close();
    		    std::cout << "[INFO] CSV log file closed.\n";
	        }
            if (arm_log_file_.is_open()) {
    	        arm_log_file_.close();
    	        std::cout << "[INFO] arm CSV log file closed.\n";
	        }
        }
        else{
                                       
            // variables' change
            x1_ref = x_ref;
            x2_ref = tan(phi_ref)/(wheels_distance_*std::pow(cos(theta_ref),3));
            x3_ref = tan(theta_ref);
            x4_ref = y_ref;
            u1_ref = v_ref * (std::cos(theta_ref));
            u2_ref = (omega_ref+3 * std::sin(theta_ref) * std::pow(std::sin(phi_ref), 2) * u1_ref/(wheels_distance_* std::pow(std::cos(theta_ref), 2)))/(wheels_distance_ * std::pow(std::cos(theta_ref), 3) * std::pow(std::cos(phi_ref), 2));
            
            if(command_interface_ == "approximate_linearization"){
            	// Compute current state variables
                x1 = x_;
                x2 = (tan(phi_)/(wheels_distance_*std::pow(cos(theta_),3)));
                x3 = tan(theta_);
                x4 = y_;	                                
            
                // Compute state errors
                e_1 = x1_ref - x1;
                e_2 = x2_ref - x2;
                e_3 = x3_ref - x3;
                e_4 = x4_ref - x4;
            
                // Control gains
                k1 = 0.001; 
                k2 = 0.2722;

                // Control inputs correction terms
                u1_tilde = -k1*e_1;
                u2_tilde = -3*k2*std::abs(u1_ref)*e_2-3*(std::pow(k2,2))*u1_ref*e_3-(std::pow(k2,3))*std::abs(u1_ref)*e_4;
                
                // Total control inputs 
                u1 = u1_ref+u1_tilde;
                u2 = u2_ref+u2_tilde;

                // Compute velocity commands from control inputs
                v = u1 / std::cos(theta_);
                omega = -3 * std::sin(theta_) * std::pow(std::sin(phi_), 2) * u1/ (wheels_distance_ * std::pow(std::cos(theta_), 2))
                            + wheels_distance_ * std::pow(std::cos(theta_), 3) * std::pow(std::cos(phi_), 2) * u2;  
                
                //v and omega saturation
                v = std::clamp(v, -v_max, v_max);
                omega = std::clamp(omega, -omega_max, omega_max);                            
            }

            else if (command_interface_ == "IO_linearization"){
            
                // Gains and parameter delta for input-output linearization
                delta = 20;
                k1 = 0.2; 
                k2 = 0.15;
                
                // z_ref
                z_ref(0) = x_ref + wheels_distance_ * std::cos(theta_ref) + delta * std::cos(theta_ref + phi_ref);
                z_ref(1) = y_ref + wheels_distance_ * std::sin(theta_ref) + delta * std::sin(theta_ref + phi_ref);

                // dot_z_ref 
                dot_z_ref(0) = (std::cos(theta_ref) - std::tan(phi_ref) * (std::sin(theta_ref) + delta * std::sin(theta_ref + phi_ref) / wheels_distance_)) * v_ref
                        - delta * std::sin(theta_ref + phi_ref) * omega_ref;

                dot_z_ref(1) = (std::sin(theta_ref) + std::tan(phi_ref) * (std::cos(theta_ref) + delta * std::cos(theta_ref + phi_ref) / wheels_distance_)) * v_ref
                        + delta * std::cos(theta_ref + phi_ref) * omega_ref;

                // z (real) 
                z(0) = x_ + wheels_distance_ * std::cos(theta_) + delta * std::cos(theta_ + phi_);
                z(1) = y_ + wheels_distance_ * std::sin(theta_) + delta * std::sin(theta_ + phi_);                        
                                            
                        
                // r
                r(0) = dot_z_ref(0) + k1 * (z_ref(0) - z(0));
                r(1) = dot_z_ref(1) + k2 * (z_ref(1) - z(1));

                // Matrix A(θ, φ) 
                A(0, 0) = std::cos(theta_) - std::tan(phi_) * (std::sin(theta_) + delta * std::sin(theta_ + phi_) / wheels_distance_);
                A(0, 1) = -delta * std::sin(theta_ + phi_);
                A(1, 0) = std::sin(theta_) + std::tan(phi_) * (std::cos(theta_) + delta * std::cos(theta_ + phi_) / wheels_distance_);
                A(1, 1) = delta * std::cos(theta_ + phi_);
                
                // Compute velocity commands                                                            
                v_cmd = A.inverse() * r;
                v = v_cmd(0);
                omega = v_cmd(1);        
            } 
            
            else if (command_interface_ == "full_state_feedback_linearization"){        
                
                // Control gain
                ka1 = 0.5;
                kv1 = 2; 
                kp1 = 0.65;  
                ka2 = 0.5; 
                kv2 = 2; 
                kp2 = 0.65;
                                      	
                // Desired values from trajectory
                z1d = x_ref;
                z2d = y_ref;
                
                dot_z1d = dot_x_ref;
                ddot_z1d = ddot_x_ref;
                dddot_z1d = dddot_x_ref;
                dot_z2d = dot_y_ref;
                ddot_z2d = ddot_y_ref;
                dddot_z2d = dddot_y_ref;
            
		
                if (flag == false){
                    
                    // Initialization on the first run
                    flag = true;     	      	        	        	        
                    x1 = x_ref;
                    x2 = (dot_z1d*ddot_z2d-ddot_z1d*dot_z2d)/std::pow(dot_z1d,3);
                    x3 = dot_z2d/dot_z1d; 
                    x4 = y_ref;
                    
                    xi1 = dot_z1d;
                    xi2 = ddot_z1d;
	            } 
                else{
                
                    // Update states based on current robot pose and orientation	               
	            x1 = x_;
                    x2 = (tan(phi_)/(wheels_distance_*std::pow(cos(theta_),3)));
                    x3 = tan(theta_);
                    x4 = y_;	                            
	            }
	            
                // Change of variables to internal coordinates z1, z2 and their derivatives
                z1 = x1;
                z2 = x4;
                dot_z1 = xi1;
                dot_z2 = x3*xi1;
                ddot_z1 = xi2;
                ddot_z2 = x2*std::pow(xi1,2) + x3*xi2;
                
                // r COMPUTATION
                r1 = dddot_z1d + ka1*(ddot_z1d-ddot_z1) + kv1*(dot_z1d-dot_z1) + kp1*(z1d-z1);
                r2 = dddot_z2d + ka2*(ddot_z2d-ddot_z2) + kv2*(dot_z2d-dot_z2) + kp2*(z2d-z2);
                                
                
                // VIRTUAL INPUTS COMPUTATION
                u1 = xi1;
                u2 = (r2-x3*r1-3*x2*xi1*xi2)/std::pow(xi1,2);
                
                // REAL INPUTS COMPUTATION        
                v = u1 / std::cos(theta_);
                omega = -3 * std::sin(theta_) * std::pow(std::sin(phi_), 2) * u1/ ((wheels_distance_) * std::pow(std::cos(theta_), 2))
                            + wheels_distance_ * std::pow(std::cos(theta_), 3) * std::pow(std::cos(phi_), 2) * u2;
                                                     

                 // Check if theta is close to ±π/2 or ±3π/2 to avoid singularities and activate saturation
		        if (std::abs(std::abs(theta_) - theta_limit) < threshold) {
		            std::cout << "Warning: theta close to ±π/2, saturation activated!" << std::endl;

		            v = std::clamp(v, -v_max, v_max);
		            omega = std::clamp(omega, -omega_max, omega_max);   
		        }
		        else if (std::abs(std::abs(theta_) - theta_limit) < threshold) {
		            std::cout << "Warning: theta close to ±3π/2, saturation activated!" << std::endl;

		            v = std::clamp(v, -v_max, v_max);
		            omega = std::clamp(omega, -omega_max, omega_max);	
		        }	
		                                        
                // UPDATE XI
                xi1 = xi1 + xi2*0.01;
                xi2 = xi2 + r1*0.01;
        
        
            }

            // Publish base velocity commands
            cmd.linear.x = v;
            cmd.angular.z = omega;
            cmd_pub_->publish(cmd); 
        }
         
        std::cout<<"X_"<<std::endl;
        std::cout<<x_<<std::endl;
        std::cout<<"Y_"<<std::endl;
        std::cout<<y_<<std::endl;   
        std::cout<<"theta_"<<std::endl;
        std::cout<<theta_<<std::endl;     
         
         
        // Lool-At-Point behavior
        if (calc_traj == true){                 
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            
            // Compute direction and angle error between vectors
            Eigen::Vector3d s= toEigen(aruco_cam_frame.p)/ toEigen(aruco_cam_frame.p).norm();
            Eigen::Vector3d sd(0, 0, 1); 
            angle_error = std::acos(s.dot(sd));
            
            // Control matrix L for position and orientation
            Eigen::Matrix3d I3 =  Eigen::Matrix3d::Identity();
            Eigen::MatrixXd L  = Eigen::Matrix<double,3,6>::Zero();
            L.block(0,0,3,3) = (-1/toEigen(aruco_cam_frame.p).norm())*(I3 - s*s.transpose());
            L.block(0,3,3,3) = skew(s);                                                                                      
            Eigen::Matrix3d Rc = toEigen(robot_->getEEFrame().M);
            Eigen::MatrixXd R = Eigen::Matrix<double,6,6>::Zero();
            R.block(0,0,3,3) = Rc;
            R.block(3,3,3,3) = Rc;
            L=L*R;
            
            // Pseudoinverse and nullspace projection matrix                                                                                                          
            Eigen::MatrixXd I6 =  Eigen::MatrixXd::Identity(6,6);
            Eigen::MatrixXd LJ = L*robot_->getEEJacobian().data;
            Eigen::MatrixXd LJpinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::MatrixXd N = (I6 - (LJpinv*LJ));
            
            // Compute joint velocities
            double k = 2;
            joint_velocities_.data = k*LJpinv*sd+ N*(joint_pos.data - joint_positions_.data);
            joint_positions_.data = joint_positions_.data + joint_velocities_.data*0.01;
            
            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
       
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_(i);
            }

             // Publish joint velocity commands
            std_msgs::msg::Float64MultiArray cmd_msg_look;
            cmd_msg_look.data = desired_commands_;
            arm_look_pub_->publish(cmd_msg_look);   
            
            if (arm_log_file_.is_open()) {
            
		        arm_log_file_ << time_ << "," 
          	    << angle_error << "," << desired_commands_[1] << "," << desired_commands_[2] << "," << desired_commands_[3] << ","  
          	    << desired_commands_[4] << "," << desired_commands_[5] << "," << desired_commands_[6] <<"\n";

            }                 
        }

        time_ += 0.01; // 10ms timestep
    }

    void compute_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) // === retrieve aruco pose === 
    {
        last_aruco_time_ = this->now();
	    if (aruco_lost_) {
            // If the aruco marker is detected again within one second after being lost, switch from position to velocity mode
		    switch_controllers_async({"ur5_arm_velocity_controller"}, {"ur5_arm_controller"});
		    aruco_lost_ = false; // 
		    calc_traj = true;
	    }
        else if (!calc_traj) {
		    // When the aruco marker is detected for the first time, switch from position to velocity mode 
		    switch_controllers_async({"ur5_arm_velocity_controller"}, {"ur5_arm_controller"});
		    calc_traj = true;
	    }

        aruco_quat_cam_frame.resize(4);

	    // Extract position
        aruco_trasl_cam_frame[0] = pose_msg->pose.position.x ;
        aruco_trasl_cam_frame[1] = pose_msg->pose.position.y ;
        aruco_trasl_cam_frame[2] = pose_msg->pose.position.z ;
                
        // Extract orientation quaternion        
        aruco_quat_cam_frame[0] = pose_msg->pose.orientation.x;
        aruco_quat_cam_frame[1] = pose_msg->pose.orientation.y;
        aruco_quat_cam_frame[2] = pose_msg->pose.orientation.z;
        aruco_quat_cam_frame[3] = pose_msg->pose.orientation.w;
                
            
                
        aruco_cam_frame.p = aruco_trasl_cam_frame;
                            
     
        calc_traj=true;            
            
    }

    void aruco_watchdog_callback() {  // === switch from velocity controller to position controller when the aruco is lost for more than 1s
        // check if the aruco is detected at least once
        if (!calc_traj) return;

        auto elapsed = this->now() - last_aruco_time_;
        if (elapsed.seconds() > 1.0 && !aruco_lost_) {
            // Marker "lost" for more than 1 second!
            RCLCPP_WARN(this->get_logger(), "Aruco marker lost for more than 1s, I switch to position controller!");
            switch_controllers_async({"ur5_arm_controller"}, {"ur5_arm_velocity_controller"});
            aruco_lost_ = true;
            calc_traj = false; // Stop look-at-point logic
            auto pos = toStdVector(joint_positions_.data); // actual arm position
	        initial_arm_position_ = pos;
            arm_wakeup_complete_ = false;
            arm_wakeup_time_ = 0;
            arm_wakeup_timer_ = this->create_wall_timer(
            20ms, std::bind(&BicycleTracker::arm_wakeup_step, this));
        }
   }
    
    // parameters initialization
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    double wheels_distance_;
    double time_;
    double phi_ = 0.0;
    double v = 0;
    double omega = 0;
    double x1 = 0, x2 = 0, x3 = 0, x4 = 0;
    double dot_z1 = 0, ddot_z1 = 0, dot_z2 = 0, ddot_z2 = 0 ;
    double dot_z1d = 0, ddot_z1d = 0, dddot_z1d = 0, dot_z2d = 0, ddot_z2d = 0, dddot_z2d = 0 ;
    double xi1 = 0, xi2 = 0;
    double z1 = 0, z2 = 0, z1d = 0, z2d = 0;
    double r1 = 0, r2 = 0;
    double angle_error = 0;
    double phi_left = 0.0, phi_right = 0.0;
    double x_ref = 0;
    double y_ref = 0;
    double theta_ref = 0;
    double v_ref = 0;  
    double phi_ref = 0;
    double omega_ref = 0;
    double dot_x_ref= 0;
    double ddot_x_ref= 0;
    double dddot_x_ref= 0;
    double dot_y_ref= 0;   
    double ddot_y_ref= 0; 
    double dddot_y_ref= 0;
    double ex = 0;
    double ey = 0;
    double etheta = 0;
    double ephi = 0;
    double tmp_phi_left = 0.0, tmp_phi_right = 0.0;
    double x_map = 0.0;
    double y_map = 0.0;
    double z_map = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;
    double x1_ref = 0.0;
    double x2_ref = 0.0;
    double x3_ref = 0.0;
    double x4_ref = 0.0;
    double u1_ref = 0.0;
    double u2_ref = 0.0;
    double e_1 = 0.0;
    double e_2 = 0.0;
    double e_3 = 0.0;
    double e_4 = 0.0;
    double k1 = 0.0, k2 = 0.0;
    double u1_tilde = 0.0;
    double u2_tilde = 0.0;
    double u1 = 0.0;
    double u2 = 0.0;
    double delta = 0.0;
    double ka1 = 0.0, kv1 = 0.0, kp1 = 0.0; 
    double ka2 = 0.0, kv2 = 0.0, kp2 = 0.0;

    const double v_max = 0.5;     // [m/s]
    const double omega_max = 2.0; // [rad/s] 
    const double theta_limit = PI / 2.0;
    const double threshold = 0.01; 
    const double theta_limit_2 = PI / 2.0 + PI; 
                

    bool calc_traj=false;
    bool aruco_lost_ = false;
    bool found_left = false, found_right = false;
    bool joint_state_available_ = false;
    bool odom_available_ = false;
    bool trajectory_available_ = false;
    bool flag = false;

    std::string command_interface_;
    std::string traj_;
    std::string file_path;

    Eigen::Vector2d z = {0,0};
    Eigen::Vector2d z_ref = {0,0};
    Eigen::Vector2d dot_z_ref = {0,0};
    Eigen::Vector2d r;
    Eigen::Matrix2d A;
    Eigen::Vector2d v_cmd;
    
       
    // ROS 2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  
    std::ofstream log_file_;
    std::ofstream arm_log_file_;
 
    
    // trajectory
    std::vector<double> t_d, x_d, y_d, theta_d, v_d, omega_d, phi_d;
    std::vector<double> dot_xd, ddot_xd, dddot_xd;
    std::vector<double> dot_yd, ddot_yd,dddot_yd;
    double dot_z1d_prev, ddot_z1d_prev, dddot_z1d_prev, dot_z2d_prev, ddot_z2d_prev, dddot_z2d_prev;    
    
    
    // arm
    std::vector<double> initial_arm_position_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   
    std::vector<double> final_arm_position_{0.0, -1.2, 1.2, 0.0, 0.0, 0.0};      
    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   
    std::vector<double> errors_ = {0.0, 0.0, 0.0, 0.0};  
    double arm_wakeup_duration_ = 2.0; // seconds
    double arm_wakeup_time_ = 0.0;
    bool arm_wakeup_complete_ = false;
    std::string trajectory_profile_ = "cubic"; // "cubic" or "trap"
    rclcpp::TimerBase::SharedPtr arm_wakeup_timer_;
    rclcpp::TimerBase::SharedPtr hold_arm_timer_;

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    rclcpp::Time last_aruco_time_;
    rclcpp::TimerBase::SharedPtr aruco_watchdog_timer_;
 
    // publishers and subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_mark_pose_sub_ ;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_look_pub_;	
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr err_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    KDL::Frame init_cart_pose_;
    KDL::Frame cartpos;
    KDL::Frame aruco_cam_frame;

    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_efforts_;
    KDL::JntArray joint_pos;
    KDL::JntArray joint_vel;
   
    std::shared_ptr<KDLRobot> robot_;
    std::shared_ptr<KDLController> controller_;   

    //msg
    geometry_msgs::msg::TransformStamped tf_map_base;
    std_msgs::msg::Float64MultiArray cmd_msg_angle;
    std_msgs::msg::Float64MultiArray cmd_msg_position;
    std_msgs::msg::Float64MultiArray cmd_msg_err;
    geometry_msgs::msg::Twist cmd;


    //ArUco
    KDL::Vector aruco_trasl_cam_frame;
    Eigen::VectorXd aruco_quat_cam_frame;
};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BicycleTracker>());
    rclcpp::shutdown();
    return 0;
}

