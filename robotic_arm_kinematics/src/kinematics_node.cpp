#include <stdio.h>
#include <math.h>
#include <string.h>
#include <vector>

#include "jsoncpp/json/json.h"
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


using namespace std::chrono_literals;
using namespace std;

class KinematicsNode : public rclcpp::Node{
	public:
		KinematicsNode(double _link_1, double _link_2, double _link_3, double _link_4, double _link_5, double x, double y, double z) : Node("kinematics_node")
		{
			link_1 = _link_1;
			link_2 = _link_2;
			link_3 = _link_3;
			link_4 = _link_4;
			link_5 = _link_5;
					
			pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(publish_topic, 10);
			timer_ = create_wall_timer(1s, std::bind(&KinematicsNode::timerCallback, this));
			RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
			get_angles(x, y, z);

		}

		void timerCallback()
		{
			auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
			trajectory_msg.joint_names = joints;
			auto point = trajectory_msgs::msg::JointTrajectoryPoint();
			point.positions = goal_positions;
			point.time_from_start = builtin_interfaces::msg::Duration();
			trajectory_msg.points.push_back(point);
			pub_->publish(trajectory_msg);
			cout<<"\n Trajectory Sent! \n"<<endl;
			// auto message = std_msgs::msg::String();
			// message.data = "Hello " + std::to_string(counter_++);
			// pub_->publish(message);
		}

		// Variable to hold joint angles solved
		vector<double> goal_positions = {0.0, 0.0, 0.0, 0.0};

	private:
		// publish topic name
        string publish_topic = "/robot_joints_joint_trajectory_controller/joint_trajectory";
		// Robotic arm joint names
		vector<string> joints = {		
				"base_waist_joint",
                "waist_link1_joint",
                "link1_link2_joint",
                "link2_gripper_base_joint",
				};


		// Robotic arm links
		double link_1, link_2, link_3, link_4, link_5;
		
		// Robotic arm joint angles
		double base_waist_joint, waist_link1_joint, link1_link2_joint, link2_gripper_base_joint, right_gripper_joint;

		// Pitch angle to determine orientation of the end effector
		double pitch = 1.57;

		// Spatial coordinates
		double x, y, z;

		// Method to calculate the forward kinematics
		int forward_kinematics(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double* x, double* y, double* z);
		
		// Method to calculate the inverse kinematics of
		int inverse_kinematics(double x, double y, double z, double pitch, double yaw, double* theta_1, double* theta_2, double* theta_3, double* theta_4, double* theta_5);

		// Method to get the angles from the inverse kinematics
		void get_angles(double x, double y, double z);

		rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
		rclcpp::TimerBase::SharedPtr timer_;
		unsigned int counter_;

};


int KinematicsNode::forward_kinematics(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double* x, double* y, double* z)
{
	//the forward is offset by by 90 degree
	*x = cos(theta_1) * ((link_2 * cos(theta_2)) + (link_3 * cos(theta_2 + theta_3)) + (link_5 * sin(theta_2 + theta_3 + theta_4)));
	*y = sin(theta_1) * ((link_2 * sin(theta_2)) + (link_3 * cos(theta_2 + theta_3)) + (link_2 * cos(theta_2 + theta_3 + theta_4)));
	*z = link_1 - (link_2 * sin(theta_2)) - (link_3 * sin(theta_2 + theta_3)) - (link_5 * cos(theta_2 + theta_3 + theta_4));
	return 1;
}

int KinematicsNode::inverse_kinematics(double x, double y, double z, double pitch, double yaw, double* theta_1, double* theta_2, double* theta_3, double* theta_4, double* theta_5)
{

	//fix to avoid devision by zero error
	if (pitch == 0)
	{
		pitch = 0.00000001;
	}

	double _theta_1 = atan(y / x);
	double c_1 = cos(_theta_1);
	double s_1 = sin(_theta_1);

	//intermediate break
	double b_2 = link_1 - (link_5 * cos(pitch)) - z;
	double b_1 = c_1 * x + s_1 * y + link_5 * sin(pitch);

	//calculations for theta 3
	double c_3 = ((b_1 * b_1) + (b_2 * b_2) - (link_2 * link_2) - (link_3 * link_3)) / (2 * link_2 * link_3);
	double _theta_3 = acos(c_3);
	double s_3 = sin(_theta_3);

	//calculations for theta 2
	double c_2 = (((link_2 + link_3 * c_3) * b_1) + ((link_3 * s_3) * b_2)) / ((link_2 * link_2) + (link_3 * link_3) + (2 * link_2 * link_3 * c_3));
	double _theta_2 = -acos(c_2);

	//calculations for theta_4
	double _theta_4 = pitch - _theta_2 - _theta_3;//this is where we handle the pitch from
	printf("Theta 1 = %f\r\n", _theta_1);
	printf("Theta 2 = %f\r\n", _theta_2 + 1.570796);
	printf("Theta 3 = %f\r\n", (-1 * _theta_3)  + 1.570796);
	printf("Theta 4 = %f\r\n", _theta_4);


	//calculations for theta_5
	double R_31 = -(cos(yaw) * sin(_theta_2 + _theta_3 + _theta_4));
	double R_32 = (sin(_theta_2 + _theta_3 + _theta_4) * sin(yaw));

	//printf("R-31 is %f\r\n", R_31);
	//printf("R-32 IS %f\r\n", R_32);

	//double R_31 = 0.233;
	//double R_32 = 0.3536;

	double _theta_5 = atan(((R_32) / (-(R_31))));//this is where we handle the roll from

	//fixing the points
	*theta_1 = _theta_1;
	*theta_2 = _theta_2 + 1.570796;
	*theta_3 = (-1 * _theta_3) + 1.570796;
	*theta_4 = _theta_4;
	*theta_5 = _theta_5;


	return 1;
}


void KinematicsNode::get_angles(double x, double y, double z){
	inverse_kinematics(x, y, z, pitch, 0, &base_waist_joint, &waist_link1_joint, &link1_link2_joint, &link2_gripper_base_joint, &right_gripper_joint);
	forward_kinematics(base_waist_joint, waist_link1_joint, link1_link2_joint, link2_gripper_base_joint, right_gripper_joint, &x, &y, &z);
	goal_positions[0] = base_waist_joint;
	goal_positions[1] = waist_link1_joint;
	goal_positions[2] = link1_link2_joint;
	goal_positions[3] = link2_gripper_base_joint;
	
}


// Main function
int main(int argc, char* argv[]) {

	// Create a vector to store the command-line arguments (excluding the program name)
    std::vector<std::string> arguments;

    for (int i = 1; i < argc; ++i) {
        arguments.push_back(argv[i]);
    }
	
	// Length of the robotic arm links in mm
	double waist = 42.661;
	double link_1 = 120;
	double link_2 = 117.352; 
	double gripper_base = 99.676;
	double link_4 = 0.0;

	// Desired end effector position
	double x, y, z;
	
	// Convert arguments to double and use them
    std::vector<double> numericArguments;
    for (const std::string& arg : arguments) {
        try {
            double numericValue = std::stod(arg);
            numericArguments.push_back(numericValue);
        } catch (const std::invalid_argument& e) {
            // Handle conversion errors here
            std::cerr << "Error: Argument '" << arg << "' is not a valid double." << std::endl;
        }
    }

	x = numericArguments[0];
	y = numericArguments[1];
	z = numericArguments[2];
	
	printf("x is %f\r\n", x);
	printf("y is %f\r\n", y);
	printf("z is %f\r\n", z);

	std::cout << " Desired Position " << endl;
	std::for_each(numericArguments.begin(), numericArguments.end(), [](int num) {
        std::cout << " " << num;
    });

	rclcpp::init(argc, argv);
	auto ik_node = std::make_shared<KinematicsNode>(waist, link_1, link_2, link_4, gripper_base, x, y, z);

	// std::cout<<"----------------------------------------------------------------"<<endl;
	// std::cout << " Inverse Kinematics Solution: " << endl;
	
	// std::for_each(ik_node->goal_positions.begin(), ik_node->goal_positions.end(), [](int num) {
    //     std::cout << " " << num;
    // });
	// std::cout<<"\n----------------------------------------------------------------"<<endl;

	rclcpp::spin(ik_node);
	rclcpp::shutdown();

	return 0;
}
