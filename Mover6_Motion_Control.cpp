/*
Title: MOVER6 Joint Controller
Date: 10/17/2024
Authors: Ali Mohamed & Elion Selko
This C++ code implements a motion controller for the MOVER6 robot.
It accepts an N x 6 vector of waypoint joint configurations, provided either via a terminal window or
MATLAB.
The code reconstructs the N x 6 waypoint configuration matrix, sequentially calculates the required joint
velocities, and publishes them to the joint jog topic to actuate the robot.
The implementation follows an Object-Oriented Programming (OOP) architecture with distinct `Robot` and
`Joint` classes, each encapsulating their respective attributes and methods.
Various operational flags can be toggled, such as selecting between relative or trapezoidal velocity motion
profiles.
*/


#include <ros/ros.h>
#include "std_msgs/String.h"
#include "control_msgs/JointJog.h"
#include "std_msgs/MultiArrayLayout.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float32MultiArray.h>
#include <boost/bind.hpp>
#include <algorithm>
#include <chrono>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>



using namespace std;

// JOINT CLASS
class Joint {
public:
    string Joint_Name;
    float Joint_C_Value = 0; // Joint current value
    float Joint_Demand = 0;
    float init_state = 0; // Joint initial state
    float Velocity = 0;

	float Sign() const
	{
		return (Joint_Demand - Joint_C_Value) / abs(Joint_Demand - Joint_C_Value);
	}
	
 // Trapezoidal Velocity Profiling
    float Trapezoidal_Velocity(std::chrono::time_point<std::chrono::steady_clock> t_start, float t_max, float t_blend) const 
	{

        float current_time =  (std::chrono::duration<float>(std::chrono::steady_clock::now() - t_start)).count(); // Get current time
        float v_max = 2*abs(Joint_Demand - init_state) / (t_max - t_blend);
        // cout << "Current Time: " << current_time << endl;
         ROS_INFO("Joint: %s, Current_T: %s, V_Max = %s,", Joint_Name.c_str(),to_string(current_time).c_str(), to_string(v_max/2).c_str());
        
		float t_acc = t_blend;  // Time for acceleration/deceleration

		// If current time is within the acceleration phase
		if ((current_time < t_acc)) {
			return Sign() * (v_max / t_acc) * current_time;  // Accelerate
		}
		// If current time is in the constant velocity phase
		else if (current_time < t_max - t_acc) {
			return Sign() * v_max;  // Constant velocity
		}
		// If current time is in the deceleration phase
		else if (current_time < t_max) {
			return Sign() * v_max * (1 - (current_time - (t_max - t_acc)) / t_acc);  // Decelerate
		} 
		// After t_max, stop
		else {
			return 0;
		}
	}
		
};

// ROBOT CLASS 
class Robot {
public:
    const int Num_of_Joints; // Number of Joints
    vector<Joint> Robot_Joints; // Vector of ROBOT JOINTS
	control_msgs::JointJog msg_start;
	std::stringstream ss;
	bool know_states = false;
	bool know_demands = false;
	float max_distance = 0;
	float lambda = 2; // Maximum velocity unit for normalization
	float t_max = 0; // time for furthest joint movement - for RELATIVE motion
	float Maximum_Time_T = 1; // Maximum time - for trapizodial velocity motion
	float T_Blend = Maximum_Time_T/4; // Blending time
	bool Vel_P = true; // true for trapizoidal velocity profile and false for relative velocity
	std::chrono::time_point<std::chrono::steady_clock> Start_Time; // clock
	vector<vector<float>> joint_matrix; // Joints configuration matrix
	

// Robot constructor
    Robot(int num_of_joints) : Num_of_Joints(num_of_joints), Robot_Joints(num_of_joints){
        for (int i = 0; i < Num_of_Joints; ++i) {
            Robot_Joints[i].Joint_Name = "joint" + to_string(i + 1);
        }
		msg_start.duration = 5; // Duration default to function unless otherwise specified.
    }

/* 
The Max_Time() function determines the maximum time needed for the joint that must travel the greatest distance,
based on the parameter lambda (as defined above).
*/
	float Max_Time() {
		max_distance = 0;
		for (const auto& joint : Robot_Joints) {
			float distance = abs(joint.Joint_Demand - joint.Joint_C_Value);
			if (distance > max_distance) { // Compare manually to find the max
				max_distance = distance;
			}
		}
		t_max = max_distance / lambda; 
		ROS_INFO("Max time %s", to_string(t_max).c_str());
		return t_max; // Return the maximum distance
	}

// Starts clock timer	
	 void Start_Timer() {
        Start_Time = std::chrono::steady_clock::now();
    }
// Prints time
    void Print_Start_Time() {
        // Convert Start_Time to duration since epoch and print it as a float
        auto duration_since_epoch = Start_Time.time_since_epoch();
        float seconds = std::chrono::duration<float>(duration_since_epoch).count();
        cout << "Start time (seconds since epoch): " << seconds << " seconds" << endl;
    }

// Method to print the joint demand matrix (joint_matrix)
    void PrintJointMatrix() const {
        ROS_INFO("Joint Demand Matrix:\n");
        for (const auto& row : joint_matrix) {
            for (float value : row) {
                ROS_INFO("%s", to_string(value).c_str());
            }
            cout << endl;
        }
    }


};

// Joints states callback function, updates the robot joint states.
void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg,  Robot* robot) {
	
	int i=0;
	string joint_info = "";

	for (const auto& position : msg->position)
	{
		if(i < robot->Num_of_Joints)
		{
			robot->Robot_Joints[i].Joint_C_Value = position;
			joint_info += std::to_string(robot->Robot_Joints[i].Joint_C_Value);
			joint_info += "\t";
		}
		i++;
	}

	robot->know_states = true; 
	ROS_INFO("Received State %s", joint_info.c_str());
}

/*
The Multi_Array_CB callback function processes the N x 6 joint configuration vector, 
reconstructs the robot joint matrix for sequential processing, 
and acknowledges the received demand values to initiate execution.
*/ 
void Multi_Array_CB(const std_msgs::Float32MultiArray::ConstPtr& msg, Robot* robot) {
	int i = 0;  // Index for traversing the flattened demand array
    int set_count = 0; // Track how many sets of 6 values we've processed
    string joint_info = "";

/*
The following commented for loop can be utilized when MATLAB provides individual joint configurations instead of an N x 6 array.
*/
	/*for (const auto& demand : msg->data) 
	{
		if (i < robot->Num_of_Joints)
		{
			robot->Robot_Joints[i].Joint_Demand = demand;
			joint_info += std::to_string(robot->Robot_Joints[i].Joint_Demand);
			joint_info += "\t";
		}
		i++;
    }
	*/

       // Iterate through the received flattened data
    for (size_t j = 0; j < msg->data.size(); ++j) {
        // Calculate the row (configuration) index and the column (joint) index
        int row = j / robot->Num_of_Joints;  // Each row corresponds to a joint configuration
        int col = j % robot->Num_of_Joints;  // Each column corresponds to a specific joint

        // If we have not yet reached a new row, initialize it
        if (row ==  robot->joint_matrix.size()) { // matrix size is zero originally
            robot->joint_matrix.push_back(vector<float>(robot->Num_of_Joints, 0.0f));  // Initialize new row with 6 joints
        }

        // Store the demand in the appropriate joint of the current configuration
         robot->joint_matrix[row][col] = msg->data[j];

		// Update joint information for logging
        joint_info += std::to_string(msg->data[j]) + "\t";

        // If we've processed a full set (6 values), increment the set count
        if (col == robot->Num_of_Joints - 1) {
            set_count++;
        }
    }
	
	robot->know_demands = true; // Demands acknowledgment
	ROS_INFO("Received Demands %s", joint_info.c_str());
}




int main(int argc, char **argv) {

	Robot Mover6(6);  // Creates a Robot with 6 joints 

	// ROS Initialization
	ros::init(argc, argv, "goal_movement_example");
	ros::NodeHandle n;

	/* Create publisher to attach to JointJog */
	ros::Publisher chatter_pub = n.advertise<control_msgs::JointJog>("/JointJog",1);

	// Subscribe to joint states and joint demands, passing the Mover6 object to the callbacks
    ros::Subscriber chatter_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(&jointsCallback, _1, &Mover6));
    
    ros::Subscriber chatter_sub_2 = n.subscribe<std_msgs::Float32MultiArray>("/joint_demands", 1000, boost::bind(&Multi_Array_CB, _1, &Mover6));

	ros::Rate loop_rate(10);

	ros::Duration(2.0).sleep();
	// Mover6.Max_Time(); // computes maximum time
	
	while(ros::ok()) {
		
		if(Mover6.know_states && Mover6.know_demands) {
			
			Mover6.Start_Timer();
			// Mover6.PrintJointMatrix();
			// ROS_INFO("Matrix Size: %s", to_string(Mover6.joint_matrix.size()).c_str());
			// sleep(10);
			for(int N=0; N < Mover6.joint_matrix.size(); N++)
			{
				for(int p = 0; p < 6; p++)
				{
				     Mover6.Robot_Joints[p].init_state = Mover6.Robot_Joints[p].Joint_C_Value; 
				}
				Mover6.Max_Time(); 
				bool all_within_tolerance = false; // All joints within tolerance check
				 while (!all_within_tolerance)
				 {
					int l = 0;  // Counter
					all_within_tolerance = true;
					for(int j=0; j<4; j++)
					{
						Mover6.Robot_Joints[j].Joint_Demand = Mover6.joint_matrix[N][j];
						const auto& joint = Mover6.Robot_Joints[j];
						Mover6.msg_start.joint_names.clear(); 
						Mover6.msg_start.velocities.clear(); 
						ROS_INFO("Setting message");
						Mover6.msg_start.joint_names.push_back(joint.Joint_Name);
						float V = 0;
						if(Mover6.Vel_P) // Trapizoid
						{
							 V = joint.Trapezoidal_Velocity(Mover6.Start_Time, Mover6.Maximum_Time_T, Mover6.T_Blend);
						}
						else // Relative motion
						{
							 V = joint.Sign() * (abs(joint.Joint_Demand - joint.Joint_C_Value) / Mover6.t_max);
						}
						ROS_INFO("Joint: %s, Joint_D: %s, Joint_S: %s, Velocity: %s, l:%s", joint.Joint_Name.c_str(), to_string(joint.Joint_Demand).c_str(), to_string(joint.Joint_C_Value).c_str(), to_string(V).c_str(), to_string(l).c_str());
						// sleep(3);
						Mover6.msg_start.velocities.push_back( (abs(joint.Joint_Demand - joint.Joint_C_Value) > 0.05) ? V : 0 );
						ROS_INFO("Sending message");
						chatter_pub.publish(Mover6.msg_start);

						if (abs(joint.Joint_C_Value - joint.Joint_Demand) < 0.05) {
                            l++;  // Increment the counter if the joint is within tolerance
                        } else {
                            all_within_tolerance = false;  // If any joint is out of tolerance, set the flag to false
                        }
					}
					// If all joints are within tolerance, break out of the loop
                    if (l >= 4) {
                        all_within_tolerance = true;  // All joints are within tolerance, set flag to true
                    }
				ros::spinOnce();
				loop_rate.sleep(); 
				 }
			}
			Mover6.know_demands = false; 
			Mover6.joint_matrix.clear();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

