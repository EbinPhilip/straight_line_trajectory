#ifndef LINE_H
#define LINE_H

#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <straight_line_trajectory/line_followerConfig.h>
#include <straight_line_trajectory/PIDtuning.h>

namespace line_follower 
{
	
	class line_follower 
	{
		
		public:
		
		line_follower(ros::NodeHandle &n);
		
		~line_follower();
		
		void Run();
		
		private:
		
		void Update(const nav_msgs::Odometry &odom);
		
		void ReconfigCallback(straight_line_trajectory::line_followerConfig &config,  uint32_t level);
		
		void SetReconfServerConfig();
		
		void SetSpeed(double speed);
		
		void SetHeading(double heading);
		
		void SetMaxSpeed(double speed);
		
		void SetMaxTurnRate(double turn_rate);
		
		void SetMinStep(double minstep);
		
		//void SetThrottle(double throttle);
		
		//void SetThrottleTrim(double trim);
		
		double ComputeHeadingError(double x, double y, double curr_heading
		);
		
		void CalculatePID(double heading_error, double speed_error, double &left_wheel_command, double &right_wheel_command);
		
		void SendMotorCommands(double left_wheel_command, double right_wheel_command);
		
		double constrain_double(double val, double limit=1.0);
	
		control_toolbox::Pid speed_controller;
		control_toolbox::Pid heading_controller;
		
		ros::NodeHandle *node_handle;
		ros::Subscriber odom_subscriber;
		//ros::Publisher front_left_publisher;
		//ros::Publisher front_right_publisher;
		//ros::Publisher rear_left_publisher;
		//ros::Publisher rear_right_publisher;
		ros::Publisher speed_PID_publisher;
		ros::Publisher heading_PID_publisher;
		ros::Publisher cmd_vel_publisher;
		ros::Publisher crosstrack_error_publisher;
		
		ros::Time controller_time_stamp;
		
		dynamic_reconfigure::Server<straight_line_trajectory::line_followerConfig> *reconfig_server;
		dynamic_reconfigure::Server<straight_line_trajectory::line_followerConfig>::CallbackType reconfig_callback;	
		
		double heading_setpoint;
		double speed_setpoint;
		//double max_throttle;
		double max_speed;
		double max_turn_rate;
		double min_step;
		//double throttle_trim;
	};
}

#endif
