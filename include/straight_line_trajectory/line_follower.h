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
		
		// to be called in main()
		void Run();
		
		private:
		// called as callback to odom message. Effectively functions as the main loop
		void Update(const nav_msgs::Odometry &odom);
		
		// callback for dynamic reconfigure 
		void ReconfigCallback(straight_line_trajectory::line_followerConfig &config,  uint32_t level);
		
		// loads rosparam values into dynamic reconfigure server
		void SetReconfServerConfig();
		
		// Setter functions for dynamic reconfigure-able parameters
		void SetSpeed(double speed);		
		void SetHeading(double heading);		
		void SetMaxTurnRate(double turn_rate);
		void SetMinStep(double minstep);
		
		// Provides heading to follow straight line trajectory
		double ComputeHeadingError(double x, double y, double curr_heading);
		
		// Calculates PID based on heading error
		void CalculatePID(double heading_error, double &heading_cmd);
		
		// Publishes desired speed and turn rate on /cmd_vel topic
		void SendMotorCommands(double heading_cmd);
		
		// constrains val to within +-limit. Used to limit output of PID to -1,1
		double constrain_double(double val, double limit=1.0);

		// PID controller object
		control_toolbox::Pid heading_controller;
		
		// Node handle
		ros::NodeHandle *node_handle;
		
		// Subscriber objects
		ros::Subscriber odom_subscriber;

		// Publisher objects
		ros::Publisher speed_publisher;
		ros::Publisher heading_PID_publisher;
		ros::Publisher cmd_vel_publisher;
		ros::Publisher crosstrack_error_publisher;
		
		// Used to store time stamp for PID 
		ros::Time controller_time_stamp;
		
		// Dynamic reconfigure server and callback object
		dynamic_reconfigure::Server<straight_line_trajectory::line_followerConfig> *reconfig_server;
		dynamic_reconfigure::Server<straight_line_trajectory::line_followerConfig>::CallbackType reconfig_callback;	
		
		// Dynamic reconfigure capable parameters
		double heading_setpoint;
		double speed_setpoint;
		double max_turn_rate;
		double min_step;
	};
}

#endif
