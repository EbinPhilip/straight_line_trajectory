#include <line_follower.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

namespace line_follower
{
	line_follower::line_follower(ros::NodeHandle &n)
	{
		this->node_handle = new ros::NodeHandle(n);
		
		// Connect publishers to topics
		this->cmd_vel_publisher = this->node_handle->advertise<geometry_msgs::Twist>("/cmd_vel",1);
		this->speed_publisher = this->node_handle->advertise<straight_line_trajectory::PIDtuning>(this->node_handle->resolveName("speed_tuning"), 1);
		this->heading_PID_publisher = this->node_handle->advertise<straight_line_trajectory::PIDtuning>(this->node_handle->resolveName("heading_PID_tuning"), 1);
		this->crosstrack_error_publisher = this->node_handle->advertise<std_msgs::Float64>(this->node_handle->resolveName("cross_track_error"),1);
		
		// Initialize PID controller object
		this->heading_controller.init(ros::NodeHandle(n,"heading_pid"));
		
		// Connect subscribers to topic and callback: Update()
		this->odom_subscriber = this->node_handle->subscribe("/odom", 1, &line_follower::Update, this);
		
		//Intialize PID time stamp
		this->controller_time_stamp = ros::Time::now();
		
		// Setup dynamic reconfigure
		this->reconfig_server = new dynamic_reconfigure::Server<straight_line_trajectory::line_followerConfig>(n);
		// Load values from rosparam server
		this->SetReconfServerConfig();
		//Set dynamic reconfigure callback 
		this->reconfig_callback = boost::bind(&line_follower::ReconfigCallback, this, _1, _2);
		this->reconfig_server->setCallback(this->reconfig_callback);
	}
		
	line_follower::~line_follower()
	{
	}
	
	// blocking function, to be called in main() 
	void line_follower::Run()
	{
		ros::spin();
	}
	
	// Callback to odom messages
	// calculates heading
	// uses PID to calculate turn rate based on heading error
	// Sends speed and turn rate(Twist) to motors
	void line_follower::Update(const nav_msgs::Odometry &odom)
	{
		// publisher messages
		straight_line_trajectory::PIDtuning speed_msg, heading_msg;
		
		// get speed and heading from odometry message
		double current_speed = odom.twist.twist.linear.x;
		double current_heading = tf::getYaw(odom.pose.pose.orientation);
		
		// get desired heading for straight line trajectory
		double heading = this->ComputeHeadingError(odom.pose.pose.position.x , odom.pose.pose.position.y, current_heading);
		
		// populate speed and heading publisher messages
		speed_msg.desired = this->speed_setpoint;
		heading_msg.desired = angles::to_degrees(heading) + this->heading_setpoint;
		speed_msg.actual = current_speed;
		heading_msg.actual = angles::to_degrees(current_heading);
		
		// calculate heading error, send to PID
		double heading_error = angles::from_degrees(this->heading_setpoint) + heading - current_heading;
		double heading_cmd;
		this->CalculatePID(heading_error, heading_cmd);
		
		// sends heading command and speed setpoint via /cmd_vel topic
		this->SendMotorCommands(heading_cmd);
		
		// publish speed and heading messages
		this->speed_publisher.publish(speed_msg);
		this->heading_PID_publisher.publish(heading_msg);	
	}
	
	// dynamic reconfigure server callback
	void line_follower::ReconfigCallback(straight_line_trajectory::line_followerConfig &config, uint32_t level)
	{
		// Update values obtained to current object
		this->SetSpeed(config.speed_setpoint);
		this->SetHeading(config.heading_setpoint);
		this->SetMaxTurnRate(config.max_turn_rate);
		this->SetMinStep(config.min_step);
	}
	
	// Initilize dynamic reconfigure server from ros parameter server
	void line_follower::SetReconfServerConfig()
	{
		double speed, heading, max_turn, minstep;
		straight_line_trajectory::line_followerConfig config;
		
		//retrieve rosparams
		this->node_handle->getParam(this->node_handle->resolveName("speed_setpoint"), speed);
		this->node_handle->getParam(this->node_handle->resolveName("heading_setpoint"), heading);
		this->node_handle->getParam(this->node_handle->resolveName("max_turn_rate"), max_turn);
		this->node_handle->getParam(this->node_handle->resolveName("min_step"), minstep);

		// populate dynamic reconfigure server message
		config.speed_setpoint = speed;
		config.heading_setpoint = heading;
		config.max_turn_rate = max_turn;
		config.min_step = minstep;
		
		// Set values obtained to current object
		this->SetSpeed(config.speed_setpoint);
		this->SetHeading(config.heading_setpoint);
		this->SetMaxTurnRate(config.max_turn_rate);
		this->SetMinStep(config.min_step);
		
		// Update values in server
		this->reconfig_server->updateConfig(config);
		
	}
	
	//// setter functions for dynamic reconfigure ////
	void line_follower::SetSpeed(double speed)
	{
		this->speed_setpoint = speed;
	}
	
	void line_follower::SetHeading(double heading)
	{
		this->heading_setpoint = heading;
	}
	
	void line_follower::SetMaxTurnRate(double turn_rate)
	{
		this->max_turn_rate = turn_rate;
	}
	
	void line_follower::SetMinStep(double minstep)
	{
		this->min_step = minstep;
	}
	/////////////////////////////////////////////////
	
	// Compute heading for straight line trajectory
	double line_follower::ComputeHeadingError(double x, double y, double curr_heading)
	{
		//return if desired speed is zero
		if (this->speed_setpoint == 0)
		{
			return 0.0;
		}
		
		//calculate distance of vehicle from origin
		double dist_origin = std::sqrt(x*x + y*y);
		
		// calculate and send cross track error
		std_msgs::Float64 cross_track_error;
		cross_track_error.data = dist_origin * std::atan2(y, x);
		this->crosstrack_error_publisher.publish(cross_track_error);
		
		// calculate steps to next closest point on straight line trajectory
		double step_no = std::floor(dist_origin/this->min_step) + (this->speed_setpoint>=0 ? 1:-1)*(x>=0 ? 1:-1);
		
		// adjust for both half of x-y plane
		if (x<0)
			step_no = -step_no;
		
		// calculate heading to next closest point on straight line trajectory
		double heading ;
		if (this->speed_setpoint>=0)
		{
			heading = std::atan2( (-y), (step_no*this->min_step - x) );
		}
		else
		{
			heading = std::atan2( (y), (x - step_no*this->min_step) );
		}
		return heading;
	}
	
	// Calculate PID from heading error
	void line_follower::CalculatePID(double heading_error, double &heading_cmd)
	{
		// calculate time difference from last PID call
		ros::Duration dt = ros::Duration( (ros::Time::now() - this->controller_time_stamp).toSec() );
		if (dt > ros::Duration(1.0))
		{
			this->heading_controller.reset();
		}	
		// PID output in -1,1 range
		heading_cmd = this->constrain_double( this->heading_controller.computeCommand(heading_error, dt) );
		// update current time
		this->controller_time_stamp = ros::Time::now();
	}
	
	// publish speed and turn rate on /cmd_vel topic
	void line_follower::SendMotorCommands(double heading_cmd)
	{
		geometry_msgs::Twist msg;
		msg.linear.x = this->speed_setpoint;
		// get turn rate from PID output by multiplying with max_turn_rate parameter
		msg.angular.z = heading_cmd * angles::from_degrees(this->max_turn_rate);
		this->cmd_vel_publisher.publish(msg);
	}
	
	// to constrain PID output to -1,1 range
	double line_follower::constrain_double(double val, double limit)
	{
		if (std::abs(val) > std::abs(limit))
		{
			if (val > 0)
			{
				return limit;
			}
			else
			{
				return -limit;
			}
		}
		else
			return val;
	}
}
