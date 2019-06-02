#include <line_follower.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

namespace line_follower
{
	line_follower::line_follower(ros::NodeHandle &n)
	{
		this->node_handle = new ros::NodeHandle(n);
		
		this->cmd_vel_publisher = this->node_handle->advertise<geometry_msgs::Twist>("/cmd_vel",1);
		//this->front_left_publisher = this->node_handle->advertise<std_msgs::Float64>("/front_left_controller/command", 1);
		//this->front_right_publisher = this->node_handle->advertise<std_msgs::Float64>("/front_right_controller/command", 1);
		//this->rear_left_publisher = this->node_handle->advertise<std_msgs::Float64>("/rear_left_controller/command", 1);
		//this->rear_right_publisher = this->node_handle->advertise<std_msgs::Float64>("/rear_right_controller/command", 1);
		this->speed_PID_publisher = this->node_handle->advertise<straight_line_trajectory::PIDtuning>(this->node_handle->resolveName("speed_PID_tuning"), 1);
		this->heading_PID_publisher = this->node_handle->advertise<straight_line_trajectory::PIDtuning>(this->node_handle->resolveName("heading_PID_tuning"), 1);
		this->crosstrack_error_publisher = this->node_handle->advertise<std_msgs::Float64>(this->node_handle->resolveName("cross_track_error"),1);
		
		this->speed_controller.init(ros::NodeHandle(n,"speed_pid"));
		this->heading_controller.init(ros::NodeHandle(n,"heading_pid"));
		
		this->odom_subscriber = this->node_handle->subscribe("/odom", 1, &line_follower::Update, this);
		
		this->controller_time_stamp = ros::Time::now();
		
		this->reconfig_server = new dynamic_reconfigure::Server<straight_line_trajectory::line_followerConfig>(n);
		this->SetReconfServerConfig();
		this->reconfig_callback = boost::bind(&line_follower::ReconfigCallback, this, _1, _2);
		this->reconfig_server->setCallback(this->reconfig_callback);
	}
		
	line_follower::~line_follower()
	{
	}
	
	void line_follower::Run()
	{
		ros::spin();
	}
	
	
	void line_follower::Update(const nav_msgs::Odometry &odom)
	{
		straight_line_trajectory::PIDtuning speed_msg, heading_msg;
		
		double current_speed = odom.twist.twist.linear.x;
		double current_heading = tf::getYaw(odom.pose.pose.orientation);
		
		double heading = this->ComputeHeadingError(odom.pose.pose.position.x , odom.pose.pose.position.y, current_heading);
		
		speed_msg.desired = this->speed_setpoint;
		heading_msg.desired = angles::to_degrees(heading);
		
		speed_msg.actual = current_speed;
		heading_msg.actual = angles::to_degrees(current_heading);
		
		double speed_error = this->speed_setpoint - current_speed;
		double heading_error = heading - current_heading;
		
		double heading_cmd, speed_cmd;
		this->CalculatePID(heading_error, speed_error, heading_cmd, speed_cmd);
		this->SendMotorCommands(heading_cmd, speed_cmd);
		
		this->speed_PID_publisher.publish(speed_msg);
		this->heading_PID_publisher.publish(heading_msg);
		
	}

	void line_follower::ReconfigCallback(straight_line_trajectory::line_followerConfig &config, uint32_t level)
	{
		this->SetSpeed(config.speed_setpoint);
		this->SetHeading(config.heading_setpoint);
		this->SetMaxSpeed(config.max_speed);
		this->SetMaxTurnRate(config.max_turn_rate);
		this->SetMinStep(config.min_step);
		//this->SetThrottle(config.max_throttle);
		//this->SetThrottleTrim(config.throttle_trim);
		//this->reconfig_server->updateConfig(config);
	}
	
	void line_follower::SetReconfServerConfig()
	{
		double speed, heading, max_spd, max_turn, minstep;
		straight_line_trajectory::line_followerConfig config;
		
		this->node_handle->getParam(this->node_handle->resolveName("speed_setpoint"), speed);
		this->node_handle->getParam(this->node_handle->resolveName("heading_setpoint"), heading);
		this->node_handle->getParam(this->node_handle->resolveName("max_speed"), max_spd);
		this->node_handle->getParam(this->node_handle->resolveName("max_turn_rate"), max_turn);
		this->node_handle->getParam(this->node_handle->resolveName("min_step"), minstep);

		config.speed_setpoint = speed;
		config.heading_setpoint = heading;
		config.max_speed = max_spd;
		config.max_turn_rate = max_turn;
		config.min_step = minstep;
		
		this->SetSpeed(config.speed_setpoint);
		this->SetHeading(config.heading_setpoint);
		this->SetMaxSpeed(config.max_speed);
		this->SetMaxTurnRate(config.max_turn_rate);
		this->SetMinStep(config.min_step);
		//this->SetThrottle(config.max_throttle);
		//this->SetThrottleTrim(config.throttle_trim);
		
		this->reconfig_server->updateConfig(config);
		
	}
	
	void line_follower::SetSpeed(double speed)
	{
		this->speed_setpoint = speed;
	}
	
	void line_follower::SetHeading(double heading)
	{
		this->heading_setpoint = heading;
	}
	
	//void line_follower::SetThrottle(double throttle)
	//{
		//this->max_throttle = throttle;
	//}
	
	//void line_follower::SetThrottleTrim(double trim)
	//{
		//this->throttle_trim = trim;
	//}
	
	void line_follower::SetMaxSpeed(double speed)
	{
		this->max_speed = speed;
	}
	
	void line_follower::SetMaxTurnRate(double turn_rate)
	{
		this->max_turn_rate = turn_rate;
	}
	
	void line_follower::SetMinStep(double minstep)
	{
		this->min_step = minstep;
	}
	
	double line_follower::ComputeHeadingError(double x, double y, double curr_heading)
	{
		if (this->speed_setpoint == 0)
		{
			return curr_heading;
		}
		double dist_origin = std::sqrt(x*x + y*y);
		
		std_msgs::Float64 cross_track_error;
		cross_track_error.data = dist_origin * std::atan2(y, x);
		this->crosstrack_error_publisher.publish(cross_track_error);
		
		double step_no = std::floor(dist_origin/this->min_step) + (this->speed_setpoint>=0 ? 1:-1)*(x>=0 ? 1:-1);
		if (x<0)
			step_no = -step_no;
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
	
	void line_follower::CalculatePID(double heading_error, double speed_error, double &heading_cmd, double &speed_cmd)
	{
		ros::Duration dt = ros::Duration( (ros::Time::now() - this->controller_time_stamp).toSec() );
		if (dt > ros::Duration(1.0))
		{
			this->speed_controller.reset();
			this->heading_controller.reset();
		}
		speed_cmd = this->constrain_double( this->speed_controller.computeCommand(speed_error, dt) );
		heading_cmd = this->constrain_double( this->heading_controller.computeCommand(heading_error, dt) );
		//if ( (std::abs(speed_cmd) + std::abs(heading_cmd) ) > 1.0)
		//{
			//speed_cmd = speed_cmd/(std::abs(speed_cmd) + std::abs(heading_cmd));
			//heading_cmd = heading_cmd/(std::abs(speed_cmd) + std::abs(heading_cmd));
		//}
		//left_wheel_command = speed_cmd - heading_cmd;
		//right_wheel_command = speed_cmd + heading_cmd;
		this->controller_time_stamp = ros::Time::now();
	}
	
	void line_follower::SendMotorCommands(double heading_cmd, double speed_cmd)
	{
		geometry_msgs::Twist msg;
		msg.linear.x = this->speed_setpoint;
		msg.angular.z = heading_cmd * angles::from_degrees(this->max_turn_rate);
		this->cmd_vel_publisher.publish(msg);
		//std_msgs::Float64 left_wheel_command, right_wheel_command;
		//left_wheel_command.data = left_wheel_output * this->max_throttle + this->throttle_trim * (left_wheel_output>=0 ? 1:-1);
		//right_wheel_command.data = right_wheel_output * this->max_throttle + this->throttle_trim * (right_wheel_output>=0 ? 1:-1);
		//this->front_left_publisher.publish(left_wheel_command);
		//this->front_right_publisher.publish(right_wheel_command);
		//this->rear_left_publisher.publish(left_wheel_command);
		//this->rear_right_publisher.publish(right_wheel_command);
	}
	
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
