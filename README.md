# straight_line_trajectory ROS package
Moves ROSBot on a straight line trajectory along the x axis at a given speed. 

1) download the ROSBot package and add to path: https://github.com/husarion/rosbot_description. Do the same with this repo.
2) Install all dependencies using rosdep
3) run ``` roslaunch straight_line_trajectory straight_line.launch ```
4) Change the speed_setpoint parameter on rqt_reconfigure window to move the robot. Donot change the heading_setpoint, except for PID tuning. Leave it at 0.0
5) Published topic can be viewed using rqt_plot:
 - /line_follower_node/speed_tuning 
 - /line_follower_node/heading_PID_tuning
 - /line_follower_node/cross_track_error
 - /cmd_vel
  
