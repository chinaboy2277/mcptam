#include <mcptam/PanTiltControl.h>
#include <fstream>

//This class provides functionality to move the FLIR pan tilt unit

PanTiltControl::PanTiltControl(ros::NodeHandle &nh, std::string sub_name, std::string pub_name)
{
	//set default values
 pan_angle_current_ = 0;
 tilt_angle_current_ = 0;
 pan_angle_setpoint_= 0;
 tilt_angle_setpoint_=0;
 pan_velocity_current_ = 0;
 tilt_velocity_current_ = 0;
 pan_velocity_setpoint_= 0.5; //default velocity
 tilt_velocity_setpoint_=0.5; //default velocty

 //setup subscribers and publishers

 ROS_INFO_STREAM("PanTiltControl:: setting up subscriber for topic: " << sub_name);
 angle_sub_ = nh.subscribe("/ptu/state", 1000, &PanTiltControl::JointAngleCallback, this);
 ROS_INFO_STREAM("PanTiltControl:: setting up publisher for topic: " << pub_name);
 angle_pub_ = nh.advertise<sensor_msgs::JointState>(pub_name, 1000);

}

void PanTiltControl::JointAngleCallback(const sensor_msgs::JointState & msg)
{
	//service the callback
	pan_angle_current_ = msg.position[0];
	tilt_angle_current_ = msg.position[1];
	pan_velocity_current_ = msg.velocity[0];
	tilt_velocity_current_ = msg.velocity[1];
	
}

void PanTiltControl::get_pan_tilt_angle_current(double &pan, double &tilt)
{
	pan = pan_angle_current_;
	tilt = tilt_angle_current_;
}

void PanTiltControl::get_pan_tilt_angle_setpoint(double &pan, double &tilt)
{
	pan = pan_angle_setpoint_;
	tilt = tilt_angle_setpoint_;
}

void PanTiltControl::get_pan_tilt_velocty_current(double &pan, double &tilt)
{
	pan = pan_velocity_current_;
	tilt = tilt_velocity_current_;
}

void PanTiltControl::get_pan_tilt_velocty_setpoint(double &pan, double &tilt)
{
	pan = pan_velocity_setpoint_;
	tilt = tilt_velocity_setpoint_;
}

 void PanTiltControl::set_pan_tilt_setpoint(double pan_angle, double tilt_angle)
 {

 	pan_angle_setpoint_ = pan_angle;
 	tilt_angle_setpoint_ = tilt_angle;

 	sensor_msgs::JointState joint_state_message;
 	//resize the message
 	joint_state_message.position.resize(2);
 	joint_state_message.velocity.resize(2);
 	joint_state_message.position[0] = pan_angle_setpoint_;
 	joint_state_message.position[1] = tilt_angle_setpoint_;
 	joint_state_message.velocity[0] = pan_velocity_setpoint_;
 	joint_state_message.velocity[1] = tilt_velocity_setpoint_;
 	angle_pub_.publish(joint_state_message);
 	

 }
 void PanTiltControl::set_axis_velocity(double pan_velocity, double tilt_velocity)
 {

 	pan_velocity_setpoint_ = pan_velocity;
 	tilt_velocity_setpoint_= tilt_velocity;

 	sensor_msgs::JointState joint_state_message;
 	joint_state_message.position[0] = pan_angle_setpoint_;
 	joint_state_message.position[1] = tilt_angle_setpoint_;
 	joint_state_message.velocity[0] = pan_velocity_setpoint_;
 	joint_state_message.velocity[1] = tilt_velocity_setpoint_;
 	angle_pub_.publish(joint_state_message);
 }

