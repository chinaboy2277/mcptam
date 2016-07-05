/****************************************************************************************
 *
 * \file GimbalControl.h
 * \brief Declaration of Gimbal class
 *
 * Copyright 2015   Arun Das, University of Waterloo (adas@uwaterloo.ca)
 * 
 *PanTiltControl class provides functionality to move the FLIR pan tilt unit, including 
 *reading pan and tilt angles, as well as sending velocity commands to the unit
 *
 ****************************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <deque>


#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#define MAX_BUFFER_SIZE 200


 class GimbalControl
 {
 public:

 GimbalControl(ros::NodeHandle &nh,std::string sub_name); 	///constructor
 void get_gimbal_angles_current(double &roll,double &pitch,double &yaw);
 //void get_pan_tilt_angle_setpoint(double &pan, double &tilt);
 //void get_pan_tilt_velocty_current(double &pan, double &tilt);
 //void get_pan_tilt_velocty_setpoint(double &pan, double &tilt);
 //void set_pan_tilt_setpoint(double pan, double tilt);
 //void set_axis_velocity(double pan, double tilt);
 //ros::Time get_latest_timestamp(){return latest_timestamp;}
 //void get_gimbal_angles_interpolated(double &roll,double &pitch,double &yaw, ros::Time int_time);
 

 //variables
private:
 double pitch_angle_; 	///current pitch angle, as reported by the driver
 double roll_angle_;	///current roll angle, as reported by the driver
 double yaw_angle_;		///current yaw angle, as reported by the driver
 //double tilt_angle_current_;	///current tilt angle, as reported by the driver
 //double pan_velocity_current_; 	///current pan velocity, as reported by the driver
 //double tilt_velocity_current_;	///current tilt velocity, as reported by the driver
 //double pan_angle_setpoint_;	///setpoint for the pan angle
 //double tilt_angle_setpoint_; 	///setpoint for the tilt angle
 //double pan_velocity_setpoint_;	///setpoint for the pan velocity
 //double tilt_velocity_setpoint_; 	///setpoint for the tilt velocity

 std::deque<sensor_msgs::JointState> msg_buffer;	//buffer for holding a fixed number of joint messages

 ros::Time latest_timestamp;

 ros::Subscriber angle_sub_;		///subscriber to the joint angles
 ros::Publisher	angle_pub_; 		///publisher for joint angles

 //functions
 void JointAngleCallback(const sensor_msgs::JointState & msg); 	///callback which sets the current joint angles
 //int FindClosestIndex(ros::Time time); //returns the index of the sample with the closest time
 //int InterpolateFromBufferIndex(int idx, ros::Time time, std::string dir, double &int_pan, double &int_tilt);
 //double LinearInterpolation(ros::Time t0,ros::Time t1,double y0, double y1,ros::Time x);



 };

 #endif