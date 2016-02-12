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
 pan_velocity_setpoint_= 0.25; //default velocity
 tilt_velocity_setpoint_=0.25; //default velocty

 //setup subscribers and publishers

 ROS_INFO_STREAM("PanTiltControl:: setting up subscriber for topic: " << sub_name);
 angle_sub_ = nh.subscribe("/ptu/state", 1000, &PanTiltControl::JointAngleCallback, this);
 ROS_INFO_STREAM("PanTiltControl:: setting up publisher for topic: " << pub_name);
 angle_pub_ = nh.advertise<sensor_msgs::JointState>(pub_name, 1000);

}

void PanTiltControl::JointAngleCallback(const sensor_msgs::JointState & msg)
{
	//service the callback
	latest_timestamp =msg.header.stamp;
	pan_angle_current_ = msg.position[0];
	tilt_angle_current_ = msg.position[1];
	pan_velocity_current_ = msg.velocity[0];
	tilt_velocity_current_ = msg.velocity[1];

	//buffer the callback
	sensor_msgs::JointState m = msg;
	msg_buffer.push_back(m);

	if(msg_buffer.size()>MAX_BUFFER_SIZE)
		msg_buffer.pop_front();
	
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

void PanTiltControl::get_pan_tilt_angle_interpolated(double &pan_angle, double &tilt_angle, ros::Time int_time)
{
    //first check the size of the buffer 
    if(msg_buffer.size()==0 || msg_buffer.size() ==1 ) //can't interpolate
    {
        get_pan_tilt_angle_current(pan_angle,tilt_angle);
        return;
    }
    
    //find the closest time index
    unsigned int best_idx = FindClosestIndex(int_time);
    std::string interpol_dir = "forward";

    
    if(best_idx == 0)
        interpol_dir = "forward";
    else if(best_idx == (msg_buffer.size()-1))
        interpol_dir = "backward";

    //ROS_INFO_STREAM("interpol" << "size: " <<msg_buffer.size() << "best idx: " << best_idx);
    //ROS_INFO_STREAM("interpol dir: " << interpol_dir);

    //perform the interpolation
    InterpolateFromBufferIndex(best_idx, int_time, interpol_dir, pan_angle, tilt_angle);


}

int PanTiltControl::FindClosestIndex(ros::Time time)
{
    int best_idx = -1;
    double best_score = 1;
	for(unsigned int i=0; i<msg_buffer.size(); i++)
    {
        ros::Time ct = msg_buffer[i].header.stamp;
        //double current_time = ct.toSec();
        ros::Duration diff = ct-time;
        double dt = fabs(diff.toSec());

        if(dt<best_score)
        {
            best_score = dt;
            best_idx = i;
        }
    }
    return best_idx;
}

int PanTiltControl::InterpolateFromBufferIndex(int idx, ros::Time time, std::string dir, double &int_pan, double &int_tilt)
{
    ros::Time rt0;
    ros::Time rt1;
    //double t0=0;
    //double t1=0;
    double yp0=0;
    double yp1=0; 
    double yt0=0;
    double yt1=0; 


     //ROS_INFO_STREAM("interpol" << "size: " <<msg_buffer.size() << "idx: " << idx);
    if(dir=="forward")
    {
         //ROS_INFO_STREAM("f" << "size: " <<msg_buffer.size() << "idx: " << idx);
        rt0 = msg_buffer[idx].header.stamp;
        rt1 = msg_buffer[idx+1].header.stamp;
        //t0 = rt0.toSec();
        //t1 = rt1.toSec(); 
        yp0 = msg_buffer[idx].position[0];
        yp1 = msg_buffer[idx+1].position[0];
        yt0 = msg_buffer[idx].position[1];
        yt1 = msg_buffer[idx+1].position[1];
    }
    else if(dir == "backward")
    {
        //ROS_INFO_STREAM("b" << "size: " <<msg_buffer.size() << "idx: " << idx);
        rt0 = msg_buffer[idx-1].header.stamp;
        rt1 = msg_buffer[idx].header.stamp;
        //t0 = rt0.toSec();
        //t1 = rt1.toSec();
        yp0 = msg_buffer[idx-1].position[0];
        yp1 = msg_buffer[idx].position[0];
        yt0 = msg_buffer[idx-1].position[1];
        yt1 = msg_buffer[idx].position[1]; 
    }
    else
        return -1;

    //ROS_INFO_STREAM("t0: " << rt0<<"t1: " << rt1 <<"y0: " << yp0 << "y1: " << yp1 <<"int time: " << time);


    //linear interpolation on pan axis
    int_pan = LinearInterpolation(rt0,rt1,yp0,yp1,time);
    //linear interpolation in tilt axis
    int_tilt = LinearInterpolation(rt0,rt1,yt0,yt1,time);

    return 0;

}

double PanTiltControl::LinearInterpolation(ros::Time x0,ros::Time x1,double y0, double y1,ros::Time x)
{

    //implementing y0 + (y1-y0)*( (x-x0)/(x1-x0));

    ros::Duration num = x-x0;
    ros::Duration den = x1-x0;

    return y0 + (y1-y0)*( num.toSec()/den.toSec());
}