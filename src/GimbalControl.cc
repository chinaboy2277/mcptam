#include <mcptam/GimbalControl.h>
#include <fstream>


//This class provides functionality to move the FLIR pan tilt unit

GimbalControl::GimbalControl(ros::NodeHandle &nh, std::string sub_name)
{
	//set default values
  pitch_angle_ = 0; 	
  roll_angle_ = 0;
  yaw_angle_ = 0;	
 
 //setup subscribers and publishers

 ROS_INFO_STREAM("GimbalControl:: setting up subscriber for topic: " << sub_name);
 angle_sub_ = nh.subscribe("/gimbal/encoders", 1000, &GimbalControl::JointAngleCallback, this);
 //ROS_INFO_STREAM("GimbalControl:: setting up publisher for topic: " << pub_name);
 //angle_pub_ = nh.advertise<sensor_msgs::JointState>(pub_name, 1000);

}

void GimbalControl::JointAngleCallback(const sensor_msgs::JointState & msg)
{
	//service the callback
	latest_timestamp =msg.header.stamp;
	pitch_angle_ = msg.position[0]; 	
 	roll_angle_ = msg.position[1];
 	yaw_angle_ = msg.position[2];	

	//buffer the callback
	sensor_msgs::JointState m = msg;
	msg_buffer.push_back(m);

	if(msg_buffer.size()>MAX_BUFFER_SIZE)
		msg_buffer.pop_front();
	
}

void GimbalControl::get_gimbal_angles_current(double &roll,double &pitch,double &yaw)
{
	roll = roll_angle_;
	pitch = pitch_angle_;
	yaw = yaw_angle_;
}


/*void GimbalControl::get_pan_tilt_angle_interpolated(double &pan_angle, double &tilt_angle, ros::Time int_time)
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


}*/

/*int GimbalControl::FindClosestIndex(ros::Time time)
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

int GimbalControl::InterpolateFromBufferIndex(int idx, ros::Time time, std::string dir, double &int_pan, double &int_tilt)
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

double GimbalControl::LinearInterpolation(ros::Time x0,ros::Time x1,double y0, double y1,ros::Time x)
{

    //implementing y0 + (y1-y0)*( (x-x0)/(x1-x0));

    ros::Duration num = x-x0;
    ros::Duration den = x1-x0;

    return y0 + (y1-y0)*( num.toSec()/den.toSec());
}*/