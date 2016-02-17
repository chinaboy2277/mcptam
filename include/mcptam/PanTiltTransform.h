/****************************************************************************************
 *
 * \file PanTiltTransform.h
 * \brief Declaration of PanTiltTransform class
 *
 * Copyright 2015   Arun Das, University of Waterloo (adas@uwaterloo.ca)
 * 
 *PanTiltTransform class provides functionality to compute the total rigid body transformation 
 *given a calibrated pan tilt rig
 *
 ****************************************************************************************/

#include <ros/ros.h>
#include <Eigen/Core>
#include <string>

#ifndef __PAN_TILT_TRANSFORM_H
#define __PAN_TILT_TRANSFORM_H

 typedef Eigen::Matrix< double, 6, 1 > 	Vector6d;


 class PanTiltTransform
 {
 public:

 	//constructor
 	PanTiltTransform(Eigen::Matrix< double, 18, 1 > calibration_parameters); 
 	PanTiltTransform(Eigen::Matrix< double, 12, 1 > calibration_parameters); 
 	PanTiltTransform(Eigen::Matrix< double, 6, 1 > calibration_parameters); 
 	
 	//constructs a transformation matrix given the DH parameters (theta, d, r, alpha)
 	Eigen::Matrix4d GenerateDHMatrix(double theta, double d,double r,double alpha); 
 	
 	// generate rigid body transformation matrix from paramaters:
 	// pvec[0]: rotation about x axis
 	// pvec[1]: rotation about y axis
 	// pvec[2]: rotation about z axis
 	// pvec[3]: translation about x axis
 	// pvec[4]: translation about yis
 	// pvec[5]: translation about z axis
 	Eigen::Matrix4d GenerateTransformationMatrix(Vector6d pvec); 

 	//computes the rigid body transformation through the pan tilt rig
 	Eigen::Matrix4d ComputeRigTransformation(); 
 	
 	//sets the pan angle for the unit
 	void set_pan_angle(double pan_angle){_current_pan_angle = pan_angle;} 

 	//sets the tilt angle for the unit
 	void set_tilt_angle(double tilt_angle){_current_tilt_angle = tilt_angle;}

 private:

 	std::string cal_parameterization;
 	double _current_pan_angle; //current pan_angle (dont forget to add 90 offset)
 	double _current_tilt_angle; //current tilt_angle (dont forget to add 90 offset)
 	Eigen::Matrix< double, 18, 1 > _calibration_parameters18; //calibration params
 	Eigen::Matrix< double, 12, 1 > _calibration_parameters12; //calibration params 
 	Eigen::Matrix< double, 6, 1 > _calibration_parameters6; //calibration params 
 
 };

#endif //__PAN_TILT_TRANSFORM_H
