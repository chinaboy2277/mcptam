
//this class provides functionality to get the extrinsic chain for a reconfigurable pan tilt rig

#include <mcptam/PanTiltTransform.h>


PanTiltTransform::PanTiltTransform(Eigen::Matrix< double, 18, 1 > calibration_parameters)
{
	
	//set the calibration params
	 _calibration_parameters = calibration_parameters;
	 //init the pan and tilt angles
	 _current_pan_angle = 0;
	 _current_tilt_angle=0;
}


Eigen::Matrix4d PanTiltTransform::GenerateDHMatrix(double theta, double d,double r,double alpha)
{
	Eigen::Matrix4d T_dh;

	T_dh<< 	cos(theta), 	-sin(theta)*cos(alpha), 	sin(theta)*sin(alpha), 	r*cos(theta),
    		sin(theta), 	cos(theta)*cos(alpha),		-cos(theta)*sin(alpha),	r*sin(theta),
    		0,				sin(alpha),					cos(alpha), 			d,
    		0,				0,							0,						1;
   return T_dh;
} 

Eigen::Matrix4d PanTiltTransform::GenerateTransformationMatrix(Vector6d pvec)
{
	Eigen::Matrix4d T;

	//extract parameters
  double rx = pvec(0); double ry = pvec(1); double rz = pvec(2);
  double tx = pvec(3); double ty = pvec(4); double tz = pvec(5);

  //precompute sine/cosine
  double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
  double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

  //compute rotation matrix 
  double r00    = +cy*cz;           double r01    = -cy*sz;           double r02    = +sy;
  double r10    = +sx*sy*cz+cx*sz;  double r11    = -sx*sy*sz+cx*cz;  double r12    = -sx*cy;
  double r20    = -cx*sy*cz+sx*sz;  double r21    = +cx*sy*sz+sx*cz;  double r22    = +cx*cy;
  
	T << r00, 	r01, 	r02, 	tx,
	 	 r10, 	r11, 	r12, 	ty, 
	 	 r20,	r21, 	r22, 	tz,
	 	 0,		0,		0,		1; 

 return T;
}

Eigen::Matrix4d PanTiltTransform::ComputeRigTransformation()
{
	double theta1 = _current_pan_angle; 
    double theta2 = _current_tilt_angle; 
    
    Vector6d cam1_to_base_params = _calibration_parameters.segment(0,6); //first 6 params
    Eigen::Vector3d base_to_pt1_dh = _calibration_parameters.segment(6,3); //next 3 params
    Eigen::Vector3d base_to_pt2_dh = _calibration_parameters.segment(9,3); // next 3 params
    Vector6d pt_to_cam2 = _calibration_parameters.segment(12,6); // last 6 params
 
    Eigen::Matrix4d Tcam1_to_base = GenerateTransformationMatrix(cam1_to_base_params);
    Eigen::Matrix4d Tbase_to_pt1 = GenerateDHMatrix(theta1,base_to_pt1_dh(0),base_to_pt1_dh(1),base_to_pt1_dh(2));
    Eigen::Matrix4d Tbase_to_pt2 = GenerateDHMatrix(theta2,base_to_pt2_dh(0),base_to_pt2_dh(1),base_to_pt2_dh(2));
    Eigen::Matrix4d Tpt_to_cam2 = GenerateTransformationMatrix(pt_to_cam2);
    Eigen::Matrix4d T_total = Tcam1_to_base*Tbase_to_pt1* Tbase_to_pt2*Tpt_to_cam2;

    return T_total;
}

