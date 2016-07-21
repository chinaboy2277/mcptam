/*************************************************************************
 *  
 *  
 *  Copyright 2014  Adam Harmat (McGill University) 
 *                      [adam.harmat@mail.mcgill.ca]
 *                  Michael Tribou (University of Waterloo)
 *                      [mjtribou@uwaterloo.ca]
 *
 *  Multi-Camera Parallel Tracking and Mapping (MCPTAM) is free software:
 *  you can redistribute it and/or modify it under the terms of the GNU 
 *  General Public License as published by the Free Software Foundation,
 *  either version 3 of the License, or (at your option) any later
 *  version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  
 *  MCPTAM is based on the Parallel Tracking and Mapping (PTAM) software.
 *  Copyright 2008 Isis Innovation Limited
 *  
 *  
 ************************************************************************/


//=========================================================================================
//
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <mcptam/PointCollector.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/KeyFrameViewer.h>
#include <mcptam/BundleAdjusterSingle.h>
#include <mcptam/MapMakerCalib.h>
#include <mcptam/TrackerCalib.h>
#include <mcptam/Utility.h>
#include <mcptam/OpenGL.h>
#include <mcptam/Map.h>
#include <mcptam/PanTiltControl.h>
 #include <mcptam/GimbalControl.h>
#include <gvars3/instances.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <fstream>
#include <ros/ros.h> 

using namespace GVars3;
using namespace TooN;

bool bGrabPoints = false;
double pan_tilt_angle_increment = 0.174533; //in rads, about 10 degrees

PointCollector::PointCollector(ros::NodeHandle &nodehandle)
: SystemBase("PointCollector",true, true)
{
  if(mpVideoSourceMulti->GetNumGroups() != 1)
  {
    ROS_FATAL_STREAM("PointCollector: All cameras need to be triggered together in one group.");
    ROS_FATAL_STREAM("PointCollector: Currently you have "<<mpVideoSourceMulti->GetNumGroups()<<" camera groups.");
    ros::shutdown();
    return;
  }


  //hook up the ROS nodehandle
 
  nh = nodehandle;
  //setup pan tilt control object

  capture_index = 0; //reset capture index

  //PTC = new PanTiltControl(nh,"/ptu/state", "/ptu/cmd");
  GC = new GimbalControl(nh,"/gimbal/encoders");
  
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("GrabFrame", GUICommandCallBack, this);
  
  
  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  
  GUI.ParseLine("DrawMasks=0");
  GUI.ParseLine("DrawCandidates=0");
  GUI.ParseLine("DrawLevel=0");
  GUI.ParseLine("GlareMasking=0");
  GUI.ParseLine("LevelZeroPoints=1");
  
  bool bLevelZeroPoints;
  mNodeHandlePriv.param<bool>("level_zero_points", bLevelZeroPoints, true);
  
  static gvar3<int> gvnLevelZeroPoints("LevelZeroPoints", 0, HIDDEN|SILENT);
  *gvnLevelZeroPoints = bLevelZeroPoints;
  
  // Main Menu
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  
  
  mNodeHandlePriv.param<int>("pattern_width", mirPatternSize[0],0);
  mNodeHandlePriv.param<int>("pattern_height", mirPatternSize[1],0);
  mNodeHandlePriv.param<double>("square_size", mdSquareSize,0);
  
  // Advertise reset before creating mapmaker
  mResetSystemServer = mNodeHandlePriv.advertiseService("reset", &PointCollector::ResetSystemCallback, this);
  
  mpBundleAdjuster = new BundleAdjusterSingle(*mpMap, mmCameraModels);
  mpMapMaker = new MapMakerCalib(*mpMap, mmCameraModels, *mpBundleAdjuster);
  mpKeyFrameViewer = new KeyFrameViewer(*mpMap, *mpGLWindow, mmDrawOffsets, mpVideoSourceMulti->GetSizes());
  
  ImageBWMap masksMap = LoadMasks(); 
  
  // Create the CalibratorTrackers, one for each camera
  for(TaylorCameraMap::iterator it = mmCameraModels.begin(); it != mmCameraModels.end(); it++)
  {
    std::string camName = it->first;
    mmTrackers[camName] = new TrackerCalib(*mpMap, *mpMapMaker, mmCameraModels, camName, mmDrawOffsets[camName], mirPatternSize, mdSquareSize, mpGLWindow);
    mmTrackers[camName]->SetMasks(masksMap);
  }
  
  mnLastNumInit = 0;
  mbOptimizing = false;
  mbDone = false;
}

PointCollector::~PointCollector()
{
  delete mpBundleAdjuster;
  delete mpMapMaker;
  delete mpKeyFrameViewer;
  
  for(TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
  {
    delete it->second;
  }
}

// Blocking function that loops indefinitiely
void PointCollector::Run()
{
  static gvar3<std::string> gvsCurrentSubMenu("Menu.CurrentSubMenu", "", HIDDEN|SILENT);
  
  while(!mbDone && ros::ok())
  {
    mpGLWindow->SetupViewport();
    mpGLWindow->SetupVideoOrtho();
    mpGLWindow->SetupVideoRasterPosAndZoom();
       
    std::string caption;

    caption = Track();
    mpGLWindow->DrawCaption(caption);
    mpGLWindow->DrawMenus();
    mpGLWindow->swap_buffers();
    mpGLWindow->HandlePendingEvents();


    // GUI interface
    while(!mqCommands.empty())
    {

      GUICommandHandler(mqCommands.front().command, mqCommands.front().params);
      mqCommands.pop();
    }

    ros::spinOnce(); //spin to make sure we process callbacks
  }
  
}

// Perform all tracking operations
std::string PointCollector::Track()
{

  double pixel_err_thresh = 0.2;	
  // This data will be displayed on GUI
  static std::queue<ros::Time> qLoopTimes;
  static std::deque<ros::Duration> qTotalDurations;
  static unsigned int nMaxQueueSize = 10;
  std::vector<CVD::ImageRef> vOffset;
  
      
  std::stringstream captionStream;  // gather messages here
  
  // Grab new video frame...
  ros::Time timestamp;
  bool bGotNewFrame = mpVideoSourceMulti->GetAndFillFrameBW(ros::WallDuration(0.1), mmFramesBW, timestamp);

  static gvar3<std::string> gvsCurrentSubMenu("Menu.CurrentSubMenu", "", HIDDEN|SILENT); 
  bool bDrawKeyFrames = *gvsCurrentSubMenu == "View";
  

  
  if(bGotNewFrame)
  {
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT);
           
    mvInitialized.clear();
    
    ros::Time start = ros::Time::now();
    int nCamNum = 1;
    bool allCamsGotGrid = true;
    std::vector<TrackerCalib*> vTrackers; //vector of tracker data, save them so we can process them later

    //first go through all the images to see if they all found a checkerboard

    for(ImageBWMap::iterator it = mmFramesBW.begin(); it != mmFramesBW.end(); it++, nCamNum++)
    {
        TrackerCalib* pTracker = mmTrackers[it->first];
        pTracker->singleGridVisible = true;
        vTrackers.push_back(pTracker);
        pTracker->TrackFrame(it->second, timestamp, true, true);
        if(!pTracker->foundGridPattern) //one frame did not find a grid, set flag to false
        {
          allCamsGotGrid = false; 
          pTracker->singleGridVisible = false;   
        }


       // Print sequential number as overlay
      glColor3f(0,1.0,0);
      CVD::ImageRef irOffset = pTracker->mmDrawOffsets[pTracker->mCamName];
      vOffset.push_back(irOffset);
      std::stringstream ss;
      ss<<"#"<<nCamNum<<std::endl;
      mpGLWindow->PrintString(irOffset + CVD::ImageRef(10,40), ss.str(), 10);

      //print out the pan and tilt angles to screen
      //double pan_angle; 
      //double tilt_angle;
      double pitch_angle;
      double roll_angle;
      double yaw_angle;
      GC->get_gimbal_angles_current(roll_angle,pitch_angle,yaw_angle);
      std::stringstream ps;
      //ps<<"pan: "<<pan_angle*57.3<< std::endl <<"tilt: " << tilt_angle*57.3 <<std::endl;
      ps<<"pitch: "<<pitch_angle*57.3<< std::endl <<"roll: " << roll_angle*57.3 <<std::endl <<"yaw: " << yaw_angle*57.3 <<std::endl;
      mpGLWindow->PrintString(CVD::ImageRef(10,550), ps.str(), 10);

	    

    }

    for(ImageBWMap::iterator it = mmFramesBW.begin(); it != mmFramesBW.end(); it++, nCamNum++)
    {
        TrackerCalib* pTracker = mmTrackers[it->first];
        if(allCamsGotGrid)
		{

		pTracker->bothGridVisible = true;   
		}
		else
		{
		pTracker->bothGridVisible = false;   	
		}

		//draw the status borders
		pTracker->DrawStatusBorder();

    }

    //  ROS_INFO("PointCollector: Grid visible in all cameras");

    bool bothSuccess = true;
    if(bGrabPoints && allCamsGotGrid) //if we've triggered to grab points, both cameras found grids 
    {
      
      for(unsigned int i=0; i<vTrackers.size(); i++)
      {
        TrackerCalib* pCurrentTrackerFrame = vTrackers[i];
        double pitch_angle;
        double roll_angle;
        double yaw_angle;
        
        GC->get_gimbal_angles_current(roll_angle,pitch_angle,yaw_angle);
        // Call map maker's ComputeGridPoints
       // ROS_INFO_STREAM("Computing Points for camera: " << pCurrentTrackerFrame->mCamName);
        bool bSuccess = mpMapMaker->ComputeGridPoints(*(pCurrentTrackerFrame->mpCalibImage), mdSquareSize, pCurrentTrackerFrame->mCamName, pCurrentTrackerFrame->mpCurrentMKF->mse3BaseFromWorld,roll_angle,pitch_angle,yaw_angle, capture_index,pixel_err_thresh);
       	ros::Duration(0.5).sleep(); // sleep for half a second
       	vPxErr[i] = sqrt(mpMapMaker->mdMeanChiSquared);
       	vSuccess[i] = bSuccess;
       	
       //reset after all info has been dumped
        if(!bSuccess) //failure in getting tracker pose
        {
          ROS_ERROR_STREAM("PointCollector: Failed to get tracker pose");
          bothSuccess = false;
        }
       
        bGrabPoints = false;
        mpMapMaker->RequestReset();

      }

      
      if(bothSuccess)
      {
      	 // Flash screen white to indicate capture
      	glClearColor(1,1,1,0.5);
      	glClear(GL_COLOR_BUFFER_BIT);
      	capture_index++;
      }
      else
      {
      	glClearColor(1,0,0,0.5);
      	glClear(GL_COLOR_BUFFER_BIT);
      }

      	
    }

    for(int i=0;i<2;i++)
    {
          double px_err = vPxErr[i] ;
          std::stringstream es;
	   	  es<<"last capture RMS pixel error: " << px_err;

	   	  if(vSuccess[i])
	   	  {
	   	  	glColor3f(0.0,1.0,0); //display green
	   	  	mpGLWindow->PrintString(vOffset[i]+ CVD::ImageRef(10,60), es.str(), 10);
	   	  }
	   	  else{
	   	  	glColor3f(1.0,0.0,0); //display red
	   	  	mpGLWindow->PrintString(vOffset[i]+ CVD::ImageRef(10,60), es.str(), 10);
	   	  }

	}
    vOffset.clear();
    
    

    
    qTotalDurations.push_back(ros::Time::now() - start);
    if(qTotalDurations.size() > nMaxQueueSize)
      qTotalDurations.pop_front();
      
    qLoopTimes.push(ros::Time::now());
    if(qLoopTimes.size() > nMaxQueueSize)
      qLoopTimes.pop();
      
    PublishPoses();
  }  // end if got new frame
  
    
  
  return captionStream.str();
  
}


// Deals with user interface commands
void PointCollector::GUICommandHandler(std::string command, std::string params) 
{

  
  if(command=="exit" || command=="quit")
  {
    mbDone = true;
    return;
  }
  
   if(command=="Reset")
  {
    mpMapMaker->RequestReset();
    
    for(TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
      it->second->Reset(false);
    
    return;
  }
  
  // KeyPress commands are issued by GLWindow
  if(command=="KeyPress")
   {

    if(params == "Space")
    {
      //for(TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
       // it->second->RequestInit(false);
      ROS_INFO("Capturing Grid Points");
      bGrabPoints = true;

      //dump the camera params to a file
      DumpCamerasToFile("/home/adas/mcptam_output/camparams.txt");
    }
    
    else if(params == "q" || params == "Escape")
    {
      mbDone = true;
    }

    //keypresses for moving ptu unit 
    /*else if(params == "i" ) //tilt up
    {
        ROS_INFO_STREAM("p uppppppppppppppppppppppppp");
        double current_pan_angle_setpoint;
        double current_tilt_angle_setpoint;
        PTC->get_pan_tilt_angle_setpoint(current_pan_angle_setpoint,current_tilt_angle_setpoint);
        double tilt_angle_setpoint = current_tilt_angle_setpoint + pan_tilt_angle_increment;
        PTC->set_pan_tilt_setpoint(current_pan_angle_setpoint, tilt_angle_setpoint);

    }
    else if(params == "k" ) //tilt down
    {
        double current_pan_angle_setpoint;;
        double current_tilt_angle_setpoint;
        PTC->get_pan_tilt_angle_setpoint(current_pan_angle_setpoint,current_tilt_angle_setpoint);
        double tilt_angle_setpoint = current_tilt_angle_setpoint - pan_tilt_angle_increment;
        PTC->set_pan_tilt_setpoint(current_pan_angle_setpoint, tilt_angle_setpoint);
    }
    else if(params == "j" ) //pan left
    {
        double current_pan_angle_setpoint;
        double current_tilt_angle_setpoint;
        PTC->get_pan_tilt_angle_setpoint(current_pan_angle_setpoint,current_tilt_angle_setpoint);
        double pan_angle_setpoint = current_pan_angle_setpoint + pan_tilt_angle_increment;
        PTC->set_pan_tilt_setpoint(pan_angle_setpoint, current_tilt_angle_setpoint);

      
    }
    else if(params == "l" ) //pan right
    {
        double current_pan_angle_setpoint;
        double current_tilt_angle_setpoint;
        PTC->get_pan_tilt_angle_setpoint(current_pan_angle_setpoint,current_tilt_angle_setpoint);
        double pan_angle_setpoint = current_pan_angle_setpoint - pan_tilt_angle_increment;
        PTC->set_pan_tilt_setpoint(pan_angle_setpoint, current_tilt_angle_setpoint);
      
    }*/



    return;
  }
      
  ROS_FATAL_STREAM("System: Unhandled command in GUICommandHandler: " << command);
  ros::shutdown();
}

bool PointCollector::ResetSystemCallback(mcptam::Reset::Request &request, mcptam::Reset::Response &response)
{
  ROS_INFO("PointCollector: In reset callback, calling reset on individual trackers");
  for(TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
    it->second->Reset(false);
  
  return true;
}

void PointCollector::PublishPoses()
{
  static tf::TransformBroadcaster br;
  
  for(TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
  {
    std::string camName = it->first;
    TrackerCalib* pTracker = it->second;
    geometry_msgs::Pose poseMsg;
    poseMsg = util::SE3ToPoseMsg(pTracker->GetCurrentPose().inverse());
    
    
    tf::Transform transform;
    tf::poseMsgToTF(poseMsg, transform);
    br.sendTransform(tf::StampedTransform(transform, pTracker->GetCurrentTimestamp(), "vision_world", camName));
  }
}
