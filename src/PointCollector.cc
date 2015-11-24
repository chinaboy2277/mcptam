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
#include <gvars3/instances.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <fstream>

using namespace GVars3;
using namespace TooN;

bool bGrabPoints = false;

PointCollector::PointCollector()
: SystemBase("PointCollector",true, true)
{
  if(mpVideoSourceMulti->GetNumGroups() != 1)
  {
    ROS_FATAL_STREAM("PointCollector: All cameras need to be triggered together in one group.");
    ROS_FATAL_STREAM("PointCollector: Currently you have "<<mpVideoSourceMulti->GetNumGroups()<<" camera groups.");
    ros::shutdown();
    return;
  }
  
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
  }
  
}

// Perform all tracking operations
std::string PointCollector::Track()
{
  // This data will be displayed on GUI
  static std::queue<ros::Time> qLoopTimes;
  static std::deque<ros::Duration> qTotalDurations;
  static unsigned int nMaxQueueSize = 10;
      
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
    bool processedCamera[2] = {0}; //TODO: Generalize to the total number of rig cameras
    
    // Go through all the images that we got
    for(ImageBWMap::iterator it = mmFramesBW.begin(); it != mmFramesBW.end(); it++, nCamNum++)
    {
      TrackerCalib* pTracker = mmTrackers[it->first];
      
      // If this tracker has not yet found a checkerboard
      pTracker->TrackFrame(it->second, timestamp, !bDrawKeyFrames, true);
      //std::cout<<pTracker->meCheckerboardStage << std::endl;
      
      
      // Print sequential number as overlay
      glColor3f(0,1.0,0);
      CVD::ImageRef irOffset = pTracker->mmDrawOffsets[pTracker->mCamName];
      std::stringstream ss;
      ss<<"#"<<nCamNum;
      mpGLWindow->PrintString(irOffset + CVD::ImageRef(10,40), ss.str(), 10);

      //if(pTracker->meCheckerboardStage == TrackerCalib::CHECKERBOARD_SECOND_STAGE)
      //{
      //  ROS_INFO("found board");
     // }
      
      bool foundGrid = pTracker->foundGridPattern;

      if(foundGrid)
        ROS_INFO_STREAM("camera " <<  pTracker->mCamName << " found a grid");

      if(bGrabPoints && foundGrid && !processedCamera[nCamNum]) //if we've triggered to grab points, the camera has found a grid, and we haven't already processed this camera 
      {
      //std::string  CamName = pTracker->mCamName; 
      //TrackerCalib* pFirstTracker = mmTrackers.begin()->second;
      ROS_INFO_STREAM("computing points and tracker pose for " << pTracker->mCamName);
        
        // Call map maker's init from calib image
      bool bSuccess = mpMapMaker->ComputeGridPoints(*(pTracker->mpCalibImage), mdSquareSize, pTracker->mCamName, pTracker->mpCurrentMKF->mse3BaseFromWorld);
      
      //reset after all info has been dumped
      mpMapMaker->RequestReset();
      //flag that we've serviced this camera
      processedCamera[nCamNum] = true;

      }
    }

    bGrabPoints = false;
    
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

  std::cout<<command<<std::endl;

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
    else
    {
      int nResetNum = atoi(params.c_str());
      
      // Only cameras #2 and greater can be reset individually. If you tried to 
      // reset camera #1, it would destroy the fixed points extracted
      // from the checkerboard, and calibration would be impossible.
      
      if(nResetNum > 1)  // reset the corresponding tracker
      {
        TrackerCalibPtrMap::iterator it = std::next(mmTrackers.begin(), nResetNum-1);
        TrackerCalib* pTracker = it->second;
        std::string camName = it->first;
        ROS_INFO_STREAM("Resetting "<<camName<<" tracker");
        pTracker->Reset(false);
        mpMapMaker->PauseRun();
        mpMapMaker->RemoveMultiKeyFrames(camName, false);
        mpMapMaker->ResumeRun();
      }
    }
    return;
  }
  
  if(command=="InitTracker")
  {
    for(TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
        it->second->RequestInit(false);
        
    return;
  }
  
  if(command=="SaveCalib")
  {
    mbDone = mpVideoSourceMulti->SavePoses(mmFinalPoses);
    
    // If we want to output poses for processing in another program do it now
    std::string poseFileName;
    if(mNodeHandlePriv.getParam("pose_out_file", poseFileName))
    {
      // Write kf poses to file
      std::ofstream file(poseFileName.c_str());
      if(!file.is_open())
      {
        ROS_ERROR_STREAM("Could not open "<<poseFileName<<" for writing poses");
      }
      else
      {
        ROS_INFO_STREAM("Writing poses to "<<poseFileName);
        
        for(SE3Map::iterator it = mmFinalPoses.begin(); it != mmFinalPoses.end(); it++)
        {
          std::string camName = it->first;
          TooN::SE3<> se3Pose = (it->second).inverse();
          file<<camName<<std::endl;
          file<<se3Pose<<std::endl;
        }
        file.close();
      }
    }
      
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
