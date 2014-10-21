///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include "Client.h"
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <vector>
#include <string.h>

#include <time.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

using namespace ViconDataStreamSDK::CPP;
using namespace std;
using namespace pcl;
namespace
{
  string Adapt( const bool i_Value )
  {
    return i_Value ? "True" : "False";
  }

  string Adapt( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
      return "Forward";
      case Direction::Backward:
      return "Backward";
      case Direction::Left:
      return "Left";
      case Direction::Right:
      return "Right";
      case Direction::Up:
      return "Up";
      case Direction::Down:
      return "Down";
      default:
      return "Unknown";
    }
  }

  string Adapt( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
      return "ForcePlate";
      case DeviceType::Unknown:
      default:
      return "Unknown";
    }
  }

  string Adapt( const Unit::Enum i_Unit )
  {
    switch( i_Unit )
    {
      case Unit::Meter:
      return "Meter";
      case Unit::Volt:
      return "Volt";
      case Unit::NewtonMeter:
      return "NewtonMeter";
      case Unit::Newton:
      return "Newton";
      case Unit::Kilogram:
      return "Kilogram";
      case Unit::Second:
      return "Second";
      case Unit::Ampere:
      return "Ampere";
      case Unit::Kelvin:
      return "Kelvin";
      case Unit::Mole:
      return "Mole";
      case Unit::Candela:
      return "Candela";
      case Unit::Radian:
      return "Radian";
      case Unit::Steradian:
      return "Steradian";
      case Unit::MeterSquared:
      return "MeterSquared";
      case Unit::MeterCubed:
      return "MeterCubed";
      case Unit::MeterPerSecond:
      return "MeterPerSecond";
      case Unit::MeterPerSecondSquared:
      return "MeterPerSecondSquared";
      case Unit::RadianPerSecond:
      return "RadianPerSecond";
      case Unit::RadianPerSecondSquared:
      return "RadianPerSecondSquared";
      case Unit::Hertz:
      return "Hertz";
      case Unit::Joule:
      return "Joule";
      case Unit::Watt:
      return "Watt";
      case Unit::Pascal:
      return "Pascal";
      case Unit::Lumen:
      return "Lumen";  
      case Unit::Lux:
      return "Lux";
      case Unit::Coulomb:
      return "Coulomb";
      case Unit::Ohm:
      return "Ohm";
      case Unit::Farad:
      return "Farad";
      case Unit::Weber:
      return "Weber";
      case Unit::Tesla:
      return "Tesla";
      case Unit::Henry:
      return "Henry";
      case Unit::Siemens:
      return "Siemens";
      case Unit::Becquerel:
      return "Becquerel";
      case Unit::Gray:
      return "Gray";
      case Unit::Sievert:
      return "Sievert";
      case Unit::Katal:
      return "Katal";

      case Unit::Unknown:
      default:
      return "Unknown";
    }
  }

}
PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>);
PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>);
boost::shared_ptr<visualization::CloudViewer> viewer;
Grabber* kinectGrabber;
unsigned int filesSaved = 0;
bool saveCloud(false), noColour(false);
Client MyClient;



void
printUsage(const char* programName)
{
  cout << "Usage: " << programName << " [options]"
  << endl
  << endl
  << "Options:\n"
  << endl
  << "\t<none>     start capturing from a Kinect device.\n"
  << "\t-v NAME    visualize the given .pcd file.\n"
  << "\t-h         shows this help.\n";
}


#define output_stream cout 
Eigen::Matrix4f gimmeDaFrame(Client &MyClient){

      // Get a frame

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  while( MyClient.GetFrame().Result != Result::Success )
  {
        // Sleep a little so that we don't lumber the CPU with a busy poll

    usleep(1000000);

    output_stream << ".";
  }
  output_stream << endl;


      // Get the frame number
  Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();

      // Get the timecode
  Output_GetTimecode _Output_GetTimecode  = MyClient.GetTimecode();


  for( unsigned int LatencySampleIndex = 0 ; LatencySampleIndex < MyClient.GetLatencySampleCount().Count ; ++LatencySampleIndex )
  {
    string SampleName  = MyClient.GetLatencySampleName( LatencySampleIndex ).Name;
    double      SampleValue = MyClient.GetLatencySampleValue( SampleName ).Value;


  }


      // Count the number of subjects
  unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;

  for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
  {
    string SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
    string str2 ("scaner_xtion");
    if (SubjectName.compare(str2) !=0){

      continue;
    }


        // Get the root segment
    string RootSegment = MyClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;


        // Count the number of segments
    unsigned int SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;

    for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
    {


          // Get the segment name
      string SegmentName = MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;


          // Get the segment parent
      string SegmentParentName = MyClient.GetSegmentParentName( SubjectName, SegmentName ).SegmentName;


          // Get the segment's children
      unsigned int ChildCount = MyClient.GetSegmentChildCount( SubjectName, SegmentName ).SegmentCount;

      for( unsigned int ChildIndex = 0 ; ChildIndex < ChildCount ; ++ChildIndex )
      {
        string ChildName = MyClient.GetSegmentChildName( SubjectName, SegmentName, ChildIndex ).SegmentName;

      }


          // Get the global segment translation
      Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = 
      MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );

transform_1 (0,3)=_Output_GetSegmentGlobalTranslation.Translation[ 0 ]*0.001;
transform_1 (1,3)=_Output_GetSegmentGlobalTranslation.Translation[ 1 ]*0.001;
transform_1 (2,3)=_Output_GetSegmentGlobalTranslation.Translation[ 2 ]*0.001;


          // Get the global segment rotation as a matrix
Output_GetSegmentGlobalRotationMatrix _Output_GetSegmentGlobalRotationMatrix = 
MyClient.GetSegmentGlobalRotationMatrix( SubjectName, SegmentName );

transform_1(0,0)= _Output_GetSegmentGlobalRotationMatrix.Rotation[ 0 ];        
transform_1(0,1)= _Output_GetSegmentGlobalRotationMatrix.Rotation[ 1 ]; 
transform_1(0,2)= _Output_GetSegmentGlobalRotationMatrix.Rotation[ 2 ]; 

transform_1(1,0)= _Output_GetSegmentGlobalRotationMatrix.Rotation[ 3 ];        
transform_1(1,1)= _Output_GetSegmentGlobalRotationMatrix.Rotation[ 4 ]; 
transform_1(1,2)= _Output_GetSegmentGlobalRotationMatrix.Rotation[ 5 ]; 

transform_1(2,0)= _Output_GetSegmentGlobalRotationMatrix.Rotation[ 6 ];        
transform_1(2,1)= _Output_GetSegmentGlobalRotationMatrix.Rotation[ 7 ]; 
transform_1(2,2)= _Output_GetSegmentGlobalRotationMatrix.Rotation[ 8 ]; 

}


}
return transform_1;
}
void
grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
  if (! viewer->wasStopped())
    viewer->showCloud(cloud);

  if (saveCloud)
  {
   
   gimmeDaFrame(MyClient);
   gimmeDaFrame(MyClient);
   Eigen::Matrix4f transform_1=gimmeDaFrame(MyClient);
   
   if (transform_1(0,3)!=0){
    cout << transform_1 <<endl;


     Eigen::Matrix4f transform_2= Eigen::Matrix4f::Identity();
     transform_2 (0,3)=-0.017;
     transform_2 (1,3)=0.011;
     transform_2 (2,3)=-0.115;

     transform_2(0,0)=0;
     transform_2(0,1)=-1;
     transform_2(0,2)=0;
     transform_2(1,0)=-1;
     transform_2(1,1)=0;
     transform_2(1,2)=0;
     transform_2(2,0)=0;
     transform_2(2,1)=0;
     transform_2(2,2)=-1;
     cout << transform_2 <<endl;
     

     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA> ());
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA> ());


     pcl::transformPointCloud (*cloud, *transformed_cloud1, transform_2);
     pcl::transformPointCloud (*transformed_cloud1, *transformed_cloud2, transform_1);
     stringstream stream;
     stream << "inputCloud" << filesSaved << ".pcd";
     string filename = stream.str();
     if (io::savePCDFile(filename, *transformed_cloud2, true) == 0)
     {
      filesSaved++;
      cout << "Saved " << filename << "." << endl;
    }
    else PCL_ERROR("Problem saving %s.\n", filename.c_str());

    saveCloud = false;
  }else{
    saveCloud = false;
    cout<<"no data from vicon"<<endl;
  }
}
}

void
keyboardEventOccurred(const visualization::KeyboardEvent& event,
  void* nothing)
{
  if (event.getKeySym() == "space" && event.keyDown())
    saveCloud = true;

}

boost::shared_ptr<visualization::CloudViewer>
createViewer()
{
  boost::shared_ptr<visualization::CloudViewer> v
  (new visualization::CloudViewer("3D Viewer"));
  v->registerKeyboardCallback(keyboardEventOccurred);

  return(v);
}






void connectToVicon(string hostname){
  string HostName = hostname;
  
  
  // Make a new client
    /*Client MyClient;*/


    // Connect to a server
  cout << "Connecting to " << HostName << " ..." << flush;
  while( !MyClient.IsConnected().Connected )
  {
      // Direct connection
    bool ok = false;

    ok =( MyClient.Connect( HostName ).Result == Result::Success );

    if(!ok)
    {
      cout << "Warning - connect failed..." << endl;
    }


    cout << ".";

    usleep(1000000);

  }
  cout << endl;

    // Enable some different data types
  MyClient.EnableSegmentData();
  MyClient.EnableMarkerData();
  MyClient.EnableUnlabeledMarkerData();
  MyClient.EnableDeviceData();


    // Set the streaming mode
  MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
    // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
    // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

    // Set the global up axis
  MyClient.SetAxisMapping( Direction::Forward, 
   Direction::Left, 
                             Direction::Up ); // Z-up


  Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
  cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
  << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
  << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << endl;

    // Discover the version number
  Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
  cout << "Version: " << _Output_GetVersion.Major << "." 
  << _Output_GetVersion.Minor << "." 
  << _Output_GetVersion.Point << endl;



    size_t FrameRateWindow = 1000; // frames
    size_t Counter = 0;
    clock_t LastTime = clock();
    // return MyClient;
  }

  int main( int argc, char* argv[] )
  {
    if (console::find_argument(argc, argv, "-h") >= 0)
    {
      printUsage(argv[0]);
      return 0;
    }
    bool justVisualize(false);
    string filename;
    if (console::find_argument(argc, argv, "-v") >= 0)
    {
      if (argc != 3)
      {
        printUsage(argv[0]);
        return 0;
      }

      filename = argv[2];
      justVisualize = true;
    }
    else if (argc != 1)
    {
      printUsage(argv[0]);
      return 0;
    }
    
    cout<<"Conneting to Vicon"<<endl;
    connectToVicon("131.176.25.20:801");

    if (justVisualize)
    {
      try
      {
        io::loadPCDFile<PointXYZRGBA>(filename.c_str(), *cloudptr);
      }
      catch (PCLException e1)
      {
        try
        {
          io::loadPCDFile<PointXYZ>(filename.c_str(), *fallbackCloud);
        }
        catch (PCLException e2)
        {
          return -1;
        }

        noColour = true;
      }

      cout << "Loaded " << filename << "." << endl;
      if (noColour)
        cout << "This file has no RGBA colour information present." << endl;
    }
    else
    {
      kinectGrabber = new OpenNIGrabber();
      if (kinectGrabber == 0)
        return false;
      boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
      boost::bind(&grabberCallback, _1);
      kinectGrabber->registerCallback(f);
    }
    
    viewer = createViewer();
    
    if (justVisualize)
    {
      if (noColour)
        viewer->showCloud(fallbackCloud);
      else viewer->showCloud(cloudptr);
    }
    else kinectGrabber->start();
    
    while (! viewer->wasStopped())
      boost::this_thread::sleep(boost::posix_time::seconds(1));
    
    if (! justVisualize)
      kinectGrabber->stop();
    
    // Loop until a key is pressed
    



    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();

    // Disconnect and dispose
    int t = clock();
    cout << " Disconnecting..." << endl;
    MyClient.Disconnect();
    int dt = clock() - t;
    double secs = (double) (dt)/(double)CLOCKS_PER_SEC;
    cout << " Disconnect time = " << secs << " secs" << endl;


  }
