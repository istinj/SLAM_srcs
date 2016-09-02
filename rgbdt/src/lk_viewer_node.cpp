#include "txt_io/message_reader.h"
#include "txt_io/message_seq_synchronizer.h"
#include "txt_io/message_enlister_trigger.h"
#include "txt_io/pinhole_image_message.h"
#include "ros_wrappers/image_message_listener.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include "rgbdt/FrameInfoMessage.h"
#include "globals/system_utils.h"
#include <cstring>
#include "opencv2/opencv.hpp"
#include "lk_tracker.h"
#include "lk_viewer.h"
#include "qapplication.h"
#include <pthread.h>
#include "lk_ros_bridge.h"

using namespace std;
using namespace txt_io;
using namespace ros_wrappers;
using namespace system_utils;
using namespace rgbdt;

int max_size=1000;
std::list<FrameInfo*> frame_info_list;
void processFrameInfo(const rgbdt::FrameInfoMessageConstPtr& msg) {
  FrameInfo* frame_info = new FrameInfo;
  msg2frameInfo(*frame_info, *msg);
  frame_info_list.push_back(frame_info);
  while (frame_info_list.size()>max_size) {
    FrameInfo* killed_frame=frame_info_list.front();
    frame_info_list.pop_front();
    delete killed_frame;
  }
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "lk_client");
  ros::NodeHandle nh;

  ros::Subscriber sub=nh.subscribe("frame_info", 100, processFrameInfo);
  bool use_gui=true;
    /*
  image_transport::ImageTransport itr(nh);
  std::string topic1=argv[1];
  std::string topic2=argv[2];
  ImageMessageListener *listener1 = 
    new ImageMessageListener(&nh,  &itr, &sorter,0,"","");
  ImageMessageListener *listener2 = 
    new ImageMessageListener(&nh,  &itr, &sorter,0,"","");

  listener1->subscribe(topic1);
  listener2->subscribe(topic2);
    */

  QApplication* app=0;
  LKViewer* viewer=0;
  if (use_gui){
    app=new QApplication(argc, argv);
    viewer = new LKViewer;
    viewer->_frame_infos=&frame_info_list;
    viewer->show();
  }

  ros::Rate loop_rate(30);
  while (ros::ok()){
    loop_rate.sleep();
    viewer->updateGL();
    app->processEvents();
    ros::spinOnce();
  }
}
