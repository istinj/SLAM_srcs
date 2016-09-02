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

const char* banner[]={
  "lk_node <rgb_topic> <depth_topic>",
  0
};

MessageSeqSynchronizer synchronizer;
SensorMessageList message_list;
SensorMessageSorter sorter;
MessageEnlisterTrigger enlister(&sorter,1,&message_list);
LKTracker tracker(500);
ros::Publisher pub;

FrameInfoMessage pub_msg;

int last_seq=0;
void processMessages() {
  while (!message_list.empty()){
    synchronizer.putMessage(message_list.front());
    std::tr1::shared_ptr<txt_io::BaseSensorMessage> msg=message_list.front();
    msg->untaint();
    if (synchronizer.messagesReady()) {
      PinholeImageMessage *rgb=dynamic_cast <PinholeImageMessage*>(synchronizer.messages()[0].get());
      PinholeImageMessage *depth=dynamic_cast <PinholeImageMessage*>(synchronizer.messages()[1].get());     tracker.addImage(rgb->image(), depth->image(), rgb->seq(), depth->cameraMatrix());
      synchronizer.reset();
      if (rgb->seq()-last_seq>1) {
	cerr << "WARNING, skipping " << rgb->seq()-last_seq << " frames" << endl;
      }
      last_seq = rgb->seq();
      frameInfo2Msg(pub_msg, tracker.currentFrameInfo());
      pub.publish(pub_msg);
    }
    message_list.pop_front();
  }
}


int main(int argc, char ** argv) {
  if (argc<3 || ! strcmp(argv[1],"-h")) {
    printBanner(banner);
    return 0;
  }
  sorter.setTimeWindow(0.001);
  ros::init(argc, argv, "lk_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport itr(nh);
  
  std::string topic1=argv[1];
  std::string topic2=argv[2];
  ImageMessageListener *listener1 = 
    new ImageMessageListener(&nh,  &itr, &sorter,0,"","");
  ImageMessageListener *listener2 = 
    new ImageMessageListener(&nh,  &itr, &sorter,0,"","");
  
  listener1->subscribe(topic1);
  listener2->subscribe(topic2);


  std::vector<std::string> sync_topics;
  sync_topics.push_back(topic1);
  sync_topics.push_back(topic2);
  synchronizer.setTopics(sync_topics);
  pthread_t tracker_thread;
  pub=nh.advertise<rgbdt::FrameInfoMessage>("frame_info", 10);
  ros::Rate loop_rate(30);
  while (ros::ok()){
    processMessages();
    loop_rate.sleep();
    ros::spinOnce();
  }
}
