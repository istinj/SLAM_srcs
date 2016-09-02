#include "txt_io/message_reader.h"
#include "txt_io/message_seq_synchronizer.h"
#include "txt_io/base_image_message.h"
#include "txt_io/pinhole_image_message.h"
#include "txt_io/message_enlister_trigger.h"

#include "globals/system_utils.h"
#include <cstring>
#include "opencv2/opencv.hpp"
#include "lk_tracker.h"
#include "lk_viewer.h"
#include "qapplication.h"

using namespace system_utils;
using namespace std;
using namespace txt_io;
using namespace rgbdt;


const char* banner[]={
  "lk_app: a simple example on reading a dump file written with txt io",
  " it reads sequentially all elements in the file and dowd rgb+depth tracking of features",
  "",
  "usage: lk_app  <dump_file>",
  0
};

MessageSeqSynchronizer synchronizer;
LKTracker tracker(200);
SensorMessageList message_list;
FrameInfoManager manager;

void callTracker(){
  if (synchronizer.messagesReady()){
    PinholeImageMessage *rgb=dynamic_cast <PinholeImageMessage*>(synchronizer.messages()[0].get());
    PinholeImageMessage *depth=dynamic_cast <PinholeImageMessage*>(synchronizer.messages()[1].get());
    rgb->untaint();
    depth->untaint();
    tracker.addImage(rgb->image(), depth->image(), rgb->seq(), depth->cameraMatrix());
    FrameInfo* finfo=new FrameInfo(tracker.currentFrameInfo());
    manager.putFrame(finfo, tracker.currentGray(), tracker.currentDepth());
  }
}

int main(int argc, char ** argv) {
  if (argc<1 || ! strcmp(argv[1],"-h")) {
    printBanner(banner);
    return 0;
  }
  bool use_gui=true;
  MessageReader reader;
  reader.open(argv[1]);
  std::string topic1="/camera/rgb/image_raw";
  std::string topic2="/camera/depth/image_raw";
  
  std::vector<std::string> sync_topics;
  sync_topics.push_back(topic1);
  sync_topics.push_back(topic2);

  synchronizer.setTopics(sync_topics);
  QApplication* app=0;
  LKViewer* viewer=0;
  if (use_gui){
    app=new QApplication(argc, argv);
    viewer = new LKViewer(&manager);
    viewer->show();
    viewer->_closures=&manager.closures;
  }
  BaseMessage* msg=0;
  int run=false;
  while ( (msg = reader.readMessage()) ) {
    BaseSensorMessage* sensor_msg=dynamic_cast<BaseSensorMessage*>(msg);
    if (! sensor_msg)
      delete msg;
    if (sensor_msg->topic()==topic1) {
      synchronizer.putMessage(sensor_msg);
    } else if (sensor_msg->topic()==topic2) {
      synchronizer.putMessage(sensor_msg);
    } else
      delete sensor_msg;
    if (synchronizer.messagesReady()) {
      callTracker();
      synchronizer.reset();
      if (use_gui){
	tracker.drawFeatures();
	tracker.drawCorrespondences();
	tracker.drawSolverState();
	imshow("LK Demo", tracker.currentFrame());
	int key=0;
	if (! run) {
	  key=(char) cv::waitKey();
	} else
	  cv::waitKey(10);
	
	switch (key) {
	case 'c':
	  run=!run;
	  break;

	case 'p': 
	  viewer->_points_drawn=!viewer->_points_drawn; 
	  cerr << "points:"  << viewer->_points_drawn;
	  break;
	case 's': 
	  viewer->_camera_pov=!viewer->_camera_pov; 
	  cerr << "camera:"  << viewer->_camera_pov;
	  break;
	case 'k': 
	  viewer->_keyframes_drawn=!viewer->_keyframes_drawn; 
	  cerr << "kf:"  << viewer->_keyframes_drawn;
	  break;
	case 'i': 
	  viewer->_intermediate_frames_drawn=!viewer->_intermediate_frames_drawn; 
	  cerr << "i:"  << viewer->_intermediate_frames_drawn;
	  break;
	case 'm':
	  viewer->_map_drawn=!viewer->_map_drawn; 
	  cerr << "m:"  << viewer->_map_drawn;
	  break;
	case 'n':
	  viewer->_normals_drawn=!viewer->_normals_drawn; 
	  cerr << "n:"  << viewer->_normals_drawn;
	  break;
	}
	viewer->updateGL();
	app->processEvents();
      }
    }
  }
  if (use_gui)
    app->exec();
}
