#pragma once
#include "lk_tracker.h"
#include "simple_viewer.h"
#include "frame_info_manager.h"

namespace rgbdt {

  class LKViewer: public SimpleViewer{
  public:
    LKViewer(FrameInfoManager* manager);
    void drawFrameInfo(const FrameInfo* finfo, bool draw_points=false);
    void drawKeyFrame(const KeyFrame* kframe, 
		      const Eigen::Vector3f& color=Eigen::Vector3f(0.5, 1, 1));
    void drawMap();
    void drawKeyFrameMap();
    virtual void draw();
    //protected:
    bool _points_drawn;
    bool _keyframes_drawn;
    bool _intermediate_frames_drawn;
    bool _normals_drawn;
    bool _camera_pov;
    bool _map_drawn;
    FrameInfoManager* _manager;
    std::set<int>* _closures;
    FrameMatchList _matches;
  };

}
