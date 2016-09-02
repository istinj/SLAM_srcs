#include "frame_info.h"
#include <opencv2/features2d/features2d.hpp>
#include <stdexcept>

namespace rgbdt {
  using namespace std;

  FrameInfo::FrameInfo() {
    reset();
  }
  

  void FrameInfo::reset() {
    seq=-1;
    parent_seq=-1;
    keyframe_seq=-1;
    parent_keyframe_seq=-1;
    frame_id="";
    stamp=0;
    camera_matrix.setIdentity();
    global_T.setIdentity();
    keyframe_T.setIdentity();
    transform_good=false;
    epoch=0;
  }

  FrameInfo::FrameInfo(int seq_, 
	    int parent_seq_, 
	    const std::string& frame_id_,
	    double stamp_,
	    const Eigen::Matrix3f& camera_matrix_,
	    const Eigen::Isometry3f& global_T_):
    seq(seq_),
    parent_seq(parent_seq_),
    keyframe_seq(-1),
    parent_keyframe_seq(-1),
    frame_id(frame_id_),
    stamp(stamp_),
    camera_matrix(camera_matrix_),
    global_T(global_T_){
    transform_good=false;
    epoch=0;
  }
  
void FrameInfo::toCvPoints(std::vector<cv::Point2f>& cv_points, int status_mask) {
    cv_points.resize(points.size());
    int k=0;
    for(size_t i=0; i<points.size(); i++){
      if(! status_mask || (points[i].status&status_mask)) {
	cv_points[k].x=points[i].image_coordinates.x();
	cv_points[k].y=points[i].image_coordinates.y();
	k++;
      }
    }
    cv_points.resize(points.size());
  }

  void FrameInfo::fromCvPoints(const std::vector<cv::Point2f>& cv_points, bool add_points){
    size_t k=0;
    if (add_points){
      k=points.size();
      points.resize(points.size()+cv_points.size());
    } else {
      points.resize(cv_points.size());
    }
    for (size_t i=0; i<cv_points.size(); ++i, ++k){
      points[k].reset();
      points[k].image_coordinates=Eigen::Vector3f(cv_points[i].x, cv_points[i].y, 1);
      points[k].status|=PointTrack::Created;
    }
  }

  void FrameInfo::fromKeyPoints(const std::vector<cv::KeyPoint>& cv_keypoints, bool add_points) {
    size_t k=0;
    if (add_points){
      k=points.size();
      points.resize(points.size()+cv_keypoints.size());
    } else {
      points.resize(cv_keypoints.size());
    }
    for (size_t i=0; i<cv_keypoints.size(); ++i, ++k){
      points[k].reset();
      points[k].size=cv_keypoints[i].size;
      points[k].angle=cv_keypoints[i].angle;
      points[k].response=cv_keypoints[i].response;
      points[k].octave=cv_keypoints[i].octave;
      points[k].image_coordinates=Eigen::Vector3f(cv_keypoints[i].pt.x, cv_keypoints[i].pt.y, 0);
      points[k].status|=PointTrack::Created;
    }
  }

  void FrameInfo::toKeyPoints(std::vector<cv::KeyPoint>& cv_keypoints, int status_mask) {
    cv_keypoints.resize(points.size());
    int k=0;
    for(size_t i=0; i<points.size(); i++){
        if(! status_mask || (points[i].status&status_mask) ) {
	cv_keypoints[k].pt.x=points[i].image_coordinates.x();
	cv_keypoints[k].pt.y=points[i].image_coordinates.y();
	cv_keypoints[k].size=points[i].size;
	cv_keypoints[k].angle=points[i].angle;
	cv_keypoints[k].response=points[i].response;
	cv_keypoints[k].octave=points[i].octave;
	k++;
      }
    }
    cv_keypoints.resize(k);
  }


  FrameInfo* FrameInfoMap::get(int seq) {
    iterator it=find(seq);
    if (it==end())
      return 0;
    return it->second;
  }

  PointTrack* FrameInfoMap::getPoint(int seq, int point_index) {
    FrameInfo* finfo=get(seq);
    if (! finfo)
      return 0;
    if (point_index<0||point_index>=finfo->points.size()) {
      throw std::runtime_error("error, no point index in frame info");
    }
    return &finfo->points[point_index];
  }

  void FrameInfoMap::put(FrameInfo* finfo) {
    FrameInfo* element=get(finfo->seq);
    if (! element) {
      insert(make_pair(finfo->seq, finfo));
      return;
    }
    if (element==finfo)
	return;
    throw std::runtime_error("error, duplicate insertion");
  }

}
