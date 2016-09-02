#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "point_track.h"
#include <opencv2/features2d/features2d.hpp>

namespace rgbdt {

  /** A FrameInfo comprises all the information about a features extracted from one image, 
      as well as the tracker status

      It has:
      - a unique sequence number
      - the sequence number of the previous FrameInfo
      - frame_id/timestamp/camera_matrix
      - global position of the camera
      - bookkeeping informations to keep track of what's happening
      - a collection of features
      - a tracker status, used to output the status of the tracker to modules that are cascaded to it.
   */

  struct FrameInfo{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameInfo();
    void reset();
    FrameInfo(int seq, 
	      int parent_seq, 
	      const std::string& frame_id,
	      double stamp,
	      const Eigen::Matrix3f& camera_matrix=Eigen::Matrix3f::Identity(),
	      const Eigen::Isometry3f& global_t=Eigen::Isometry3f::Identity());
    void toCvPoints(std::vector<cv::Point2f>& cv_points, int status_mask=0);
    void fromCvPoints(const std::vector<cv::Point2f>& cv_points, bool add_points);

    void toKeyPoints(std::vector<cv::KeyPoint>& cv_keypoints, int status_mask=0);
    void fromKeyPoints(const std::vector<cv::KeyPoint>& cv_keypoints, bool add_points);

    int epoch; //< number of restarts of the tracker. An epoch is a time intervall where the tracker never breaks
    int seq;   //< sequence number
    int parent_seq; //< sequence number of the previous frame
    int keyframe_seq; //< sequence number of the last keyframe. if seq==keyframe_seq, the instance is a keyframe
    int parent_keyframe_seq; //< keyframe before the current frame
    std::string frame_id; //< ROS string telling the frame_id where the camera is mounted
    double stamp; //< ROS timestamp
    Eigen::Matrix3f camera_matrix; //< 3x3 camera matrix
    Eigen::Isometry3f global_T;    //< global transform of the camera
    Eigen::Isometry3f keyframe_T;  //< transform of the current frame w.r.t. the last keyframe
    bool transform_good;           //< true if the tracker behaves well
    int tracker_status;            //< more articulated information of the tracker (See lk_tracker.h)
    PointTrackVector points;       //< vector of tracks
  };    

  class FrameInfoMap: public std::map<int, FrameInfo*> {
  public:
    FrameInfo* get(int seq);
    PointTrack* getPoint(int seq, int point_index);
    void put(FrameInfo* finfo);
  };

}
