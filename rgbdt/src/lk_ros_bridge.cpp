#include "lk_ros_bridge.h"
#include <Eigen/Geometry>

namespace rgbdt {

  void msg2frameInfo (FrameInfo& dest, const FrameInfoMessage& src) {
    dest.frame_id=src.header.frame_id;
    dest.stamp =src.header.stamp.toSec();
    dest.seq=src.seq;
    dest.parent_seq=src.parent_seq;
    dest.global_T.translation().x()=src.global_T.position.x;
    dest.global_T.translation().y()=src.global_T.position.y;
    dest.global_T.translation().z()=src.global_T.position.z;
    

    Eigen::Quaternionf q;
    q.x()=src.global_T.orientation.x;
    q.y()=src.global_T.orientation.y;
    q.z()=src.global_T.orientation.z;
    q.w()=src.global_T.orientation.w;
    dest.global_T.linear()=q.toRotationMatrix();
    dest.points.resize(src.points.size());
    for (size_t i=0; i<src.points.size(); i++) {
      const PointTrackMessage& src_pt =src.points[i];
      PointTrack& dest_pt=dest.points[i];
      dest_pt.status=src_pt.status;
      dest_pt.image_coordinates.x()=src_pt.image_coordinates.x;
      dest_pt.image_coordinates.y()=src_pt.image_coordinates.y;
      dest_pt.image_coordinates.z()=src_pt.image_coordinates.z;


      dest_pt.camera_coordinates.x()=src_pt.camera_coordinates.x;
      dest_pt.camera_coordinates.y()=src_pt.camera_coordinates.y;
      dest_pt.camera_coordinates.z()=src_pt.camera_coordinates.z;

      dest_pt.world_coordinates.x()=src_pt.world_coordinates.x;
      dest_pt.world_coordinates.y()=src_pt.world_coordinates.y;
      dest_pt.world_coordinates.z()=src_pt.world_coordinates.z;

      dest_pt.reprojected_coordinates.x()=src_pt.reprojected_coordinates.x;
      dest_pt.reprojected_coordinates.y()=src_pt.reprojected_coordinates.y;
      dest_pt.reprojected_coordinates.z()=src_pt.reprojected_coordinates.z;

      dest_pt.depth_covariance=src_pt.depth_covariance;
      dest_pt.posit_error = src_pt.posit_error;
      dest_pt.parent_index = src_pt.parent_index;
      dest_pt.age = src_pt.age;
      dest_pt.landmark_id = src_pt.landmark_id;
    }
  }

  void frameInfo2Msg (FrameInfoMessage& dest, const FrameInfo& src) {
    dest.header.frame_id=src.frame_id;
    dest.header.stamp = ros::Time(src.stamp);
    dest.seq=src.seq;
    dest.parent_seq=src.parent_seq;
    dest.keyframe_seq=src.keyframe_seq;
    dest.global_T.position.x=src.global_T.translation().x();
    dest.global_T.position.y=src.global_T.translation().y();
    dest.global_T.position.z=src.global_T.translation().z();
    Eigen::Quaternionf q(src.global_T.linear());
    dest.global_T.orientation.x=q.x();
    dest.global_T.orientation.y=q.y();
    dest.global_T.orientation.z=q.z();
    dest.global_T.orientation.w=q.w();
    dest.transform_good=src.transform_good;
    dest.tracker_status=src.tracker_status;
    dest.points.resize(src.points.size());
    for (size_t i=0; i<src.points.size(); i++) {
      const PointTrack& src_pt =src.points[i];
      PointTrackMessage& dest_pt=dest.points[i];
      dest_pt.status=src_pt.status;
      dest_pt.image_coordinates.x=src_pt.image_coordinates.x();
      dest_pt.image_coordinates.y=src_pt.image_coordinates.y();
      dest_pt.image_coordinates.z=src_pt.image_coordinates.z();


      dest_pt.camera_coordinates.x=src_pt.camera_coordinates.x();
      dest_pt.camera_coordinates.y=src_pt.camera_coordinates.y();
      dest_pt.camera_coordinates.z=src_pt.camera_coordinates.z();

      dest_pt.world_coordinates.x=src_pt.world_coordinates.x();
      dest_pt.world_coordinates.y=src_pt.world_coordinates.y();
      dest_pt.world_coordinates.z=src_pt.world_coordinates.z();

      dest_pt.reprojected_coordinates.x=src_pt.reprojected_coordinates.x();
      dest_pt.reprojected_coordinates.y=src_pt.reprojected_coordinates.y();
      dest_pt.reprojected_coordinates.z=src_pt.reprojected_coordinates.z();

      dest_pt.depth_covariance=src_pt.depth_covariance;
      dest_pt.posit_error = src_pt.posit_error;
      dest_pt.parent_index = src_pt.parent_index;
      dest_pt.age = src_pt.age;
      dest_pt.landmark_id = src_pt.landmark_id;
    }
  }

}
