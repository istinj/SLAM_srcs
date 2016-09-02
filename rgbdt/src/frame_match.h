#pragma once
#include <Eigen/Core>
#include <list>
namespace rgbdt {
  
  /**
     spatial constraint between two keyframes
   */
  struct FrameMatch{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    int from_seq; //< sequence number of the start keyframe
    int to_seq;   //< sequence number of the ending keyframe
    int type;     //< type
    Eigen::Isometry3f transform; //< transformation that expresses the to keyframe in the reference frame of the from keyframe
  };

  typedef std::list<FrameMatch*> FrameMatchList;
  
}
