#include "point_track.h"

namespace rgbdt {
  using namespace std;

  PointTrack::PointTrack() {
    reset();
  }

  void PointTrack::reset(){
    camera_coordinates.setZero();
    image_coordinates<< 0,0,1;
    image_normal << 0,0,0;
    world_coordinates.setZero();
    reprojected_coordinates.setZero();
    depth_covariance=-1;
    status=Unknown;
    posit_error=-1;
    parent_index=-1;
    age=1;
    landmark_id=-1;
    size=1;
    angle=-1;
    response=0;
    octave=0;
  }

}
