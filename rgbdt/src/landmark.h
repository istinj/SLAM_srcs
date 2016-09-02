#pragma once
#include <list>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>

namespace rgbdt {

  typedef std::pair<int, int> IntPair;
  
  /**
     A landmark is a unique point in the world.
     It is identified by an id unique among all landmarks.
     The data fields are:
     - the coordinates of the point in the world frame
     - the normal to the surface computed around the point (for visualization purposes)
     - the observations made about the landmark. An observation is a pair of int,
       <i,j>, where 
       i denotes the image where the landmark was seen, and 
       j denotes the feature within the frame 
   */
  struct Landmark{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id; //< unique id
    Eigen::Vector3f world_coordinates; //< xyz coordinates in the world
    Eigen::Vector3f world_normal;      //< normal in world frame
    std::list<IntPair> observations;   //< list of observations
  };

  /**
     Collection of landmark pointers, indexed by id
   */
  class LandmarkMap: public std::map<int, Landmark*> {
  public:
    //! returns the landmark having the id passed as argument, if it exists. OTherwise it returns 0
    Landmark* get(int id);
    //! puts a landmark in the map. If another landmark with the same id is already int the map
    //! it throws a nice exception
    void put(Landmark* landmark);
  };
}
