#pragma once
#include "opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <map>

namespace rgbdt 
{

	/**
	This class represents a tracked feature in one image.
	A feature has
	- a status that summarizes what happened to the feature in relation to the previous detection
	- a parent_id, that tells the index of the feature in the previous feame
	- the parameters of the detector (size, response, position etc)
	- the position of the point in the image
	- the position of the point in the camera coordinates (meters w.r.t the camera pose)
	- the position of the feature in the world (redundant)
	- the position of the feature landmark reprojected in image coordinates
	- a landmark_id, telling to which landmark the feature is associated
	*/

	struct PointTrack
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		//! constants detecting the state. The state is a bitwise or of these fields
		static const int Unknown=0x0,
		Created=0x1,
		BadDepth=0x2, 
		ConfirmedFromImage=0x4, 
		ConfirmedFromPosit=0x8, 
		UsedByPosit=0x10;

		//! calls reset to set the feature to an unspecified value
		PointTrack();

		//! initializes a point_track, and sets all indices to invalid values
		void reset();

		//! FIXME image and camera coordinates need to be swapped. 
		Eigen::Vector3f camera_coordinates; //< coordinates in image frame (u,v,1)
		Eigen::Vector3f image_coordinates;  //< coordinates in the camera frame (x,y,z)
		Eigen::Vector3f image_normal;       //< normal in the camera frame (x,y,z)
		Eigen::Vector3f world_coordinates;  //< coordinates in the world frame (x,y,z) used for caching purposes
		Eigen::Vector3f reprojected_coordinates; //< used by posit to provide an estimate of the landmark reprojection

		float size; //< size of the feature area (depends on the detector)
		float angle; //< angle of the feature (depends on the detector)
		float response; //< (depends on the detector)
		float octave;   //< (depends on the detector)

		float depth_covariance; //< covariance of the depth measurement computed around the feature in the depth_image
		int status;             //< status of the feature
		float posit_error;      //< error after posit (in pixels^2)
		int parent_index;       //< index of the feature in the previous frame
		int age;                //< tells how long am i tracking this feature
		int landmark_id;        //< if>0 tells that this feature was used to spawn a landmark
	};

	//! vector of the things above
	typedef std::vector<PointTrack, Eigen::aligned_allocator<PointTrack> > PointTrackVector;
}