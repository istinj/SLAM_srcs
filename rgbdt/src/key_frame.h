#pragma once
#include "opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include "posit_solver.h"

namespace rgbdt 
{

	//! structure used to describe a keyframe
	//! This thing does not exist in the tracker but it is introduced at later stages
	//! A keyframe is characterized by
	//! - a position in the world (that might be optimized)
	//! - a bunch of features detected together with their descriptors
	//! - an epoch (telling the number of consistent graphs made so far)
	//! - other stuff for convenience

	struct KeyFrame
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		int epoch; //< tracker epoch. All keyframes with the same epoch belong to a fully connected subset
		int seq;   //< unique sequence of a keyframe. Must match the seq of the FrameInfo used to generate it
		int parent_seq; //< sequence number of the previous keyframe
		cv::Mat gray; //< gray version of the rgb image
		cv::Mat depth; //< depth map
		Eigen::Isometry3f global_T; //< global transformation 
		Eigen::Matrix3f camera_matrix; //< camera matrix
		std::vector<cv::KeyPoint> key_points; //< positions of the detected features
		Vector3fVector image_points; //< redundant position of the points in the image frame (u,v,d)
		Vector3fVector camera_points; //< redundant position of the points in the camera frame  (x,y,z)
		Vector3fVector camera_normals; //< redundant position of the normals in the camera frame  (x,y,z)
		cv::Mat descriptors; //< descriptor matrix (see opencv)
		void updateDepths(); 
	};

	//! convenience class to handle the keyframes
	class KeyFrameMap: public std::map<int, KeyFrame*> 
	{
		public:
		//! return if it exists a keyframe with sequence number as in the argument
		KeyFrame* get(int seq);
		//! put a new unique keyframe in the pool (kf_info->seq should have been valorized)
		void put(KeyFrame* kf_info);
	};

}
