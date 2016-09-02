#pragma once
#include "opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "posit_solver.h"
#include "frame_info.h"

namespace rgbdt {

	/**
	Implementation of a simplistic rgbd tracker for rgb+depth images.
	To use it, create an instance of the tracker.
	add pairs of images (depth+rgb), together with the camera matrix and the  number sequence
	*/

	class LKTracker
	{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			enum Status {Initializing, Tracking}; //< defines one of the two tracker states
			LKTracker(int max_num_features=200);  //< ctor
			void reset();                         //< clears the status

			//! adds an image to the pipeline and does all processing
			//! @param frame: rgb image
			//! @param depth: depth image (16 bit mono, each pixel expresses a depth in mm, should have the same size)
			//! @param seq: unique sequence number of the image pair
			//! @param camera_matrix: 3x3 camera matrix. Last row should be 0,0,1
			void addImage(const cv::Mat frame, const cv::Mat depth, int seq, const Eigen::Matrix3f& camera_matrix);

			//! returns the current grayscale conversion of the last rgb image 
			inline const cv::Mat& currentGray() const {return _current_gray;}

			//! returns the current depth image 
			inline const cv::Mat& currentDepth() const {return _current_depth;}

			//! returns the current rgb image 
			inline const cv::Mat& currentFrame() const {return _current_frame;}

			//! returns the status of the tracker
			inline Status status() const {return _status;}

			//! if true, the tracker tries to follow the descriptors after the 1st detection,
			//! instead of replaicing them. Default: disables
			inline bool incrementalTrackingEnabled() const {return _incremental_tracking;}

			//! setter for incremental tracking
			inline void setIncrementalTracking(bool enable) {_incremental_tracking=enable;}

			//! returns the status of the tracker after adding an image
			inline const FrameInfo& currentFrameInfo() const {return _current_frame_info;}

			//! internally used, global position of the tracker
			inline const Eigen::Isometry3f& globalT() const {return _global_T;}

			//! minimum number of redetection of a feature before considering it stable
			inline int minAge() const {return _min_age;}
			inline void setMinAge(float ma) {_min_age=ma;}

			//! maximum number of tracked features (set through the ctor)
			inline int maxNumFeatures() const {return _max_num_features;}
			inline void setMaxNumFeatures(int n) {_max_num_features=n;}

			//! minimum fraction of redetected features among frames to tell that the tracker is not misbehaving
			inline float minNumFeaturesFraction() const {return _min_num_features_fraction;}
			inline void setMinNumFeaturesFraction(float f) {_min_num_features_fraction=f;}

			//! minimum distance between the detected features
			inline void setMinFeatureDistance(float d) {_min_feature_distance=d;}
			inline float minFeatureDistance() const {return _min_feature_distance;}

			//! region of the depth image used to compute depth statistics of a feature
			inline float minCoveredRegion() const {return _min_covered_region;}
			inline void setMinCoveredRegion(float min_covered_region) {_min_covered_region=min_covered_region;}

			//! minimum rotation between subsequent keyframes
			inline float keyframeRotation() const {return _keyframe_rotation;}
			inline void setKeyframeRotation(float rotation) {_keyframe_rotation=rotation;}

			//! minimum translation between subsequent keyframes
			inline float keyframeTranslation() const {return _keyframe_translation;}
			inline void setKeyframeTranslation(float translation) {_keyframe_translation=translation;}

			// gui stuff
			void drawFeatures();
			void drawCorrespondences();
			void drawSolverState();

		protected:
			//! extracts features in the region of the image where there are no tracked points
			//! does side effect on _current_frame_info
			void extractFeatures();

			//! tracks features from the previous frame 
			//! does side effect on _current_frame_info
			void trackFeatures();

			//! computes the depth statistics around the current features
			//! does side effect on _current_frame_info
			void updateDepths();

			//! calls the least squares posit solver and verifies the solution
			//! @returns false if posit failed (not enough inliers)
			bool callSolver();

			//! updates the estimate of the points in the world
			//! if _incremental_tracking, is uses the previous frame estimate
			//! otherwise it uses the first detection of a landmark to propagate the pose
			void updateWorldPoints();

			//! kills all points that are not confirmed by posit or do not have a decent depth or... see the code
			void suppressBadTracks();

			//! called when the system needs to spawn a new keyframe, after initialization
			void initializeNewKeyframe();

			//! calles when the system spawns a new keyframe without loosing track
			void handleKeyframeUpdate();

			//! RGB posit solver
			PositSolver _solver;

			FrameInfo 
			_current_frame_info, 
			_previous_frame_info,
			_last_keyframe_info;

			cv::TermCriteria _termcrit;
			float _min_feature_distance;
			cv::Size _sub_pixel_win_size;
			cv::Size _win_size;
			cv::Mat _current_frame;
			cv::Mat _previous_frame;
			cv::Mat _current_gray;
			cv::Mat _previous_gray;
			cv::Mat _current_depth;
			cv::Mat _mask;
			std::vector<int> _correspondence_map;
			int _max_num_features;
			float _min_num_features_fraction;
			int _solver_min_num_correspondences;
			float _solver_min_num_inliers_fraction;
			Status _status;
			int _depth_region;
			int _min_good_depths;
			int _min_age;
			int _min_landmark_age;
			float _keyframe_rotation, _keyframe_translation;
			float _posit_min_inlier_fraction;
			int _tracked_features;
			int _solver_iterations;
			int _last_landmark_id;
			bool _incremental_tracking;
			Eigen::Isometry3f _global_T;
			bool _verbose;
			bool _verbose_profile;
			cv::GFTTDetector _detector;
			int _current_epoch;
			float _min_covered_region;
	};
}
