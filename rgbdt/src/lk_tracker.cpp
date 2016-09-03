#include "lk_tracker.h"

#include <iostream>
#include <globals/system_utils.h>
#include <stdexcept>
#include <Eigen/Eigenvalues> 

namespace rgbdt {

	using namespace cv;
	using namespace std;


	double getTimeMs() {
		return system_utils::getTime()*1000;
	}

	static int kf_count=0;

	LKTracker::LKTracker(int max_num_features):
	_termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03),
	_sub_pixel_win_size(10,10),
	_win_size(31,31)
	{
		_max_num_features=max_num_features;
		_min_num_features_fraction=0.25;
		_min_feature_distance=10;
		_status=Initializing;
		_depth_region=5;
		_min_good_depths=10;
		_min_age=10;
		_solver_iterations=10;
		_solver.setMaxError(10.0f);
		_solver_min_num_correspondences=20;
		_solver_min_num_inliers_fraction=0.3;
		_global_T.setIdentity();
		_incremental_tracking=false;
		_min_landmark_age=10;
		_last_landmark_id=0;
		_keyframe_rotation=M_PI/8;
		_keyframe_translation=0.2;
		_verbose=false;
		_verbose_profile=false;
		_current_epoch=0;
		_min_covered_region=0.7;
	}


	void LKTracker::extractFeatures(){
		cv::Point2f side=cv::Point2f(_min_feature_distance,_min_feature_distance);
		std::vector<cv::Point2f> current_cv_points;
		_current_frame_info.toCvPoints(current_cv_points);
		_mask=(_current_depth>0);
		for (size_t i =0; i<current_cv_points.size(); i++){
			cv::rectangle(_mask, 
				current_cv_points[i]-side,
				current_cv_points[i]+side,
				cv::Scalar(0),
				CV_FILLED);
		}
		float uncovered_region=((float)cv::countNonZero(_mask)/(float)(_mask.rows*_mask.cols));

		if (_current_frame_info.points.size()>_max_num_features*_min_num_features_fraction &&
			uncovered_region<_min_covered_region) 
			return;

		cerr << "Uncovered region: " << uncovered_region << endl;
		int num_new_features_to_find=_max_num_features-_current_frame_info.points.size();
		if(num_new_features_to_find<=0)
			return;

		std::vector<cv::Point2f> new_cv_points;
		std::vector<cv::KeyPoint> keypoints;

		_detector = GFTTDetector(num_new_features_to_find, 0.01, _min_feature_distance, 3, 0, 0.04);
		_detector.detect(_current_gray, keypoints, _mask);

    // goodFeaturesToTrack(_current_gray, new_cv_points, num_new_features_to_find, 0.01, _min_feature_distance, _mask, 3, 0, 0.04);
		_current_frame_info.fromKeyPoints(keypoints, true);
    // for (size_t i=0; i<keypoints.size(); i++) {
    //   cerr << i 
    // 	   << "  " << keypoints[i].pt.x
    // 	   << "  " << keypoints[i].pt.y
    // 	   << "  " << keypoints[i].size
    // 	   << "  " << keypoints[i].angle
    // 	   << "  " << keypoints[i].response
    // 	   << "  " << keypoints[i].octave << endl;
    // }

    //_current_frame_info.fromCvPoints(new_cv_points, true);
	}

	void LKTracker::trackFeatures(){
		std::vector<cv::Point2f> current_cv_points;
		std::vector<cv::Point2f> previous_cv_points;

		std::vector<uchar> status;
		std::vector<float> error;

		_previous_frame_info.toCvPoints(previous_cv_points);
		calcOpticalFlowPyrLK(_previous_gray, _current_gray, 
			previous_cv_points, current_cv_points, 
			status, error, 
			_win_size, 3, _termcrit, 0, 0.001);
		_current_frame_info.points.resize(previous_cv_points.size());
		int k=0;
		for(size_t i=0; i<status.size(); ++i) {
			PointTrack& parent_track = _previous_frame_info.points[i];
			if(status[i]){
				PointTrack& current_track=_current_frame_info.points[k];
				current_track.reset();
				current_track.image_coordinates.x()=current_cv_points[i].x;
				current_track.image_coordinates.y()=current_cv_points[i].y;
				current_track.image_coordinates.z()=0;
				current_track.parent_index=i;
				current_track.age=parent_track.age+1;
				current_track.status|=PointTrack::ConfirmedFromImage;
				current_track.landmark_id=parent_track.landmark_id;
				current_track.size=parent_track.size;
				current_track.angle=parent_track.angle;
				current_track.response=parent_track.response;
				current_track.octave=parent_track.octave;
				++k;
			}
		}
		if (k) {
			_current_frame_info.points.resize(k);
		} else {
			_current_frame_info.points.clear();
		}
		_tracked_features=k;
	}

	void LKTracker::addImage(cv::Mat frame, cv::Mat depth, int seq, const Eigen::Matrix3f& camera_matrix) {
		double t_start = getTimeMs();
		_current_frame.copyTo(_previous_frame);
		_current_gray.copyTo(_previous_gray);
		depth.copyTo(_current_depth);
		cvtColor(frame, _current_frame, CV_BGR2RGB);
		cvtColor(_current_frame, _current_gray, CV_BGR2GRAY);
		double t_prepare = getTimeMs()-t_start;

		double t_track=getTimeMs();
		_previous_frame_info=_current_frame_info;
		_current_frame_info=FrameInfo(seq, _previous_frame_info.seq, std::string(""), 0, camera_matrix, Eigen::Isometry3f::Identity());
		_current_frame_info.epoch=_current_epoch;
		if (_previous_frame_info.points.size())
			trackFeatures();
		t_track=getTimeMs()-t_track;
		double t_repopulate = getTimeMs();
		extractFeatures();
		t_repopulate = getTimeMs()-t_repopulate;

		double t_depth=getTimeMs();
		updateDepths();
		t_depth=getTimeMs()-t_depth;

		double t_posit= getTimeMs();
		bool transform_good=false;
		if (_previous_frame_info.points.size()){
			transform_good=callSolver();
			suppressBadTracks();
			updateWorldPoints();
		} 

		t_posit=getTimeMs()-t_posit;
		double t_total=getTimeMs()-t_start;
		if (_verbose_profile) {
			cerr << "landmarks: " << _last_landmark_id << " ";
			cerr << "hz: " << 1000.0f/t_total << " ";
			cerr << "total: " << t_total << " ";
			cerr << "prepare: " << t_prepare << " ";
			cerr << "track: " << t_track << " ";
			cerr << "repopulate: " << t_repopulate << " ";
			cerr << "depth: " << t_depth << " ";
			cerr << "posit: " << t_posit << endl;
		}
		_current_frame_info.transform_good=transform_good;

		switch(_status){
			case Initializing:
			cerr << "initializing, t good: " << transform_good << endl;
			if (transform_good){
				_status=Tracking;
				initializeNewKeyframe();
			}
			break;
			case Tracking:
			if (! transform_good) {
				reset();
			} else {
				handleKeyframeUpdate();
			}
			break;
		}
		_current_frame_info.tracker_status=_status;
	}

	void LKTracker::initializeNewKeyframe() {
		_last_keyframe_info=_current_frame_info;
		_current_frame_info.keyframe_T.setIdentity();
		_current_frame_info.keyframe_seq=_current_frame_info.seq;
		_current_frame_info.parent_keyframe_seq = -1;
		kf_count++;
		cerr << "creating keyframe: " << kf_count << " " << _current_frame_info.epoch << " "  << _current_frame_info.keyframe_seq << endl;
	}

	void LKTracker::handleKeyframeUpdate() {
		Eigen::Isometry3f delta=_last_keyframe_info.global_T.inverse()*_current_frame_info.global_T;
		float rotation=Eigen::AngleAxisf(delta.linear()).angle();
		float translation=delta.translation().norm();
		_current_frame_info.keyframe_T=delta;
		_current_frame_info.parent_keyframe_seq = _previous_frame_info.keyframe_seq;
		_current_frame_info.keyframe_seq=_current_frame_info.seq;
		if(fabs(rotation)>_keyframe_rotation||translation>_keyframe_translation) {
			_current_frame_info.keyframe_T.setIdentity();
			_last_keyframe_info=_current_frame_info;
			kf_count++;
			cerr << "updating keyframe: " << kf_count << " " << _current_frame_info.epoch << " " << _current_frame_info.keyframe_seq << endl;
		}
		_current_frame_info.keyframe_seq=_last_keyframe_info.seq;
	}

	void LKTracker::drawFeatures(){
		cv::Scalar color;
		for(size_t i=0; i<_current_frame_info.points.size(); i++){
			const PointTrack& point=_current_frame_info.points[i];
			if (point.age>_min_age)
				color= cv::Scalar(0 ,255,0);
			else
				color=cv::Scalar(255,0,0);
			cv::circle(_current_frame,
				cv::Point(point.image_coordinates.x(), 
					point.image_coordinates.y()), 
				3, color);
		}
	}

	void LKTracker::drawCorrespondences(){
		cv::Scalar color(0,255,0);
		for(size_t i=0; i<_current_frame_info.points.size(); i++){
			const PointTrack& current_point=_current_frame_info.points[i];
			int idx=current_point.parent_index;
			if (idx<0)
				continue;
			if (idx>=_previous_frame_info.points.size())
				continue;
			const PointTrack& previous_point=_previous_frame_info.points[idx];
			cv::Point current_pt(current_point.image_coordinates.x(), 
				current_point.image_coordinates.y());
			cv::Point previous_pt(previous_point.image_coordinates.x(), 
				previous_point.image_coordinates.y());

			cv::line(_current_frame, current_pt,previous_pt, color); 
		}
	}

	void LKTracker::drawSolverState(){
		cv::Scalar color(0,0,255);
		Vector3fVector projected_model_points;
		_solver.project(projected_model_points);
		for(size_t i=0; i<_current_frame_info.points.size(); i++){
			const PointTrack& point=_current_frame_info.points[i];
			if (point.status&PointTrack::ConfirmedFromPosit) {
				if (point.reprojected_coordinates.z()<=0)
					continue;
				cv::Point reprojected_pt(point.reprojected_coordinates.x(), 
					point.reprojected_coordinates.y());
				cv::Point image_pt(point.image_coordinates.x(), 
					point.image_coordinates.y());
				Eigen::Vector3f dp=point.reprojected_coordinates-point.image_coordinates;
				cv::circle(_current_frame,reprojected_pt, 3, color);
				cv::line(_current_frame, reprojected_pt, image_pt, color); 
			}
		}
	}

	void LKTracker::reset() {
		cerr << "reset" << endl;
		_current_epoch++;
		_global_T.setIdentity();
		_current_frame_info.epoch=_current_epoch;
		_current_frame_info.keyframe_T.setIdentity();
		_current_frame_info.global_T.setIdentity();
		_status=Initializing;
		_previous_frame_info.points.clear();
		_current_frame_info.parent_seq=-1;
		_current_frame_info.points.clear();
		extractFeatures();
		updateDepths();
    //updateWorldPoints();
	}

	void LKTracker::updateDepths() 
	{
		int good_depths=0;
		Eigen::EigenSolver<Eigen::Matrix3f> eigensolver;
		Eigen::Matrix3f inverse_camera_matrix=_current_frame_info.camera_matrix.inverse();
		for (size_t i=0; i<_current_frame_info.points.size(); i++)
		{
			PointTrack &ptrack=_current_frame_info.points[i];
			float u=ptrack.image_coordinates.x();
			float v=ptrack.image_coordinates.y();

			int rmin=v-_depth_region;
			if (rmin<0)
				rmin=0;
			int rmax=v+_depth_region;
			if(rmax>=_current_gray.rows)
				rmax=_current_gray.rows-1;

			int cmin=u-_depth_region;
			if (cmin<0)
				cmin=0;
			int cmax=u+_depth_region;
			if(cmax>=_current_gray.cols)
				cmax=_current_gray.cols-1;
			int num_points=0;
			float depth=0; 
			float depth2=0;
			Eigen::Vector3f point_mean;
			Eigen::Matrix3f point_squared_mean;
			point_mean.setZero();
			point_squared_mean.setZero();
			for(;rmin<=rmax;rmin++)
			{
				uint16_t* row_ptr=_current_depth.ptr<uint16_t>(rmin)+cmin;
				for (int i=0; i<cmax-cmin; i++, row_ptr++)
				{
					uint16_t d_raw=*row_ptr;
					if(d_raw)
					{
						float d=d_raw*1e-3;
						Eigen::Vector3f world_point=inverse_camera_matrix*Eigen::Vector3f(u*d, v*d,d);
						point_mean+=world_point;
						point_squared_mean+=world_point*world_point.transpose();
						num_points++;
						depth+=d;
						depth2+=d*d;
					}
				}
			}
			if (num_points>_min_good_depths)
			{
				float inv_num_points= 1/num_points;
				point_mean*=inv_num_points;
				Eigen::Matrix3f point_covarince=inv_num_points*point_squared_mean-point_mean*point_mean.transpose();
				eigensolver.compute(point_covarince, true);
				int ev_imin=-1;
				float ev_min=1e9;
				for(int i=0; i<3; i++)
				{
					float ev_real=eigensolver.eigenvalues()(i).real();
					if (ev_min>ev_real)
					{
						ev_min=ev_real;
						ev_imin=i;
					}
				}
				ptrack.image_normal=eigensolver.eigenvectors().col(ev_imin).real();
				if (ptrack.image_normal.dot(point_mean)>0)
				{
					ptrack.image_normal*=-1.0f;
				}
				float mean=depth/num_points;
				float sigma=depth2/num_points-mean*mean;
				ptrack.image_coordinates.z()=mean;
				ptrack.depth_covariance=sigma;
				good_depths++;
				Eigen::Vector3f image_point(u*mean,v*mean,mean);
				ptrack.camera_coordinates=inverse_camera_matrix*image_point;
				ptrack.world_coordinates=ptrack.camera_coordinates;
			} else {
				ptrack.status|=PointTrack::BadDepth;
			}
		}
	}

	//! Added normal here (pisit init)
	bool LKTracker::callSolver()
	{
		int num_correspondences=0;
		Vector3fVector model_points;
		Vector3fVector image_points;

		Vector3fVector normals;
		size_t num_points=_current_frame_info.points.size();
		model_points.resize(num_points);
		image_points.resize(num_points);
		_correspondence_map.resize(num_points);
		for (size_t i=0; i<_current_frame_info.points.size(); i++)
		{
			_correspondence_map[i]=-1;
			PointTrack& current_point=_current_frame_info.points[i];
			if (current_point.age<_min_age)
				continue;
			if (current_point.parent_index<0)
				continue;
			const PointTrack& parent_point=_previous_frame_info.points[current_point.parent_index];
			if(parent_point.depth_covariance<0)
				continue;
			_correspondence_map[i]=num_correspondences;
			current_point.status|=PointTrack::UsedByPosit;
			if (_incremental_tracking) 
			{
				model_points[num_correspondences]=parent_point.camera_coordinates;
			} else {
				model_points[num_correspondences]=parent_point.world_coordinates;
			}
			image_points[num_correspondences]=current_point.image_coordinates;
			num_correspondences++;

			// normals.push_back(parent_point.image_normal); //????
		}
		model_points.resize(num_correspondences);
		image_points.resize(num_correspondences);
		Eigen::Isometry3f initial_guess;

		if (_incremental_tracking) 
		{
			initial_guess.setIdentity();
		} else {
			initial_guess=_global_T.inverse();
		}

		//MOD overload
		_solver.init(model_points, 
			image_points,
			_current_frame.rows,
			_current_frame.cols,
			_current_frame_info.camera_matrix,
			initial_guess);
		//cerr << "seq: " << _current_frame_info.seq;
		if (num_correspondences < _solver_min_num_correspondences) 
		{
			cerr << "solver fail1: " << num_correspondences << " " << _solver_min_num_correspondences << endl;
			return false;
		}


		for (int i=0; i<_solver_iterations; i++)
		{
			_solver.oneRound(i==_solver_iterations-1);
			if (_verbose && (i==0 || i==_solver_iterations-1) ) 
			{
				cerr << "\tn:" << num_correspondences;
				cerr << "\te:" << _solver.error();
				cerr << "\ti:" << _solver.numInliers();
				cerr << "\tie:" << _solver.inliersError()/_solver.numInliers();
			}
		}
		if (_solver.numInliers()<(num_correspondences*_solver_min_num_inliers_fraction)) 
		{
			cerr << "solver fail: " << _solver.numInliers() << " " << (num_correspondences*_solver_min_num_inliers_fraction) << endl;
			return false;
		}
		if (_incremental_tracking) 
		{
			_global_T=_global_T*_solver.T().inverse();
			Eigen::Matrix3f R = _global_T.linear();
			Eigen::Matrix3f E = R.transpose() * R;
			E.diagonal().array() -= 1;
			_global_T.linear() -= 0.5 * R * E;
		} else {
			_global_T=_solver.T().inverse();
		}
		_current_frame_info.global_T=_global_T;


		Vector3fVector projected_model_points;
		_solver.project(projected_model_points);

		for (size_t i=0; i<_correspondence_map.size(); i++) 
		{
			PointTrack& current_point=_current_frame_info.points[i];
			current_point.reprojected_coordinates.setZero();
			int idx=_correspondence_map[i];
			if (idx<0)
				continue;
			current_point.reprojected_coordinates=projected_model_points[idx];
			current_point.posit_error=_solver.errors()[idx];
		}
		return true;
	}

	void LKTracker::suppressBadTracks() {

		for (size_t i=0; i<_correspondence_map.size(); i++) {
			PointTrack& current_point=_current_frame_info.points[i];
			current_point.posit_error=-1;
			int idx=_correspondence_map[i];
			if (idx<0) {
				continue;
			}
			if ( !_solver.inliers()[idx]){
				current_point.parent_index=-1;
				current_point.age=1;
				current_point.status&= ~PointTrack::ConfirmedFromPosit;
			} else {
				current_point.status|=PointTrack::ConfirmedFromPosit;
			}
		}
	}
	void LKTracker::updateWorldPoints() {
		for (size_t i=0; i<_current_frame_info.points.size(); i++){
			PointTrack& current_point=_current_frame_info.points[i];
			if (current_point.parent_index<0 || current_point.age<_min_age){
				current_point.world_coordinates=_global_T*current_point.camera_coordinates;
				continue;
			}
			const PointTrack& parent_point=_previous_frame_info.points[current_point.parent_index];
			if(parent_point.depth_covariance<0 && current_point.depth_covariance>0) {
				current_point.world_coordinates=_global_T*current_point.camera_coordinates;
				continue;
			}
			current_point.landmark_id=parent_point.landmark_id;
			current_point.world_coordinates=parent_point.world_coordinates;
			if (current_point.landmark_id<0 &&
				current_point.age>_min_landmark_age &&
				(current_point.status&PointTrack::ConfirmedFromPosit) ){
				current_point.landmark_id=_last_landmark_id;
			_last_landmark_id++;
		}
	}
}



} // end namespace
