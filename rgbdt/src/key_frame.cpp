#include "key_frame.h"
#include <stdexcept>
#include <Eigen/Eigenvalues> 

namespace rgbdt {
	using namespace std;


	void KeyFrame::updateDepths(){
		if (depth.empty()){
			throw std::runtime_error("cant update the depths if no imag is provided");
		}
		Eigen::EigenSolver<Eigen::Matrix3f> eigensolver;

		Eigen::Matrix3f iK=camera_matrix.inverse();
		image_points.resize(key_points.size());
		camera_points.resize(key_points.size());
		camera_normals.resize(key_points.size());
		for (size_t i=0; i<key_points.size(); i++){
			const cv::KeyPoint& key_point=key_points[i];
			Eigen::Vector3f& image_point=image_points[i];
			Eigen::Vector3f& camera_point=camera_points[i];
			Eigen::Vector3f& camera_normal=camera_normals[i];

			float u = image_point.x() = key_point.pt.x;
			float v = image_point.y() = key_point.pt.y;
			image_point.z()=1;

			int _depth_region=5;
			int rmin=v-_depth_region;
			if (rmin<0)
				rmin=0;
			int rmax=v+_depth_region;
			if(rmax>=depth.rows)
				rmax=depth.rows-1;

			int cmin=u-_depth_region;
			if (cmin<0)
				cmin=0;
			int cmax=u+_depth_region;
			if(cmax>=depth.cols)
				cmax=depth.cols-1;

			int num_points=0;
			Eigen::Vector3f point_mean;
			Eigen::Matrix3f point_squared_mean;
			point_mean.setZero();
			point_squared_mean.setZero();
			float d_mean=0;
			for(int r=rmin;r<=rmax;r++){
				uint16_t* row_ptr=depth.ptr<uint16_t>(r)+cmin;
				for (int i=cmin; i<=cmax; i++, row_ptr++){
					uint16_t d_raw=*row_ptr;
					if(d_raw){
						float d=d_raw*1e-3;
						d_mean+=d;
						Eigen::Vector3f world_point=iK*Eigen::Vector3f(i*d, r*d,d);
						point_mean+=world_point;
						point_squared_mean+=world_point*world_point.transpose();
						num_points++;
					}
				}
			}
			int _min_good_depths=5;
			if (num_points>_min_good_depths){
				float inv_num_points= 1./num_points;
				point_mean*=inv_num_points;
				d_mean*=inv_num_points;
				Eigen::Matrix3f point_covarince=inv_num_points*point_squared_mean-point_mean*point_mean.transpose();
				eigensolver.compute(point_covarince, true);
				int ev_imin=-1;
				float ev_min=1e9;
				for(int i=0; i<3; i++){
					float ev_real=eigensolver.eigenvalues()(i).real();
					if (ev_min>ev_real){
						ev_min=ev_real;
						ev_imin=i;
					}
				}
				camera_normal=eigensolver.eigenvectors().col(ev_imin).real();
				if (camera_normal.dot(point_mean)>0){
					camera_normal*=-1.0f;
				}
				camera_point=point_mean;
				image_point.z()=d_mean;
			} else {
				float z=1e-3 * depth.at<uint16_t>(key_point.pt.y, key_point.pt.x);
				camera_point=iK*(image_point*z);
				image_point.z()=z;
				camera_normal.setZero();
			}

		}
	} 




	KeyFrame* KeyFrameMap::get(int seq) {
		iterator it=find(seq);
		if (it==end())
			return 0;
		return it->second;
	}
	
	void KeyFrameMap::put(KeyFrame* kf_info){
		KeyFrame* element = get(kf_info->seq);
		if (! element) {
			insert(make_pair(kf_info->seq, kf_info));
			return;
		}
		if (element==kf_info)
			return;
		throw std::runtime_error("error, duplicate insertion");
	}
}
