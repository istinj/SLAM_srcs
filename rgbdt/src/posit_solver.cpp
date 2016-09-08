#include "posit_solver.h"
#include <iostream>
#include <stdexcept>

namespace rgbdt {
	using namespace std;

	inline Eigen::Isometry3f v2t(const Vector6f& t)
	{
		Eigen::Isometry3f T;
		T.setIdentity();
		T.translation()=t.head<3>();
		float w=t.block<3,1>(3,0).squaredNorm();
		if (w<1) 
		{
			w=sqrt(1-w);
			T.linear()=Eigen::Quaternionf(w, t(3), t(4), t(5)).toRotationMatrix();
		} else {
			T.linear().setIdentity();
		}
		return T;
	}

	inline Eigen::Matrix3f skew(const Eigen::Vector3f& p)
	{
		Eigen::Matrix3f s;
		s << 
			0,  -p.z(), p.y(),
			p.z(), 0,  -p.x(), 
			-p.y(), p.x(), 0;
		return s;
	}

	PositSolver::PositSolver()
	{
		_T.setIdentity();
		_K <<  
			525, 0, 319.5, 
			0, 525, 239.5, 
			0, 0, 1;
		_image_cols=640;
		_image_rows=480;
		_max_error = 1;
		_error = 0;
		_num_inliers = 0;
		_use_depth=false;
	}

	//! Normals added
	void PositSolver::init(const Vector3fVector& model_points, 
		const Vector3fVector& image_points,
		const Vector3fVector& normals,
		int image_rows, int image_cols,
		const Eigen::Matrix3f& camera_matrix,
		const Eigen::Isometry3f& initial_guess)
	{
		if (model_points.size()!=image_points.size())
			throw std::runtime_error("num of points in the model and in the image should match");
		_model_points=model_points;
		_image_points=image_points;
		_normals = normals;
		_is_tracker = false;
		_image_rows = image_rows;
		_image_cols = image_cols;
		_K=camera_matrix;
		_T=initial_guess;
		_num_inliers=0;
		_error=-1;
		_inliers_error=-1;
		_inliers.resize(model_points.size());
		_errors.resize(model_points.size());
	}

	void PositSolver::init(const Vector3fVector& model_points, 
		const Vector3fVector& image_points,
		int image_rows, int image_cols,
		const Eigen::Matrix3f& camera_matrix,
		const Eigen::Isometry3f& initial_guess)
	{
		if (model_points.size()!=image_points.size())
			throw std::runtime_error("num of points in the model and in the image should match");
		_model_points=model_points;
		_image_points=image_points;
		_normals.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
		_is_tracker = true;
		_image_rows = image_rows;
		_image_cols = image_cols;
		_K=camera_matrix;
		_T=initial_guess;
		_num_inliers=0;
		_error=-1;
		_inliers_error=-1;
		_inliers.resize(model_points.size());
		_errors.resize(model_points.size());
	}


	void PositSolver::project(Vector3fVector& dest)
	{
		Eigen::Matrix3f KT = _K*_T.linear();
		Eigen::Vector3f Kt = _K*_T.translation();
		dest.resize(_model_points.size());

		for (size_t i = 0; i<_model_points.size(); i++){
			const Eigen::Vector3f& p=_model_points[i];
			Eigen::Vector3f pp=KT*p + Kt;
			pp *= (1./pp.z());
			if (pp.z()<0 || 
				pp.x()<0 || pp.x()>_image_cols ||
				pp.y()<0 || pp.y()>_image_rows)
			{
				pp.z()=0;
				continue;
			}
			dest[i]=pp;
		}
	}

	//! MOD
	bool PositSolver::errorAndJacobian(Vector6f&  error, Matrix6f&  J, 
		const Eigen::Vector3f& modelPoint, 
		const Eigen::Vector3f& imagePoint,
		const Eigen::Vector3f& normal)
	{
		J.setZero();

		//! Reprojection error, between xi and P*Xi
		Eigen::Vector3f rep_error;
		Matrix3_6f J_reproj;

		// apply the transform to the point
		Eigen::Vector3f tp=_T*modelPoint;

		// if the points is behind the camera, drop it;
		if (tp.z()<0)
			return false;

		float z = tp.z();
		float iz=1./z;

		// apply the projection to the transformed point
		Eigen::Vector3f pp = _K*tp;
		if (pp.z()<=0)
			return false;

		float iw = 1./pp.z();
		pp *= iw;

		// if the projected point is outside the image, drop it
		if (pp.x()<0 || pp.x()>_image_cols ||
			pp.y()<0 || pp.y()>_image_rows)
			return false;

		// compute the error given the projection
		rep_error = pp - imagePoint;
		if (! _use_depth)
			rep_error(2)=0;

		// jacobian of the transform part = [ I 2*skew(T*modelPoint) ]
		Eigen::Matrix<float, 3, 6> Jt;
		Jt.setZero();
		Jt.block<3,3>(0,0).setIdentity();
		Jt.block<3,3>(0,3)=-2*skew(tp);

		// jacobian of the homogeneous division
		// 1/z   0    -x/z^2
		// 0     1/z  -y/z^2
		Eigen::Matrix<float, 3, 3> Jp;
		Jp << 
			iz, 0,   -pp.x()*iz,
			0,   iz, -pp.y()*iz,
			0,   0,   (_use_depth)?1:0;

		// apply the chain rule and get the damn jacobian
		J_reproj=Jp*_K*Jt;


		//! Error between normals
		Eigen::Vector3f norm_error;
		Matrix3_6f J_normals;

		// Take just the rotation of the T matrix (since normal is a direction)
		Eigen::Matrix3f R = _T.matrix().block<3,3>(0,0);
		Eigen::Vector3f tn = R*normal;

		norm_error = tn - normal; // this way??

		J_normals.setZero();
		J_normals.block<3,3>(0,3) = 2 * skew(tn);

		//! Composing error and jacobian
		J.block<3,6>(0,0) = J_reproj;
		J.block<3,6>(3,0) = J_normals;

		error.block<3,1>(0,0) = rep_error;
		error.block<3,1>(3,0) = norm_error;

		return true;
	}

	bool PositSolver::errorAndJacobian(Eigen::Vector3f&  error, Matrix3_6f&  J, 
		const Eigen::Vector3f& modelPoint, 
		const Eigen::Vector3f& imagePoint)
	{
		J.setZero();
		// apply the transform to the point
		Eigen::Vector3f tp=_T*modelPoint;

		// if the points is behind the camera, drop it;
		if (tp.z()<0)
			return false;

		float z = tp.z();
		float iz=1./z;

		// apply the projection to the transformed point
		Eigen::Vector3f pp = _K*tp;
			if (pp.z()<=0)
		return false;

		float iw = 1./pp.z();
		pp *= iw;

		// if the projected point is outside the image, drop it
		if (pp.x()<0 || pp.x()>_image_cols ||
			pp.y()<0 || pp.y()>_image_rows)
			return false;

		// compute the error given the projection
		error = pp - imagePoint;
		if (! _use_depth)
			error(2)=0;

		// jacobian of the transform part = [ I 2*skew(T*modelPoint) ]
		Eigen::Matrix<float, 3, 6> Jt;
		Jt.setZero();
		Jt.block<3,3>(0,0).setIdentity();
		Jt.block<3,3>(0,3)=-2*skew(tp);

		// jacobian of the homogeneous division
		// 1/z   0    -x/z^2
		// 0     1/z  -y/z^2
		Eigen::Matrix<float, 3, 3> Jp;
		Jp << 
			iz, 0,   -pp.x()*iz,
			0,   iz, -pp.y()*iz,
			0,   0,   (_use_depth)?1:0;

		// apply the chain rule and get the damn jacobian
		J=Jp*_K*Jt;
		return true;
	  }

	//! Added normals
	void PositSolver::linearize(Matrix6f& H,Vector6f& b, bool suppress_outliers)
	{
		H.setZero();
		b.setZero();
		_error = 0;
		_inliers_error=0;
		_num_inliers = 0;

		if (_is_tracker)
		{
			Eigen::Vector3f e;
			Matrix3_6f J;

			for (size_t i = 0; i<_model_points.size(); i++)
			{
				_inliers[i]=false;
				if (errorAndJacobian(e,J,_model_points[i], _image_points[i]));
				{
					float en = e.squaredNorm();
					_errors[i]=en;
					_error += en;
					float scale = 1;
					if (en>_max_error)
					{
						scale  = _max_error/en;
					} else {
						_inliers[i]=true;
						_num_inliers++;
						_inliers_error+=en;
					}
					if(! suppress_outliers || _inliers[i])
					{
						H.noalias() += J.transpose()*J*scale;
						b.noalias() += J.transpose()*e*scale;

					}
				}
			}
		} else {
			Vector6f e;
			Matrix6f J;

			for (size_t i = 0; i<_model_points.size(); i++)
			{
				_inliers[i]=false;
				if (errorAndJacobian(e,J,_model_points[i], _image_points[i], _normals[i]))
				{
					float en = e.squaredNorm();
					_errors[i]=en;
					_error += en;
					float scale = 1;
					if (en>_max_error)
					{
						scale  = _max_error/en;
					} else {
						_inliers[i]=true;
						_num_inliers++;
						_inliers_error+=en;
					}
					if(! suppress_outliers || _inliers[i])
					{
						H.noalias() += J.transpose()*J*scale;
						b.noalias() += J.transpose()*e*scale;

						// std::cout << "linearize w/ normals" << endl;
						// std::cout << "H \n" << H << endl;
						// std::cout << "b \n" << b << endl;
						// std::cout << "J \n" << J << endl << endl;
					}
				}
			}
		}


	}


	void PositSolver::oneRound(bool suppress_outliers)
	{
		Matrix6f H;
		Vector6f b;
		linearize(H,b, suppress_outliers);

		//H += sqrt(error)*Matrix6f::Identity();
		//cerr << H << endl;
		// add damping?
		Vector6f dt = H.ldlt().solve(-b);
		_T = v2t(dt)*_T;
		Eigen::Matrix3f R = _T.linear();
		Eigen::Matrix3f E = R.transpose() * R;
		E.diagonal().array() -= 1;
		_T.linear() -= 0.5 * R * E;
	}

}// end namespace
