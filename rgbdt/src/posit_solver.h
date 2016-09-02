#pragma once
#include <iostream>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <vector>

namespace rgbdt {
	// convenience definitions
	typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
	typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
	typedef Eigen::Matrix<float, 6, 6> Matrix6f;
	typedef Eigen::Matrix<float, 6, 1> Vector6f;
	typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;
	typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;


	/**
   	Class implementing a posit solver (+ depth, optional) . Good old least squares stuff :)
   */

	class PositSolver
	{
		public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		//! ctor, sets parameters to default values
		PositSolver();


		//! initializes an optimization run
		//! the points of model and image should have the same number of elements
		//! the correspondences are dictated by the corresponding index in the arrays as in
		//! model_points[0]--image_points[0], model_points[1]--image_points[1], ... etc.
		//! @param model_points: the (x,y,z) 3d points in world coordinates
		//! @param image_points: the (u,v,d) points in camera coordinates
		//! @param image_rows: rows of the image
		//! @param image_cols: cols of the image
		//! @param camera_matrix: 3x3 camera matrix
		//! @param initial_guess: transform from camera_to_world
		void init(const Vector3fVector& model_points, 
			const Vector3fVector& image_points,
			int image_rows, int image_cols,
			const Eigen::Matrix3f& camera_matrix,
			const Eigen::Isometry3f& initial_guess=Eigen::Isometry3f::Identity());

		//! does one round of optimization
		//! @param suppress_outliers: if true it ignores all points whoose error is
		//! higher than maxError()
		void oneRound(bool suppress_outliers=false);

		inline int imageRows() const {return _image_rows;}
		inline int imageCols() const {return _image_cols;}
		inline const Vector3fVector& modelPoints() const {return _model_points;}
		inline const Vector3fVector& imagePoints() const {return _image_points;}

		//!returns a boolean vector telling if the ith point is an inlier or not
		inline const std::vector<bool>& inliers() const {return _inliers;}

		//!returns float vector containing the errors of all points
		inline const std::vector<float>& errors() const {return _errors;}

		//!returns the global error of the optimization
		inline double error() const {return _error;}

		//! returns the error of only the inliers
		inline double inliersError() const {return _inliers_error;}

		//! returns the number of inliers
		inline int numInliers() const {return _num_inliers;}

		//! returns the maximum error after which the point is considered an inlier (and the robust kernel kicks in)
		inline float maxError() const {return _max_error;}
		inline void setMaxError(float max_error_) {_max_error = max_error_;}

		//!current transform of the solver
		inline const Eigen::Isometry3f& T() const {return _T;}

		//!current transform of the solver
		inline void setT(const Eigen::Isometry3f& T_)  {_T=T_;}

		//!if 1 it uses the depth in the optimization
		inline bool useDepth() const {return _use_depth;}
		inline void setUseDepth(bool use_depth) { _use_depth=use_depth;}


		//! projects the points of the model in  dest (u,v,d) , given the current transform
		//! good for visualization purposes, and to compute the reprojections
		void project(Vector3fVector& dest);

		
		protected:

		// computes the reprojection error and the jacobian of a model point
		// returns false if the point is outside the image plane
		bool errorAndJacobian(Eigen::Vector3f&  error, Matrix3_6f&  J, 
			const Eigen::Vector3f& modelPoint, 
			const Eigen::Vector3f& imagePoint);

		// computes the hessian and the coefficient vector of the model points between imin and imax
		// it also computes the error and the number of inliers
		void linearize(Matrix6f& H,Vector6f& b, bool suppress_outliers=false);    

		// does one round of optimization
		// updates the transform T, the error and the numver of inliers
		std::vector<bool> _inliers;
		std::vector<float> _errors;
		Vector3fVector _model_points;
		Vector3fVector _image_points;

		//! Normals
		Vector3fVector _normals;

		Eigen::Isometry3f _T; // position of the world w.r.t the camera
		Eigen::Matrix3f _K;
		float _image_rows, _image_cols;
		float _max_error;
		int _num_inliers; 
		double _error;
		double _inliers_error;
		bool _use_depth;
	};
}
