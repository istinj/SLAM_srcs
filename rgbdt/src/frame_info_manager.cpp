#include "frame_info_manager.h"

#include <iostream>
#include <stdexcept>
#include <opencv2/features2d/features2d.hpp>

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

namespace rgbdt 
{
	using namespace std;

	FrameInfoManager::FrameInfoManager() 
	{
		detector = new cv::GFTTDetector(1000, 0.01, 1., 3);
		extractor = new cv::BriefDescriptorExtractor();
		matcher = new cv::BFMatcher(cv::NORM_HAMMING);
		solver_iterations=100;
		solver.setMaxError(3);
		closure_frame_window=5;
		min_closure_inliers=20;
		min_closure_matches=50;
		optimizer = g2oInit();
  }

	FrameInfoManager::~FrameInfoManager()
	{
		delete detector;
		delete extractor;
		delete matcher;
	}


	//! Added normal here (pisit init)
	bool FrameInfoManager::tryMatch(Eigen::Isometry3f& result,
		const KeyFrame* current, 
		const KeyFrame* parent, 
		const std::list<cv::DMatch>& matches)
	{
		Vector3fVector model_points(matches.size());
		Vector3fVector image_points(matches.size());
		int i=0;
		for (std::list<cv::DMatch>::const_iterator it=matches.begin(); it!=matches.end(); ++it)
		{
			const cv::DMatch& match=*it;
			model_points[i]=current->camera_points[match.queryIdx];
			image_points[i]=parent->image_points[match.trainIdx];
			i++;
		}

		Eigen::Isometry3f initial_t=parent->global_T.inverse()*current->global_T;

		if (initial_t.translation().norm()>1)
			initial_t.setIdentity();

		solver.init(model_points, 
			image_points,
			current->camera_normals, //???
			current->depth.rows,
			current->depth.cols,
			parent->camera_matrix,
			initial_t);

		cerr << "matches: " << model_points.size() << endl;
		for (int i=0; i<solver_iterations; i++)
		{
			solver.oneRound(i==solver_iterations-1);
			if (! (i%5) || i==solver_iterations-1)
			{
				cerr << "  iteration: " << i;
				cerr << "  inliers: " << solver.numInliers();
				cerr << "  error: ";
				if (solver.numInliers())
				{
					cerr << solver.inliersError()/solver.numInliers();
				} else
					cerr << "NaN";
				cerr << endl;
			}
		}
		if (solver.numInliers()<min_closure_inliers)
			return false;
		result=solver.T();
		return true;
	}


	void FrameInfoManager::manageKeyframe(KeyFrame* kframe)
	{
		if(kframe->gray.empty())
			return;
		if(kframe->depth.empty())
			return;

		FrameInfo* finfo=frames.get(kframe->seq);
		cerr << "got keyframe: " << kframe->seq << endl;
		if (detector) 
		{
			cerr << "  running detector" << endl;
			cv::Mat mask=kframe->depth>0;
			detector->detect(kframe->gray, kframe->key_points, mask);
		} else {
			cerr << "  finfo " << finfo->points.size() <<  endl;
			finfo->toKeyPoints(kframe->key_points, PointTrack::ConfirmedFromPosit);
		}


		cerr << "  running exractor" << endl;
		cerr << "  good " << kframe->key_points.size() << " keypoints" << endl;
		extractor->compute(kframe->gray, kframe->key_points, kframe->descriptors);
		cerr << "  descriptors " << kframe->key_points.size() << " keypoints" << endl;
		kframe->updateDepths();

		cerr << "  running matcher" << endl;
		cerr << "  pool size: " << matcher->getTrainDescriptors().size() << endl;
		std::vector<cv::DMatch> matches;
		matcher->match(kframe->descriptors, matches);

		cerr << "  matches size: " << matches.size() << endl;

		closures.clear();
		std::vector< std::list<cv::DMatch> > image_stats(keyframe_seq_index.size());
		for (size_t i=0; i<matches.size(); i++) 
		{
			const cv::DMatch& match = matches[i];
			image_stats[match.imgIdx].push_back(match);
		}

		bool closures_found=false;
		for (int i=0; i<(int) image_stats.size()-closure_frame_window; i++) 
		{
			if (image_stats[i].size()<min_closure_matches)
				continue;
			if (image_stats[i].size()<0.2*kframe->key_points.size())
				continue;

			cerr << "  closure: " << keyframe_seq_index[i] 
				<< "  score: " << image_stats[i].size() << endl;

			FrameInfo* finfo=frames.get(kframe->seq);
			FrameInfo* parent_info=frames.get(keyframe_seq_index[i]);
			if (! parent_info || ! finfo)
				continue;
			Eigen::Isometry3f closure_transform;
			KeyFrame* parent_keyframe = keyframes.get(keyframe_seq_index[i]);
			bool match_good=tryMatch(closure_transform, kframe, parent_keyframe, image_stats[i]);
			if (! match_good) 
			{
				cerr << "  REJECTED " << endl; 
				continue;
			}
			cerr << "  ACCEPTED " << endl; 

			closures.insert(keyframe_seq_index[i]);
			Eigen::Isometry3f initial_offset=Eigen::Isometry3f::Identity();

			FrameMatch* frame_match=new FrameMatch;
			frame_match->from_seq=parent_info->seq;
			frame_match->to_seq=finfo->seq;
			frame_match->type=1;
			frame_match->transform=closure_transform;
			this->matches.push_back(frame_match);
			closures_found=true;
			kframe->depth.release();
			kframe->gray.release();
		}


		// Adding to the matcher the current frame
		std::vector<cv::Mat> d;
		d.push_back(kframe->descriptors);
		matcher->add(d);
		keyframe_seq_index.push_back(kframe->seq);

		if (closures_found)
			optimize();
  }


	void FrameInfoManager::putFrame(FrameInfo* finfo,
		const cv::Mat& current_gray, 
		const cv::Mat& current_depth) 
	{
		frames.put(finfo);
		int max_chain=0;
		int max_chain_id=-10;
		for (size_t i = 0; i< finfo->points.size(); i++)
		{
			PointTrack& point=finfo->points[i];
			if (point.landmark_id<0)
				continue;
			if (! (point.status&PointTrack::ConfirmedFromPosit) )
				continue;
			Landmark* landmark=landmarks.get(point.landmark_id);
			if (!landmark) 
			{
				landmark = new Landmark;
				landmark->id=point.landmark_id;
				landmark->world_coordinates=point.world_coordinates;
				landmark->world_normal=(finfo->global_T.translation() - point.world_coordinates).normalized();
				landmarks.put(landmark);

				FrameInfo* current_frame_info=finfo;
				PointTrack* current_track=&point;
				int chain=0;
				do 
				{
					if(current_frame_info->parent_seq<0)
						break;
					if (current_track->parent_index<0)
						break;
					FrameInfo* parent_frame_info=frames.get(current_frame_info->parent_seq);
					if (! parent_frame_info)
						break;
					PointTrack* parent_track=&parent_frame_info->points[current_track->parent_index];
					if (! (parent_track->status&PointTrack::ConfirmedFromPosit) )
						break;
					parent_track->landmark_id=landmark->id;
					landmark->observations.push_front(std::make_pair(parent_frame_info->seq, current_track->parent_index));
					/*
					cerr << "(" << parent_frame_info->seq << " " << current_track->parent_index << " "  <<
					" " << parent_track->age << " " << parent_track->status << ") ";
					*/
					current_frame_info=parent_frame_info;
					current_track=parent_track;
					chain++;
				} while(1);
				if(chain>max_chain) 
				{
					max_chain = chain;
					max_chain_id=landmark->id;
				}
				//cerr << endl;
			}
			landmark->observations.push_front(std::make_pair(finfo->seq, point.parent_index));
		}

		if (finfo->seq==finfo->keyframe_seq) 
		{
			Eigen::Isometry3f keyframe_global_T=Eigen::Isometry3f::Identity();
			if (finfo->parent_keyframe_seq>-1) 
			{
				FrameInfo* parent_info=frames.get(finfo->parent_keyframe_seq);
				KeyFrame* parent_keyframe=keyframes.get(finfo->parent_keyframe_seq);
				if (parent_info && parent_keyframe) 
				{
					FrameMatch* match=new FrameMatch;
					match->from_seq=parent_info->seq;
					match->to_seq=finfo->seq;
					match->type=0;
					match->transform=parent_info->global_T.inverse()*finfo->global_T;
					keyframe_global_T=parent_keyframe->global_T*match->transform;
					matches.push_back(match);
				}
			}
			KeyFrame* kframe=new KeyFrame;
			kframe->camera_matrix=finfo->camera_matrix;
			kframe->global_T = keyframe_global_T;
			kframe->seq=finfo->seq;
			kframe->parent_seq=finfo->parent_keyframe_seq;
			current_gray.copyTo(kframe->gray);
			current_depth.copyTo(kframe->depth);
			keyframes.put(kframe);
			manageKeyframe(kframe);
			cv::Mat image;
			current_gray.copyTo(image);
			cv::drawKeypoints(current_gray , kframe->key_points, image);
			cv::imshow("features", image);
		}

		if(max_chain>20)
		throw std::runtime_error("max chain");
	}


	FrameInfo* FrameInfoManager::lastFrame()
	{
		if (frames.empty())
			return 0;
		FrameInfoMap::reverse_iterator it=frames.rbegin();
		return it->second;
	}


	g2o::SparseOptimizer * FrameInfoManager::g2oInit()
	{
		// graph construction
		typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
		typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
		SlamLinearSolver* linearSolver = new SlamLinearSolver();
		linearSolver->setBlockOrdering(false);
		SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
		g2o::OptimizationAlgorithmLevenberg* solverGauss  = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
		//OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
		g2o::SparseOptimizer * g = new g2o::SparseOptimizer();
		g->setAlgorithm(solverGauss);
		return g;
	}


	void FrameInfoManager::optimize()
	{
		optimizer->clear();
		// copy the vertices
		for(KeyFrameMap::iterator it=keyframes.begin(); it!=keyframes.end(); ++it) 
		{
			const KeyFrame* keyframe = it->second;
			g2o::VertexSE3* v=new g2o::VertexSE3;
			v->setId(keyframe->seq);
			v->setEstimate(keyframe->global_T.cast<double>());
			optimizer->addVertex(v);
		}
		// copy the edges
		for (FrameMatchList::iterator it=matches.begin(); it!=matches.end(); ++it)
		{
			const FrameMatch* match=*it;
			g2o::EdgeSE3* e=new g2o::EdgeSE3;
			g2o::OptimizableGraph::Vertex* from=optimizer->vertex(match->from_seq);
			g2o::OptimizableGraph::Vertex* to=optimizer->vertex(match->to_seq);
			e->setVertex(0,from);
			e->setVertex(1,to);
			e->setMeasurement(match->transform.cast<double>());
			e->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
			optimizer->addEdge(e);
		}
		optimizer->initializeOptimization();
		optimizer->setVerbose(1);
		optimizer->optimize(10);

		// copy the vertices
		for(KeyFrameMap::iterator it=keyframes.begin(); it!=keyframes.end(); ++it)
		{
			KeyFrame* keyframe = it->second;
			const g2o::VertexSE3* v=static_cast<const g2o::VertexSE3*> (optimizer->vertex(keyframe->seq));
			keyframe->global_T=v->estimate().cast<float>();
		}
	}
}
