#include "lk_viewer.h"
#include "opengl_primitives.h"
#include <iostream>

using namespace std;

namespace rgbdt {
	LKViewer::LKViewer(FrameInfoManager* manager){
		_manager=manager;

		_points_drawn = false;
		_keyframes_drawn = true;
		_intermediate_frames_drawn = false;
		_map_drawn=true;
		_normals_drawn=true;
		_camera_pov=false;
	}

	void LKViewer::drawFrameInfo(const FrameInfo* finfo, bool draw_points) {
		if (finfo->keyframe_seq<0) 
			return;
		Eigen::Isometry3f frame_transform=finfo->global_T;
		KeyFrame* keyframe=_manager->keyframes.get(finfo->keyframe_seq);
		if(keyframe) {
			frame_transform=keyframe->global_T*finfo->keyframe_T;
		}

		Eigen::Matrix4f global_T=frame_transform.matrix();
		global_T.row(3) << 0,0,0,1;
		glPushMatrix();
		glMultMatrixf(global_T.data());
		glPushAttrib(GL_LINE_WIDTH|GL_POINT_SIZE);
		if (finfo->seq==finfo->keyframe_seq) {
			glColor3f(0.8, 0.5, 0.5);
			drawPyramid(0.1, 0.1);
		} else {
			glColor3f(0.5, 0.5, 0.8);
			glLineWidth(0.1);
			drawPyramidWireframe(0.1, 0.1);
		}
		glPopAttrib();


		if (draw_points) {

			glBegin(GL_POINTS);
			for (size_t i=0; i<finfo->points.size(); i++){
				const PointTrack& p=finfo->points[i];
				const Eigen::Vector3f wc=p.camera_coordinates;
				if (! (p.status&PointTrack::ConfirmedFromPosit))
					continue;
				if (p.depth_covariance>0){
	  // Eigen::Vector3f normal=-wc;
	  // normal.normalize();
	  // glNormal3f(-normal.x(), -normal.y(), -normal.z());

					Eigen::Vector3f n=p.image_normal;
					glNormal3f(n.x(), n.y(), n.z());
					glVertex3f(wc.x(), wc.y(), wc.z());
				}
			}
			glEnd();

		} 


		glPopMatrix();
	}

	void LKViewer::drawKeyFrame(const KeyFrame* kframe, const Eigen::Vector3f& color) {
		Eigen::Matrix4f global_T=kframe->global_T.matrix();
		global_T.row(3) << 0,0,0,1;
		glPushMatrix();
		glMultMatrixf(global_T.data());
		glPushAttrib(GL_LINE_WIDTH|GL_POINT_SIZE|GL_COLOR);
		glColor3f(color.x(), color.y(), color.z());
		drawPyramid(0.1, 0.1);
		glBegin(GL_POINTS);
		for (size_t i=0; i<kframe->camera_points.size(); i++){
			const Eigen::Vector3f& p=kframe->camera_points[i];
			const Eigen::Vector3f& n=kframe->camera_normals[i];
			if (p.z()>0){
	// Eigen::Vector3f normal = (-p).normalized();
	// glNormal3f(normal.x(), normal.y(), normal.z());
				glNormal3f(n.x(), n.y(), n.z());
				glVertex3f(p.x(), p.y(), p.z());
			}
		}
		glEnd();

		if (_normals_drawn) {
			glColor3f(0.0, 0.36, 0.99);
			glBegin(GL_LINES);
			for (size_t i=0; i<kframe->camera_points.size(); i++){
				const Eigen::Vector3f& p=kframe->camera_points[i];
				const Eigen::Vector3f n=kframe->camera_normals[i]*0.1+p;
				glVertex3f(p.x(), p.y(), p.z());
				glVertex3f(n.x(), n.y(), n.z());
			}
			glEnd();
		}

		glPopAttrib();
		glPopMatrix();
	}

	void LKViewer::drawMap() {
		if (! _manager)
			return;

		glColor3f(0.8,0.3,0.3);
		glBegin(GL_POINTS);
		for(LandmarkMap::iterator it=_manager->landmarks.begin(); it!=_manager->landmarks.end(); it++){
			Landmark* l=it->second;
			glNormal3f(l->world_normal.x(), l->world_normal.y(), l->world_normal.z());
			glVertex3f(l->world_coordinates.x(), l->world_coordinates.y(), l->world_coordinates.z());
		}
		glEnd();
	}


	void LKViewer::drawKeyFrameMap() {
		if (! _manager)
			return;
		for (KeyFrameMap::iterator it=_manager->keyframes.begin(); it!=_manager->keyframes.end(); it++){
			const KeyFrame* kf=it->second;
			drawKeyFrame(kf, Eigen::Vector3f(0.8, 0.1, 0.1));
		}
	}

	void LKViewer::draw(){
		if (! _manager)
			return;
		FrameInfo * current_frame=_manager->lastFrame();
		if (! current_frame)
			return;

		Eigen::Isometry3f world_to_camera;
		world_to_camera.setIdentity();
		if(_camera_pov) {
			world_to_camera=current_frame->global_T.inverse();
		}
		world_to_camera.matrix().row(3) << 0,0,0,1;

		glPushMatrix();
		glMultMatrixf(world_to_camera.data());

		glPushAttrib(GL_POINT_SIZE|GL_COLOR);
		glPointSize(3);
		glColor3f(1,0,0);
		drawFrameInfo(current_frame, true);
		glPopAttrib();


		glPushAttrib(GL_COLOR);
		glBegin(GL_LINES);
		for (FrameMatchList::iterator it=_manager->matches.begin(); it!=_manager->matches.end(); it++){
			FrameMatch* match=*it;
			KeyFrame* from_frame=_manager->keyframes.get(match->from_seq);
			if (! from_frame)
				continue;
			KeyFrame* to_frame=_manager->keyframes.get(match->to_seq);
			if (! to_frame)
				continue;
			if (match->type==0){
				glColor3f(0.1,0,0.1);
			} else {
				glColor3f(0.5,1,1);
			}
			glVertex3f(from_frame->global_T.translation().x(),
				from_frame->global_T.translation().y(),
				from_frame->global_T.translation().z());
			glVertex3f(to_frame->global_T.translation().x(),
				to_frame->global_T.translation().y(),
				to_frame->global_T.translation().z());
		}
		glEnd();
		glPopAttrib();


		for (FrameInfoMap::iterator it=_manager->frames.begin(); it!=_manager->frames.end(); it++){
			FrameInfo * frame = it->second;
			if (frame->seq==frame->keyframe_seq && !_keyframes_drawn) {
				continue;
			}

			if (frame->seq!=frame->keyframe_seq && !_intermediate_frames_drawn)
				continue;
			drawFrameInfo(frame, _points_drawn);

			if (_manager->closures.find(frame->seq)!=_manager->closures.end()){
				KeyFrame* kframe=_manager->keyframes.get(frame->seq);
				drawKeyFrame(kframe);
			} 
		}

		if (_map_drawn)
			drawKeyFrameMap();
		glPopMatrix();
	}

}
