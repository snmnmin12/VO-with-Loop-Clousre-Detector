#pragma once

#include "common_headers.h"
#include "frame.h"
#include "util.h"
#include "concurrent_queue.h"

#include <functional>
#include <thread>

class Track {
public:
	enum Status {
		INITIALIZED,
		OK,
		LOST
	};

	Track(FrameReader& para): frameReader(para) {
		initialized = false;
		t_thread = make_shared<thread>(bind(&Track::run,this));
	}

	void track(Frame::Ptr& frame) {
		inque.push(frame);
	}

	//run track in the separate thread
    void run() {

    		while(!shutdownflag) {
	    		Frame::Ptr frame = inque.pop();
	    		cout <<"Start tracking frame now!" << endl; 
				if (!initialized) {
					refs.push_back(frame);
					initialized = true;
					outque.push(frame);
					continue;
				}
				vector<int> inliers;
				trackRefFrame(frame, inliers);
				cout <<"inlier: " << inliers.size() << endl;
				if (inliers.size() >= 10) {
					outque.push(frame);
				}
			}
	}

	//track the 
	void trackRefFrame(Frame::Ptr& frame, vector<int>& inliers) {
		//tracked ptrs
		vector<cv::Point2f> trackedPts;
		vector<cv::Point3f> Obj;
		for (int i = 0; i < refs.size(); i++) {
			auto ref = refs[i];
			vector<cv::Point2f> nextPts;
			vector<uchar> status;
			vector<float> err;
			calcOpticalFlowPyrLK(ref->left, frame->left, ref->keyPts1, nextPts, status, err);
			//find the common landmarks and feature points
			auto inv_pose = ref->getTransform();
			for (int  j = 0; j < status.size(); j++) {
				if (status[j] == 1) {
					trackedPts.push_back(nextPts[j]);
					auto& obj = ref->WorldPts[j];
					Eigen::Vector4d O_W  = inv_pose * Eigen::Vector4d(obj.x, obj.y, obj.z, 1);
					Obj.push_back(cv::Point3f(O_W(0), O_W(1), O_W(2)));
					inliers.push_back(j);
				}
			}
		}
		if (inliers.size() < 5) return;

		cv::Mat dist_coeffs = cv::Mat::zeros(4,1,CV_64F);
		cv::Mat rvec, tvec;
		cv::Mat K = frameReader.K;
		inliers.clear();

		solvePnPRansac(Obj, trackedPts, K, dist_coeffs,rvec, tvec,false, 100, 8.0, 0.99, inliers);// inliers);
		if (inliers.size() < 10) return;
		//update the original trajectory
		cv::Mat R, tvec2;
		Rodrigues(rvec,R);
		R = R.t(); tvec2 = -R*tvec;
		Eigen::Isometry3d T = cvMat2Eigen(R, tvec2);
		frame->T_f_w  = T;

		 refs.push_back(frame);
		 //relase the memory to reduce space
		 if (refs.size() > frame_size) {
		 	refs.front()->left.release();
		 	refs.front()->right.release();
		 	refs.erase(refs.begin());
		}
	}

	void shutdown() {
		shutdownflag=true;
		if (t_thread != nullptr) {
			t_thread->join();
		}
	}
	//get the processed frameout
	Frame::Ptr getFrame() {
		Frame::Ptr frame = outque.pop();
		return frame;
	}

public:
	bool shutdownflag=false;
	shared_ptr<thread> t_thread=nullptr;
private:
	vector<Frame::Ptr> refs;
	int frame_size = 2;
	bool initialized;
	const FrameReader& frameReader;	
	Queue<Frame::Ptr> inque;
	Queue<Frame::Ptr> outque;
};