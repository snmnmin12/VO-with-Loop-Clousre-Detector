#pragma once

#include "common_headers.h"
#include "frame.h"
#include "GraphOptimization.h"

#define WHITE   cv::Scalar(255,255,255)
#define RED     cv::Scalar(0,   0, 255)
#define BLUE    cv::Scalar(255, 0,  0)
#define GREEN   cv::Scalar(0, 255, 0)
#define DarkRed cv::Scalar(0, 0, 127)

class Viewer {

public:

	Viewer (const FrameReader& frameReader, const GraphOptimizer& _optimizer): poses(frameReader.poses), optimizer(_optimizer) {

		traj = cv::Mat::zeros(600, 600, CV_8UC3);
	}

	void show() {

		traj = WHITE;
		vector<Frame::Ptr> frames = optimizer.getFrames();
		for (int i = 0; i < frames.size(); i++) {
			Eigen::Isometry3d T = frames[i]->getTransform();
			int id = frames[i]->id;
			cv::Point2f center = cv::Point2f(int(T(0,3)) + 300, int(T(2,3)) + 100);
			cv::circle(traj, center ,1, RED, 1);
			if (i == frames.size() - 1) {
				cout << "[:" << T(0,3) << ", " << T(1,3) << ", " << T(2,3) << "]" << endl;
				cout << "["<<poses[i](0, 3) << ", " << poses[id](1, 3) << ", " << poses[id](2, 3) <<"]"<<endl;
			}
   			cv::Point2f t_center = cv::Point2f(int(poses[id](0, 3)) + 300, int(poses[id](2, 3))+ 100);
			cv::circle(traj, t_center,1, BLUE, 1);
			cv::putText(traj, estimated_text, cv::Point2f(10,50), cv::FONT_HERSHEY_PLAIN, 1, RED, 1, 5);
			cv::putText(traj, ground_text, cv::Point2f(10,70), cv::FONT_HERSHEY_PLAIN, 1, BLUE, 1, 5);
		}
		cv::imshow(name, traj);
		cv::waitKey(1);
	}

	void pause () {
		imwrite("map2.png", traj);
		cv::imshow(name, traj);
		cout <<"You are approaching the end of this session!"<< endl;
		cv::waitKey(0);
	} 

private:
	const string estimated_text  = "Red color: estimated trajectory";
	const string ground_text	   = "Blue color: Groundtruth trajectory";
	const string loop_detected   = "GREEN Loop detected here";
	const string optimized_text  = "DarkRed Optimized trajectory";
	const string name = "Trajectory";
	cv::Mat traj;
	int frame_size = 0;
	const vector<Eigen::Isometry3d>& poses;
	const GraphOptimizer& optimizer;
	vector<Frame::Ptr> frames;
};