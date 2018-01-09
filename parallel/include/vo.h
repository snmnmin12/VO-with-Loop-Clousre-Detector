#ifndef VO2_H
#define VO2_H

#include "common_headers.h"

#include "DBoW2/DBoW2.h"

// #include "PlaceRecognition/PlaceDetector.h"
// #include "KeyFrameSelection.h"
#include "GraphOptimization.h"
#include "parameter_reader.h"
#include "feature.h"
#include "tracking.h"
#include "view.h"
#include "concurrent_queue.h"
//#include "tracking.h"


class VO2 {
public:

	VO2(ParameterReader& param): paramReader(param), frameReader(param) {

		K = frameReader.K;

		M1 = cv::Mat::zeros(3,4, CV_64F);
	    M2 = cv::Mat::zeros(3,4, CV_64F);
	    M1.at<double>(0,0) =1; 
	    M1.at<double>(1,1) =1;
	    M1.at<double>(2,2) =1;

	   	M2.at<double>(0,0) =1;
	    M2.at<double>(1,1) =1;
	    M2.at<double>(2,2) =1;
	    M2.at<double>(0,3) =-frameReader.baseline;

	}
    void initialize();
	//input is the feature points 1 and feature points 2 in each image, output is the 3D points, stereo images
	void get3D_Points(Frame::Ptr& frame);

	//input  is two stere image pair, output is the feature points of image1 and generated 3D points of landmarks
	void extract_keypoints(Frame::Ptr& frame);

	//this is to play on the image sequence
	void playSequence();

	cv::Mat getK() const {return K;}
	float getBaseline() const {return baseline;}
	void setK(const cv::Mat& _K) {K = _K;}
	void setBaseline(float _baseline) {baseline = _baseline;}
	void set_Max_frame(int maxframe) {max_frame = maxframe;}
	
private:

	cv::Mat K;
	float baseline;
	int max_frame;

	//concurrent queue for passing values;
	Queue<Frame::Ptr> queue;

	const ParameterReader& paramReader;
	FrameReader frameReader;

	//feature detector
	FDetector featureDetector;
	//set up the tracker
	shared_ptr<Track> tracker;

    shared_ptr<GraphOptimizer> optimizer;

    //poses
    vector<Eigen::Isometry3d> poses;

    //camera matrix
	cv::Mat M1, M2;

	//viewer
	shared_ptr<Viewer> viewer;
};


#endif
