#ifndef VO2_H
#define VO2_H

#include "DBoW2/DBoW2.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "PlaceRecognition/PlaceDetector.h"
#include "KeyFrameSelection.h"
#include "GraphOptimization.h"

typedef TemplatedKeyFrameSelection
<DBoW2::FORB::TDescriptor, DBoW2::FORB> KeyFrameSelection;


class VO2 {
public:

	VO2(){}
	VO2(const cv::Mat& _k, float _baseline, const std::string&, const std::string&);
        void initialize(int max_frame, const std::string& voc_file, int num);
	//input is the feature points 1 and feature points 2 in each image, output is the 3D points, stereo images
	std::vector<cv::Point3f> get3D_Points(const std::vector<cv::Point2f>& feature_p1, const std::vector<cv::Point2f>& feature_p2) const ;

	//input  is two stere image pair, output is the feature points of image1 and generated 3D points of landmarks
	void extract_keypoints(int index, const cv::Mat& img1, const cv::Mat& img2, std::vector<cv::Point3f>&landmarks, std::vector<cv::Point2f>& feature_points);

	//this is to play on the image sequence
	void playSequence(const std::vector<std::vector<float>>& poses);

	cv::Mat getK() const {return K;}
	float getBaseline() const {return baseline;}
	void setK(const cv::Mat& _K) {K = _K;}
	void setBaseline(float _baseline) {baseline = _baseline;}
	void set_Max_frame(int maxframe) {max_frame = maxframe;}

	//get the left image
	static cv::Mat getImage(const std::string& path, int i);

private:
	cv::Mat K;
	float baseline;
	int max_frame;
	std::string left_image_path;
	std::string right_image_path;
	std::shared_ptr<OrbLoopDetector> loopDetector;
    std::shared_ptr<OrbVocabulary> voc;
    std::shared_ptr<KeyFrameSelection> keyFrameSelection;
    std::shared_ptr<GraphOptimizer> optimizer;


	const string estimated_text  = "Red color: estimated trajectory";
	const string ground_text	   = "Blue color: Groundtruth trajectory";
	const string loop_detected   = "GREEN Loop detected here";
	const string optimized_text  = "DarkRed Optimized trajectory";
	//to save the detection sequences in 
	map<int, int> detect_sequence;
	int detect_start;
	bool loopfound;
	int d_count;
	int matchingIdx;
};


#endif
