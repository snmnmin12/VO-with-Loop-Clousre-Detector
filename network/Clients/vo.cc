#include "vo.h"

#include <fstream>
#include <string>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <algorithm>
#include <map>
#include <thread>
#include <time.h>

#define IMAGE_W 1241
#define IMAGE_H 376

using namespace std;
using namespace cv;
using namespace DBoW2;
using namespace DLoopDetector;

//help to chang the descriptor to descriptor format required by DBoW2 class
void restructure (cv::Mat& plain, vector<FORB::TDescriptor> &descriptors)
{  
  const int L = plain.rows;
  descriptors.resize(L);
  for (unsigned int i = 0; i < (unsigned int)plain.rows; i++) {
  	descriptors[i] = plain.row(i);
  }
}

void VO2::initialize(int max_frame, const std::string& m_vocfile, int num) {
   
    this->max_frame = max_frame;
    //parameters
    Params params;
    params.image_rows = IMAGE_W;
    params.image_cols = IMAGE_H;
    params.use_nss = true; // use normalized similarity score instead of raw score
    params.alpha = 0.5; // nss threshold
    params.k = 1; // a loop must be consistent with 1 previous matches
    params.geom_check = GEOM_DI; // use direct index for geometrical checking
    params.di_levels = 2; // use two direct index levels

    // Load the vocabulary to use
    voc.reset(new OrbVocabulary());
    cout << "Loading " <<m_vocfile << " vocabulary..." << endl;
    if (m_vocfile.size() > 3 && m_vocfile.substr(m_vocfile.size()-4,4).compare(".txt") == 0) {
    bool bVocLoad = voc->loadFromTextFile(m_vocfile);
    if(!bVocLoad)
    {
	std::cerr << "Wrong path to vocabulary. " << std::endl;
	std::cerr << "Falied to open at: " << m_vocfile << std::endl;
	exit(-1);
    }
    std::cout << "I am here now!" << std::endl;
  } else {
     voc->load(m_vocfile);
  }

  //loop detector initialized
  loopDetector.reset(new OrbLoopDetector(*voc, params));
  loopDetector->allocate(num);
}

VO2::VO2(const cv::Mat& _K, float _baseline, const string& _left_path, const string& _right_path):
		left_image_path(_left_path), right_image_path(_right_path), K(_K), baseline(_baseline) {
	loopfound = false;
	d_count = 0;
	matchingIdx = 0;
	detect_start = 0;
	optimizer.reset(new GraphOptimizer(false));
}

vector<Point3f> VO2::get3D_Points(const vector<Point2f>& feature_p1, const vector<Point2f>& feature_p2) const {

	// This is to generate the 3D points
    Mat M_left  = Mat::zeros(3,4, CV_64F);
    Mat M_right = Mat::zeros(3,4, CV_64F);
    M_left.at<double>(0,0) =1; 
    M_left.at<double>(1,1) =1;
    M_left.at<double>(2,2) =1;
    M_right.at<double>(0,0) =1;
    M_right.at<double>(1,1) =1;
    M_right.at<double>(2,2) =1;
    M_right.at<double>(0,3) =-baseline;

    M_left  = K*M_left;
    M_right = K*M_right;

    Mat landmarks;

    triangulatePoints(M_left, M_right, feature_p1, feature_p2, landmarks); 

    std::vector<Point3f> output;
    
    //normalize the 3D points
    for (int i = 0; i < landmarks.cols; i++) {
    Point3f p;
    p.x = landmarks.at<float>(0, i)/landmarks.at<float>(3, i);
	p.y = landmarks.at<float>(1, i)/landmarks.at<float>(3, i);
	p.z = landmarks.at<float>(2, i)/landmarks.at<float>(3, i);
	output.push_back(p);
    }

    return output;
}

void VO2::extract_keypoints(int index, const Mat& img1, const Mat& img2, vector<Point3f>&landmarks, vector<Point2f>& feature_points)  {

	assert(landmarks.size() == 0);
	assert(feature_points.size() == 0);

	// Feature Detection and Extraction
    static Ptr<FeatureDetector> orb = ORB::create(1500);
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;

	//loop detector here
	vector<FORB::TDescriptor> descriptors;

	std::thread left([&](){
		orb->detectAndCompute(img1,cv::Mat(), keypoints1, descriptors1);
		restructure (descriptors1, descriptors);
	});
	std::thread right([&](){
		orb->detectAndCompute(img2,cv::Mat(), keypoints2, descriptors2);
	});
	left.join();
	right.join();

	DetectionResult result;
	loopDetector->detectLoop(keypoints1, descriptors, result);
	detect_sequence[detect_start++] = index;
	if(result.detection() && (result.query - result.match > 100)) {
		d_count++;
		loopfound = true;
		matchingIdx = detect_sequence[result.match];
    	cout << "- Loop found with image " << matchingIdx << "!"<< endl;
	} else {
		d_count = 0;
		loopfound = false;
		cout << "No loop detected " << endl;
	}
   
     //to generate the mathcing points and 3D points
	descriptors1.convertTo(descriptors1, CV_32F);
    descriptors2.convertTo(descriptors2, CV_32F);
	
	//matching feature points
	static FlannBasedMatcher matcher;

    vector<vector<DMatch>> matches;
    matcher.knnMatch(descriptors1, descriptors2, matches, 2);

    vector<Point2f> match_points1;
    vector<Point2f> match_points2;
    vector<DMatch> bestMatches;
	for (int i = 0; i < matches.size(); i++) {
    	const DMatch& bestMatch = matches[i][0];  
		const DMatch& betterMatch = matches[i][1];  

	if (bestMatch.distance < 0.7*betterMatch.distance){
	    match_points1.push_back(keypoints1[bestMatch.queryIdx].pt);
        match_points2.push_back(keypoints2[bestMatch.trainIdx].pt);
        bestMatches.push_back(bestMatch);
	}
    }
	
   	feature_points = match_points1;
   	landmarks = get3D_Points(match_points1, match_points2);
}


//track the feature points in the next frame
vector<int> tracking(const Mat& ref_img, const Mat& curImg, vector<Point2f>& featurePoints, vector<Point3f>& landmarks) {

	vector<Point2f> nextPts;
	vector<uchar> status;
	vector<float> err;

	calcOpticalFlowPyrLK(ref_img, curImg, featurePoints, nextPts, status, err);

	std::vector<int> res;
	featurePoints.clear();

	//find the common landmarks and feature points
	for (int  j = status.size()-1; j > -1; j--) {
		if (status[j] != 1) {
			landmarks.erase(landmarks.begin()+j);		
		} else {
			featurePoints.push_back(nextPts[j]);
			res.push_back(j);
		}
	}
	std::reverse(res.begin(),res.end()); 
	std::reverse(featurePoints.begin(),featurePoints.end()); 

	return res;

}


void VO2::playSequence(const std::vector<std::vector<float>>& poses, Client* client){

        //load the first image pair for initialization
    int startIndex = 0;

    std::cout << max_frame << std::endl;
    //start poses
    Mat R_f = (cv::Mat_<double>(3, 3) << poses[startIndex][0], poses[startIndex][1], poses[startIndex][2],
                         poses[startIndex][4], poses[startIndex][5], poses[startIndex][6],
                         poses[startIndex][8], poses[startIndex][9], poses[startIndex][10]);
    //Mat::eye(3,3,CV_64F);
    Mat t_f = (cv::Mat_<double>(3,1)<<poses[startIndex][3], poses[startIndex][7], poses[startIndex][11]);

    //initializing
    g2o::Isometry3D pose = optimizer->cvMat2Eigen(R_f, t_f);
    optimizer->addVertex(startIndex, pose);

    int prevIndex = startIndex;
   	std::vector<PoseRT> oposes;
   	bool optim_start = true;

   	Mat left_img = getImage(left_image_path, startIndex);
	Mat right_img = getImage(right_image_path,startIndex);
	Mat& ref_img  = left_img;
	Mat curImg;
        
    //initialize the landmarks and feature points
	vector<Point3f> landmarks;
	vector<Point2f> featurePoints;
	extract_keypoints(startIndex, left_img, right_img, landmarks, featurePoints);
	vector<Point2f> h_plots;

	clock_t begin = clock();
    //loop through all the images here
	for(int i = startIndex + 1; i < max_frame; i++) {

		cout << i << endl;
		curImg = getImage(left_image_path, i);
		cv::Mat rcurImg = getImage(right_image_path, i);
		vector<Point3f> tempmarks; vector<Point2f> tempPoints;
		extract_keypoints(i, curImg, rcurImg, tempmarks, tempPoints);
	    //track the feature points and landmarks
		vector<int> tracked_points = tracking(ref_img, curImg, featurePoints, landmarks);
		if (landmarks.size() < 10) continue;
		//solve the pnp 
		Mat dist_coeffs = Mat::zeros(4,1,CV_64F);
		Mat rvec, tvec;
		vector<int> inliers;
		solvePnPRansac(landmarks, featurePoints, K, dist_coeffs,rvec, tvec,false, 100, 8.0, 0.99, inliers);// inliers);
		if (inliers.size() < 5) continue;

		float inliers_ratio = inliers.size()/float(landmarks.size());
		cout << "inliers ratio: " << inliers_ratio << endl;

		//update the original trajectory
		Mat R, tvec2;
		Rodrigues(rvec,R);
		R = R.t(); tvec2 = -R*tvec;
		t_f +=  R_f * tvec2;
		R_f = R*R_f;

		// if (isKeyFrame) {
		if (loopfound)  { 
			optim_start = true;
			optimizer->addVertex(i, pose, matchingIdx);
		} else {
	   		Eigen::Isometry3d T = optimizer->cvMat2Eigen(R_f, t_f);
			optimizer->addVertex(i, T);
	    }

		Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity()*0.5;
		Rodrigues(rvec,R);
		Eigen::Isometry3d T = optimizer->cvMat2Eigen(R, tvec);
		g2o::Isometry3D rel_pose = T.inverse();
	    optimizer->addEdge(prevIndex, i, rel_pose, information);
	   	prevIndex = i;

	   	//vector string
	   	std::string data = "#" + std::to_string(i) + "[";
	   	//loop found and two loop detection with 100 frames to trigger optimization again
	   	if (loopfound && (d_count == 1 || d_count > 100)) {
	   		d_count = 0;
	   		// optim_start = false;
	   		cout <<"processing optimizer"<<endl;
	   		optimizer->optimizeGraph(true);
	   		oposes.clear();h_plots.clear();
	   		optimizer->getPoses(oposes);
	   		PoseRT RT = optimizer->getPoses(matchingIdx);
			t_f = RT.t; R_f = RT.R;
	   		cout <<"Processing the rotation and transformation!"<< oposes.size() <<endl;
	   		for (int i = 0; i < oposes.size(); i++) {
	   			Point2f o_center = Point2f(int(oposes[i].t.at<double>(0)) + 300, int(oposes[i].t.at<double>(2)) + 100);
	   			data += std::to_string(int(o_center.x)) + " ";
				data += std::to_string(int(o_center.y)) + " ";
   			}
	    } else {
	    	Point2f center = Point2f(int(t_f.at<double>(0)) + 300, int(t_f.at<double>(2)) + 100);
			data += std::to_string(int(center.x)) + " ";
			data += std::to_string(int(center.y)) + " ";
	    }

	    data += "]\r\n";
	    client->send_msg(data);
	    //edge 
	    //visualization
	    cout << t_f << endl;
		cout << "["<<poses[i][3] << ", " << poses[i][7] << ", " << poses[i][11] <<"]"<<endl;

		featurePoints = tempPoints;
		landmarks = tempmarks;
		cout <<"feature point size is "<< featurePoints.size() << endl;
		ref_img = curImg;
	}

	clock_t end = clock();
	double runtime = 1000* (double)(end - begin) / CLOCKS_PER_SEC; 
	printf("image processing time is %fms\n", runtime);
}

//############################################################
// static methods here

Mat VO2::getImage(const string& raw_path, int i) {
    char path[100];
    sprintf(path, raw_path.c_str(), i);
    Mat img = imread(path);
    if(! img.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
    }
    return img;
}