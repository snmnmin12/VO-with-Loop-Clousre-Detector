#pragma once

#include "common_headers.h"
#include "frame.h"

class FDetector {
private:
	cv::Ptr<cv::FeatureDetector> orb;
	cv::FlannBasedMatcher matcher;

public:
	FDetector() {
		orb = cv::ORB::create(1500);
	}

	//help to chang the descriptor to descriptor format required by DBoW2 class
	void restructure (cv::Mat& plain, vector<cv::Mat> &descriptors)
	{  
	  const int L = plain.rows;
	  descriptors.resize(L);
	  for (unsigned int i = 0; i < (unsigned int)plain.rows; i++) {
	  	descriptors[i] = plain.row(i);
	  }
	}

	void detectAndCompute(Frame::Ptr& frame) {

	    vector<cv::KeyPoint> keypoints1, keypoints2;
		cv::Mat descriptors1, descriptors2;

	 	orb->detectAndCompute(frame->left, cv::Mat(), keypoints1, descriptors1);
		orb->detectAndCompute(frame->right,cv::Mat(), keypoints2, descriptors2);

		//change to Bow feature
		frame->KeyPoints = keypoints1;
		restructure (descriptors1, frame->descriptors);

		//to generate the mathcing points and 3D points
		descriptors1.convertTo(descriptors1, CV_32F);
	    descriptors2.convertTo(descriptors2, CV_32F);
		
		//matching feature points
	    vector<vector<cv::DMatch>> matches;
	    // std::vector<cv::DMatch> bestMatches;
	    matcher.knnMatch(descriptors1, descriptors2, matches, 2);

	    //find the matching points
		for (int i = 0; i < matches.size(); i++) {
	    	const cv::DMatch& bestMatch = matches[i][0];  
			const cv::DMatch& betterMatch = matches[i][1];  
			if (bestMatch.distance < 0.7*betterMatch.distance) {
			    frame->keyPts1.push_back(keypoints1[bestMatch.queryIdx].pt);
		        frame->keyPts2.push_back(keypoints2[bestMatch.trainIdx].pt);
		        // bestMatches.push_back(bestMatch);
			}
		}
		// cv::Mat out;
		// cv::drawMatches(frame->left, keypoints1,
  //                       frame->right, keypoints2,
  //                       bestMatches, out );
  //       cv::imshow( "inlier matches", out );
  //       cv::waitKey(0);

	}
};