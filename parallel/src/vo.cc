#include "vo.h"
#include <condition_variable>
#include <mutex>

using namespace cv;
using namespace DBoW2;
using namespace DLoopDetector;

void VO2::initialize() {
   //feature detector
	poses = frameReader.poses;
	tracker.reset(new Track(frameReader));
    this->max_frame = frameReader.maxframe;
  //optimizer
  	optimizer.reset(new GraphOptimizer(paramReader));
 	 viewer.reset(new Viewer(frameReader, *optimizer));
}

void VO2::get3D_Points(Frame::Ptr& frame) {
	// This is to generate the 3D points
    cv::Mat P1  = K* M1;
    cv::Mat P2  = K* M2;

    cv::Mat landmarks;
    triangulatePoints(P1, P2, frame->keyPts1, frame->keyPts2, landmarks); 
    //normalize the 3D points
    for (int i = 0; i < landmarks.cols; i++) {
    cv::Point3f p;
    p.x = landmarks.at<float>(0, i)/landmarks.at<float>(3, i);
	p.y = landmarks.at<float>(1, i)/landmarks.at<float>(3, i);
	p.z = landmarks.at<float>(2, i)/landmarks.at<float>(3, i);
	//cout << p << endl;
	frame->WorldPts.push_back(p);
    }
}

void VO2::extract_keypoints(Frame::Ptr& frame)  {
	// Feature Detection and Extraction
	featureDetector.detectAndCompute(frame);
	get3D_Points(frame);
}

void VO2::playSequence(){

	mutex frame_mutex;
	condition_variable frame_updated;
	bool shutdownflag=false;

	thread mainloop=([&](){
	    int startIndex = 0;
	    std::cout << max_frame << std::endl;
	    cout << "max_frame: " << max_frame << endl;
	    //loop through all the images here
		for(int i = startIndex; i < max_frame; i++) {

			cout << i << endl;
			Frame::Ptr frame = frameReader.read(i);
			extract_keypoints(frame);

			tracker->track(frame);		
			Frame::Ptr frame_ = tracker->getFrame();
			optimizer->insertFrame(frame_);
			frame_updated.notify_one();
			//viewer->show();
		}
			optimizer->shutdown();
			tracker->shutdown();
			frame_updated.notify_all();
		}
	);	

	while(!shutdownflag) {
		unique_lock<mutex> lck(frame_mutex);
    	frame_updated.wait(lck);
    	cout <<"frame updated!"<<endl;
    	viewer->show();
	}

	mainloop,join();
	cout <<"System ended!"<< endl;
	viewer->pause();
}