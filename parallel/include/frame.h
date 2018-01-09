#pragma once

#include "common_headers.h"
#include <mutex>
#include <iomanip>
#include "parameter_reader.h"


class Frame {

public:
	typedef shared_ptr<Frame> Ptr;
	int id  =-1;                //-1表示该帧不存在
	cv::Mat left, right;
	vector<cv::Point2f> keyPts1, keyPts2;
	vector<cv::Point3f>  WorldPts;
	//for loop detection
	vector<cv::KeyPoint> KeyPoints;
	vector<cv::Mat> descriptors;
	//从当前帧到世界坐标系的变换
    Eigen::Isometry3d   T_f_w = Eigen::Isometry3d::Identity();
    std::mutex   mutexT;

public:
	void setTransform( const Eigen::Isometry3d& T )
    {
        std::unique_lock<std::mutex> lck(mutexT);
        T_f_w = T;
    }
    
    Eigen::Isometry3d getTransform()  
    {
        std::unique_lock<std::mutex> lck(mutexT);
        return T_f_w;
    }
};



class FrameReader {
public:
	//for stereo image folder
	FrameReader(ParameterReader& para): parameterReader(para) {

		baseline = para.getData<double>("baseline");
		pose_file = para.getData<string>("pose_file");
		stereo_dir = para.getData<string>("stereo_dir");

		camera = para.getCamera();
		K = (cv::Mat_<double>(3, 3) << camera.fx, 0,camera.cx,
                         0, camera.fy, camera.cy,
                         0, 0, 1);
		baseline = para.getData<double>("baseline");
		maxframe = para.getData<int>("maxframe");
		cout << "maxframe: " << maxframe << endl;
		m_vocfile= para.getData<string>("m_vocfile");
		LoadPathFiles();
	}

	void LoadPathFiles() {

		//load the path files
		  ifstream myfile(pose_file);
		  string line;
		  if (myfile.is_open())
		  {
		    while (getline(myfile,line) )
		    {    
			  std::stringstream ss;
			  Eigen::Isometry3d pose;
			  Eigen::Matrix3d r;
			  float tx, ty, tz;
			  ss << line;
			  ss>>r(0,0);
			  ss>>r(0,1);
			  ss>>r(0,2);
			  ss>>tx;
			  ss>>r(1,0);
			  ss>>r(1,1);
			  ss>>r(1,2);
			  ss>>ty;
			  ss>>r(2,0);
			  ss>>r(2,1);
			  ss>>r(2,2);
			  ss>>tz;

			  Eigen::AngleAxisd angle(r);
			  pose = angle;
			  pose(0,3)=tx;
			  pose(1,3)=ty;
			  pose(2,3)=tz;
			  poses.push_back(pose);

		    }
		    myfile.close();
		  } else {
		    cout << "Unable to open file"; 
		    exit(1);
		  } 

	}

	Frame::Ptr read(int currentIndex) {

		if (currentIndex >= maxframe) {
			cout << "maximum frame reached! " << endl;
			return nullptr;
		}

		Frame::Ptr frame(new Frame);
		frame->id    = currentIndex;
		stringstream ss;
		ss << setfill('0') << setw(6) << currentIndex;
		string left = stereo_dir+"/image_0/"+ss.str()+".png";
		string right= stereo_dir+"/image_1/"+ss.str()+".png";
		frame->left  = cv::imread(left);
		frame->right = cv::imread(right);
		//frame->camera = K;
		//frame->baseline = baseline;
		//frame->setTransform(poses[currentIndex]);
		return frame;
	}


public:
    
    int maxframe;
    
    string pose_file;
    string stereo_dir;
    string m_vocfile;

    //vector<string>  leftFiles, rightFiles;
    vector<Eigen::Isometry3d> poses;

    const ParameterReader&  parameterReader;
    CAMERA_INTRINSIC_PARAMETERS camera;
    cv::Mat  K;
    double baseline;
};