
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <random>
#include <exception>
#include <stdexcept>
#include "Server.h"
#include "opencv2/opencv.hpp"


using namespace std;

#define WHITE   cv::Scalar(255,255,255)
#define RED     cv::Scalar(0,   0, 255)
#define BLUE    cv::Scalar(255, 0,  0)
#define GREEN   cv::Scalar(0, 255, 0)
#define DarkRed cv::Scalar(0, 0, 127)

const string estimated_text  = "Red color: estimated trajectory";
const string ground_text	 = "Blue color: Groundtruth trajectory";
const string pose_path = "/Users/HJK-BD/Downloads/kitti/poses/00.txt";
static vector<vector<float>> true_poses;
int curIndex = 0;


//load the poses from ground truth pose data files
void load_Pose(const std::string& path, vector<vector<float>>& poses);
//draw the circles
void draw(cv::Mat & traj, vector<int>& data);


int main() {

	cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
    traj = cv::Scalar(255,255,255);

   	load_Pose(pose_path, true_poses);

    int port_num = 5000;
 //    cout << "input port number of server\n";
	// cin  >> port_num;
	cv::imshow( "Trajectory", traj);

	Server *myServer;
	cout << "waiting for connection\n";
	myServer = new Server(port_num);
	cout << "Connected!\n" << endl;
	myServer->send_msg("Connected!\r\n");

	while (1) {
		string s = myServer->read_msg();
		if (s.size() == 0 || s[0] !='#') break;

		auto b1 = s.find_first_of("[");
		auto b2 = s.find_first_of("]");	

		if (b1 == string::npos) exit(1);
		if (b2 == string::npos) exit(1);

		curIndex = stoi(s.substr(1, b1-1));
		// cout << curIndex << endl;

		string data = s.substr(b1 + 1, b2 - b1 - 1);

		std::istringstream iss(data);
		vector<int> coords;
		string token;
		char delimiter=' ';
		while (std::getline(iss, token, delimiter))
	    {
	      coords.push_back(stoi(token));
	    }

	    draw(traj, coords);
		// cout << results << endl;
	}
	cout << "Program finished\n";
    cv::waitKey(0);
    return 0;
}


void draw(cv::Mat & traj, vector<int>& data) {

	if (data.size() > 2) {
		traj = WHITE;
		for (int i = 0; i < curIndex; i++) {
			cv::circle(traj, cv::Point2f(true_poses[i][3] + 300, true_poses[i][11] + 100), 1, BLUE, 1);
		}
	}
	//draw the ground truth data
	cv::circle(traj, cv::Point2f(true_poses[curIndex][3]+300, true_poses[curIndex][11]+100), 1, BLUE, 1);

	//draw the estimated data
	for (int i = 0; i < data.size(); i += 2) {
		cv::circle(traj, cv::Point2f(data[i], data[i+1]), 1, RED, 1);
	}

	putText(traj, estimated_text, cv::Point2f(10,50), cv::FONT_HERSHEY_PLAIN, 1, RED, 1, 5);
	putText(traj, ground_text, cv::Point2f(10,70), cv::FONT_HERSHEY_PLAIN, 1, BLUE, 1, 5);

	cv::imshow( "Trajectory", traj);
	//string img_path = to_string(curIndex) + ".png";
	cv::imwrite(img_path, traj);
	cv::waitKey(1);
}


void load_Pose(const std::string& path, vector<vector<float>>& poses) {

  cout << "Loading path file: " << path << endl;
  ifstream myfile(path);
  string line;
  if (myfile.is_open())
  {
    while (getline(myfile,line) )
    {    
        std::stringstream ss;
        ss << line;
        std::vector<float> v;
        v.resize(12);
        for (int i = 0; i < 12; i++) {
     	float value;
           ss>>value;
       v[i] = value;
        }
         poses.push_back(v);

    }
    myfile.close();
  } else {
    cout << "Unable to open file\n"; 
    exit(1);
  } 

}