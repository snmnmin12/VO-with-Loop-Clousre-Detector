#include "vo.h"
#include <fstream>
using namespace std;


string left_path =  "/Users/HJK-BD/Downloads/kitti/00/image_0/%06d.png";
string right_path = "/Users/HJK-BD/Downloads/kitti/00/image_1/%06d.png"; 
string pose_path =  "/Users/HJK-BD/Downloads/kitti/poses/00.txt";
string  voc_file  = "./resources/ORBvoc.txt";
//load the poses from ground truth pose data files
void load_Pose(const std::string& path, vector<vector<float>>& poses);

int main(int argc, char** argv) {
   
    int max_frame = 4541;
    if (argc == 2) max_frame = atoi(argv[1]);

	//you have to configure your own path

    cout <<"Program starts!"<<endl;

    cv::Mat K = (cv::Mat_<double>(3, 3) << 7.188560000000e+02, 0, 6.071928000000e+02,
                         0, 7.188560000000e+02, 1.852157000000e+02,
                         0, 0, 1);
    float baseline = 0.54;

    //vo.set_Max_frame(max_frame);
   
    vector<vector<float>> poses;
    load_Pose(pose_path, poses);
    
    VO2 vo(K, baseline, left_path, right_path);
    vo.initialize(max_frame, voc_file, poses.size());

    cout << "start the sequence" << endl;
    vo.playSequence(poses);
    cout << "You come to the end!" << endl;
    return 0;
}


void load_Pose(const std::string& path, vector<vector<float>>& poses) {

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
    cout << "Unable to open file"; 
  } 

}
