
#include "parameter_reader.h"
#include "vo.h"
#include "Client.h"

ParameterReader param;

//load the poses from ground truth pose data files
void load_Pose(const std::string& path, vector<vector<float>>& poses);

int main(int argc, char** argv) {
   
    string folder = param.getData<string>("stereo_dir");
    string left_path = folder + "/image_0/%06d.png";
    string right_path = folder + "/image_1/%06d.png";
    string pose_path = param.getData<string>("pose_file") + "/poses/00.txt";
    string voc_file  = param.getData<string>("m_vocfile");

    int max_frame    = param.getData<int>("maxframe");
    //you have to configure your own path

    std::string host = "localhost";
    int port_num = 5000;
    Client* client = new Client(host, port_num);

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
    vo.playSequence(poses, client);
    cout << "You come to the end!" << endl;
    return 0;
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
  } 

}
