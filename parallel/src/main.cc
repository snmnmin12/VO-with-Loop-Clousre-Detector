#include "vo.h"
#include <fstream>
#include "parameter_reader.h"
using namespace std;

int main(int argc, char** argv) {
   
    //vo.set_Max_frame(max_frame);
    ParameterReader parameter_reader;
    VO2 vo(parameter_reader);
    vo.initialize();
    cout << "start the sequence" << endl;
    vo.playSequence();
    cout << "You come to the end!" << endl;
    return 0;
}

