#pragma once

#include <thread>
#include <mutex>
#include <functional>
#include <condition_variable>

#include "common_headers.h"
#include "frame.h"
#include "PlaceRecognition/PlaceDetector.h"
#include "KeyFrameSelection.h"
#include "concurrent_queue.h"


typedef TemplatedKeyFrameSelection<DBoW2::FORB::TDescriptor, DBoW2::FORB> KeyFrameSelection;
typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

class GraphOptimizer
{

public:
GraphOptimizer(const ParameterReader& para, bool verbose = false);

void insertFrame(Frame::Ptr frame);
int addVertex(int Id, const g2o::Isometry3D& pose, int existId = -1);
  /*!Adds an edge that defines a spatial constraint between the vertices "fromId" and "toId" with Identity information matrix of the added edge.*/
void addEdge(const int fromId, const int toId, const g2o::Isometry3D &rel_pose, const Eigen::Matrix<double,6,6> &information);
  /*find the index of the node for given frame index.*/
int find(int id);

void mainLoop();
//thread shutdown
void shutdown() {
	shutdownflag = true;
	if (opthread != nullptr) {opthread->join();}
}

vector<Frame::Ptr> getFrames() const {
	unique_lock<mutex> lck(keyframes_mutex);
	vector<Frame::Ptr> copy = keyFrames;
	lck.unlock();
	return copy;
}

public:
	//thread signa
	bool shutdownflag = false;

	//main thread for running optimizaiton
	shared_ptr<thread> opthread;
 	//key frames
	mutable mutex keyframes_mutex;

	// mutex keyframe_update_mutex;
	// condition_variable keyframe_updated;
	Queue<Frame::Ptr> que;

	Frame::Ptr ref_frame;
	vector<Frame::Ptr> keyFrames;

	//buffered to avoid keyframes resource contention
	vector<Frame::Ptr> buffer;
	int frame_size=0;

	//detector for loop detection
	shared_ptr<OrbLoopDetector> loopDetector;

	//Orb vocabulary to help loop detection
    shared_ptr<OrbVocabulary> voc;

    //help to select key frame
    shared_ptr<KeyFrameSelection> keyFrameSelection;

    //read the parameter reader
    const ParameterReader& paraReader;

    //detection sequences
    map<int, int> detect_sequence;
	int 		  detect_start = 0;
	bool 		  loopfound=false;
	int 		  d_count =0;
	int 		  matchingIdx=0;

private:
	//used for g2o
	int n_vertices;
	SlamBlockSolver* solver_ptr;
	SlamLinearSolver* linearSolver;
	g2o::OptimizationAlgorithmLevenberg *solver;
	g2o::SparseOptimizer optimizer;
	vector<int> poseIndexes;
	map<int, int> sequences;
	set<pair<int,int>> Edges;
	vector<int> history;

};