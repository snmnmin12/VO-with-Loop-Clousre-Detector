#ifndef __GRAPH_OPTIMIZATION__
#define __GRAPH_OPTIMIZATION__

//g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"

#include <map>
#include <set>

typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

struct PoseRT {
	cv::Mat R;
	cv::Mat t;
};

class GraphOptimizer
{

private:
	int n_vertices;
	SlamBlockSolver* solver_ptr;
	SlamLinearSolver* linearSolver;
	g2o::OptimizationAlgorithmLevenberg *solver;
	g2o::SparseOptimizer optimizer;
	std::vector<int> poseIndexes;
	std::map<int, int> sequences;
	std::set<std::pair<int,int>> Edges;
	std::vector<int> history;
public:
	GraphOptimizer(bool verbose = true);
	int addVertex(int Id, const g2o::Isometry3D& pose, int existId = -1);
	  /*!Adds an edge that defines a spatial constraint between the vertices "fromId" and "toId" with Identity information matrix of the added edge.*/
	void addEdge(const int fromId, const int toId, const g2o::Isometry3D &rel_pose, const Eigen::Matrix<double,6,6> &information);
	  /*!Calls the graph optimization process to determine the pose configuration that best satisfies the constraints defined by the edges.*/
	void optimizeGraph(bool local = false);
	  /*!Returns a vector with all the optimized poses of the graph.*/
	void getPoses(std::vector<PoseRT> &poses);
	/*!Returns the poses of the optimizer.*/
	PoseRT getPoses(int index);
	  /*find the index of the node for given frame index.*/
	int find(int id);
	static Eigen::Isometry3d cvMat2Eigen( const cv::Mat& rvec, const cv::Mat& tvec );
	static void Eigen2cvMat(const Eigen::Isometry3d& matrix, PoseRT& position);
};

#endif