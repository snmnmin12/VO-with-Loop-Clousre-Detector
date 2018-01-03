

#include "GraphOptimization.h"
#include <g2o/types/slam3d/types_slam3d.h>

// #include "bundle.h"
#include <unordered_map>
#include <unordered_set>

GraphOptimizer::GraphOptimizer(bool verbose)
{   
    linearSolver = new SlamLinearSolver();
    solver_ptr = new SlamBlockSolver(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    solver->setUserLambdaInit(1e-10);
    optimizer.setVerbose(verbose);
    n_vertices = 0;
}

int GraphOptimizer::addVertex(int Id, const g2o::Isometry3D &pose, int exisitId)
{
    
    history.push_back(Id);

    if (exisitId != -1) {
        sequences[Id] = find(exisitId);
        return n_vertices-1;
    }
    //Set up node
    g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
    v_se3->setId(Id);
    v_se3->setMarginalized(false);

    v_se3->setEstimate(pose);

    if (sequences.size() == 0)
    { //First node
        v_se3->setFixed(true);
    }
    // add to optimizer
    optimizer.addVertex(v_se3);
    sequences[Id] = Id;
    n_vertices++;
    return (n_vertices - 1);
}

void GraphOptimizer::addEdge(const int fromId, const int toId, const g2o::Isometry3D &rel_pose, const Eigen::Matrix<double,6,6> &information)
{

    g2o::VertexSE3* p1 = static_cast<g2o::VertexSE3*>(optimizer.vertex(sequences[fromId]));
    g2o::VertexSE3* p2 = static_cast<g2o::VertexSE3*>(optimizer.vertex(sequences[toId]));
    if (p1 == nullptr || p2 == nullptr)  {
        std::cout << fromId << ": " << sequences[fromId] <<", "<< toId << " "<< sequences[toId] << std::endl;
        std::cout << "Nul ptr detected" << std::endl;
        getchar();
    }
    
    int from = sequences[fromId];
    int to   = sequences[toId];

    std::pair<int, int> edge = {from, to};
    if (Edges.find(edge) != Edges.end()) {
        return;
    }
    Edges.insert(edge);

    g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3;
    e_se3->setVertex(0, optimizer.vertex(from));
    e_se3->setVertex(1, optimizer.vertex(to));
    e_se3->setMeasurement(rel_pose);
    e_se3->setRobustKernel( new g2o::RobustKernelHuber() );
    //Set the information matrix
    e_se3->setInformation(information);
    optimizer.addEdge(e_se3);
}


void GraphOptimizer::getPoses(std::vector<PoseRT> &poses)
{
    poses.clear();
    poses.resize(sequences.size());

    int id = 0;
    for(auto& ele : sequences)
    {
        //Get the Isometry3D estimate from the G2O vertex pose
        g2o::VertexSE3* v_se3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(ele.second));
        g2o::Isometry3D optimized_pose = v_se3->estimate();
        //Set the optimized pose to the vector of poses
        Eigen2cvMat(optimized_pose, poses[id++]);
    }
}

PoseRT GraphOptimizer::getPoses(int index) {

    PoseRT RT;
    g2o::VertexSE3* v_se3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(sequences[index]));
    g2o::Isometry3D optimized_pose = v_se3->estimate();
    //Set the optimized pose to the vector of poses
    Eigen2cvMat(optimized_pose, RT);
    return RT;
}

void GraphOptimizer::optimizeGraph(bool local)
{   
    for (int i = 0; i < history.size(); i++) {
        optimizer.vertex(sequences[history[i]])->setFixed(false);
    }

    optimizer.vertex(sequences[history[0]])->setFixed(true);

    if (local) {
        //local optimization
        int large = history.size()/2;
        large = std::min(large, 1000);
        for (int i = 0; i < history.size() - large; i++) {
            if (i < history.size() - large) {
                optimizer.vertex(sequences[history[i]])->setFixed(true);
            } else {
                optimizer.vertex(sequences[history[i]])->setFixed(false);
            }
        }
    }

    optimizer.initializeOptimization();
    optimizer.computeInitialGuess();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);
}


int GraphOptimizer::find(int id) {
    if (sequences[id] == id) return id;
    return find(sequences[id]); 
}

// ################################################################
// static methods here

Eigen::Isometry3d GraphOptimizer::cvMat2Eigen( const cv::Mat& R, const cv::Mat& tvec ){

    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ ) 
            r(i,j) = R.at<double>(i,j);
  
//     // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(1,0); 
    T(2,3) = tvec.at<double>(2,0);

    return T;

}

// //将eigen 矩阵转换为opencv矩阵
void GraphOptimizer::Eigen2cvMat(const Eigen::Isometry3d& matrix, PoseRT& position) {

    cv::Mat R = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3,1,CV_64F);

    for ( int i=0; i<3; i++ )
    for ( int j=0; j<3; j++ ) 
        R.at<double>(i,j) = matrix(i,j);

    tvec.at<double>(0) = matrix(0, 3); 
    tvec.at<double>(1) = matrix(1, 3);  
    tvec.at<double>(2) = matrix(2, 3);

    position.R = R;
    position.t = tvec;
}
