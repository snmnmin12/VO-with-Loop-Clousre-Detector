#include "util.h"

Eigen::Isometry3d cvMat2Eigen( const cv::Mat& R, const cv::Mat& tvec ){

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


g2o::SE3Quat    toSE3Quat(const Eigen::Isometry3d& T)
{
    Eigen::Matrix3d R = T.rotation();
    
    Eigen::Vector3d t( T(0,3), T(1,3), T(2,3) );

    return g2o::SE3Quat( R, t );
};


void Eigen2cvMat(const Eigen::Isometry3d& matrix, cv::Mat& R, cv::Mat& tvec) {

    R = cv::Mat::zeros(3,3,CV_64F);
    tvec = cv::Mat::zeros(3,1,CV_64F);

    for ( int i=0; i<3; i++ )
    for ( int j=0; j<3; j++ ) 
        R.at<double>(i,j) = matrix(i,j);

    tvec.at<double>(0) = matrix(0, 3); 
    tvec.at<double>(1) = matrix(1, 3);  
    tvec.at<double>(2) = matrix(2, 3);
}