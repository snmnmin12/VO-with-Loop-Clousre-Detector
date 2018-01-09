#pragma once

#include "common_headers.h"

Eigen::Isometry3d cvMat2Eigen( const cv::Mat& R, const cv::Mat& tvec );
g2o::SE3Quat    toSE3Quat(const Eigen::Isometry3d& T);
void Eigen2cvMat(const Eigen::Isometry3d& matrix, cv::Mat& R, cv::Mat& T);