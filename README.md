# VO-with-Loop-Clousre-Detector
=============

##Overview
This is the simple implementation for the stereo vo with loop closure detection and subsequent bundle adjustment optimization.

DLoopDetector requires [DLib](https://github.com/dorian3d/)
Bag of Word model is used for place recognition [DBoW2](https://github.com/dorian3d/DBoW2).

ORB feature extractor for feature extraction and the vocabulary is from ORBSLAM2.

When running bundle adjustment after loop detection, instead of running global, local with the most recent frames are used.

### Requirement
* OpenCV 3.2
* Kitti data 00 visual odometry [Kitty](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
* g2o optimizaiton package
* DBow2 and DLoop Detector for Loop closure detection 

## Install and usage notes
install g2o and opencv, for the other dependences, the files are included in the third party folder, run ./build.sh to run and execute the files.
![Loop Closure](path.png)
