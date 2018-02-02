/*
 * File: DLoopDetector.h
 * Date: November 2017
 */

#ifndef __D_T_LOOP_DETECTOR__
#define __D_T_LOOP_DETECTOR__

#include "DBoW2/DBoW2.h"
#include "TemplatedLoopDetector.h"
#include "DBoW2/FORB.h"

using namespace DLoopDetector;
using namespace DBoW2;
using namespace std;

/// SURF64 Loop Detector
typedef DLoopDetector::TemplatedLoopDetector
  <FORB::TDescriptor, FORB> OrbLoopDetector;

typedef OrbLoopDetector::Parameters Params;

/// Generic class to create functors to extract features
template<class TDescriptor>
class FeatureExtractor
{
public:
  /**
   * Extracts features
   * @param im image
   * @param keys keypoints extracted
   * @param descriptors descriptors extracted
   */
  virtual void operator()(const cv::Mat &im, 
    vector<cv::KeyPoint> &keys, vector<TDescriptor> &descriptors) const = 0;
};
//-----------------------------------------------------------------------

#endif

