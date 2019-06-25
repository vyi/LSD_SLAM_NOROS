
#ifdef HAVE_FABMAP
#pragma once
#include <opencv2/core/core.hpp>

namespace of2 {
class FabMap;
}

namespace cv {
    class FeatureDetector;
    class BOWImgDescriptorExtractor;
}


namespace lsd_slam
{


class Frame;

/** Interface to openFabMap. */
class FabMap
{
public:
    /** Initializes FabMap. */
    FabMap();

    /** Writes out the confusion matrix if enabled. */
    ~FabMap();

    /** Adds the keyframe to the set of frames to compare against and returns
     *  its (non-negative) ID in FabMap (different to the keyframe ID).
     *  Returns -1 if the frame cannot be added due to an error. */
// 	int add(KeyFrame* keyframe);

    /** Checks if the keyframe is determined to be the same as an already
     *  added frame and if yes, returns its ID. If not, returns -1.
     *  Does not directly return a KeyFrame pointer to allow for KeyFrames
     *  being deleted. */
// 	int compare(KeyFrame* keyframe);

    /** Combination of compare() followed by add() (more efficient). */
    void compareAndAdd(Frame* keyframe, int* out_newID, int* out_loopID);

    /** Returns if the class is initialized correctly (i.e. if the required
     *  files could be loaded). */
    bool isValid() const;

private:
    int nextImageID;
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::BOWImgDescriptorExtractor> bide;
    cv::Ptr<of2::FabMap> fabMap;

    bool printConfusionMatrix;
    cv::Mat confusionMat;

    bool valid;
};

}
#endif
