#include "opencv_tools.hpp"

Ptr<DescriptorExtractor> getDescriptorExtractor() {
    Ptr<DescriptorExtractor> extractor;
#if CV_MAJOR_VERSION == 2
    // The extractor can be any of (see OpenCV features2d.hpp):
    // extractor = Ptr(new BriefDescriptorExtractor());
    // extractor = Ptr(new ORB());
    extractor = Ptr<DescriptorExtractor>(new SIFT());
    // extractor = Ptr(new SURF(600.0));
    // extractor = Ptr(new BRISK());
    // extractor = Ptr(new FREAK());
#elif CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 3)
    extractor = xfeatures2d::SIFT::create();
#else  // >= 4.3.0
    extractor = SIFT::create();
#endif
    return extractor;
}

Ptr<FeatureDetector> getFeatureDetector() {
    Ptr<FeatureDetector> detector;
#if CV_MAJOR_VERSION == 2
    // detector = Ptr(new DenseFeatureDetector());
    // detector = Ptr(new FastFeatureDetector());
    // detector = Ptr(new GFTTDetector());
    // detector = Ptr(new MSER());
    // detector = Ptr(new ORB());
    detector = Ptr<FeatureDetector>(new SIFT());
    // detector = Ptr(new StarFeatureDetector());
    // detector = Ptr(new SURF(600.0));
    // detector = Ptr(new BRISK());
#elif CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 3)
    detector = xfeatures2d::SIFT::create();
#else  // >= 4.3.0
    detector = SIFT::create();
#endif

    return detector;
}
