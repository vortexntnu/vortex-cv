#ifndef LINDETECTORPIPE_HPP
#define LINDETECTORPIPE_HPP

#include <opencv2/opencv.hpp>

struct Line {
    double slope;
    double intercept;
    double score;
    cv::Point start = {0, 0};
    cv::Point end = {0, 0};
};

struct HoughParams {
    float minTurnAngle;
    int size;
    int morph_close_size;
    float dist_thresh;
    int min_skeleton_component_size;
    int hough_threshold;
};

class LinedetectorPipe {
    float minTurnAngle_;
    int size_;
    cv::Mat orgImg_;
    double scaleX_;
    double scaleY_;
    int morph_close_size_;
    float dist_thresh_;
    int min_skeleton_component_size_;
    int hough_threshold_;
    cv::Mat processedImg_;

   public:
    void preprocess(cv::Mat& img, bool dist = true);
    void getEndPoints(Line& line, bool swap = false);
    std::vector<Line> detect(const cv::Mat& img, const int maxLines);

    LinedetectorPipe();
    ~LinedetectorPipe();
    LinedetectorPipe(HoughParams params) {
        minTurnAngle_ = params.minTurnAngle;
        size_ = params.size;
        morph_close_size_ = params.morph_close_size;
        dist_thresh_ = params.dist_thresh;
        min_skeleton_component_size_ = params.min_skeleton_component_size;
        hough_threshold_ = params.hough_threshold;
    }
};

#endif  // LINDETECTORPIPE_HPP
