#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <pipeline_line_fitting/linedetectorPipe.hpp>

static std::ofstream& ransac_log() {
    static std::ofstream f("/tmp/ransac_debug.log", std::ios::app);
    return f;
}

static std::string now_ms() {
    auto tp = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  tp.time_since_epoch()).count();
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(ms));
    return buf;
}

void removeBorderArtifacts(cv::Mat& img) {
    img.row(0).setTo(cv::Scalar(0));
    img.row(img.rows - 1).setTo(cv::Scalar(0));
    img.col(0).setTo(cv::Scalar(0));
    img.col(img.cols - 1).setTo(cv::Scalar(0));
}

void LinedetectorPipe::preprocess(cv::Mat& img, bool dist) {
    // Calculate scaling factors
    scaleX_ = static_cast<double>(size_) / img.cols;
    scaleY_ = static_cast<double>(size_) / img.rows;

    cv::resize(img, img, cv::Size(size_, size_));

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(morph_close_size_, morph_close_size_));
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);

    // Apply distance transform to get the center lines
    if (dist) {
        cv::Mat dist_img;
        cv::distanceTransform(img, dist_img, cv::DIST_L2, 5);
        cv::normalize(dist_img, dist_img, 0, 1.0, cv::NORM_MINMAX);

        // Threshold the distance transform image to get the skeleton
        cv::threshold(dist_img, img, dist_thresh_, 1.0, cv::THRESH_BINARY);
    }

    // Convert the image to 8-bit
    img.convertTo(img, CV_8U, 255);

    // Apply morphological operations to clean up the result
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,
    // 3));
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);

    // Skeletonize the image using Zhang-Suen thinning algorithm
    cv::ximgproc::thinning(img, img, cv::ximgproc::THINNING_ZHANGSUEN);

    // Drop skeleton connected components smaller than the threshold — these are
    // noise blobs that survived thinning and would seed bad RANSAC lines.
    if (min_skeleton_component_size_ > 0) {
        cv::Mat labels, stats, centroids;
        int n_labels =
            cv::connectedComponentsWithStats(img, labels, stats, centroids);
        for (int lbl = 1; lbl < n_labels; ++lbl) {
            if (stats.at<int>(lbl, cv::CC_STAT_AREA) <
                min_skeleton_component_size_) {
                img.setTo(0, labels == lbl);
            }
        }
    }

    removeBorderArtifacts(img);
}

LinedetectorPipe::LinedetectorPipe() {};
LinedetectorPipe::~LinedetectorPipe() {};

void LinedetectorPipe::postprocess() {}

int LinedetectorPipe::detectSingleLine(const arma::mat& points,
                                       const arma::mat& values,
                                       const std::vector<Line>& lines,
                                       const int i,
                                       bool flipped) {
    // Extract columns and reshape
    if (points.n_rows < 5) {
        return 1;
    }

    arma::mat X = points.col(1);
    X.reshape(points.n_rows, 1);
    arma::mat y = points.col(0);
    y.reshape(points.n_rows, 1);

    // Set the d parameter for RANSAC
    int d = points.n_elem * fracOfPoints_;
    randsac_.d = d;

    // Fit the RANSAC model
    randsac_.fit(X, y, values, lines, flipped);

    // Check the best_fit and bestValue conditions
    if (randsac_.bestFit.params.size() == 0 ||
        randsac_.bestScore < finalScorethresh_) {
        return 1;
    }

    return 0;
}

void LinedetectorPipe::getEndPoints(Line& line, bool swap) {
    int minX = -1;
    int maxX = -1;
    int minX_yval = -1;
    int maxX_yval = -1;
    for (double x = 0; x < size_; x += 0.05) {
        int y = line.slope * x + line.intercept;

        if (y < 0 || y >= size_) {
            continue;
        }
        int pixel;
        if (swap) {
            pixel = orgImg_.at<uchar>(x, y);
        } else {
            pixel = orgImg_.at<uchar>(y, x);
        }
        if (pixel > 0) {
            if (minX == -1) {
                minX = x;
                minX_yval = y;
            }
            if (x > maxX) {
                maxX = x;
                maxX_yval = y;
            }
        }
    }

    if (swap) {
        line.start = cv::Point(static_cast<int>(minX_yval / scaleX_),
                               static_cast<int>(minX / scaleY_));
        line.end = cv::Point(static_cast<int>(maxX_yval / scaleX_),
                             static_cast<int>(maxX / scaleY_));
    } else {
        line.start = cv::Point(static_cast<int>(minX / scaleX_),
                               static_cast<int>(minX_yval / scaleY_));
        line.end = cv::Point(static_cast<int>(maxX / scaleX_),
                             static_cast<int>(maxX_yval / scaleY_));
    }
}

std::vector<Line> LinedetectorPipe::detect(const cv::Mat& img,
                                           const int maxLines = 3) {
    orgImg_ = img.clone();
    cv::resize(orgImg_, orgImg_, cv::Size(size_, size_));
    processedImg_ = img.clone();
    preprocess(processedImg_);

    // Find points where img > 0
    std::vector<cv::Point> pointList;
    cv::findNonZero(processedImg_, pointList);

    // Convert points to arma::mat
    arma::mat points(pointList.size(), 2);
    for (size_t i = 0; i < pointList.size(); ++i) {
        points(i, 0) = pointList[i].y;
        points(i, 1) = pointList[i].x;
    }

    // Extract values from the image at the points
    arma::mat values(pointList.size(), 1);
    for (size_t i = 0; i < pointList.size(); ++i) {
        values(i, 0) = processedImg_.at<uchar>(pointList[i].y, pointList[i].x);
    }

    std::vector<Line> lines;

    auto& rlog = ransac_log();
    rlog << "FRAME t=" << now_ms()
         << " skel_pts=" << points.n_rows
         << " max_lines=" << maxLines << "\n";

    // Returns true if `candidate` intersects all lines in `lines` within the
    // image frame. Two segments of the same pipe are parallel/collinear and
    // their intersection lies far outside the image. A genuine junction arm
    // meets the pipe at the junction point which must be visible in frame.
    auto intersects_in_frame = [&](const Line& candidate) -> bool {
        for (const auto& prev : lines) {
            double ax = prev.start.x * scaleX_, ay = prev.start.y * scaleY_;
            double bx = prev.end.x * scaleX_, by = prev.end.y * scaleY_;
            double cx = candidate.start.x * scaleX_,
                   cy = candidate.start.y * scaleY_;
            double ex = candidate.end.x * scaleX_,
                   ey = candidate.end.y * scaleY_;
            // General form ax+by+c=0 for each line.
            double p1 = -(by - ay), q1 = bx - ax,
                   r1 = (by - ay) * ax - (bx - ax) * ay;
            double p2 = -(ey - cy), q2 = ex - cx,
                   r2 = (ey - cy) * cx - (ex - cx) * cy;
            double det = p1 * q2 - p2 * q1;
            if (std::abs(det) < 1e-6)
                return false;  // parallel → same pipe
            double xi = (-r1 * q2 + r2 * q1) / det;
            double yi = (-r2 * p1 + r1 * p2) / det;
            if (xi < 0.0 || xi > size_ || yi < 0.0 || yi > size_)
                return false;
        }
        return true;
    };

    for (int i = 0; i < maxLines; ++i) {
        const size_t pts_before = points.n_rows;
        int returnCode = detectSingleLine(points, values, lines, i);
        Line line;

        if (returnCode) {
            // rotate points and retry
            arma::mat newPoints(points.n_cols, points.n_rows);
            for (size_t j = 0; j < points.n_rows; ++j) {
                newPoints(0, j) = points(j, 1);
                newPoints(1, j) = points(j, 0);
            }

            newPoints = newPoints.t();

            returnCode = detectSingleLine(newPoints, values, lines, i, true);

            if (returnCode) {
                rlog << "  LINE[" << i << "] pts_in=" << pts_before
                     << " -> REJECTED(no_fit)\n";
                rlog.flush();
                continue;
            }

            line = Line{randsac_.bestFit.params[1], randsac_.bestFit.params[0],
                        randsac_.bestScore};
            // use a rotated image to get end points also
            getEndPoints(line, true);

        } else {
            line = Line{randsac_.bestFit.params[1], randsac_.bestFit.params[0],
                        randsac_.bestScore};
            getEndPoints(line);
        }

        // For second+ lines: reject if intersection with previous lines is
        // outside the image frame (same pipe with segmentation gap, not a
        // genuine junction arm).
        if (i > 0 && !intersects_in_frame(line)) {
            rlog << "  LINE[" << i << "] pts_in=" << pts_before
                 << " score=" << randsac_.bestScore
                 << " -> REJECTED(intersect_out_of_frame)\n";
            rlog.flush();
            continue;
        }

        // For second+ lines: reject phantom diagonals created when both arms
        // are visible at a junction. A real pipe arm has nearly all skeleton
        // pixels as inliers; a phantom diagonal crossing two perpendicular arms
        // gets a fraction. Real arms are consistently >85%, phantoms are <35%.
        if (i > 0 && pts_before > 0 &&
            static_cast<double>(randsac_.bestScore) / static_cast<double>(pts_before) < min_inlier_ratio_) {
            rlog << "  LINE[" << i << "] pts_in=" << pts_before
                 << " score=" << randsac_.bestScore
                 << " ratio=" << (randsac_.bestScore / pts_before)
                 << " -> REJECTED(low_inlier_ratio<" << min_inlier_ratio_ << ")\n";
            rlog.flush();
            continue;
        }

        // Remove points for next iteration
        arma::mat newPoints;
        arma::mat newValues;
        for (size_t j = 0; j < points.n_rows; ++j) {
            if (std::find(randsac_.rempointids.begin(),
                          randsac_.rempointids.end(),
                          j) == randsac_.rempointids.end()) {
                newPoints.insert_rows(newPoints.n_rows, points.row(j));
                newValues.insert_rows(newValues.n_rows, values.row(j));
            }
        }
        const size_t pts_stripped = pts_before - newPoints.n_rows;
        points = newPoints;
        values = newValues;

        rlog << "  LINE[" << i << "] pts_in=" << pts_before
             << " score=" << randsac_.bestScore
             << " stripped=" << pts_stripped
             << " pts_remain=" << points.n_rows
             << " -> ACCEPTED"
             << " slope=" << line.slope
             << " start=(" << line.start.x << "," << line.start.y << ")"
             << " end=(" << line.end.x << "," << line.end.y << ")"
             << "\n";
        rlog.flush();

        lines.push_back(line);
    }

    return lines;
}
