#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <pipeline_line_fitting/linedetectorPipe.hpp>

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

    // Keep only the two largest skeleton components. The pipeline is always the
    // dominant structure; any third+ component is noise regardless of size.
    // Keeping two allows junctions where the arms thin into separate components.
    {
        cv::Mat labels, stats, centroids;
        int n_labels =
            cv::connectedComponentsWithStats(img, labels, stats, centroids);

        std::vector<std::pair<int, int>> areas;  // (area, label)
        for (int lbl = 1; lbl < n_labels; ++lbl) {
            int area = stats.at<int>(lbl, cv::CC_STAT_AREA);
            if (area >= min_skeleton_component_size_)
                areas.push_back({area, lbl});
        }
        std::sort(areas.rbegin(), areas.rend());

        std::set<int> keep;
        for (int i = 0; i < std::min(2, (int)areas.size()); ++i)
            keep.insert(areas[i].second);

        for (int lbl = 1; lbl < n_labels; ++lbl) {
            if (keep.find(lbl) == keep.end())
                img.setTo(0, labels == lbl);
        }
    }

    removeBorderArtifacts(img);
}

LinedetectorPipe::LinedetectorPipe() {};
LinedetectorPipe::~LinedetectorPipe() {};

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

    // Hough line detection on thinned skeleton.
    std::vector<cv::Vec2f> raw_lines;
    cv::HoughLines(processedImg_, raw_lines, 1, CV_PI / 180.0, hough_threshold_);

    if (raw_lines.empty()) return {};

    // Cluster by angle: lines within minTurnAngle_ rad are the same pipe arm.
    // raw_lines is sorted by vote count so the first entry per cluster is the
    // best representative.
    struct Cluster {
        float rho, theta;
        int count = 1;
    };
    std::vector<Cluster> clusters;

    for (const auto& hl : raw_lines) {
        float rho = hl[0], theta = hl[1];
        bool merged = false;
        for (auto& c : clusters) {
            double dtheta = std::fmod(std::abs(theta - c.theta), CV_PI);
            if (dtheta > CV_PI / 2.0) dtheta = CV_PI - dtheta;
            if (dtheta < minTurnAngle_) {
                c.count++;
                merged = true;
                break;
            }
        }
        if (!merged) clusters.push_back({rho, theta, 1});
    }

    std::sort(clusters.begin(), clusters.end(),
              [](const Cluster& a, const Cluster& b) {
                  return a.count > b.count;
              });

    std::vector<Line> lines;

    // Reject a second line whose infinite extension doesn't intersect the first
    // inside the image frame — means they're parallel (same pipe, not a corner).
    auto intersects_in_frame = [&](const Line& candidate) -> bool {
        for (const auto& prev : lines) {
            double ax = prev.start.x * scaleX_, ay = prev.start.y * scaleY_;
            double bx = prev.end.x * scaleX_,   by = prev.end.y * scaleY_;
            double cx = candidate.start.x * scaleX_, cy = candidate.start.y * scaleY_;
            double ex = candidate.end.x * scaleX_,   ey = candidate.end.y * scaleY_;
            double p1 = -(by - ay), q1 = bx - ax,
                   r1 = (by - ay) * ax - (bx - ax) * ay;
            double p2 = -(ey - cy), q2 = ex - cx,
                   r2 = (ey - cy) * cx - (ex - cx) * cy;
            double det = p1 * q2 - p2 * q1;
            if (std::abs(det) < 1e-6) return false;
            double xi = (-r1 * q2 + r2 * q1) / det;
            double yi = (-r2 * p1 + r1 * p2) / det;
            if (xi < 0.0 || xi > size_ || yi < 0.0 || yi > size_) return false;
        }
        return true;
    };

    for (const auto& c : clusters) {
        if ((int)lines.size() >= maxLines) break;

        // Convert Hough (rho, theta) to slope/intercept.
        // Normal form: col*cos(theta) + row*sin(theta) = rho
        // -> row = slope*col + intercept  when sin(theta) is not near zero
        // -> col = slope*row + intercept  (swap=true) for near-vertical lines
        Line line;
        bool swapped = false;

        if (std::abs(std::sin(c.theta)) > 0.1) {
            line.slope     = -std::cos(c.theta) / std::sin(c.theta);
            line.intercept =  c.rho / std::sin(c.theta);
        } else {
            line.slope     = -std::sin(c.theta) / std::cos(c.theta);
            line.intercept =  c.rho / std::cos(c.theta);
            swapped = true;
        }
        line.score = c.count;

        getEndPoints(line, swapped);

        if (line.start.x < 0 || line.start.y < 0 ||
            line.end.x < 0   || line.end.y < 0) continue;

        if (!lines.empty() && !intersects_in_frame(line)) continue;

        lines.push_back(line);
    }

    return lines;
}
