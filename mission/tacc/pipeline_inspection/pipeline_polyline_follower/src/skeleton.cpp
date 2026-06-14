#include "pipeline_polyline_follower/skeleton.hpp"

#include <functional>
#include <unordered_set>

// ── Zhang-Suen thinning ───────────────────────────────────────────────────

namespace {

// Returns number of 0→1 transitions in the cyclic 8-neighbourhood
// P2 P3 P4 P5 P6 P7 P8 P9 starting from top-centre going clockwise.
int transitions(const cv::Mat& b, int r, int c) {
    // clang-format off
    const uchar seq[9] = {
        b.at<uchar>(r-1, c  ),  // P2
        b.at<uchar>(r-1, c+1),  // P3
        b.at<uchar>(r  , c+1),  // P4
        b.at<uchar>(r+1, c+1),  // P5
        b.at<uchar>(r+1, c  ),  // P6
        b.at<uchar>(r+1, c-1),  // P7
        b.at<uchar>(r  , c-1),  // P8
        b.at<uchar>(r-1, c-1),  // P9
    };
    // clang-format on
    int cnt = 0;
    for (int i = 0; i < 8; ++i)
        if (seq[i] == 0 && seq[(i + 1) % 8] != 0)
            ++cnt;
    return cnt;
}

// Number of foreground 8-neighbours
int nonzero_neighbours(const cv::Mat& b, int r, int c) {
    int cnt = 0;
    for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc)
            if ((dr != 0 || dc != 0) && b.at<uchar>(r + dr, c + dc) != 0)
                ++cnt;
    return cnt;
}

}  // namespace

void zhang_suen_thin(cv::Mat& img) {
    // Work on a 0/1 image with a 1-pixel border of zeros to avoid bounds checks
    cv::Mat bin;
    cv::threshold(img, bin, 127, 1, cv::THRESH_BINARY);
    bin.convertTo(bin, CV_8U);
    cv::copyMakeBorder(bin, bin, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);

    bool any_deleted = true;
    while (any_deleted) {
        any_deleted = false;
        for (int step = 0; step < 2; ++step) {
            std::vector<cv::Point> to_delete;
            for (int r = 1; r < bin.rows - 1; ++r) {
                for (int c = 1; c < bin.cols - 1; ++c) {
                    if (bin.at<uchar>(r, c) == 0)
                        continue;
                    const int B = nonzero_neighbours(bin, r, c);
                    if (B < 2 || B > 6)
                        continue;
                    if (transitions(bin, r, c) != 1)
                        continue;

                    const uchar p2 = bin.at<uchar>(r - 1, c);
                    const uchar p4 = bin.at<uchar>(r, c + 1);
                    const uchar p6 = bin.at<uchar>(r + 1, c);
                    const uchar p8 = bin.at<uchar>(r, c - 1);

                    if (step == 0) {
                        if ((p2 * p4 * p6) != 0)
                            continue;
                        if ((p4 * p6 * p8) != 0)
                            continue;
                    } else {
                        if ((p2 * p4 * p8) != 0)
                            continue;
                        if ((p2 * p6 * p8) != 0)
                            continue;
                    }
                    to_delete.push_back({c, r});
                }
            }
            for (auto& p : to_delete)
                bin.at<uchar>(p.y, p.x) = 0;
            if (!to_delete.empty())
                any_deleted = true;
        }
    }

    // Remove border and scale back to 0/255
    img = bin(cv::Rect(1, 1, img.cols, img.rows)) * 255;
}

// ── Skeleton tracing ──────────────────────────────────────────────────────

std::vector<cv::Point> trace_skeleton(const cv::Mat& skeleton) {
    // Collect foreground pixels
    std::vector<cv::Point> pixels;
    for (int r = 0; r < skeleton.rows; ++r)
        for (int c = 0; c < skeleton.cols; ++c)
            if (skeleton.at<uchar>(r, c) > 0)
                pixels.push_back({c, r});

    if (pixels.empty())
        return {};

    const int W = skeleton.cols;
    auto key = [W](cv::Point p) { return p.y * W + p.x; };

    std::unordered_set<int> pixel_set;
    pixel_set.reserve(pixels.size());
    for (auto& p : pixels)
        pixel_set.insert(key(p));

    auto neighbours = [&](cv::Point p) {
        std::vector<cv::Point> nb;
        for (int dr = -1; dr <= 1; ++dr)
            for (int dc = -1; dc <= 1; ++dc) {
                if (dr == 0 && dc == 0)
                    continue;
                cv::Point q(p.x + dc, p.y + dr);
                if (q.x >= 0 && q.x < skeleton.cols && q.y >= 0 &&
                    q.y < skeleton.rows && pixel_set.count(key(q)))
                    nb.push_back(q);
            }
        return nb;
    };

    // Classify pixels
    std::vector<cv::Point> endpoints;
    for (auto& p : pixels) {
        int n = static_cast<int>(neighbours(p).size());
        if (n == 1)
            endpoints.push_back(p);
    }

    // If the skeleton is a closed loop, just pick any pixel as start
    cv::Point start = endpoints.empty() ? pixels[0] : endpoints[0];

    // DFS to collect an ordered path.  At branch points we follow the branch
    // with the most remaining pixels (greedy longest-path heuristic).
    std::unordered_set<int> visited;
    visited.reserve(pixels.size());
    std::vector<cv::Point> path;
    path.reserve(pixels.size());

    std::function<void(cv::Point)> dfs = [&](cv::Point cur) {
        visited.insert(key(cur));
        path.push_back(cur);
        auto nbs = neighbours(cur);
        // Sort neighbours: prefer unvisited ones with fewer of their own
        // neighbours first (follow the thin branch, not a junction back-edge)
        std::sort(nbs.begin(), nbs.end(), [&](cv::Point a, cv::Point b_pt) {
            const bool av = visited.count(key(a)) == 0;
            const bool bv = visited.count(key(b_pt)) == 0;
            if (av != bv)
                return av > bv;  // unvisited first
            return neighbours(a).size() < neighbours(b_pt).size();
        });
        for (auto& nb : nbs)
            if (!visited.count(key(nb)))
                dfs(nb);
    };
    dfs(start);
    return path;
}
