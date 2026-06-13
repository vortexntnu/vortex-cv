#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

struct NMConfig {
    int confirm_n{2};
    int confirm_m{4};
    int delete_n{4};
    int delete_m{4};
};

struct Track {
    int id{-1};
    bool confirmed{false};
    Eigen::Matrix<double, 2, 2> line_points;
    int id1{-1};  ///< Parent line id (intersection tracks only)
    int id2{-1};  ///< Parent line id (intersection tracks only)

    std::deque<bool> hit_history{};
    bool marked_for_deletion{false};

    Eigen::Vector2d midpoint() const {
        return (line_points.col(0) + line_points.col(1)) / 2.0;
    }

    double orientation() const {
        return std::atan2(line_points(1, 1) - line_points(1, 0),
                          line_points(0, 1) - line_points(0, 0));
    }

    int hits() const {
        return static_cast<int>(
            std::count(hit_history.begin(), hit_history.end(), true));
    }

    bool operator<(const Track& other) const {
        if (confirmed != other.confirmed) return confirmed > other.confirmed;
        return hits() > other.hits();
    }

    bool operator==(const Track& other) const { return id == other.id; }
};

class TrackManager {
   public:
    TrackManager() : tracker_id_(0) {}

    /**
     * @brief Update line tracks with new detections (nearest-neighbour + N/M).
     *
     * measurements: 2×(2N) array — column pairs (p0, p1) for each of N lines.
     * Confirmed tracks are matched first. Each track gets a hit if a gated
     * measurement is nearest to it, a miss otherwise. Unmatched measurements
     * spawn new tracks.
     */
    void update_line_tracks(
        const Eigen::Array<double, 2, Eigen::Dynamic>& measurements,
        double min_gate,
        double max_gate);

    /**
     * @brief Delete tracks via N/M logic or those marked for deletion.
     */
    void delete_tracks();

    void delete_track_by_id(int id);

    void set_nm_config(const NMConfig& nm) { nm_ = nm; }

    /**
     * @brief Orientation gate for line association (radians, mod pi).
     * Prevents a perpendicular line at a junction absorbing the current track.
     * Set to >= pi/2 to disable.
     */
    void set_orientation_gate(double rad) { orientation_gate_threshold_ = rad; }

    std::vector<Track> get_tracks() const { return tracks_; }

   private:
    void record_hit_miss(Track& track, bool hit);
    void confirm_tracks();

    std::vector<Track> tracks_;
    NMConfig nm_;
    double orientation_gate_threshold_ = M_PI;
    int tracker_id_{0};
};
