#include <pipeline_intersection_following/track_manager.hpp>

void TrackManager::update_line_tracks(
    const Eigen::Array<double, 2, Eigen::Dynamic>& measurements,
    double min_gate,
    double max_gate) {
    const int n_lines = static_cast<int>(measurements.cols()) / 2;

    // Confirmed tracks and those with more recent hits get first pick.
    std::sort(tracks_.begin(), tracks_.end());

    std::vector<bool> meas_used(n_lines, false);

    for (auto& track : tracks_) {
        const Eigen::Vector2d track_mid = track.midpoint();
        const double track_ang = track.orientation();

        int best_idx = -1;
        double best_dist = std::numeric_limits<double>::infinity();

        for (int i = 0; i < n_lines; ++i) {
            if (meas_used[i]) continue;

            const Eigen::Vector2d meas_p0 = measurements.col(2 * i).matrix();
            const Eigen::Vector2d meas_p1 = measurements.col(2 * i + 1).matrix();
            const Eigen::Vector2d meas_mid = (meas_p0 + meas_p1) / 2.0;

            const double dist = (meas_mid - track_mid).norm();
            if (dist > max_gate) continue;

            // Skip orientation check for very close measurements (within
            // min_gate), since the pipe is clearly the same object.
            if (dist > min_gate) {
                const double meas_ang =
                    std::atan2(meas_p1(1) - meas_p0(1), meas_p1(0) - meas_p0(0));
                double dang = std::fmod(std::fabs(track_ang - meas_ang), M_PI);
                if (dang > M_PI / 2.0) dang = M_PI - dang;
                if (dang > orientation_gate_threshold_) continue;
            }

            if (dist < best_dist) {
                best_dist = dist;
                best_idx = i;
            }
        }

        if (best_idx >= 0) {
            track.line_points.col(0) = measurements.col(2 * best_idx).matrix();
            track.line_points.col(1) = measurements.col(2 * best_idx + 1).matrix();
            meas_used[best_idx] = true;
            record_hit_miss(track, true);
        } else {
            record_hit_miss(track, false);
        }
    }

    // Unmatched measurements become new candidate tracks.
    for (int i = 0; i < n_lines; ++i) {
        if (meas_used[i]) continue;
        Track track;
        track.id = tracker_id_++;
        track.line_points.col(0) = measurements.col(2 * i).matrix();
        track.line_points.col(1) = measurements.col(2 * i + 1).matrix();
        track.confirmed = false;
        track.hit_history.push_back(true);
        tracks_.push_back(track);
    }

    confirm_tracks();
}

void TrackManager::record_hit_miss(Track& track, bool hit) {
    const int max_window = std::max(nm_.confirm_m, nm_.delete_m);
    track.hit_history.push_back(hit);
    while (static_cast<int>(track.hit_history.size()) > max_window) {
        track.hit_history.pop_front();
    }
}

void TrackManager::confirm_tracks() {
    for (Track& track : tracks_) {
        if (track.confirmed) continue;
        if (static_cast<int>(track.hit_history.size()) < nm_.confirm_m) continue;
        int recent_hits = 0;
        auto it = track.hit_history.rbegin();
        for (int i = 0; i < nm_.confirm_m; ++i, ++it) {
            if (*it) ++recent_hits;
        }
        if (recent_hits >= nm_.confirm_n) track.confirmed = true;
    }
}

void TrackManager::delete_tracks() {
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [this](const Track& track) {
                           if (track.marked_for_deletion) return true;
                           if (static_cast<int>(track.hit_history.size()) <
                               nm_.delete_m)
                               return false;
                           int recent_misses = 0;
                           auto it = track.hit_history.rbegin();
                           for (int i = 0; i < nm_.delete_m; ++i, ++it) {
                               if (!*it) ++recent_misses;
                           }
                           return recent_misses >= nm_.delete_n;
                       }),
        tracks_.end());
}

void TrackManager::delete_track_by_id(int id) {
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [id](const Track& t) { return t.id == id; }),
        tracks_.end());
}
