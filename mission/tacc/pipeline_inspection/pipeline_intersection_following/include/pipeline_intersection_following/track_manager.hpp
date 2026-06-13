#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>
#include <vortex_filtering/filters/ipda.hpp>
#include <vortex_filtering/vortex_filtering.hpp>

using State2d = vortex::prob::Gauss<2>;
using DynMod = vortex::models::ConstantPosition;
using SensorMod = vortex::models::IdentitySensorModel<2, 2>;
using IPDA = vortex::filter::IPDA<DynMod, SensorMod>;

/**
 * @brief 2D position gate for line / intersection measurements.
 *
 * Replaces the removed PDAF min/max_gate_threshold config fields. A measurement
 * is accepted when it is closer than @c min_gate_threshold (always inside),
 * rejected when it is farther than @c max_gate_threshold (always outside), and
 * otherwise tested against the Mahalanobis gate. Mirrors the gate-function
 * pattern used by vortex::filtering::PoseGate6D in pose_filtering.
 */
struct LineGate2D {
    double min_gate_threshold = 0.0;
    double max_gate_threshold = std::numeric_limits<double>::infinity();
    double mahalanobis_threshold = std::numeric_limits<double>::infinity();

    using Vec_z = IPDA::Vec_z;
    using Gauss_z = IPDA::Gauss_z;

    bool operator()(const Vec_z& z, const Gauss_z& z_pred) const {
        const double dist = (z - z_pred.mean()).norm();
        if (dist > max_gate_threshold) {
            return false;
        }
        if (dist <= min_gate_threshold) {
            return true;
        }
        return z_pred.mahalanobis_distance(z) <= mahalanobis_threshold;
    }
};

/**
 * @brief N/M track confirmation / deletion window sizes.
 *
 * A track is confirmed once it has @c confirm_n hits within the last
 * @c confirm_m steps, and deleted once it has @c delete_n misses within the
 * last
 * @c delete_m steps. Mirrors the N/M logic used by pose_filtering.
 */
struct NMConfig {
    int confirm_n{2};
    int confirm_m{4};
    int delete_n{4};
    int delete_m{4};
};

struct Track {
    int id{-1};
    State2d state;
    double existence_probability{0.0};
    bool confirmed{false};
    Eigen::Matrix<double, 2, 2> line_points;
    int id1{-1};  ///< Parent line id (intersection tracks only)
    int id2{-1};  ///< Parent line id (intersection tracks only)

    /// Sliding window of hit/miss history (true = hit, false = miss).
    std::deque<bool> hit_history{};
    /// Force deletion regardless of the N/M window (e.g. gating conflicts).
    bool marked_for_deletion{false};

    /// Number of hits in the current window.
    int hits() const {
        return static_cast<int>(
            std::count(hit_history.begin(), hit_history.end(), true));
    }

    // For sorting tracks: confirmed first, then by number of recent hits.
    bool operator<(const Track& other) const {
        if (confirmed != other.confirmed) {
            return confirmed > other.confirmed;
        }
        return hits() > other.hits();
    }

    bool operator==(const Track& other) const { return id == other.id; }
};

class TrackManager {
   public:
    /**
     * @brief Default constructor for the TrackManager class.
     */
    TrackManager();

    /**
     * @brief Updates the tracks based on the measurements received from the
     * sensor.
     *
     * @param measurements The measurements received from the sensor.
     * @param update_interval The time interval between updates.
     * @param confirmation_threshold The threshold for confirming a track.
     * @param gate_theshhold The threshold for gating measurements.
     * @param prob_of_detection The probability of detection.
     * @param prob_of_survival The probability of survival.
     * @param clutter_intensity The intensity of clutter.
     */
    void update_line_tracks(
        Eigen::Array<double, 2, Eigen::Dynamic> measurements,
        Eigen::Array<double, 2, Eigen::Dynamic> line_params,
        int update_interval,
        double confirmation_threshold,
        double gate_theshhold,
        double min_gate_threshold,
        double max_gate_threshold,
        double prob_of_detection,
        double prob_of_survival,
        double clutter_intensity,
        double initial_existence_probability);

    void update_line_intersection_tracks(
        Eigen::Array<double, 2, Eigen::Dynamic> intersections,
        Eigen::Array<int, 2, Eigen::Dynamic> current_intersection_ids,
        Eigen::Array<double, 2, Eigen::Dynamic>
            current_line_intersection_points,
        int update_interval,
        double confirmation_threshold,
        double gate_theshhold,
        double min_gate_threshold,
        double max_gate_threshold,
        double prob_of_detection,
        double prob_of_survival,
        double clutter_intensity,
        double initial_existence_probability);

    /**
     * @brief Creates new tracks for every measurements.
     *
     * @param measurements The measurements received.
     */
    void create_line_tracks(
        Eigen::Array<double, 2, Eigen::Dynamic> measurements,
        Eigen::Array<double, 2, Eigen::Dynamic> line_params,
        double initial_existence_probability);

    void create_line_intersection_tracks(
        Eigen::Array<double, 2, Eigen::Dynamic> intersections,
        Eigen::Array<int, 2, Eigen::Dynamic> current_intersection_ids,
        Eigen::Array<double, 2, Eigen::Dynamic>
            current_line_intersection_points,
        double initial_existence_probability);

    /**
     * @brief Deletes tracks via N/M logic (delete_n misses within the last
     * delete_m steps) or those explicitly marked for deletion.
     */
    void delete_tracks();

    /**
     * @brief Sets the dynamic model for estimating target motion.
     *
     * @param std_velocity The standard deviation of the target velocity.
     */
    void set_dyn_model(double std_velocity);

    /**
     * @brief Sets the sensor model for estimating target measurements.
     *
     * @param std_measurement The standard deviation of the target measurement.
     */
    void set_sensor_model(double std_measurement);

    /**
     * @brief Sets the N/M confirmation / deletion window sizes.
     *
     * @param nm The N/M configuration.
     */
    void set_nm_config(const NMConfig& nm) { nm_ = nm; }

    /**
     * @brief Sets the orientation association gate for line tracks.
     *
     * A measurement is only associated to a line track if its orientation
     * differs from the track's by less than this threshold (radians, compared
     * modulo pi since lines are undirected). This prevents a perpendicular line
     * at a junction from being absorbed into the current line track just
     * because their midpoints are close. Set to >= pi/2 to disable.
     *
     * @param rad The orientation gate threshold in radians.
     */
    void set_orientation_gate(double rad) { orientation_gate_threshold_ = rad; }

    /**
     * @brief Retrieves the current tracks.
     *
     * @return A vector of Track objects representing the current tracks.
     */
    std::vector<Track> get_tracks() const { return tracks_; }

    void delete_track_by_id(int id);

   private:
    /// Appends a hit/miss to a track's history, trimming to the N/M window.
    void record_hit_miss(Track& track, bool hit);

    /// Confirms tracks that satisfy the N/M confirmation criterion.
    void confirm_tracks();

    std::vector<Track> tracks_;  ///< The vector of tracks.

    NMConfig nm_;  ///< N/M confirmation / deletion window sizes.

    /// Max orientation difference (rad, mod pi) for line-measurement
    /// association.
    double orientation_gate_threshold_ = M_PI;  // disabled by default

    std::shared_ptr<DynMod>
        dyn_model_;  ///< The dynamic model for estimating target motion.

    std::shared_ptr<SensorMod>
        sensor_model_;  ///< The sensor model for estimating target
                        ///< measurements.

    int tracker_id_;  ///< The tracker id.
};
