#ifndef VORTEX_BT_NODES__MAP__LANDMARK_CACHE_HPP_
#define VORTEX_BT_NODES__MAP__LANDMARK_CACHE_HPP_

#include <behaviortree_cpp/tree_node.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/landmark_track_array.hpp>

#include "vortex_bt_nodes/common/types.hpp"

namespace vortex_bt_nodes::map {

struct LandmarkView {
    int id{0};
    std::uint16_t type{0};
    std::uint16_t subtype{0};
    Pose pose;  // map frame
    /// Position part is relative to the vehicle.
    std::array<double, 36> cov{};
    std::uint32_t n_obs{0};  // 0 = prior map only
    bool has_orientation{false};
};

/**
 * @brief Latest map from landmark_server/landmarks plus TF, shared by the
 * landmark nodes of a tree. Nodes only read it.
 */
class LandmarkCache {
   public:
    explicit LandmarkCache(
        const rclcpp::Node::SharedPtr& node,
        const std::string& topic = "landmark_server/landmarks")
        : tf_buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
          tf_listener_(
              std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node)) {
        std::string ns = node->get_namespace();
        ns.erase(0, ns.find_first_not_of('/'));
        prefix_ = ns.empty() ? "" : ns + "/";
        base_frame_ = prefix_ + "base_link";
        odom_frame_ = prefix_ + "odom";
        sub_ = node->create_subscription<vortex_msgs::msg::LandmarkTrackArray>(
            topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](vortex_msgs::msg::LandmarkTrackArray::ConstSharedPtr msg) {
                map_ = msg;
            });
    }

    std::optional<LandmarkView> by_id(int id) const {
        if (map_) {
            for (const auto& t : map_->landmark_tracks) {
                if (t.landmark.id == id) {
                    return view(t);
                }
            }
        }
        return std::nullopt;
    }

    /// The most observed landmark of the class. subtype -1 = any.
    std::optional<LandmarkView> best_of_class(std::uint16_t type,
                                              int subtype) const {
        std::optional<LandmarkView> best;
        if (!map_) {
            return best;
        }
        for (const auto& t : map_->landmark_tracks) {
            if (!matches(t, type, subtype)) {
                continue;
            }
            const LandmarkView v = view(t);
            if (!best || v.n_obs > best->n_obs) {
                best = v;
            }
        }
        return best;
    }

    /// Empty before the first map.
    std::string frame_id() const {
        return map_ ? map_->header.frame_id : std::string();
    }

    std::optional<Pose> lookup(const std::string& target,
                               const std::string& frame) const {
        try {
            const auto tf =
                tf_buffer_->lookupTransform(target, frame, tf2::TimePointZero);
            Pose p;
            p.position.x = tf.transform.translation.x;
            p.position.y = tf.transform.translation.y;
            p.position.z = tf.transform.translation.z;
            p.orientation = tf.transform.rotation;
            return p;
        } catch (const tf2::TransformException&) {
            return std::nullopt;
        }
    }

    std::optional<Pose> vehicle_pose() const {
        if (!map_) {
            return std::nullopt;
        }
        return lookup(map_->header.frame_id, base_frame_);
    }

    std::optional<Pose> vehicle_in_odom() const {
        return lookup(odom_frame_, base_frame_);
    }

    /** @brief A map-frame pose in odom. */
    std::optional<Pose> map_to_odom(const Pose& in_map) const {
        const auto map_in_odom =
            map_ ? lookup(odom_frame_, map_->header.frame_id) : std::nullopt;
        if (!map_in_odom) {
            return std::nullopt;
        }
        return compose(*map_in_odom, in_map);
    }

    /// Adds the namespace prefix unless the name already has one.
    std::string frame_name(const std::string& name) const {
        return name.find('/') == std::string::npos ? prefix_ + name : name;
    }
    const std::string& odom_frame() const { return odom_frame_; }

    static Pose compose(const Pose& a, const Pose& b) {
        tf2::Transform ta;
        tf2::Transform tb;
        tf2::fromMsg(a, ta);
        tf2::fromMsg(b, tb);
        Pose out;
        tf2::toMsg(ta * tb, out);
        return out;
    }

    static double sigma_xy(const LandmarkView& l) {
        const double a = l.cov[0];
        const double b = l.cov[1];
        const double d = l.cov[7];
        const double half_trace = 0.5 * (a + d);
        const double disc = std::sqrt(0.25 * (a - d) * (a - d) + b * b);
        return std::sqrt(std::max(0.0, half_trace + disc));
    }

    static bool confirmed(const LandmarkView& l, double max_sigma_xy) {
        return l.n_obs > 0 && sigma_xy(l) < max_sigma_xy;
    }

    /**
     * @brief Pose at offset from the landmark, facing it. Without a known
     * yaw the offset is turned toward the vehicle.
     */
    static Pose approach_pose(const LandmarkView& l,
                              const Pose& vehicle,
                              const Pose& offset,
                              int symmetry_deg) {
        const double lx = l.pose.position.x;
        const double ly = l.pose.position.y;
        const double ox = offset.position.x;
        const double oy = offset.position.y;
        const auto at = [&](double yaw) {
            return std::make_pair(lx + std::cos(yaw) * ox - std::sin(yaw) * oy,
                                  ly + std::sin(yaw) * ox + std::cos(yaw) * oy);
        };
        const auto dist = [&](double yaw) {
            const auto [x, y] = at(yaw);
            return std::hypot(x - vehicle.position.x, y - vehicle.position.y);
        };

        double yaw = yaw_of(l.pose);
        if (!l.has_orientation || symmetry_deg >= 360) {
            yaw = std::atan2(vehicle.position.y - ly, vehicle.position.x - lx) -
                  std::atan2(oy, ox);
        } else if (symmetry_deg > 0) {
            const double step = symmetry_deg * M_PI / 180.0;
            const double base = yaw;
            for (int n = 1; n * symmetry_deg < 360; ++n) {
                if (dist(base + n * step) < dist(yaw)) {
                    yaw = base + n * step;
                }
            }
        }
        const auto [x, y] = at(yaw);
        const double heading = std::hypot(lx - x, ly - y) > 1e-3
                                   ? std::atan2(ly - y, lx - x)
                                   : yaw_of(vehicle);
        return make_pose(x, y, l.pose.position.z + offset.position.z, heading);
    }

    static BT::PortsList ports() {
        return {BT::InputPort<int>("id", "Landmark id (wins over type)"),
                BT::InputPort<std::string>("type", "", "LandmarkType name"),
                BT::InputPort<std::string>("subtype", "ANY",
                                           "LandmarkSubtype name or ANY")};
    }

    std::optional<LandmarkView> resolve(const BT::TreeNode& node) const {
        if (const auto id = node.getInput<int>("id")) {
            return by_id(*id);
        }
        const auto type_name = node.getInput<std::string>("type");
        const auto subtype_name = node.getInput<std::string>("subtype");
        if (!type_name || !subtype_name) {
            return std::nullopt;
        }
        const auto type = landmark_type_from_string(*type_name);
        const auto subtype = landmark_subtype_from_string(*subtype_name);
        if (!type || !subtype) {
            return std::nullopt;
        }
        return best_of_class(*type, *subtype);
    }

   private:
    static LandmarkView view(const LandmarkTrack& t) {
        LandmarkView v;
        v.id = t.landmark.id;
        v.type = t.landmark.type.value;
        v.subtype = t.landmark.subtype.value;
        v.pose = t.landmark.pose.pose;
        std::copy(t.landmark.pose.covariance.begin(),
                  t.landmark.pose.covariance.end(), v.cov.begin());
        v.n_obs = static_cast<std::uint32_t>(std::max(0, t.observations));
        v.has_orientation = t.has_orientation;
        return v;
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr sub_;
    vortex_msgs::msg::LandmarkTrackArray::ConstSharedPtr map_;
    std::string prefix_;
    std::string base_frame_;
    std::string odom_frame_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__LANDMARK_CACHE_HPP_
