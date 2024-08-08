#ifndef VEHICLE_PLUGINS_GAZEBO_VEHICLE_PLUGIN_INCLUDE_GAZEBO_CONE_DETECTION_PLUGIN_GAZEBO_CONE_DETECTION_HPP_
#define VEHICLE_PLUGINS_GAZEBO_VEHICLE_PLUGIN_INCLUDE_GAZEBO_CONE_DETECTION_PLUGIN_GAZEBO_CONE_DETECTION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

// ROS msgs
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ROS srvs
#include <std_srvs/srv/trigger.hpp>

// QUTMS messages
#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>

#include "gazebo_cone_detection_plugin/utils.hpp"

namespace gazebo_plugins {
namespace vehicle_plugins {

enum ConeColorOption { CONE = 0, FLAT = 1 };

class ConeDetectionPlugin : public gazebo::ModelPlugin {
   public:
    ConeDetectionPlugin();

    ~ConeDetectionPlugin() override;

    void Load(gazebo::physics::ModelPtr gz_model, sdf::ElementPtr sdf) override;

   private:
    void update();
    void initParams();
    bool resetConePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    gazebo::physics::WorldPtr world;
    gazebo::physics::ModelPtr model;
    gazebo_ros::Node::SharedPtr node;
    gazebo::event::ConnectionPtr update_connection;

    std::string map_frame;
    std::string base_frame;

    gazebo::physics::ModelPtr track_model;
    gazebo::physics::LinkPtr car_link;
    ignition::math::Pose3d car_inital_pose;

    double track_update_rate;
    double detection_update_rate;
    gazebo::common::Time last_track_update;
    gazebo::common::Time last_detection_update;
    driverless_msgs::msg::ConeDetectionStamped initial_track;

    // ROS Publishers
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr track_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr detection_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detection_marker_pub;

    // ROS Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_cone_pos_srv;

    SensorConfig_t detection_config;

    // visuals
    void initMarkers();
    void setConeMarker(const driverless_msgs::msg::Cone &cone, const std_msgs::msg::Header &header, const int &id,
                       visualization_msgs::msg::Marker *marker);
    void publishMarkerArray(driverless_msgs::msg::ConeDetectionStamped msg, bool is_track);

    int id;
    ConeColorOption cone_color_option;

    visualization_msgs::msg::Marker blue_cone_marker;
    visualization_msgs::msg::Marker yellow_cone_marker;
    visualization_msgs::msg::Marker orange_cone_marker;
    visualization_msgs::msg::Marker big_orange_cone_marker;
    visualization_msgs::msg::Marker unknown_cone_marker;
    visualization_msgs::msg::Marker covariance_marker;
    visualization_msgs::msg::Marker delete_all_marker;
    visualization_msgs::msg::MarkerArray marker_array;
};

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins

#endif  // VEHICLE_PLUGINS_GAZEBO_VEHICLE_PLUGIN_INCLUDE_GAZEBO_CONE_DETECTION_PLUGIN_GAZEBO_CONE_DETECTION_HPP_