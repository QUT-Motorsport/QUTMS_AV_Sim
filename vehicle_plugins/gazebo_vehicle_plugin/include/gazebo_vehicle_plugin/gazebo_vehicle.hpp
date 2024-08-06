
#ifndef VEHICLE_PLUGINS_GAZEBO_VEHICLE_PLUGIN_INCLUDE_GAZEBO_VEHICLE_PLUGIN_GAZEBO_VEHICLE_HPP_
#define VEHICLE_PLUGINS_GAZEBO_VEHICLE_PLUGIN_INCLUDE_GAZEBO_VEHICLE_PLUGIN_GAZEBO_VEHICLE_HPP_

#include <algorithm>
#include <fstream>
#include <mutex>   // NOLINT(build/c++11)
#include <thread>  // NOLINT(build/c++11)

#include <memory>
#include <queue>
#include <string>
#include <vector>
// ROS Includes
#include "rclcpp/rclcpp.hpp"

// ROS msgs
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// ROS TF2
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS  srvs
#include <std_srvs/srv/trigger.hpp>

// Gazebo Includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

// Local includes
#include "gazebo_vehicle_plugin/noise.hpp"
#include "gazebo_vehicle_plugin/utils.hpp"
#include "gazebo_vehicle_plugin/vehicle_model_bike.hpp"
#include "gazebo_vehicle_plugin/vehicle_state.hpp"

namespace gazebo_plugins {
namespace vehicle_plugins {

class VehiclePlugin : public gazebo::ModelPlugin {
   public:
    VehiclePlugin();

    ~VehiclePlugin() override;

    void Load(gazebo::physics::ModelPtr gz_model, sdf::ElementPtr sdf) override;

   private:
    void initParams();
    void setPositionFromWorld();
    bool resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setModelState();
    void publishVehicleOdom();
    void publishTf();
    void update();
    void onAckermannCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void onTwistCmd(const geometry_msgs::msg::Twist::SharedPtr msg);

    nav_msgs::msg::Odometry stateToOdom(const State &state);
    State odomToState(const nav_msgs::msg::Odometry &odom);

    std::shared_ptr<rclcpp::Node> node;

    // Vehicle Motion
    VehicleModelBikePtr vehicle_model;
    nav_msgs::msg::Odometry state_odom;
    Control input, output;
    State state;
    std::unique_ptr<Noise> motion_noise;
    ignition::math::Pose3d offset;

    // Gazebo
    gazebo::physics::WorldPtr world;
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr update_connection;
    gazebo::common::Time last_sim_time, last_cmd_time, last_published_time;

    // Rate to publish ros messages
    double update_rate;
    double publish_rate;

    // ROS TF
    bool pub_tf;
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br;

    // ROS Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_odometry_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;

    // ROS Subscriptions
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_cmd_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub;

    // ROS Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_vehicle_pos_srv;

    // Steering joints state
    gazebo::physics::JointPtr left_steering_joint;
    gazebo::physics::JointPtr right_steering_joint;

    // Command queue for control delays
    ackermann_msgs::msg::AckermannDriveStamped last_cmd;
    double control_delay;
    // Steering rate limit variables
    double max_steering_rate, steering_lock_time;
};

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins

#endif  // VEHICLE_PLUGINS_GAZEBO_VEHICLE_PLUGIN_INCLUDE_GAZEBO_VEHICLE_PLUGIN_GAZEBO_VEHICLE_HPP_
