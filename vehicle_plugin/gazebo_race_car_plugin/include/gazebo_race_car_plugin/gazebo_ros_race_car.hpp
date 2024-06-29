
#ifndef EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_
#define EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_

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
#include "eufs_models/eufs_models.hpp"
#include "gazebo_race_car_plugin/helpers_gazebo.hpp"
#include "gazebo_race_car_plugin/helpers_ros.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

class RaceCarPlugin : public gazebo::ModelPlugin {
   public:
    RaceCarPlugin();

    ~RaceCarPlugin() override;

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

   private:
    void initParams();
    void setPositionFromWorld();
    bool resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setModelState();
    void publishVehicleMotion();
    void publishTf();
    void update();
    void onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> _rosnode;
    eufs::models::VehicleModelPtr _vehicle_model;

    // States
    eufs::models::State _state;
    eufs::models::Input _des_input, _act_input;
    std::unique_ptr<eufs::models::Noise> _noise;
    ignition::math::Pose3d _offset;

    // Gazebo
    gazebo::physics::WorldPtr _world;
    gazebo::physics::ModelPtr _model;
    gazebo::event::ConnectionPtr _update_connection;
    gazebo::common::Time _last_sim_time, _last_cmd_time;

    // Rate to publish ros messages
    double _update_rate;
    double _publish_rate;
    gazebo::common::Time _time_last_published;

    // ROS TF
    bool _pub_tf;
    std::string _map_frame;
    std::string _odom_frame;
    std::string _base_frame;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_br;

    // ROS Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_odom;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_joint_state;

    // ROS Subscriptions
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _sub_cmd;

    // ROS Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _reset_vehicle_pos_srv;

    // Steering joints state
    gazebo::physics::JointPtr _left_steering_joint;
    gazebo::physics::JointPtr _right_steering_joint;

    // Command queue for control delays
    ackermann_msgs::msg::AckermannDriveStamped _last_cmd;
    double _control_delay;
    // Steering rate limit variables
    double _max_steering_rate, _steering_lock_time;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_
