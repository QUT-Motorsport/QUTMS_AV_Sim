
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

namespace gazebo_plugins {
namespace vehicle_plugins {

class VehiclePlugin : public gazebo::ModelPlugin {
   public:
    VehiclePlugin();

    ~VehiclePlugin() override;

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

    nav_msgs::msg::Odometry stateToOdom(const eufs::models::State &state);

    std::shared_ptr<rclcpp::Node> rosnode_;
    eufs::models::VehicleModelPtr vehicle_model_;

    // States
    eufs::models::State state_;
    eufs::models::Input des_input_, act_input_;
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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_gt_odom;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;

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

    bool is_initalised(rclcpp::PublisherBase::SharedPtr publisher) { return (bool)publisher; }

    bool has_subscribers(rclcpp::PublisherBase::SharedPtr publisher) {
        return is_initalised(publisher) && publisher->get_subscription_count() > 0;
    }

    std::vector<double> to_quaternion(std::vector<double> &euler) {
        // Abbreviations for the various angular functions
        double cy = cos(euler[0] * 0.5);
        double sy = sin(euler[0] * 0.5);
        double cp = cos(euler[1] * 0.5);
        double sp = sin(euler[1] * 0.5);
        double cr = cos(euler[2] * 0.5);
        double sr = sin(euler[2] * 0.5);

        std::vector<double> q;
        q.push_back(cy * cp * sr - sy * sp * cr);  // x
        q.push_back(sy * cp * sr + cy * sp * cr);  // y
        q.push_back(sy * cp * cr - cy * sp * sr);  // z
        q.push_back(cy * cp * cr + sy * sp * sr);  // w

        return q;
    }

    double calc_dt(gazebo::common::Time start, gazebo::common::Time end) { return (end - start).Double(); }

    gazebo::physics::ModelPtr get_model(gazebo::physics::WorldPtr world, std::string name,
                                        std::optional<const rclcpp::Logger> logger = {}) {
        gazebo::physics::ModelPtr model = world->ModelByName(name);
        if (model == nullptr) {
            if (logger) {
                RCLCPP_FATAL(*logger, "Could not find required model <%s>. Exiting.", name.c_str());
            }
            exit(1);
        }
        return model;
    }

    gazebo::physics::LinkPtr get_link(gazebo::physics::ModelPtr model, std::string name,
                                    std::optional<const rclcpp::Logger> logger = {}) {
        gazebo::physics::LinkPtr link = model->GetLink(name);
        if (link == nullptr) {
            if (logger) {
                RCLCPP_FATAL(*logger, "Could not find required link <%s> on model <%s>. Exiting.", name.c_str(),
                            model->GetName().c_str());
            }
            exit(1);
        }
        return link;
    }

};

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins

#endif  // VEHICLE_PLUGINS_GAZEBO_VEHICLE_PLUGIN_INCLUDE_GAZEBO_VEHICLE_PLUGIN_GAZEBO_VEHICLE_HPP_
