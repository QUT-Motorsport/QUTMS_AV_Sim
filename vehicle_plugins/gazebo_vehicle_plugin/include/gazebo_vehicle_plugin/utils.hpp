#pragma once

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo_plugins {
namespace vehicle_plugins {

bool is_initalised(rclcpp::PublisherBase::SharedPtr publisher) { return (bool)publisher; }

bool has_subscribers(rclcpp::PublisherBase::SharedPtr publisher) {
    return is_initalised(publisher) && publisher->get_subscription_count() > 0;
}

geometry_msgs::msg::Quaternion to_quaternion(std::vector<double> &euler) {
    // Abbreviations for the various angular functions
    double cy = cos(euler[0] * 0.5);
    double sy = sin(euler[0] * 0.5);
    double cp = cos(euler[1] * 0.5);
    double sp = sin(euler[1] * 0.5);
    double cr = cos(euler[2] * 0.5);
    double sr = sin(euler[2] * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    q.w = cy * cp * cr + sy * sp * sr;

    return q;
}

std::vector<double> to_euler(geometry_msgs::msg::Quaternion &q) {
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z); 
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    double pitch = fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

double calc_dt(gazebo::common::Time start, gazebo::common::Time end) { return (end - start).Double(); }

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins
