#pragma once

#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>
#include <gazebo/gazebo.hh>
#include <string>

namespace gazebo_plugins {
namespace vehicle_plugins {

typedef struct SensorConfig {
    std::string frame_id;
    double min_view_distance;
    double max_view_distance;
    double fov;
    bool detects_colour;
    double offset_x;
    double offset_z;
} SensorConfig_t;

double calc_dt(gazebo::common::Time start, gazebo::common::Time end) { return (end - start).Double(); }

bool is_initalised(rclcpp::PublisherBase::SharedPtr publisher) { return (bool)publisher; }

bool has_subscribers(rclcpp::PublisherBase::SharedPtr publisher) {
    return is_initalised(publisher) && publisher->get_subscription_count() > 0;
}

double cone_dist(driverless_msgs::msg::Cone cone) {
    return sqrt(cone.location.x * cone.location.x + cone.location.y * cone.location.y);
}

double cone_angle(driverless_msgs::msg::Cone cone) { return atan2(cone.location.y, cone.location.x); }

// Translate a cone to the car's frame from the world frame
driverless_msgs::msg::Cone convert_cone_to_car_frame(const ignition::math::Pose3d car_pose,
                                                     const driverless_msgs::msg::Cone cone, const double offset_x = 0,
                                                     const double offset_z = 0) {
    driverless_msgs::msg::Cone translated_cone = cone;

    double x = cone.location.x - car_pose.Pos().X();
    double y = cone.location.y - car_pose.Pos().Y();
    double yaw = car_pose.Rot().Yaw();

    // Rotate the points using the yaw of the car (x and y are the other way around)
    translated_cone.location.y = (cos(yaw) * y) - (sin(yaw) * x);
    translated_cone.location.x = (sin(yaw) * y) + (cos(yaw) * x) - offset_x;
    translated_cone.location.z -= offset_z;

    return translated_cone;
}

// Calculate the distance of a cone from the car
driverless_msgs::msg::ConeDetectionStamped get_sensor_detection(SensorConfig_t sensor_config, ignition::math::Pose3d car_pose, 
                                                                driverless_msgs::msg::ConeDetectionStamped ground_truth_track) {

    driverless_msgs::msg::ConeDetectionStamped detection;
    detection.header = ground_truth_track.header;
    detection.header.frame_id = sensor_config.frame_id;

    for (auto const &cone : ground_truth_track.cones) {
        auto translated_cone = cone;
        translated_cone = convert_cone_to_car_frame(car_pose, cone, sensor_config.offset_x, sensor_config.offset_z);

        double dist = cone_dist(translated_cone);
        if (dist < sensor_config.min_view_distance || dist > sensor_config.max_view_distance) {
            continue;
        }

        double angle = cone_angle(translated_cone);
        if (abs(angle) > sensor_config.fov / 2) {
            continue;
        }

        if (!sensor_config.detects_colour) {
            translated_cone.color = driverless_msgs::msg::Cone::UNKNOWN;
        }

        detection.cones.push_back(translated_cone);
    }

    return detection;
}

// Get the track centered on the car's initial pose
// Used to update visuals when cones are hit
driverless_msgs::msg::ConeDetectionStamped get_track_centered_on_car_inital_pose(ignition::math::Pose3d car_inital_pose, 
                                                                                 driverless_msgs::msg::ConeDetectionStamped track) {
    driverless_msgs::msg::ConeDetectionStamped centered_track;
    centered_track.header = track.header;

    for (auto const &cone : track.cones) {
        auto translated_cone = cone;
        translated_cone = convert_cone_to_car_frame(car_inital_pose, cone);
        centered_track.cones.push_back(translated_cone);
    }

    return centered_track;
}

// Get the position of a cone in the world frame
driverless_msgs::msg::Cone get_cone_from_link(gazebo::physics::LinkPtr cone_link) {
    driverless_msgs::msg::Cone cone;
    cone.location.x = cone_link->WorldPose().Pos().X();
    cone.location.y = cone_link->WorldPose().Pos().Y();
    cone.location.z = 0.15;
    cone.color = driverless_msgs::msg::Cone::UNKNOWN;

    std::string link_name = cone_link->GetName();

    if (link_name.substr(0, 9) == "blue_cone") {
        cone.color = driverless_msgs::msg::Cone::BLUE;
    } else if (link_name.substr(0, 11) == "yellow_cone") {
        cone.color = driverless_msgs::msg::Cone::YELLOW;
    } else if (link_name.substr(0, 11) == "orange_cone") {
        cone.color = driverless_msgs::msg::Cone::ORANGE_SMALL;
    } else if (link_name.substr(0, 8) == "big_cone") {
        cone.color = driverless_msgs::msg::Cone::ORANGE_BIG;
        cone.location.z = 0.225;
    }

    return cone;
}

driverless_msgs::msg::ConeDetectionStamped get_ground_truth_track(gazebo::physics::ModelPtr track_model,
                                                                  gazebo::common::Time now, std::string track_frame_id) {
    driverless_msgs::msg::ConeDetectionStamped track;

    track.header.frame_id = track_frame_id;
    track.header.stamp.sec = now.sec;
    track.header.stamp.nanosec = now.nsec;

    gazebo::physics::Link_V links = track_model->GetLinks();
    for (auto const &link : links) {
        track.cones.push_back(get_cone_from_link(link));
    }

    return track;
}

} // namespace vehicle_plugins
} // namespace gazebo_plugins  
