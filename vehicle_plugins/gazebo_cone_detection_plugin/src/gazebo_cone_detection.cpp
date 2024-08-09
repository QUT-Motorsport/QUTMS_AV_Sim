#include "gazebo_cone_detection_plugin/gazebo_cone_detection.hpp"

namespace gazebo_plugins {
namespace vehicle_plugins {

ConeDetectionPlugin::ConeDetectionPlugin() {}

ConeDetectionPlugin::~ConeDetectionPlugin() { update_connection.reset(); }

void ConeDetectionPlugin::Load(gazebo::physics::ModelPtr gz_model, sdf::ElementPtr sdf) {
    node = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(node->get_logger(), "Loading ConeDetectionPlugin");

    model = gz_model;
    world = model->GetWorld();

    // Initialize parameters
    initParams();

    // Publishers
    track_pub = node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("track/ground_truth"), 1);
    detection_pub = node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("detections/ground_truth"), 1);

    track_marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(("visuals/track_markers"), 1);
    detection_marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(("visuals/detection_markers"), 1);

    // Cone position reset service
    reset_cone_pos_srv = node->create_service<std_srvs::srv::Trigger>("reset_cones",
        std::bind(&ConeDetectionPlugin::resetConePosition, this, std::placeholders::_1, std::placeholders::_2));

    last_track_update = world->SimTime();
    last_detection_update = world->SimTime();

    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ConeDetectionPlugin::update, this));

    //  Store initial track
    initial_track = get_ground_truth_track(track_model, last_track_update, map_frame);

    // Visuals
    initMarkers();

    RCLCPP_INFO(node->get_logger(), "ConeDetectionPlugin Loaded");
}

void ConeDetectionPlugin::initParams() {
    map_frame = node->declare_parameter("map_frame", "map");
    base_frame = node->declare_parameter("base_frame", "base_link");

    track_model = world->ModelByName("track");
    if (track_model == nullptr) {
        RCLCPP_FATAL(node->get_logger(), "Could not find required model 'track'. Exiting.");
        exit(1);
    }

    car_link = model->GetLink(base_frame);
    if (car_link == nullptr) {
        RCLCPP_FATAL(node->get_logger(), "Could not find required link <%s> on model <%s>. Exiting.", base_frame.c_str(),
                     model->GetName().c_str());
        exit(1);
    }

    car_inital_pose = car_link->WorldPose();

    detection_update_rate = node->declare_parameter("lidar_update_rate", 1.0);
    track_update_rate = node->declare_parameter("track_update_rate", 1.0);

    detection_config = {
        node->declare_parameter("lidar_frame_id", "sensor"),
        node->declare_parameter("lidar_min_view_distance", 0.0),
        node->declare_parameter("lidar_max_view_distance", 0.0),
        node->declare_parameter("lidar_fov", 0.0),
        node->declare_parameter("lidar_detects_colour", false),
        node->declare_parameter("lidar_offset_x", 0.0),
        node->declare_parameter("lidar_offset_z", 0.0)
    };
}

void ConeDetectionPlugin::update() {
    // publish track
    // faster rate than lidar so updates arent missed
    auto curr_time = world->SimTime();
    if (calc_dt(last_track_update, curr_time) < (1.0 / track_update_rate)) {
        return;
    }
    last_track_update = curr_time;

    auto ground_truth_track = get_ground_truth_track(track_model, curr_time, map_frame);
    if (has_subscribers(track_pub)) {
        auto centered_ground_truth = get_track_centered_on_car_inital_pose(car_inital_pose, ground_truth_track);
        track_pub->publish(centered_ground_truth);
    }
    if (has_subscribers(track_marker_pub)) {
        auto centered_ground_truth = get_track_centered_on_car_inital_pose(car_inital_pose, ground_truth_track);
        publishMarkerArray(centered_ground_truth, true);
    }

    // publish detection
    if (calc_dt(last_detection_update, curr_time) < (1.0 / detection_update_rate)) {
        return;
    }
    last_detection_update = curr_time;

    if (has_subscribers(detection_pub)) {
        auto lidar_detection = get_sensor_detection(detection_config, car_link->WorldPose(), ground_truth_track);
        detection_pub->publish(lidar_detection);
    }
    if (has_subscribers(detection_marker_pub)) {
        auto lidar_detection = get_sensor_detection(detection_config, car_link->WorldPose(), ground_truth_track);
        publishMarkerArray(lidar_detection, false);
    }
}

// Resets the position of cones to initial track model
bool ConeDetectionPlugin::resetConePosition(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    gazebo::physics::Link_V links = track_model->GetLinks();

    // Loop through all cones
    for (unsigned int i = 0; i < links.size(); i++) {
        driverless_msgs::msg::Cone cone = initial_track.cones[i];

        // Initial position and velocity variables
        const ignition::math::Pose3d pos(cone.location.x, cone.location.y, cone.location.z, 0.0, 0.0,
                                         0.0);
        const ignition::math::Vector3d vel(0.0, 0.0, 0.0);
        const ignition::math::Vector3d angular(0.0, 0.0, 0.0);

        RCLCPP_DEBUG(node->get_logger(), "Resetting cone %d to position (%f, %f, %f)", i, pos.Pos().X(),
                     pos.Pos().Y(), pos.Pos().Z());

        // Set cone position to initial position (and velocity)
        links[i]->SetWorldPose(pos);
        links[i]->SetAngularVel(vel);
        links[i]->SetLinearVel(angular);
    }

    return response->success;
}

// VISUALS

void ConeDetectionPlugin::initMarkers() {
    delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;

    blue_cone_marker.action = visualization_msgs::msg::Marker::ADD;
    blue_cone_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    blue_cone_marker.pose.orientation.x = 0.0;
    blue_cone_marker.pose.orientation.y = 0.0;
    blue_cone_marker.pose.orientation.z = 0.0;
    blue_cone_marker.pose.orientation.w = 1.0;
    blue_cone_marker.scale.x = 0.2;
    blue_cone_marker.scale.y = 0.2;
    blue_cone_marker.scale.z = 0.3;
    blue_cone_marker.color.r = 0.0;
    blue_cone_marker.color.g = 0.0;
    blue_cone_marker.color.b = 1.0;
    blue_cone_marker.color.a = 1.0;
    blue_cone_marker.ns = "cone";

    yellow_cone_marker.action = visualization_msgs::msg::Marker::ADD;
    yellow_cone_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    yellow_cone_marker.pose.orientation.x = 0.0;
    yellow_cone_marker.pose.orientation.y = 0.0;
    yellow_cone_marker.pose.orientation.z = 0.0;
    yellow_cone_marker.pose.orientation.w = 1.0;
    yellow_cone_marker.scale.x = 0.2;
    yellow_cone_marker.scale.y = 0.2;
    yellow_cone_marker.scale.z = 0.3;
    yellow_cone_marker.color.r = 1.0;
    yellow_cone_marker.color.g = 1.0;
    yellow_cone_marker.color.b = 0.0;
    yellow_cone_marker.color.a = 1.0;
    yellow_cone_marker.ns = "cone";

    orange_cone_marker.action = visualization_msgs::msg::Marker::ADD;
    orange_cone_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    orange_cone_marker.pose.orientation.x = 0.0;
    orange_cone_marker.pose.orientation.y = 0.0;
    orange_cone_marker.pose.orientation.z = 0.0;
    orange_cone_marker.pose.orientation.w = 1.0;
    orange_cone_marker.scale.x = 0.2;
    orange_cone_marker.scale.y = 0.2;
    orange_cone_marker.scale.z = 0.3;
    orange_cone_marker.color.r = 1.0;
    orange_cone_marker.color.g = 0.549;
    orange_cone_marker.color.b = 0.0;
    orange_cone_marker.color.a = 1.0;
    orange_cone_marker.ns = "cone";

    big_orange_cone_marker.action = visualization_msgs::msg::Marker::ADD;
    big_orange_cone_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    big_orange_cone_marker.pose.orientation.x = 0.0;
    big_orange_cone_marker.pose.orientation.y = 0.0;
    big_orange_cone_marker.pose.orientation.z = 0.0;
    big_orange_cone_marker.pose.orientation.w = 1.0;
    big_orange_cone_marker.scale.x = 0.2;
    big_orange_cone_marker.scale.y = 0.2;
    big_orange_cone_marker.scale.z = 0.45;
    big_orange_cone_marker.color.r = 1.0;
    big_orange_cone_marker.color.g = 0.271;
    big_orange_cone_marker.color.b = 0.0;
    big_orange_cone_marker.color.a = 1.0;
    big_orange_cone_marker.ns = "cone";

    unknown_cone_marker.action = visualization_msgs::msg::Marker::ADD;
    unknown_cone_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    unknown_cone_marker.pose.orientation.x = 0.0;
    unknown_cone_marker.pose.orientation.y = 0.0;
    unknown_cone_marker.pose.orientation.z = 0.0;
    unknown_cone_marker.pose.orientation.w = 1.0;
    unknown_cone_marker.scale.x = 0.2;
    unknown_cone_marker.scale.y = 0.2;
    unknown_cone_marker.scale.z = 0.3;
    unknown_cone_marker.color.r = 0.0;
    unknown_cone_marker.color.g = 1.0;
    unknown_cone_marker.color.b = 0.0;
    unknown_cone_marker.color.a = 1.0;
    unknown_cone_marker.ns = "cone";
}

void ConeDetectionPlugin::setConeMarker(const driverless_msgs::msg::Cone &cone,
                                        const std_msgs::msg::Header &header, const int &id,
                                        visualization_msgs::msg::Marker *marker) {
    marker->id = id;
    marker->header = header;
    marker->pose.position.x = cone.location.x;
    marker->pose.position.y = cone.location.y;
    marker->pose.position.z = cone.location.z;
}

void ConeDetectionPlugin::publishMarkerArray(driverless_msgs::msg::ConeDetectionStamped msg, bool is_track) {
    delete_all_marker.header = msg.header;
    delete_all_marker.id = id;
    marker_array.markers.push_back(delete_all_marker);

    if (is_track) {
        track_marker_pub->publish(marker_array);
    } else {
        detection_marker_pub->publish(marker_array);
    }
    marker_array.markers.clear();

    for (const auto &cone : msg.cones) {
        visualization_msgs::msg::Marker cone_marker;
        switch (cone.color) {
            case driverless_msgs::msg::Cone::BLUE:
                cone_marker = blue_cone_marker;
                break;
            case driverless_msgs::msg::Cone::YELLOW:
                cone_marker = yellow_cone_marker;
                break;
            case driverless_msgs::msg::Cone::ORANGE_BIG:
                cone_marker = big_orange_cone_marker;
                break;
            case driverless_msgs::msg::Cone::ORANGE_SMALL:
                cone_marker = orange_cone_marker;
                break;
            default:
                cone_marker = unknown_cone_marker;
                break;
        }
        setConeMarker(cone, msg.header, id, &cone_marker);
        marker_array.markers.push_back(cone_marker);
        id++;
    }

    if (is_track) {
        track_marker_pub->publish(marker_array);
    } else {
        detection_marker_pub->publish(marker_array);
    }

    marker_array.markers.clear();
}

GZ_REGISTER_MODEL_PLUGIN(ConeDetectionPlugin)

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins
