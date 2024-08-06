#include "gazebo_vehicle_plugin/gazebo_vehicle.hpp"

namespace gazebo_plugins {
namespace vehicle_plugins {

VehiclePlugin::VehiclePlugin() {}

VehiclePlugin::~VehiclePlugin() { update_connection.reset(); }

void VehiclePlugin::Load(gazebo::physics::ModelPtr gz_model, sdf::ElementPtr sdf) {
    node = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(node->get_logger(), "Loading VehiclePlugin");

    model = gz_model;
    world = model->GetWorld();

    tf_br = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    // Initialize parameters
    initParams();

    // ROS Publishers
    // Odometry
    odometry_pub = node->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
    gt_odometry_pub = node->create_publisher<nav_msgs::msg::Odometry>("odometry/ground_truth", 1);

    // RVIZ joint visuals
    joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states/steering", 1);

    // ROS Subscriptions
    ackermann_cmd_sub = node->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "control/ackermann_cmd", 1, std::bind(&VehiclePlugin::onAckermannCmd, this, std::placeholders::_1));
    twist_cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "control/twist_cmd", 1, std::bind(&VehiclePlugin::onTwistCmd, this, std::placeholders::_1));

    reset_vehicle_pos_srv = node->create_service<std_srvs::srv::Trigger>(
        "reset_vehicle",
        std::bind(&VehiclePlugin::resetVehiclePosition, this, std::placeholders::_1, std::placeholders::_2));

    // Connect to Gazebo
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&VehiclePlugin::update, this));
    last_sim_time = world->SimTime();

    max_steering_rate = (vehicle_model->getParam().input_ranges.delta.max - vehicle_model->getParam().input_ranges.delta.min) /
                         steering_lock_time;

    // Set offset
    setPositionFromWorld();

    RCLCPP_INFO(node->get_logger(), "Gazebo VehiclePlugin Loaded");
}

void VehiclePlugin::initParams() {
    // Get ROS parameters
    update_rate = node->declare_parameter("update_rate", 2.0);
    publish_rate = node->declare_parameter("publish_rate", 50.0);
    map_frame = node->declare_parameter("map_frame", "map");
    odom_frame = node->declare_parameter("odom_frame", "odom");
    base_frame = node->declare_parameter("base_frame", "base_link");
    control_delay = node->declare_parameter("control_delay", 0.5);
    steering_lock_time = node->declare_parameter("steering_lock_time", 1.0);

    // Vehicle model
    std::string vehicle_yaml_name = node->declare_parameter("vehicle_params", "null");
    if (vehicle_yaml_name == "null") {
        RCLCPP_FATAL(node->get_logger(), "gazebo_vehicle plugin missing <vehicle_config> in <config.yaml>, cannot proceed");
        exit(1);
    }
    vehicle_model = std::make_unique<VehicleModelBike>(vehicle_yaml_name);
    motion_noise = std::make_unique<Noise>(vehicle_yaml_name);

    // Steering joints
    std::string left_steering_joint_name = model->GetName() + "::left_steering_hinge_joint";
    left_steering_joint = model->GetJoint(left_steering_joint_name);
    std::string right_steering_joint_name = model->GetName() + "::right_steering_hinge_joint";
    right_steering_joint = model->GetJoint(right_steering_joint_name);
}

void VehiclePlugin::setPositionFromWorld() {
    offset = model->WorldPose();

    RCLCPP_DEBUG(node->get_logger(), "Got starting offset %f %f %f", offset.Pos().X(), offset.Pos().Y(),
                 offset.Pos().Z());

    state_odom.header.stamp.sec = last_sim_time.sec;
    state_odom.header.stamp.nanosec = last_sim_time.nsec;
    state_odom.header.frame_id = odom_frame;
    state_odom.child_frame_id = base_frame;
    state_odom.pose.pose.position.x = offset.Pos().X();
    state_odom.pose.pose.position.y = offset.Pos().Y();
    state_odom.pose.pose.position.z = offset.Pos().Z();

    state_odom.pose.pose.orientation.x = offset.Rot().X();
    state_odom.pose.pose.orientation.y = offset.Rot().Y();
    state_odom.pose.pose.orientation.z = offset.Rot().Z();
    state_odom.pose.pose.orientation.w = offset.Rot().W();

    state_odom.twist.twist.linear.x = 0.0;
    state_odom.twist.twist.linear.y = 0.0;
    state_odom.twist.twist.linear.z = 0.0;

    state_odom.twist.twist.angular.x = 0.0;
    state_odom.twist.twist.angular.y = 0.0;
    state_odom.twist.twist.angular.z = 0.0;

    last_cmd.drive.steering_angle = 0;
    last_cmd.drive.acceleration = -100;
    last_cmd.drive.speed = 0;

    state = odomToState(state_odom);
}

bool VehiclePlugin::resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    state_odom.header.stamp.sec = last_sim_time.sec;
    state_odom.header.stamp.nanosec = last_sim_time.nsec;
    state_odom.header.frame_id = odom_frame;
    state_odom.child_frame_id = base_frame;
    state_odom.pose.pose.position.x = offset.Pos().X();
    state_odom.pose.pose.position.y = offset.Pos().Y();
    state_odom.pose.pose.position.z = offset.Pos().Z();

    state_odom.pose.pose.orientation.x = offset.Rot().X();
    state_odom.pose.pose.orientation.y = offset.Rot().Y();
    state_odom.pose.pose.orientation.z = offset.Rot().Z();
    state_odom.pose.pose.orientation.w = offset.Rot().W();

    state_odom.twist.twist.linear.x = 0.0;
    state_odom.twist.twist.linear.y = 0.0;
    state_odom.twist.twist.linear.z = 0.0;

    state_odom.twist.twist.angular.x = 0.0;
    state_odom.twist.twist.angular.y = 0.0;
    state_odom.twist.twist.angular.z = 0.0;

    last_cmd.drive.steering_angle = 0;
    last_cmd.drive.speed = -1;

    const ignition::math::Vector3d vel(0.0, 0.0, 0.0);
    const ignition::math::Vector3d angular(0.0, 0.0, 0.0);

    model->SetWorldPose(offset);
    model->SetAngularVel(angular);
    model->SetLinearVel(vel);

    state = odomToState(state_odom);

    return response->success;
}

// void VehiclePlugin::setModelState() {
//     double x = offset.Pos().X() + state_odom.pose.pose.position.x * cos(offset.Rot().Yaw()) - state_odom.pose.pose.position.y * sin(offset.Rot().Yaw());
//     double y = offset.Pos().Y() + state_odom.pose.pose.position.x * sin(offset.Rot().Yaw()) + state_odom.pose.pose.position.y * cos(offset.Rot().Yaw());
//     double z = state_odom.pose.pose.position.z;

//     double yaw = to_euler(state_odom.pose.pose.orientation)[2] + offset.Rot().Yaw();

//     double vx = state_odom.twist.twist.linear.x * cos(yaw) - state_odom.twist.twist.linear.y * sin(yaw);
//     double vy = state_odom.twist.twist.linear.x * sin(yaw) + state_odom.twist.twist.linear.y * cos(yaw);

//     const ignition::math::Pose3d pose(x, y, z, 0.0, 0.0, yaw);
//     const ignition::math::Vector3d vel(vx, vy, 0.0);
//     const ignition::math::Vector3d angular(0.0, 0.0, state_odom.twist.twist.angular.z);

//     model->SetWorldPose(pose);
//     model->SetAngularVel(angular);
//     model->SetLinearVel(vel);
// }

void VehiclePlugin::setModelState() {
    double yaw = state.yaw + offset.Rot().Yaw();

    double x = offset.Pos().X() + state.x * cos(offset.Rot().Yaw()) - state.y * sin(offset.Rot().Yaw());
    double y = offset.Pos().Y() + state.x * sin(offset.Rot().Yaw()) + state.y * cos(offset.Rot().Yaw());
    double z = state.z;

    double vx = state.v_x * cos(yaw) - state.v_y * sin(yaw);
    double vy = state.v_x * sin(yaw) + state.v_y * cos(yaw);

    const ignition::math::Pose3d pose(x, y, z, 0, 0.0, yaw);
    const ignition::math::Vector3d vel(vx, vy, 0.0);
    const ignition::math::Vector3d angular(0.0, 0.0, state.r_z);

    model->SetWorldPose(pose);
    model->SetAngularVel(angular);
    model->SetLinearVel(vel);
}

void VehiclePlugin::publishVehicleOdom() {
    // Get odometry msg from state
    if (has_subscribers(gt_odometry_pub)) {
        gt_odometry_pub->publish(state_odom);
    }

    // Apply noise to state and publish
    nav_msgs::msg::Odometry odom_noisy = motion_noise->applyNoise(state_odom);
    if (has_subscribers(odometry_pub)) {
        odometry_pub->publish(odom_noisy);
    }
}

void VehiclePlugin::publishTf() {
    // Base->Odom
    // Position
    tf2::Transform base_to_odom;
    nav_msgs::msg::Odometry odom_noisy = motion_noise->applyNoise(state_odom);
    base_to_odom.setOrigin(tf2::Vector3(odom_noisy.pose.pose.position.x, odom_noisy.pose.pose.position.y, 0.0));

    // Orientation
    tf2::Quaternion base_odom_q;
    tf2::convert(odom_noisy.pose.pose.orientation, base_odom_q);
    base_to_odom.setRotation(base_odom_q);

    // Send TF
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp.sec = last_sim_time.sec;
    transform_stamped.header.stamp.nanosec = last_sim_time.nsec;
    transform_stamped.header.frame_id = odom_frame;
    transform_stamped.child_frame_id = base_frame;
    tf2::convert(base_to_odom, transform_stamped.transform);

    tf_br->sendTransform(transform_stamped);

    // Odom->Map
    // Position
    tf2::Transform odom_to_map;
    odom_to_map.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

    // Orientation
    tf2::Quaternion odom_map_q;
    odom_map_q.setRPY(0.0, 0.0, 0.0);
    odom_to_map.setRotation(odom_map_q);

    // Send TF
    transform_stamped.header.stamp.sec = last_sim_time.sec;
    transform_stamped.header.stamp.nanosec = last_sim_time.nsec;
    transform_stamped.header.frame_id = map_frame;
    transform_stamped.child_frame_id = odom_frame;
    tf2::convert(odom_to_map, transform_stamped.transform);

    tf_br->sendTransform(transform_stamped);
}

nav_msgs::msg::Odometry VehiclePlugin::stateToOdom(const State &state) {
    // convert all state field into respective odometry fields
    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = last_sim_time.sec;
    msg.header.stamp.nanosec = last_sim_time.nsec;
    msg.header.frame_id = odom_frame;
    msg.child_frame_id = base_frame;

    msg.pose.pose.position.x = state.x;
    msg.pose.pose.position.y = state.y;

    std::vector<double> orientation = {0.0, 0.0, state.yaw};
    msg.pose.pose.orientation = to_quaternion(orientation);

    msg.twist.twist.linear.x = state.v_x;
    msg.twist.twist.linear.y = state.v_y;

    msg.twist.twist.angular.z = state.yaw;

    return msg;
}

State VehiclePlugin::odomToState(const nav_msgs::msg::Odometry &odom) {
    State state;
    state.x = odom.pose.pose.position.x;
    state.y = odom.pose.pose.position.y;
    geometry_msgs::msg::Quaternion q = odom.pose.pose.orientation;
    state.yaw = to_euler(q)[2];
    state.v_x = odom.twist.twist.linear.x;
    state.v_y = odom.twist.twist.linear.y;
    state.r_z = odom.twist.twist.angular.z;

    return state;
}

void VehiclePlugin::update() {
    // Check against update rate
    gazebo::common::Time curTime = world->SimTime();
    double dt = calc_dt(last_sim_time, curTime);
    if (dt < (1 / update_rate)) {
        return;
    }

    last_sim_time = curTime;

    input.acceleration = last_cmd.drive.acceleration;
    input.velocity = last_cmd.drive.speed;
    input.steering = last_cmd.drive.steering_angle * 3.1415 / 180; // Convert to radians
    // 90* (max steering angle) = 16* (max wheel angle)
    input.steering *= (16.0 / 90.0);  // maybe use params not hardcoded?

    double current_speed = std::sqrt(std::pow(state_odom.twist.twist.linear.x, 2) + std::pow(state_odom.twist.twist.linear.y, 2));
    output.acceleration = (input.velocity - current_speed) / dt;

    // RCLCPP_INFO(node->get_logger(), "Steering Angle: %f current: %f", (input.steering, output.steering));

    // Make sure steering rate is within limits
    output.steering += (input.steering - output.steering >= 0 ? 1 : -1) *
                        std::min(max_steering_rate * dt, std::abs(input.steering - output.steering));


    // Ensure vehicle can drive
    if (input.velocity < 0) {
        output.acceleration = -100.0;
        output.velocity = 0.0;
        output.steering = 0.0;
    }

    // Update z value from simulation
    // This allows the state to have the most up to date value of z. Without this
    // the vehicle in simulation has problems interacting with the ground plane.
    // This may cause problems if the vehicle models start to take into account z
    // but because this simulation isn't for flying cars we should be ok (at least for now).
    state.z = model->WorldPose().Pos().Z();

    // odom to state
    vehicle_model->updateState(state, output, dt);

    left_steering_joint->SetPosition(0, output.steering);
    right_steering_joint->SetPosition(0, output.steering);

    // joint state visuals
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp.sec = last_sim_time.sec;
    joint_state.header.stamp.nanosec = last_sim_time.nsec;
    joint_state.name.push_back(left_steering_joint->GetName());
    joint_state.name.push_back(right_steering_joint->GetName());
    joint_state.position.push_back(left_steering_joint->Position());
    joint_state.position.push_back(right_steering_joint->Position());

    joint_state_pub->publish(joint_state);

    setModelState();

    double time_since_last_published = (last_sim_time - last_published_time).Double();
    if (time_since_last_published < (1 / publish_rate)) {
        return;
    }
    last_published_time = last_sim_time;

    state_odom = stateToOdom(state);

    // Publish car states
    publishVehicleOdom();
    publishTf();
}

void VehiclePlugin::onAckermannCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    RCLCPP_DEBUG(node->get_logger(), "Last time: %f", (world->SimTime() - last_cmd_time).Double());
    while ((world->SimTime() - last_cmd_time).Double() < control_delay) {
        RCLCPP_DEBUG(node->get_logger(), "Waiting until control delay is over");
    }
    last_cmd.drive.acceleration = msg->drive.acceleration;
    last_cmd.drive.speed = msg->drive.speed;
    last_cmd.drive.steering_angle = msg->drive.steering_angle;
    last_cmd_time = world->SimTime();
}

void VehiclePlugin::onTwistCmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_DEBUG(node->get_logger(), "Last time: %f", (world->SimTime() - last_cmd_time).Double());
    while ((world->SimTime() - last_cmd_time).Double() < control_delay) {
        RCLCPP_DEBUG(node->get_logger(), "Waiting until control delay is over");
    }
    if (msg->linear.x > 0) {
        last_cmd.drive.speed = msg->linear.x;
    } else if (msg->linear.x < 0) {
        last_cmd.drive.speed = -1;
        last_cmd.drive.steering_angle = 0;
    }
    last_cmd.drive.steering_angle += msg->angular.z;

    // last_cmd.drive.steering_angle = angular_vel_to_steering_angle(last_cmd.drive.speed, msg->angular.z, vehicle_model->getParam().kinematic.l);
    RCLCPP_INFO(node->get_logger(), "Steering Angle: %f", (last_cmd.drive.steering_angle));

    last_cmd_time = world->SimTime();
}

GZ_REGISTER_MODEL_PLUGIN(VehiclePlugin)

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins
