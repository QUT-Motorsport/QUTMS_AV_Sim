#include "gazebo_vehicle_plugin/gazebo_vehicle.hpp"

namespace gazebo_plugins {
namespace vehicle_plugins {

VehiclePlugin::VehiclePlugin() {}

VehiclePlugin::~VehiclePlugin() { _update_connection.reset(); }

void VehiclePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    rosnode_ = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(rosnode_->get_logger(), "Loading VehiclePlugin");

    _model = model;
    _world = _model->GetWorld();

    _tf_br = std::make_unique<tf2_ros::TransformBroadcaster>(rosnode_);

    // Initialize parameters
    initParams();

    // ROS Publishers
    // Odometry
    _pub_odom = rosnode_->create_publisher<nav_msgs::msg::Odometry>("/odometry", 1);
    _pub_gt_odom = rosnode_->create_publisher<nav_msgs::msg::Odometry>("/odometry/ground_truth", 1);

    // RVIZ joint visuals
    pub_joint_state_ = rosnode_->create_publisher<sensor_msgs::msg::JointState>("/joint_states/steering", 1);

    // ROS Subscriptions
    _sub_cmd = rosnode_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/control/driving_command", 1, std::bind(&VehiclePlugin::onCmd, this, std::placeholders::_1));

    // ROS Services
    _reset_vehicle_pos_srv = rosnode_->create_service<std_srvs::srv::Trigger>(
        "/system/reset_car_pos",
        std::bind(&VehiclePlugin::resetVehiclePosition, this, std::placeholders::_1, std::placeholders::_2));

    // Connect to Gazebo
    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&VehiclePlugin::update, this));
    _last_sim_time = _world->SimTime();

    _max_steering_rate = (vehicle_model_->getParam().input_ranges.delta.max - vehicle_model_->getParam().input_ranges.delta.min) /
                         _steering_lock_time;

    // Set offset
    setPositionFromWorld();

    RCLCPP_INFO(rosnode_->get_logger(), "Gazebo VehiclePlugin Loaded");
}

void VehiclePlugin::initParams() {
    // Get ROS parameters
    _update_rate = rosnode_->declare_parameter("update_rate", 2.0);
    _publish_rate = rosnode_->declare_parameter("publish_rate", 50.0);
    _map_frame = rosnode_->declare_parameter("map_frame", "map");
    _odom_frame = rosnode_->declare_parameter("odom_frame", "odom");
    _base_frame = rosnode_->declare_parameter("base_frame", "base_link");
    _control_delay = rosnode_->declare_parameter("control_delay", 0.5);
    /// SHOULD BE IN VEHICLE PARAMS FILE
    _steering_lock_time = rosnode_->declare_parameter("steering_lock_time", 1.0);

    // Vehicle model
    std::string vehicle_yaml_name = rosnode_->declare_parameter("vehicle_params", "null");
    if (vehicle_yaml_name == "null") {
        RCLCPP_FATAL(rosnode_->get_logger(), "gazebo_vehicle plugin missing <vehicle_config> in <config.yaml>, cannot proceed");
        exit(1);
    }
    vehicle_model_ = std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::DynamicBicycle(vehicle_yaml_name));


    // Steering joints
    std::string leftSteeringJointName = _model->GetName() + "::left_steering_hinge_joint";
    _left_steering_joint = _model->GetJoint(leftSteeringJointName);
    std::string rightSteeringJointName = _model->GetName() + "::right_steering_hinge_joint";
    _right_steering_joint = _model->GetJoint(rightSteeringJointName);
}

void VehiclePlugin::setPositionFromWorld() {
    _offset = _model->WorldPose();

    RCLCPP_DEBUG(rosnode_->get_logger(), "Got starting offset %f %f %f", _offset.Pos()[0], _offset.Pos()[1],
                 _offset.Pos()[2]);

    state_.x = 0.0;
    state_.y = 0.0;
    state_.z = 0.0;
    state_.yaw = 0.0;
    state_.v_x = 0.0;
    state_.v_y = 0.0;
    state_.v_z = 0.0;
    state_.r_x = 0.0;
    state_.r_y = 0.0;
    state_.r_z = 0.0;
    state_.a_x = 0.0;
    state_.a_y = 0.0;
    state_.a_z = 0.0;

    _last_cmd.drive.steering_angle = 0;
    _last_cmd.drive.acceleration = -100;
    _last_cmd.drive.speed = 0;
}

bool VehiclePlugin::resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    state_.x = 0.0;
    state_.y = 0.0;
    state_.z = 0.0;
    state_.yaw = 0.0;
    state_.v_x = 0.0;
    state_.v_y = 0.0;
    state_.v_z = 0.0;
    state_.r_x = 0.0;
    state_.r_y = 0.0;
    state_.r_z = 0.0;
    state_.a_x = 0.0;
    state_.a_y = 0.0;
    state_.a_z = 0.0;

    const ignition::math::Vector3d vel(0.0, 0.0, 0.0);
    const ignition::math::Vector3d angular(0.0, 0.0, 0.0);

    _model->SetWorldPose(_offset);
    _model->SetAngularVel(angular);
    _model->SetLinearVel(vel);

    return response->success;
}

void VehiclePlugin::setModelState() {
    double yaw = state_.yaw + _offset.Rot().Yaw();

    double x = _offset.Pos().X() + state_.x * cos(_offset.Rot().Yaw()) - state_.y * sin(_offset.Rot().Yaw());
    double y = _offset.Pos().Y() + state_.x * sin(_offset.Rot().Yaw()) + state_.y * cos(_offset.Rot().Yaw());
    double z = state_.z;

    double vx = state_.v_x * cos(yaw) - state_.v_y * sin(yaw);
    double vy = state_.v_x * sin(yaw) + state_.v_y * cos(yaw);

    const ignition::math::Pose3d pose(x, y, z, 0, 0.0, yaw);
    const ignition::math::Vector3d vel(vx, vy, 0.0);
    const ignition::math::Vector3d angular(0.0, 0.0, state_.r_z);

    _model->SetWorldPose(pose);
    _model->SetAngularVel(angular);
    _model->SetLinearVel(vel);
}

nav_msgs::msg::Odometry VehiclePlugin::stateToOdom(const eufs::models::State &state) {
    // convert all state field into respective odometry fields
    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = _last_sim_time.sec;
    msg.header.stamp.nanosec = _last_sim_time.nsec;
    msg.header.frame_id = _odom_frame;
    msg.child_frame_id = _base_frame;

    msg.pose.pose.position.x = state.x;
    msg.pose.pose.position.y = state.y;

    std::vector<double> orientation = {state.yaw, 0.0, 0.0};
    orientation = to_quaternion(orientation);

    msg.pose.pose.orientation.x = orientation[0];
    msg.pose.pose.orientation.y = orientation[1];
    msg.pose.pose.orientation.z = orientation[2];
    msg.pose.pose.orientation.w = orientation[3];

    msg.twist.twist.linear.x = state.v_x;
    msg.twist.twist.linear.y = state.v_y;

    msg.twist.twist.angular.z = state.yaw;

    return msg;
}

void VehiclePlugin::publishVehicleMotion() {
    // Get odometry msg from state
    nav_msgs::msg::Odometry odom = stateToOdom(state_);
    if (has_subscribers(_pub_gt_odom)) {
        _pub_gt_odom->publish(odom);
    }

    // Apply noise to state and publish
    nav_msgs::msg::Odometry odom_noisy = stateToOdom(_noise->applyNoise(state_));
    if (has_subscribers(_pub_odom)) {
        _pub_odom->publish(odom_noisy);
    }
}

void VehiclePlugin::publishTf() {
    // Base->Odom
    // Position
    tf2::Transform base_to_odom;
    eufs::models::State noise_tf_state = _noise->applyNoise(state_);
    base_to_odom.setOrigin(tf2::Vector3(noise_tf_state.x, noise_tf_state.y, 0.0));

    // Orientation
    tf2::Quaternion base_odom_q;
    base_odom_q.setRPY(0.0, 0.0, noise_tf_state.yaw);
    base_to_odom.setRotation(base_odom_q);

    // Send TF
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp.sec = _last_sim_time.sec;
    transform_stamped.header.stamp.nanosec = _last_sim_time.nsec;
    transform_stamped.header.frame_id = _odom_frame;
    transform_stamped.child_frame_id = _base_frame;
    tf2::convert(base_to_odom, transform_stamped.transform);

    _tf_br->sendTransform(transform_stamped);

    // Odom->Map
    // Position
    tf2::Transform odom_to_map;
    odom_to_map.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

    // Orientation
    tf2::Quaternion odom_map_q;
    odom_map_q.setRPY(0.0, 0.0, 0.0);
    odom_to_map.setRotation(odom_map_q);

    // Send TF
    transform_stamped.header.stamp.sec = _last_sim_time.sec;
    transform_stamped.header.stamp.nanosec = _last_sim_time.nsec;
    transform_stamped.header.frame_id = _map_frame;
    transform_stamped.child_frame_id = _odom_frame;
    tf2::convert(odom_to_map, transform_stamped.transform);

    _tf_br->sendTransform(transform_stamped);
}

void VehiclePlugin::update() {
    // Check against update rate
    gazebo::common::Time curTime = _world->SimTime();
    double dt = calc_dt(_last_sim_time, curTime);
    if (dt < (1 / _update_rate)) {
        return;
    }

    _last_sim_time = curTime;

    des_input_.acc = _last_cmd.drive.acceleration;
    des_input_.vel = _last_cmd.drive.speed;
    des_input_.delta = _last_cmd.drive.steering_angle * 3.1415 / 180;
    // 90* (max steering angle) = 16* (max wheel angle)
    des_input_.delta *= (16.0 / 90.0);  // maybe use params not hardcoded?

    double current_speed = std::sqrt(std::pow(state_.v_x, 2) + std::pow(state_.v_y, 2));
    act_input_.acc = (des_input_.vel - current_speed) / dt;

    // Make sure steering rate is within limits
    act_input_.delta += (des_input_.delta - act_input_.delta >= 0 ? 1 : -1) *
                        std::min(_max_steering_rate * dt, std::abs(des_input_.delta - act_input_.delta));

    // Ensure vehicle can drive
    if ((_world->SimTime() - _last_cmd_time).Double() > 0.5) {
        act_input_.acc = -100.0;
        act_input_.vel = 0.0;
        act_input_.delta = 0.0;
    }

    // Update z value from simulation
    // This allows the state to have the most up to date value of z. Without this
    // the vehicle in simulation has problems interacting with the ground plane.
    // This may cause problems if the vehicle models start to take into account z
    // but because this simulation isn't for flying cars we should be ok (at least for now).
    state_.z = _model->WorldPose().Pos().Z();

    vehicle_model_->updateState(state_, act_input_, dt);

    _left_steering_joint->SetPosition(0, act_input_.delta);
    _right_steering_joint->SetPosition(0, act_input_.delta);
    // joint states
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp.sec = _last_sim_time.sec;
    joint_state.header.stamp.nanosec = _last_sim_time.nsec;
    joint_state.name.push_back(_left_steering_joint->GetName());
    joint_state.name.push_back(_right_steering_joint->GetName());
    joint_state.position.push_back(_left_steering_joint->Position());
    joint_state.position.push_back(_right_steering_joint->Position());

    pub_joint_state_->publish(joint_state);

    setModelState();

    double time_since_last_published = (_last_sim_time - _time_last_published).Double();
    if (time_since_last_published < (1 / _publish_rate)) {
        return;
    }
    _time_last_published = _last_sim_time;

    // Publish car states
    publishVehicleMotion();
    publishTf();
}

void VehiclePlugin::onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    RCLCPP_DEBUG(rosnode_->get_logger(), "Last time: %f", (_world->SimTime() - _last_cmd_time).Double());
    while ((_world->SimTime() - _last_cmd_time).Double() < _control_delay) {
        RCLCPP_DEBUG(rosnode_->get_logger(), "Waiting until control delay is over");
    }
    _last_cmd.drive.acceleration = msg->drive.acceleration;
    _last_cmd.drive.speed = msg->drive.speed;
    _last_cmd.drive.steering_angle = msg->drive.steering_angle;
    _last_cmd_time = _world->SimTime();
}

GZ_REGISTER_MODEL_PLUGIN(VehiclePlugin)

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins
