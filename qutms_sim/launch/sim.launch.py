import os
from os.path import isfile, join

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)


def gen_world(context, *args, **kwargs):
    track = str(get_argument(context, "track") + ".world")

    tracks = get_package_share_directory("eufs_tracks")
    vehicle = get_package_share_directory("sim_vehicle")
    MODELS = os.environ.get("GAZEBO_MODEL_PATH")
    RESOURCES = os.environ.get("GAZEBO_RESOURCE_PATH")
    QUTMS = os.path.expanduser(os.environ.get("QUTMS_WS"))
    DISTRO = os.environ.get("ROS_DISTRO")

    os.environ["GAZEBO_PLUGIN_PATH"] = (
        QUTMS + "/install/vehicle_plugins:" + "/opt/ros/" + DISTRO
    )
    os.environ["GAZEBO_MODEL_PATH"] = tracks + "/models:" + str(MODELS)
    os.environ["GAZEBO_RESOURCE_PATH"] = (
        tracks
        + "/materials:"
        + tracks
        + "/meshes:"
        + vehicle
        + "/materials:"
        + vehicle
        + "/meshes:"
        + str(RESOURCES)
    )

    world_path = join(tracks, "worlds", track)

    gazebo_launch = join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )
    params_file = join(
        get_package_share_directory("qutms_sim"), "config", "user_config.yaml"
    )

    return [
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments=[
                ("verbose", "false"),
                ("pause", "false"),
                ("gui", "false"),
                ("world", world_path),
                ("params_file", params_file),
            ],
        ),
    ]


def spawn_car(context, *args, **kwargs):
    # get x,y,z,roll,pitch,yaw from track csv file
    tracks = get_package_share_directory("eufs_tracks")
    track = get_argument(context, "track")

    with open(join(tracks, "csv", track + ".csv"), "r") as f:
        # car position is last line of csv file
        for line in f:
            pass
        car_pos = line.split(",")
        x = car_pos[1]
        y = car_pos[2]
        yaw = car_pos[3]

    vehicle_config = get_argument(context, "vehicle_config")
    base_frame = get_argument(context, "base_frame")
    # enable_camera = get_argument(context, "enable_camera")
    # enable_lidar = get_argument(context, "enable_lidar")
    # enable_laserscan = get_argument(context, "enable_laserscan")

    xacro_path = join(
        get_package_share_directory("sim_vehicle"),
        "urdf",
        "robot.urdf.xacro",
    )
    urdf_path = join(
        get_package_share_directory("sim_vehicle"),
        "urdf",
        "robot.urdf",
    )

    if not isfile(urdf_path):
        os.mknod(urdf_path)

    doc = xacro.process_file(
        xacro_path,
        mappings={
            "vehicle_config": vehicle_config,
            "base_frame": base_frame,
            # "enable_camera": enable_camera,
            # "enable_lidar": enable_lidar,
            # "enable_laserscan": enable_laserscan,
        },
    )
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent="  "))

    with open(urdf_path, "r") as urdf_file:
        robot_description = urdf_file.read()

    return [
        Node(
            name="spawn_robot",
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=[
                "-entity",
                "QEV-3D",
                "-file",
                urdf_path,
                "-x",
                x,
                "-y",
                y,
                "-Y",
                yaw,
                "-spawn_service_timeout",
                "60.0",
                "--ros-args",
                "--log-level",
                "warn",
            ],
        ),
        Node(
            name="joint_state_publisher",
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "rate": 200,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "source_list": ["joint_states/steering"],
                }
            ],
            arguments=[urdf_path],
        ),
        Node(
            name="robot_state_publisher",
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "rate": 200,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
    ]


def load_rviz(context, *args, **kwargs):
    rviz_config_file = join(
        get_package_share_directory("qutms_sim"), "rviz", "default.rviz"
    )
    use_sim_time = get_argument(context, "use_sim_time")
    print(use_sim_time)

    return [
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", rviz_config_file],
                    condition=IfCondition(LaunchConfiguration("rviz")),
                    parameters=[
                        {"use_sim_time": LaunchConfiguration("use_sim_time")}
                    ],
                ),
            ],
        ),
    ]

import yaml
def generate_launch_description():
    qutms_sim_pkg = get_package_share_directory("qutms_sim")

    default_plugin_yaml = join(qutms_sim_pkg, "config", "config.yaml")

    with open(default_plugin_yaml, "r") as f:
        data = yaml.safe_load(f)

    noise_config = join(qutms_sim_pkg, "config", "motion_noise.yaml")
    vehicle_config = join(qutms_sim_pkg, "config", "vehicle_params.yaml")

    # write noise path to plugin yaml
    data["vehicle"]["ros__parameters"]["noise_config"] = noise_config
    data["vehicle"]["ros__parameters"]["vehicle_params"] = vehicle_config

    track = data["/**"]["ros__parameters"]["track"]
    sim_time = str(data["/**"]["ros__parameters"]["use_sim_time"])
    rviz = str(data["/**"]["ros__parameters"]["rviz"])
    base_frame = data["vehicle"]["ros__parameters"]["base_frame"]

    plugin_yaml = join(qutms_sim_pkg, "config", "user_config.yaml")
    with open(plugin_yaml, "w") as f:
        yaml.safe_dump(data, f)
    print(plugin_yaml)

    # load yaml file
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value=sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                name="track",
                default_value=track,
                description="Determines which track is launched",
            ),
            DeclareLaunchArgument(
                name="vehicle_config",
                default_value=vehicle_config,
                description="Determines the file from which the vehicle model parameters are read",
            ),
            DeclareLaunchArgument(
                name="rviz",
                default_value=rviz,
                description="Condition to launch the Rviz GUI",
            ),
            DeclareLaunchArgument(
                name="base_frame",
                default_value=base_frame,
                description="ROS transform frame for the vehicle base",
            ),
            DeclareLaunchArgument(
                name="enable_camera",
                default_value="false",
                description="Condition to enable camera",
            ),
            DeclareLaunchArgument(
                name="enable_lidar",
                default_value="false",
                description="Condition to enable lidar",
            ),
            DeclareLaunchArgument(
                name="enable_laserscan",
                default_value="false",
                description="Condition to enable laserscan",
            ),
            # OpaqueFunction(function=load_rviz),
            # launch the gazebo world
            OpaqueFunction(function=gen_world),
            # launch the car
            OpaqueFunction(function=spawn_car),
        ]
    )

if __name__ == "__main__":
    generate_launch_description()