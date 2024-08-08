import os
from os.path import isfile, join
import yaml

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sim_pkg = get_package_share_directory("qutms_sim")

def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)


def gen_world(context, *args, **kwargs):
    track = str(get_argument(context, "track") + ".world")

    MODELS = os.environ.get("GAZEBO_MODEL_PATH")
    RESOURCES = os.environ.get("GAZEBO_RESOURCE_PATH")
    QUTMS = os.path.expanduser(os.environ.get("QUTMS_WS"))
    DISTRO = os.environ.get("ROS_DISTRO")

    os.environ["GAZEBO_PLUGIN_PATH"] = (
        QUTMS + "/install/vehicle_plugins:" + "/opt/ros/" + DISTRO
    )
    os.environ["GAZEBO_MODEL_PATH"] = sim_pkg + "/models:" + str(MODELS)
    os.environ["GAZEBO_RESOURCE_PATH"] = (
        sim_pkg
        + "/meshes:"
        + sim_pkg
        + "/materials:"
        + sim_pkg
        + "/meshes:"
        + str(RESOURCES)
    )

    world_path = join(sim_pkg, "worlds", track)

    gazebo_launch = join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    params_file = join(sim_pkg, "config", "user_config.yaml")

    return [
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments=[
                ("verbose", "false"),
                ("pause", "false"),
                ("gui", "false"),
                ("world", world_path),
                ("params_file", params_file),
                ("gdb", "true"),
            ],
        ),
    ]


def spawn_car(context, *args, **kwargs):
    # get x,y,z,roll,pitch,yaw from track csv file
    track = get_argument(context, "track")

    with open(join(sim_pkg, "worlds", track + ".csv"), "r") as f:
        # car position is last line of csv file
        for line in f:
            pass
        car_pos = line.split(",")
        x = car_pos[1]
        y = car_pos[2]
        yaw = car_pos[3]

    vehicle_config = get_argument(context, "vehicle_config")
    base_frame = get_argument(context, "base_frame")
    display_car = get_argument(context, "display_car")
    namespace = get_argument(context, "namespace")

    xacro_path = join(sim_pkg, "urdf", "robot.urdf.xacro")
    urdf_path = join(sim_pkg, "urdf", "robot.urdf")

    if not isfile(urdf_path):
        os.mknod(urdf_path)

    doc = xacro.process_file(
        xacro_path,
        mappings={
            "vehicle_config": vehicle_config,
            "base_frame": base_frame,
            "display_car": display_car,
            "namespace": namespace,
        },
    )
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent="  "))

    with open(urdf_path, "r") as urdf_file:
        robot_description = urdf_file.read()

    return [
        TimerAction(
            period=3.0,
            actions=[
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
                        "-robot_namespace",
                        namespace,
                        "--ros-args",
                        "--log-level",
                        "warn",
                    ],
                ),
            ],
        ),
        Node(
            name="joint_state_publisher",
            package="joint_state_publisher",
            executable="joint_state_publisher",
            namespace=namespace,
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
            namespace=namespace,
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


def load_visuals(context, *args, **kwargs):
    rviz_config_file = join(sim_pkg, "rviz", "default.rviz")

    return [
        TimerAction(
            period=3.0,
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
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            condition=IfCondition(LaunchConfiguration("foxglove")),
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),
    ]

def generate_launch_description():
    default_plugin_yaml = join(sim_pkg, "config", "config.yaml")

    with open(default_plugin_yaml, "r") as f:
        data = yaml.safe_load(f)

    noise_config = join(sim_pkg, "config", "motion_noise.yaml")
    vehicle_config = join(sim_pkg, "config", "vehicle_params.yaml")

    track = data["/**"]["ros__parameters"]["track"]
    sim_time = str(data["/**"]["ros__parameters"]["use_sim_time"])
    rviz = str(data["/**"]["ros__parameters"]["rviz"])
    foxglove = str(data["/**"]["ros__parameters"]["foxglove"])
    display_car = data["/**"]["ros__parameters"]["display_car"]
    namespace = data["/**"]["ros__parameters"]["namespace"]
    base_frame = data["/**"]["ros__parameters"]["base_frame"]

    # write noise path to plugin yaml
    data[namespace]["vehicle"]["ros__parameters"]["noise_config"] = noise_config
    data[namespace]["vehicle"]["ros__parameters"]["vehicle_params"] = vehicle_config

    plugin_yaml = join(sim_pkg, "config", "user_config.yaml")
    with open(plugin_yaml, "w") as f:
        yaml.safe_dump(data, f)

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
                name="foxglove",
                default_value=foxglove,
                description="Condition to launch the Foxglove GUI",
            ),
            DeclareLaunchArgument(
                name="base_frame",
                default_value=base_frame,
                description="ROS transform frame for the vehicle base",
            ),
            DeclareLaunchArgument(
                name="namespace",
                default_value=namespace,
                description="ROS namespace for the vehicle",
            ),
            DeclareLaunchArgument(
                name="display_car",
                default_value=display_car,
                description="Determines if the car is displayed in Rviz",
            ),
            OpaqueFunction(function=load_visuals),
            # launch the gazebo world
            OpaqueFunction(function=gen_world),
            # launch the car
            OpaqueFunction(function=spawn_car),
        ]
    )

if __name__ == "__main__":
    generate_launch_description()