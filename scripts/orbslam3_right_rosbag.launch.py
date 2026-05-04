"""Launch ORB-SLAM3 monocular on the right Grasshopper images.

Pipeline:

    [ros2 bag play <bag>]  --/stereo/right/image_raw-->  bayer_to_mono.py
                                                              |
                                                              v
                                                       /stereo/right/image_mono
                                                              |
                                                              v
                                                       orbslam3 mono node

The launch file deliberately does NOT play the bag for you -- run

    ros2 bag play <bag> --clock

in a sibling terminal (or with the wrapper at
``scripts/run_orbslam3_right_rosbag.sh``) so the SLAM viewer comes up first
and the bag rate stays observable.

Default settings file is the in-tree ``config/monocular/gh.yaml`` calibrated
for the LEFT/narrow-angle Grasshopper lens; the rover's ``/stereo/right/...``
stream is from the WIDE-ANGLE lens, so the trajectory will still come out
roughly correct in shape but absolute scale and edge-distortion residuals
will be off until a wide-lens calibration replaces it.
"""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_REPO_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_VOC = (
    Path.home() / "ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt"
)
_DEFAULT_SETTINGS = (
    Path.home() / "ros2_ws/src/orbslam3_ros2/config/monocular/gh.yaml"
)


def _make_nodes(context, *_args, **_kwargs):
    bayer_topic = LaunchConfiguration("bayer_topic").perform(context)
    mono_topic = LaunchConfiguration("mono_topic").perform(context)
    voc_file = LaunchConfiguration("voc_file").perform(context)
    settings_file = LaunchConfiguration("settings_file").perform(context)
    downscale = LaunchConfiguration("downscale").perform(context)
    trajectory_prefix = LaunchConfiguration("trajectory_prefix").perform(context)

    bayer_node = Node(
        package="earth_rover",
        executable="bayer_to_mono.py",
        name="bayer_to_mono",
        output="screen",
        parameters=[{
            "input_topic": bayer_topic,
            "output_topic": mono_topic,
            "downscale": int(downscale),
        }],
    )

    bayer_fallback = Node(
        executable=str(_REPO_ROOT / "scripts" / "bayer_to_mono.py"),
        name="bayer_to_mono",
        output="screen",
        parameters=[{
            "input_topic": bayer_topic,
            "output_topic": mono_topic,
            "downscale": int(downscale),
        }],
    )

    slam_node = Node(
        package="orbslam3",
        executable="mono",
        name="orbslam3_mono",
        output="screen",
        arguments=[voc_file, settings_file],
        parameters=[{
            "image_topic": mono_topic,
            "trajectory_prefix": trajectory_prefix,
        }],
    )

    return [bayer_fallback, slam_node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "bayer_topic", default_value="/stereo/right/image_raw",
            description="Raw Bayer image topic from the bag.",
        ),
        DeclareLaunchArgument(
            "mono_topic", default_value="/stereo/right/image_mono",
            description="Topic this launch publishes the debayered mono8 stream on.",
        ),
        DeclareLaunchArgument(
            "voc_file", default_value=str(_DEFAULT_VOC),
            description="ORBvoc.txt path.",
        ),
        DeclareLaunchArgument(
            "settings_file", default_value=str(_DEFAULT_SETTINGS),
            description="ORB-SLAM3 monocular YAML (camera intrinsics + ORB + viewer).",
        ),
        DeclareLaunchArgument(
            "downscale", default_value="1",
            description="Integer downscale applied before SLAM (1 = native 2048x1536).",
        ),
        DeclareLaunchArgument(
            "trajectory_prefix", default_value="orbslam3_right",
            description="Filename prefix for *_CameraTrajectory.txt and *_KeyFrameTrajectory.txt.",
        ),
        OpaqueFunction(function=_make_nodes),
    ])
