from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    image_topic = "/head_rgbd_sensor/rgb/image_raw"
    depth_topic = "/head_rgbd_sensor/depth_registered/image_raw"
    depth_info_topic = "/head_rgbd_sensor/depth_registered/camera_info"
    cloud_topic = "/head_rgbd_sensor/depth_registered/rectified_points"

    camera_params = {
        "image_topic": image_topic,
        "depth_topic": depth_topic,
        "depth_info_topic": depth_info_topic,
        "use_sim_time": True,
    }

    return LaunchDescription([

        Node(
            package="known_locations_tf_server",
            executable="known_locations_server",
            output="screen",
            parameters=[{
                "locations_file": str(Path.home() / ".takeshi" / "rc_2025_known_locations.yaml"),
                "frame_id": "map",
                "use_sim_time": True,
            }],
        ),

        # Main vision interface (OpenCV windows)
        Node(
            package="vision",
            executable="vision_node",
            name="vision_node",
            output="screen",
            parameters=[{
                "image_topic": image_topic,
                "segment_trigger_topic": "/vision/segment_once",
                "use_sim_time": True,
            }],
        ),

        # YOLO object detection
        Node(
            package="vision",
            executable="yolo_server",
            name="yolo_server",
            output="screen",
            parameters=[{
                **camera_params,
                "confidence_threshold": 0.25,
                "yolo_input_color": "bgr",
            }],
        ),

        # Point-cloud segmentation
        Node(
            package="vision",
            executable="plane_detector",
            name="plane_detector",
            output="screen",
            parameters=[{
                "cloud_topic": cloud_topic,
                "target_frame": "",
                "trigger_topic": "/vision/segment_once",
                "process_on_trigger": True,
                "horizontal_only": False,
                "publish_object_candidates": True,
                "use_sim_time": True,
            }],
        ),

        #Face recognition — 
        Node(
            package="vision",
            executable="face_recog_service_node",
            name="face_recog_service_node",
            output="screen",
            parameters=[camera_params],
        ),

        #Human pose estimation — 
        Node(
            package="vision",
            executable="pose_service_node",
            name="pose_service_node",
            output="screen",
            parameters=[camera_params],
        ),
    ])
