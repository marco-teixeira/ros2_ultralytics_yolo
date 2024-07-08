from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

input_image_topic ="/image_raw"
output_image_topic = "/output_yolo_image"
pub_image_output = True
is_track = True
yolo_model = "yolov8x-seg.pt" #yolo'x'-pose or yolo'x'-seg or yolo'x'
output_info_topic = "/output_yolo_info"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_ultralytics_yolo',
            namespace='yolo',
            executable='yolo',
            name='yolo',
            parameters= [
                {"yolo_model":yolo_model},
                {"image_topic_sub": input_image_topic},
                {"image_topic_pub": output_image_topic},
                {"yolo_topic_info_pub": output_info_topic },
                {"pub_image_output": pub_image_output},
                {"is_track": is_track},
            ]
        )
    ])
