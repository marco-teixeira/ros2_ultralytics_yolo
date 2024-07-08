# Imports do ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ament_index_python.packages import get_package_share_directory


# Importes do ultralytics
from ultralytics import YOLO

# Demais imports
import cv2
import json



# Inicializa a câmera
# Use 0 para a câmera padrão, se tiver mais de uma camera altere para 1, 2, etc...
cap = cv2.VideoCapture(0)  


# Load a pretrained YOLOv8n model


class ros2_ultralytics_yolo(rclpy.node.Node):
    def __init__(self):
        super().__init__("yolo_node")
        package_share_directory = get_package_share_directory('ros2_ultralytics_yolo')

        # Seta parametros e seus valores iniciais caso nao sejam informados {
        self.declare_parameter(
            name="image_topic_sub",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="image_topic_pub",
            value="/camera/image_raw/yolo",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to pub to.",
            ),
        )

        self.declare_parameter(
            name="yolo_topic_info_pub",
            value="/yolo/info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to pub info.",
            ),
        )

        self.declare_parameter(
            name="yolo_model",
            value="yolov10x.pt",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Model of YoLo.",
            ),
        )

        self.declare_parameter(
            name="pub_image_output",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Pub image output?",
            ),
        )

        self.declare_parameter(
            name="is_track",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="is_track?",
            ),
        )

        


        image_topic_sub = (
            self.get_parameter("image_topic_sub").get_parameter_value().string_value
        )
        self.get_logger().info(f"image_topic_sub: {image_topic_sub}")

        image_topic_pub = (
            self.get_parameter("image_topic_pub").get_parameter_value().string_value
        )
        self.get_logger().info(f"image_topic_pub: {image_topic_pub}")

        yolo_topic_info_pub = (
            self.get_parameter("yolo_topic_info_pub").get_parameter_value().string_value
        )
        self.get_logger().info(f"yolo_topic_info_pub: {yolo_topic_info_pub}")

        yolo_model = (
            self.get_parameter("yolo_model").get_parameter_value().string_value
        )
        self.get_logger().info(f"yolo_model: {yolo_model}")

        self.pub_img_output = (
            self.get_parameter("pub_image_output").get_parameter_value().bool_value
        )
        self.get_logger().info(f"pub_image_output: {self.pub_img_output}")

        self.is_track = (
            self.get_parameter("is_track").get_parameter_value().bool_value
        )
        self.get_logger().info(f"is_track: {self.is_track}")
        # }

        # Callback {
        self.create_subscription(
            Image, image_topic_sub, self.image_callback, 1
        )
        # Callback }

        # Publisher {
        self.image_pub = self.create_publisher(Image, image_topic_pub, 10)
        self.yolo_info_pub = self.create_publisher(String, yolo_topic_info_pub, 10)
        # Publisher }

        # Modelo da YoLo a ser carregado
        self.model = YOLO(str(package_share_directory)+"/"+str(yolo_model))
        self.bridge = []
        self.bridge = CvBridge()

    def image_callback(self, img_msg):
 
        if self.is_track:        
            result = self.model.track(self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8'))
        else:
            result = self.model(self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8'))

        # publica a imagem apos a aplicação da YoLo se for True
        if self.pub_img_output:
            self.image_pub.publish( self.bridge.cv2_to_imgmsg(result[0].plot(show=False)))


        json_pub = []

        for index, cls in enumerate(result[0].boxes.cls):
            class_index = int(cls.cpu().numpy())
            class_name = result[0].names[class_index]

            # boxes
            boxes_conf = result[0].boxes.conf.cpu().numpy()[index].astype(float)
            boxes_data = list(result[0].boxes.data.cpu().numpy()[index].astype(float))
            boxes_is_track = result[0].boxes.is_track
            try:
                boxes_id = list(result[0].boxes.id.cpu().numpy()[index].astype(float))
            except:
                boxes_id = None

            # Masks
            try:
                masks_mask = result[0].masks.data.cpu().numpy()[index,:,:].astype(float).tolist()
                masks_xy = result[0].masks.xy[index].tolist()
            except:
                masks_mask = None
                masks_xy = None

            # keypoints
            try:
                keypoints_xy = result[0].keypoints.xy.cpu().numpy()[index].astype(float).tolist()  
            except:
                keypoints_xy = None

            json_pub_temp = {
                "class_index":class_index,
                "class_name":class_name,
                "Boxes":[
                    {"boxes_is_track":boxes_is_track},
                    {"track_boxes_id":boxes_id},
                    {"boxes_conf":boxes_conf},
                    {"boxes_data":boxes_data}
                ],
                "Masks":[
                    # {"masks_mask": masks_mask},
                    {"masks_xy":masks_xy}
                ],
                "Keypoints":[
                    {"xy": keypoints_xy}
                ]
            }

           
            json_pub.append(json_pub_temp)    
        
        self.yolo_info_pub.publish(String(data=str(json.dumps(json_pub))))





def main():
    rclpy.init()
    node = ros2_ultralytics_yolo()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
