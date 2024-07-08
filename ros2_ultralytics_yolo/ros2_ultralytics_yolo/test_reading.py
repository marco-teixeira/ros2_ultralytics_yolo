import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import json

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/output_yolo_info',
            self.listener_callback,
            10)
        self.subscription  

    def listener_callback(self, msg):
        json_data = json.loads(str(msg.data))

        for i in range(0,len(json_data)):   
            self.get_logger().info("---------------------------------------")
            self.get_logger().info(str(json_data[i]["class_index"]))
            self.get_logger().info(str(json_data[i]["class_name"]))
            self.get_logger().info(str(json_data[i]["Boxes"]))
            self.get_logger().info(str(json_data[i]["Masks"]))
            self.get_logger().info(str(json_data[i]["Keypoints"]))

        for data in json_data:   
            self.get_logger().info("---------------------------------------")
            self.get_logger().info(str(data["class_index"]))
            self.get_logger().info(str(data["class_name"]))
            self.get_logger().info(str(data["Boxes"]))
            self.get_logger().info(str(data["Masks"]))
            self.get_logger().info(str(data["Keypoints"]))
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()