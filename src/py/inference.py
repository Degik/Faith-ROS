import sys
# ROS2
import rclpy
# Collector Class
from collector import Collector
# YOLOInferenceNode Class
from YoloInference import YOLOInferenceNode
        
def main():
    collector = Collector()
    # Get the cameras topic from the config.ini
    cameras_topic = collector.get_cameras_topic()
    # Get the cameras ids from the config.ini
    cameras_ids = collector.get_cameras_ids()
    # Get the model name from the config.ini
    model_name = collector.get_model()

    rclpy.init(args=sys.argv)
    node = YOLOInferenceNode(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()