import numpy
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
#https://docs.hello-robot.com/0.3/ros2/example_4/
#http://docs.ros.org/en/fuerte/api/rviz/html/marker__array__test_8py_source.html
class Test_Marker(Node):
    def __init__(self):
        super().__init__('stretch_marker')
        self.publisher_ = self.create_publisher(Marker, 'test_marker', 10)  

        self.marker = Marker()
        self.marker.header.frame_id = 'base_link'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = Marker.SPHERE
        self.marker.id = 0
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 2.0
        self.get_logger().info("Publishing the test_marker topic. Use RViz to visualize.")
    def publish_marker(self):
        self.publisher_.publish(self.marker)

class Test_Marker_Array(Node):
    def __init__(self):
        super().__init__('stretch_marker')
        self.publisher_ = self.create_publisher(MarkerArray, 'test_marker_array', 10)  
        self.markerArray = MarkerArray()
        for i in range(3):
            marker = Marker() 
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.id = i 
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = 0.1*i
            marker.pose.position.y = 0.0
            marker.pose.position.z = 2.0
            self.markerArray.markers.append(marker)
        self.get_logger().info("Publishing the test_marker_array topic. Use RViz to visualize.")
    def publish_markers(self):
        self.publisher_.publish(self.markerArray)


def main(args=None):
    rclpy.init(args=args)
    test_marker_array = Test_Marker_Array()
    while rclpy.ok():
        test_marker_array.publish_markers()
    test_marker_array.destroy_node()  
    rclpy.shutdown()
if __name__ == '__main__':
    main()