import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import leap
import numpy as np
import cv2

_TRACKING_MODES = {
    leap.TrackingMode.Desktop: "Desktop",
    leap.TrackingMode.HMD: "HMD",
    leap.TrackingMode.ScreenTop: "ScreenTop",
}

class Hand_Marker_Array(Node):
    def __init__(self):
        super().__init__('stretch_marker')
        self.publisher_ = self.create_publisher(MarkerArray, 'test_marker_array', 10)  
        self.markerArray = MarkerArray()
        self.get_logger().info("Publishing the test_marker_array topic. Use RViz to visualize.")
    def publish_markers(self):
        self.publisher_.publish(self.markerArray)
    def flush_marker_array(self):
        self.markerArray = MarkerArray()
    def append_dot(self, x, y, z, index, scale, r, g, b):
        marker = Marker() 
        marker.header.frame_id = '/map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.SPHERE
        marker.id = index
        marker.action = marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        self.markerArray.markers.append(marker)
    def append_line(self, pointA, pointB, index, scale, r, g, b):
    	marker = Marker()
    	marker.header.frame_id = '/map'
    	marker.header.stamp = self.get_clock().now().to_msg()
    	marker.type = marker.LINE_LIST
    	marker.id = index
    	marker.action = marker.ADD
    	marker.scale.x = scale
    	marker.color.r = r
    	marker.color.g = g
    	marker.color.b = b
    	marker.color.a = 1.0
    	marker.pose.position.x = 0.0
    	marker.pose.position.y = 0.0
    	marker.pose.position.z = 0.0
    	point0 = Point()
    	point0.x = pointA[0]/100
    	point0.y = pointA[1]/100
    	point0.z = pointA[2]/100
    	
    	point1 = Point()
    	point1.x = pointB[0]/100
    	point1.y = pointB[1]/100
    	point1.z = pointB[2]/100
    	
    	marker.points.append(point0)
    	marker.points.append(point1)
    	
    	self.markerArray.markers.append(marker)
    
class Visualizer:
    def __init__(self):
        self.name = "RVIZ Visualiser"
        self.hands_format = "Skeleton"
        self.tracking_mode = None
        self.hand_marker_array =  Hand_Marker_Array()
    
    def set_tracking_mode(self, tracking_mode):
        self.tracking_mode = tracking_mode
    
    def toggle_hands_format(self):
        self.hands_format = "Dots" if self.hands_format == "Skeleton" else "Skeleton"
        print(f"Set hands format to {self.hands_format}")
    
    def get_joint_position(self, bone):
        if bone:
            return bone.x, bone.y, bone.z
        else:
            return None
            
    def render_hands(self, event):

        if len(event.hands) == 0:
            return
        self.hand_marker_array.flush_marker_array()
        for i in range(0, len(event.hands)):
            hand = event.hands[i]
            for index_digit in range(0, 5):
                digit = hand.digits[index_digit]
                for index_bone in range(0, 4):
                    bone = digit.bones[index_bone]
                    prev_joint = self.get_joint_position(bone.prev_joint)
                    next_joint = self.get_joint_position(bone.next_joint)
                    index = i*(5*4*3) + index_digit*4*3 + index_bone*3
                    if prev_joint:
                        print(prev_joint[0])
                        self.hand_marker_array.append_marker_array_dot(prev_joint[0]/100, prev_joint[1]/100, prev_joint[2]/100, index, 0.05, 0.0, 1.0, 0.0)
                    if next_joint:
                        self.hand_marker_array.append_marker_array_dot(next_joint[0]/100, next_joint[1]/100, next_joint[2]/100, index+1, 0.05, 0.0, 1.0, 0.0)
                    if prev_joint and next_joint:
                        self.hand_marker_array.append_marker_array_line(prev_joint, next_joint, index+1, 0.01, 0.0, 1.0, 0.0) 
        self.hand_marker_array.publish_markers()
    
    def render_hands_indexed(self, event):

        if len(event.hands) == 0:
            return
        self.hand_marker_array.flush_marker_array()
        for i in range(0, len(event.hands)):
            hand = event.hands[i]
            for index_digit in range(0, 5):
                digit = hand.digits[index_digit]
                for index_bone in range(0, 4):
                    bone = digit.bones[index_bone]
                    prev_joint = self.get_joint_position(bone.prev_joint)
                    next_joint = self.get_joint_position(bone.next_joint)
                    if (index_digit == 0 and index_bone == 0): #thumb
                    	continue
                    self.hand_marker_array.append_dot # prev joint, index
                    if(axis_display == True):
                    	# calculate yaxis
                    	#calculate zaxis
                    	self.hand_marker_array.append_line # prev y axis, index+1
                    	self.hand_marker_array.append_line # prev z axis, index+2
                    self.hand_marker_array.append_line # bone line, index+3
                    index = index + 4
                    
        self.hand_marker_array.publish_markers() 


class Canvas:
    def __init__(self):
        self.name = "Python Gemini Visualiser"
        self.screen_size = [500, 700]
        self.hands_colour = (255, 255, 255)
        self.font_colour = (0, 255, 44)
        self.hands_format = "Skeleton"
        self.output_image = np.zeros((self.screen_size[0], self.screen_size[1], 3), np.uint8)
        self.tracking_mode = None

    def set_tracking_mode(self, tracking_mode):
        self.tracking_mode = tracking_mode

    def toggle_hands_format(self):
        self.hands_format = "Dots" if self.hands_format == "Skeleton" else "Skeleton"
        print(f"Set hands format to {self.hands_format}")

    def get_joint_position(self, bone):
        if bone:
            return int(bone.x + (self.screen_size[1] / 2)), int(bone.z + (self.screen_size[0] / 2))
        else:
            return None

    def render_hands(self, event):
        # Clear the previous image
        self.output_image[:, :] = 0

        cv2.putText(
            self.output_image,
            f"Tracking Mode: {_TRACKING_MODES[self.tracking_mode]}",
            (10, self.screen_size[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            self.font_colour,
            1,
        )

        if len(event.hands) == 0:
            return

        for i in range(0, len(event.hands)):
            hand = event.hands[i]
            for index_digit in range(0, 5):
                digit = hand.digits[index_digit]
                for index_bone in range(0, 4):
                    bone = digit.bones[index_bone]
                    if self.hands_format == "Dots":
                        prev_joint = self.get_joint_position(bone.prev_joint)
                        next_joint = self.get_joint_position(bone.next_joint)
                        if prev_joint:
                            cv2.circle(self.output_image, prev_joint, 2, self.hands_colour, -1)

                        if next_joint:
                            cv2.circle(self.output_image, next_joint, 2, self.hands_colour, -1)

                    if self.hands_format == "Skeleton":
                        wrist = self.get_joint_position(hand.arm.next_joint)
                        elbow = self.get_joint_position(hand.arm.prev_joint)
                        if wrist:
                            cv2.circle(self.output_image, wrist, 3, self.hands_colour, -1)

                        if elbow:
                            cv2.circle(self.output_image, elbow, 3, self.hands_colour, -1)

                        if wrist and elbow:
                            cv2.line(self.output_image, wrist, elbow, self.hands_colour, 2)

                        bone_start = self.get_joint_position(bone.prev_joint)
                        bone_end = self.get_joint_position(bone.next_joint)

                        if bone_start:
                            cv2.circle(self.output_image, bone_start, 3, self.hands_colour, -1)

                        if bone_end:
                            cv2.circle(self.output_image, bone_end, 3, self.hands_colour, -1)

                        if bone_start and bone_end:
                            cv2.line(self.output_image, bone_start, bone_end, self.hands_colour, 2)

                        if ((index_digit == 0) and (index_bone == 0)) or (
                            (index_digit > 0) and (index_digit < 4) and (index_bone < 2)
                        ):
                            index_digit_next = index_digit + 1
                            digit_next = hand.digits[index_digit_next]
                            bone_next = digit_next.bones[index_bone]
                            bone_next_start = self.get_joint_position(bone_next.prev_joint)
                            if bone_start and bone_next_start:
                                cv2.line(
                                    self.output_image,
                                    bone_start,
                                    bone_next_start,
                                    self.hands_colour,
                                    2,
                                )

                        if index_bone == 0 and bone_start and wrist:
                            cv2.line(self.output_image, bone_start, wrist, self.hands_colour, 2)


class TrackingListener(leap.Listener):
    def __init__(self, canvas, visualizer):
        self.canvas = canvas
        self.visualizer = visualizer

    def on_connection_event(self, event):
        pass

    def on_tracking_mode_event(self, event):
        self.canvas.set_tracking_mode(event.current_tracking_mode)
        print(f"Tracking mode changed to {_TRACKING_MODES[event.current_tracking_mode]}")

    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()

        print(f"Found device {info.serial}")

    def on_tracking_event(self, event):
        self.canvas.render_hands(event)
        self.visualizer.render_hands(event)


def main(args=None):
    rclpy.init(args=args)
    canvas = Canvas()
    visualizer = Visualizer()
    
    tracking_listener = TrackingListener(canvas, visualizer)

    connection = leap.Connection()
    connection.add_listener(tracking_listener)

    running = True

    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)
        canvas.set_tracking_mode(leap.TrackingMode.Desktop)

        while running:
            cv2.imshow(canvas.name, canvas.output_image)

            key = cv2.waitKey(1)


if __name__ == "__main__":
    main()