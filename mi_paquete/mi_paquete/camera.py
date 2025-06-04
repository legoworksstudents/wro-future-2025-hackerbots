#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BlockDetectorNode(Node):
    def __init__(self):
        super().__init__('block_detector')
        
        # Declare parameter for number of blocks to publish
        self.declare_parameter('num_blocks', 2)  # Default value is 2
        self.num_blocks = self.get_parameter('num_blocks').value
        
        # Create publisher for block positions
        self.block_publisher = self.create_publisher(
            Float32MultiArray, 'block_detector/positions', 10)
            
        # Create publisher for the image
        self.image_publisher = self.create_publisher(
            Image, 'block_detector/image', 10)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create timer for periodic block detection (10 Hz)
        self.timer = self.create_timer(0.1, self.detect_and_publish)
        
        self.get_logger().info(f'Block detector node initialized. Publishing {self.num_blocks} blocks.')
        
    def detect_blocks(self, frame):
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for red and green
        # Red color range (two ranges because red wraps around in HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Green color range
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        
        # Create masks for red and green
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        
        return mask_red, mask_green

    def find_all_blocks(self, mask, frame_width, color):
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        blocks = []
        for contour in contours:
            # Get the center of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate position relative to center
                center_x = frame_width // 2
                position = (cx - center_x) / center_x * 100  # Percentage from center
                
                blocks.append({
                    'position': position,
                    'y': cy,
                    'color': color,
                    'center': (cx, cy)
                })
        
        return blocks

    def get_closest_blocks(self, all_blocks, num_blocks=3):
        if not all_blocks:
            return None
        
        # Sort blocks by y-coordinate (lower y means closer to bottom of frame)
        sorted_blocks = sorted(all_blocks, key=lambda x: x['y'], reverse=True)
        
        # Take only the specified number of closest blocks
        closest_blocks = sorted_blocks[:num_blocks]
        
        # Create the output list
        return [[block['position'], block['color']] for block in closest_blocks], closest_blocks

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Could not read frame")
            return
            
        frame_width = frame.shape[1]
        
        # Detect red and green blocks
        mask_red, mask_green = self.detect_blocks(frame)
        
        # Find all blocks
        red_blocks = self.find_all_blocks(mask_red, frame_width, "rojo")
        green_blocks = self.find_all_blocks(mask_green, frame_width, "verde")
        
        # Combine all blocks
        all_blocks = red_blocks + green_blocks
        
        # Get closest blocks using the parameter value
        result = self.get_closest_blocks(all_blocks, self.num_blocks)
        
        # Create and publish message
        msg = Float32MultiArray()
        
        # Configure layout
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = 'blocks'
        msg.layout.dim[0].size = self.num_blocks * 2  # num_blocks * 2 values (position and color)
        msg.layout.dim[0].stride = self.num_blocks * 2
        msg.layout.data_offset = 0
        
        if result is None:
            # If no blocks found, publish zeros
            msg.data = [0.0] * (self.num_blocks * 2)
        else:
            block_list, closest_blocks = result
            # Flatten the block list and convert colors to numbers (0 for red, 1 for green)
            flat_data = []
            for pos, color in block_list:
                flat_data.extend([pos, 0.0 if color == "rojo" else 1.0])
            msg.data = flat_data
        
        # Publish the message
        self.block_publisher.publish(msg)
        
        # Draw visualization
        self.draw_visualization(frame, result)
        
        # Convert and publish the image
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
        
        # Display the frame (only if running locally with GUI)
        try:
            cv2.imshow('Block Detection', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().debug(f'Could not display image (this is normal in headless mode): {e}')
    
    def draw_visualization(self, frame, result):
        frame_width = frame.shape[1]
        
        # Draw center line
        cv2.line(frame, (frame_width//2, 0), (frame_width//2, frame.shape[0]), (255, 255, 255), 2)
        
        if result is None:
            # Display "NO BLOCKS" on screen
            cv2.putText(frame, "NO BLOCKS", 
                       (frame_width//2 - 100, frame.shape[0]//2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        else:
            block_list, closest_blocks = result
            # Draw and label only the closest blocks
            for i, block in enumerate(closest_blocks):
                pos = block['center']
                color = block['color']
                color_bgr = (0, 0, 255) if color == "rojo" else (0, 255, 0)
                
                # Draw circle
                cv2.circle(frame, pos, 10, color_bgr, -1)
                
                # Draw number (1, 2, 3) to show order
                cv2.putText(frame, str(i+1), 
                           (pos[0] - 5, pos[1] + 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Draw percentage
                cv2.putText(frame, f"{block['position']:.1f}%", 
                           (pos[0] + 15, pos[1]), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
    
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BlockDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
