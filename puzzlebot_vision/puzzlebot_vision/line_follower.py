import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        
        # Image processing parameters (same as your original code)
        self.target_width, self.target_height = 640, 480
        self.roi_bottom = 0.5
        self.side_crop_percent = 0.05
        self.min_box_area = 1000
        self.max_box_area = 100000
        
        # PID controller parameters
        self.Kp = 0.002
        self.Ki = 0.0001
        self.Kd = 0.001
        self.error_integral = 0.0
        self.last_error = 0.0
        self.base_speed = 0.1  # m/s
        
        # ROS 2 setup
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Debug windows
        cv2.namedWindow('Processing Output', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Binary Image', cv2.WINDOW_NORMAL)
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Resize if needed (same as original)
            img = cv2.resize(img, (self.target_width, self.target_height))
            
            # Process image using your exact method
            centers, output_img = self.process_image(img)
            
            # If we found contours (at least 3)
            if len(centers) >= 3:
                # Sort by x position and select left, center, right
                centers.sort(key=lambda x: x[0])
                left, center, right = centers[0], centers[len(centers)//2], centers[-1]
                
                # Calculate error (deviation from expected center)
                expected_center = (left[0] + right[0]) / 2
                error = (center[0] - expected_center) / self.target_width
                
                # PID control
                control = self.pid_control(error)
                self.publish_velocity(control)
                
                # Visualization
                self.visualize(output_img, [left, center, right])
            
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
    
    def process_image(self, img):
        """Your exact image processing pipeline adapted for ROS"""
        # --- Step 1: Create trapezoidal mask ---
        roi = img[int(self.target_height * self.roi_bottom):, :]
        roi_height, roi_width = roi.shape[:2]
        
        mask = np.zeros((roi_height, roi_width), dtype=np.uint8)
        top_width = int(roi_width * 0.9)
        trapezoid = np.array([[
            ((roi_width - top_width) // 2, int(roi_height * 0.0)),
            ((roi_width + top_width) // 2, int(roi_height * 0.0)),
            (roi_width, roi_height),
            (0, roi_height)
        ]], dtype=np.int32)
        
        cv2.fillPoly(mask, trapezoid, 255)
        roi = cv2.bitwise_and(roi, roi, mask=mask)
        
        # --- Step 2: Preprocess image ---
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary_inv = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
        
        # Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        morph = cv2.erode(binary_inv, kernel, iterations=3)
        morph = cv2.dilate(morph, kernel, iterations=3)
        
        # Canny edge detection
        canny_edges = cv2.Canny(morph, 50, 150)
        
        # Side mask
        crop_x = int(roi_width * self.side_crop_percent)
        side_mask = np.zeros_like(canny_edges)
        cv2.rectangle(
            side_mask,
            (crop_x, 0),
            (roi_width - crop_x, roi_height),
            255,
            thickness=-1
        )
        canny_edges = cv2.bitwise_and(canny_edges, canny_edges, mask=side_mask)
        
        # Apply trapezoidal mask to Canny
        all_edges_roi = cv2.bitwise_and(canny_edges, canny_edges, mask=mask)
        
        # --- Step 3: Find and process contours ---
        contours, _ = cv2.findContours(all_edges_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        output = roi.copy()
        centers = []
        epsilon_factor = 0.1
        
        for cnt in contours:
            # Approximate polygon
            epsilon = epsilon_factor * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # Bounding rectangle and center
            x, y, w, h = cv2.boundingRect(approx)
            box_area = w * h
            
            # Filter by bounding box area
            if box_area < self.min_box_area or box_area > self.max_box_area:
                continue
            
            cx = x + w // 2
            cy = y + h // 2 + int(self.target_height * self.roi_bottom)  # Adjust for ROI
            
            centers.append((cx, cy))
            
            # Draw on output (for visualization)
            cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(output, (cx - x, cy - int(self.target_height * self.roi_bottom)), 4, (0, 0, 255), -1)
            cv2.putText(output, f"({cx},{cy})", (cx - x + 10, cy - int(self.target_height * self.roi_bottom)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            cv2.polylines(output, [approx], isClosed=True, color=(0, 255, 0), thickness=2)
        
        return centers, output
    
    def pid_control(self, error):
        """PID controller implementation"""
        self.error_integral += error
        derivative = error - self.last_error
        self.last_error = error
        
        control = (self.Kp * error + 
                  self.Ki * self.error_integral + 
                  self.Kd * derivative)
        
        return control
    
    def publish_velocity(self, control):
        """Publish Twist message to cmd_vel"""
        cmd = Twist()
        cmd.linear.x = self.base_speed
        cmd.angular.z = -control  # Invert because positive error needs negative turn
        self.cmd_pub.publish(cmd)
    
    def visualize(self, output_img, centers):
        """Draw visualization of the three main lines"""
        # Convert ROI image to full image size for display
        display_img = np.zeros((self.target_height, self.target_width, 3), dtype=np.uint8)
        display_img[int(self.target_height * self.roi_bottom):, :] = output_img
        
        # Draw the three main centers
        for cx, cy in centers:
            cv2.circle(display_img, (cx, cy), 5, (0, 255, 255), -1)
        
        # Draw expected path between left and right lines
        if len(centers) >= 3:
            left = centers[0]
            right = centers[2]
            cv2.line(display_img, (left[0], left[1]), (right[0], right[1]), (255, 0, 255), 2)
        
        # Show binary image for debugging
        gray = cv2.cvtColor(output_img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        binary_display = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        
        cv2.imshow('Processing Output', display_img)
        cv2.imshow('Binary Image', binary_display)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()