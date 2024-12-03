#!/usr/bin/env python3
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os, csv
import numpy as np
from datetime import datetime
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import subprocess
from rclpy.qos import QoSProfile, ReliabilityPolicy

def img_processing(img_file, plot_width_m=0.3, plot_height_m=0.3, x_value=0.5):

    try:
        # Get Package Path
        package_path = get_package_share_directory('project_two')
    except KeyError:
        # Error Handling
        print("Error: Could not find ROS 2 package 'project_two'.")
        return False
    
    # Construct Full Paths for Image and CSV
    image_path = os.path.join(package_path, img_file)
    output_csv_path = os.path.join(package_path, "csv/contours.csv")
    os.makedirs(os.path.dirname(output_csv_path), exist_ok=True)
            
    # Load Image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Error: Could not load the image at {image_path}.")
        return False
    # image = cv2.rotate(image, cv2.ROTATE_180)

    # GaussianBlur to reduce noise 
    blurred_img = cv2.GaussianBlur(image, (7, 7), 0)

    # Canny edge detection
    edges = cv2.Canny(blurred_img, 50, 150) #edge thresholding

    # Find contours based on the edges detected by Canny
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contours were found
    if len(contours) == 0:
        print("No contours found!")
    else:
        # New Empty image to draw contours 
        contour_image = np.ones_like(image) * 255 

        # Draw contours on the empty image 
        for contour in contours:
            cv2.drawContours(contour_image, [contour], -1, (0, 0, 0), 2) 
    
    # Save contour image 
    # cv2.imwrite("contour_image.png",contour_image)
    
    # Combine Contours into Array for Bounding Box
    all_points = np.vstack([contour[:, 0, :] for contour in contours])

    # Calculate Bounding Box of Image
    y_min, z_min = np.min(all_points, axis=0)
    y_max, z_max = np.max(all_points, axis=0)
    width        = y_max - y_min
    height       = z_max - z_min

    # Determine Image Scale
    scale_y = plot_width_m  / width
    scale_z = plot_height_m / height
    scale   = min(scale_y, scale_z)

    # Normalize and Scale Contours
    scaled_contours = []
    yplan = []
    zplan = []
    for contour in contours:
        
        for point in contour:
            y, z = point[0]
            y_normalized = (y - y_min) * scale
            z_normalized = -(z - z_min) * scale + 1.3
            yplan.append(y_normalized)
            zplan.append(z_normalized)
            scaled_contours.append((x_value, y_normalized, z_normalized))
    
    # Save plot of planned contour trajectory for artifacts
    plt.plot(yplan, zplan)
    plt.title("Planned Contour Trajectory")
    plt.xlabel("Y-axis")
    plt.ylabel("Z-axis")
    plt.savefig('planned_contour_traj.png')

    # Write Contours to CSV
    with open(output_csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["X", "Y_mm", "Z_mm"])  # Header
        writer.writerows(scaled_contours)

    return True

def call_inv_kin():
    
    package_name = 'project_two'
    package_prefix = get_package_prefix(package_name)
    lib_dir = os.path.join(package_prefix, 'lib', package_name)
    
    script_path = os.path.join(lib_dir, 'inverse_kinematics.py')
    
    # Running the script using subprocess.run
    result = subprocess.run(['python3', script_path], capture_output=True, text=True)

    # Output of the script
    if result.stdout:
        print(f"STDOUT: {result.stdout}")
    if result.stderr:
        print(f"STDERR: {result.stderr}")    


class ImgSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create output img directory
        package_share_dir = get_package_share_directory('project_two')
        self.img_dir = os.path.join(package_share_dir, "camera_images")
        os.makedirs(self.img_dir,exist_ok=True)
    
            
        self.last_image = None
        self.got_image = False
        self.bridge = CvBridge()

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Create Subscriber to '/camera/image_raw' topic
        self.subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
    def image_callback(self, msg):        
        """
        Callback function to handle the incoming image message.
        This function stores the latest received image.
        """
        try:
            if not self.got_image:
                
                # Convert the ROS image message to a CV2 image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 
                print("\n")               
                self.get_logger().info("Received new image")  
                
                # Save image to timestamped file
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")                
                filename = f"{self.img_dir}/image_{timestamp}.png"
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f"Image saved to {filename}")
                
                # Get sketch trajectory from image
                success = img_processing(filename)
                if success:
                    self.get_logger().info("Image processed successfully!")
                    print("\n")
                    self.get_logger().info("Generating inverse kinematics on planned end effector trajectory...")
                    call_inv_kin()
                    self.get_logger().info("Finished running inverse kinematics on planned end effector trajectory!")
                    
                    
                else:
                    self.get_logger().info("Image processing FAILED!")
                    
            
                # Set flag that image was processed
                self.got_image = True   
                                      
        except Exception as e:
            print(f"Error processing image: {e}")        
        
def main():
    rclpy.init()
    node = ImgSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
