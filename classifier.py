import rclpy
import cv2
import pickle
import sklearn
import math

from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class Classifier(Node):

    def __init__(self):
        super().__init__('classifer')
        image_qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
			history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
			depth=1
		)
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.camera_callback,
            image_qos_profile)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            image_qos_profile)
        self.sign_publisher = self.create_publisher(String, 'sign_desc', 10)
        self.dist_publisher = self.create_publisher(Float32, 'dist_val', 10)
        
        self.cur_frame = CompressedImage()
        with open('/home/nimsi/Downloads/svm_classifier.pkl','rb') as f:
            self.svm_model = pickle.load(f)
        self.camera_subscription  # prevent unused variable warning

    def camera_callback(self, msg):
        self.cur_frame = msg
        
    def crop(self,image):
        isWall = True
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to the grayscale image
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        (_, image_th) = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY)
        #cv2_imshow(image_th)
        # Apply Canny edge detection to the grayscale image
        edges = cv2.Canny(image_th, 50, 200)

        # Perform a dilation and erosion operation to close gaps in between object edges
        dilated = cv2.dilate(edges, None, iterations=2)
        eroded = cv2.erode(dilated, None, iterations=1)

        # Find contours in the edge map
        contours, _ = cv2.findContours(eroded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
        #cv2_imshow(eroded)

        height, width, _ = image.shape
        if contours:
            
          largest_contour = max(contours, key=cv2.contourArea)
          isWall = False
          # cv2.drawContours(image, cv2.convexHull(largest_contour), -1, (0, 255, 0), 3)
          # cv2_imshow(image)
          # import pdb
          # pdb.set_trace()

          # Determine the bounding box for the largest contour
          x, y, w, h = cv2.boundingRect(largest_contour)

          # Crop the image using the bounding box
          cropped_image = image[max(0,y-10):min(height,y+h+10), max(0,x-10):min(width,x+w+10)]

          # Resize the cropped image to 150x150
          cropped_image = cv2.resize(cropped_image, (150, 150))

        else:
          # If no contours are found, crop the image from the center

          start_x = width // 2 - 75
          start_y = height // 2 - 75
          cropped_image = image[start_y:start_y+150, start_x:start_x+150]

        # Save the cropped image
        #cv2.imwrite(save_path, cropped_image)
        
        return cropped_image, isWall
    
    def lidar_callback(self,lidar_msg):
        bridge = CvBridge()
        deg_ranges = [-6, -5, -4, -3,-2, -1,0, 1, 2, 3, 4, 5, 6]
        distance_list = []
        for i in deg_ranges:
            if i > 0 :
                pos_rad  = 6.283 - (i*0.0174)
            elif i < 0:
                pos_rad = abs(i*0.0174)
            else:
                pos_rad = 0.0
			
            range_index = int(pos_rad/lidar_msg.angle_increment)%len(lidar_msg.ranges)
				
            #self.get_logger().info('range index "%f"'%(range_index))
            distance = lidar_msg.ranges[range_index]
            if (math.isnan(distance)!= True):
                distance_list.append(distance)
        dist = sum(distance_list)/len(distance_list)
        pub_dist = Float32()
        pub_dist.data = dist
        self.dist_publisher.publish(pub_dist)
		
        if dist <= 0.6:

            image_cv = bridge.compressed_imgmsg_to_cv2(self.cur_frame, desired_encoding='passthrough')
            image, isWall = self.crop(image_cv)
            sign = String()
            if not isWall:
                hog_features = cv2.HOGDescriptor().compute(image)
                pred = self.svm_model.predict([hog_features])
                if pred == 0:
                    sign.data = 'wall'
                elif pred == 1:
                    sign.data = 'left'
                elif pred == 2:
                    sign.data = 'right'
                elif pred == 3:
                    sign.data = 'u_turn'
                elif pred == 4:
                    sign.data = 'stop'
                elif pred == 5:
                    sign.data = 'goal'                
                
            else:
                sign.data = 'wall'
            
            self.sign_publisher.publish(sign)
        
def main(args=None):
    rclpy.init(args=args)

    classifier = Classifier()

    rclpy.spin(classifier)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    classifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
