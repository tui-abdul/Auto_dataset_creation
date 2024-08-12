import rclpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from rclpy.node import Node
from builtin_interfaces.msg import Time
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy import qos
import message_filters
from sensor_msgs_py.point_cloud2 import read_points
import ros2_numpy
import open3d as o3d
import subprocess
import yaml

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')


        with open('../extrinsic_calibration/calibration_results/1108_603_data/40272603.yaml', "r") as file_handle:
            self.calib_data_left = yaml.safe_load(file_handle)
            self.matrix_coefficients_left =    self.calib_data_left["camera_matrix"]["data"]
            self.distortion_coefficients_left = self.calib_data_left["distortion_coefficients"]["data"]
            self.matrix_coefficients_left = np.array(self.matrix_coefficients_left).reshape(3,3)
            self.distortion_coefficients_left = np.array(self.distortion_coefficients_left)

        with open('../extrinsic_calibration/calibration_results/1108_618_data/40243618.yaml', "r") as file_handle:
            self.calib_data_right = yaml.safe_load(file_handle)
            self.matrix_coefficients_right =    self.calib_data_right["camera_matrix"]["data"]
            self.distortion_coefficients_right = self.calib_data_right["distortion_coefficients"]["data"]
            self.matrix_coefficients_right = np.array(self.matrix_coefficients_right).reshape(3,3)
            self.distortion_coefficients_right = np.array(self.distortion_coefficients_right)



        bag_file_path = "/mnt/sda/Abdul_Haq/crossing_data/pole_a_right_cam_calib/pole_A_krossing_sensors_calib_1"
        command = ["ros2", "bag", "play", bag_file_path]
        # Run the command
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("First command started, now starting the next command.")

        
        image_color ='/basler_pole_a_left_id_103_sn_603/my_camera/pylon_ros2_camera_node/image_raw'
        ouster = '/ouster_pole_a_1108/points'
        image_color1 ='/basler_pole_a_right_id_104_sn_618/my_camera/pylon_ros2_camera_node/image_raw'

        # Subscribe to topics
        image_sub = message_filters.Subscriber(self, Image, image_color)
        image_sub1 = message_filters.Subscriber(self, Image, image_color1)
        ouster_sub = message_filters.Subscriber(self, PointCloud2, ouster,qos_profile= qos.qos_profile_sensor_data)

        # Synchronize the topics by time
        ats = message_filters.ApproximateTimeSynchronizer(
            [image_sub,image_sub1,ouster_sub], queue_size=20, slop=0.1)#, allow_headerless=True)
        ats.registerCallback(self.callback)

        self.bridge = CvBridge()
        self.get_logger().info('Initialization complete')
        self.count = 0

    def transformation(self,pc_as_numpy_array):
        t_mat =np.array([
                [-1, 0, 0, 0], 
                [0, -1, 0, 0], 
                [0, 0, 1, 0.038195], 
                [0, 0, 0, 1]
            ])
        column_of_ones = np.ones((pc_as_numpy_array.shape[0] , 1))

        result_array = np.hstack((pc_as_numpy_array, column_of_ones))
        transformed_point_3d = np.dot(t_mat, result_array.T)
        #print('t_point',transformed_point_3d.shape)
        return transformed_point_3d.T

    def callback(self, image_msg,image_msg1 ,osuter_msg):
        self.count = self.count + 1
        filename = str(self.count)
        folder_image_1 = 'camera_image_0/'
        folder_image_2 = 'camera_image_1/'
        folder_pc = 'lidar_point_cloud_0/'
        self.get_logger().info('New message arrived')
        # Convert ROS Image messages to OpenCV images
        print('camera 1',image_msg.header.stamp.nanosec)
        print('camera 2',image_msg1.header.stamp.nanosec)
        print('lidar',osuter_msg.header.stamp.nanosec)
        #self.lidar_msg = osuter_msg
        #self.publisher_lidar.publish(self.lidar_msg)
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        cv_image1 = self.bridge.imgmsg_to_cv2(image_msg1, desired_encoding='bgr8')
        undistorted_image_left = cv2.undistort(cv_image, self.matrix_coefficients_left,self.distortion_coefficients_left)
        undistorted_image_right = cv2.undistort(cv_image1, self.matrix_coefficients_right,self.distortion_coefficients_right)
        cv2.imwrite(folder_image_1 + filename + '.jpg', undistorted_image_left)
        cv2.imwrite(folder_image_2 + filename +'.jpg', undistorted_image_right)

        cv2.namedWindow("Image 1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Image 2", cv2.WINDOW_NORMAL)
        # Process the images (example: show them)
        cv2.imshow("Image 1", undistorted_image_left)
        cv2.imshow("Image 2", undistorted_image_right)

        cv2.waitKey(1)  # Display images for a short period of time

        pc_as_numpy_array = np.array(ros2_numpy.point_cloud2.point_cloud2_to_array(osuter_msg)['xyz'] )
        pc_as_numpy_array = self.transformation(pc_as_numpy_array)
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pc_as_numpy_array[:,:3])
        o3d.io.write_point_cloud(folder_pc + filename + '.pcd', point_cloud, format='auto', write_ascii=False, compressed=False, print_progress=False)
        # Close all OpenCV windows

       
        '''
        vis = o3d.visualization.Visualizer()
        vis.create_window('edges')
        
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pc_as_numpy_array[:,:3] )
        # geometry is the point cloud used in your animaiton
        
        vis.add_geometry(point_cloud)
        #view_ctl = self.vis.get_view_control()
        #view_ctl.set_zoom(0.15)
        #coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        #self.vis.add_geometry(coordinate_frame)
        
        #self.vis.update_geometry(point_cloud)
        #self.vis.poll_events()
        #self.vis.update_renderer()
        vis.run()
        vis.destroy_window()
        
        while True:
            key = cv2.waitKey(0) & 0xFF
            # Close the window when 'q' is pressed
            if key == ord('q'):
                break
        
        cv2.destroyAllWindows()
        '''
        
def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
