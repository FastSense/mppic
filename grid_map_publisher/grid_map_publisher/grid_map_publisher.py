from numpy.core.defchararray import center
import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from grid_map_msgs.msg import GridMap
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


BIG_HEIGHT_VAL = 20

def numpy_to_multiarray(np_array):
    
    multiarray = Float32MultiArray()
    dim = MultiArrayDimension()
    dim.label = "column_index"
    dim.size = np_array.shape[1]
    dim.stride = np_array.shape[1] 
    multiarray.layout.dim.append(dim)

    dim = MultiArrayDimension()
    dim.label = "row_index"
    dim.size = np_array.shape[0]
    dim.stride = np_array.shape[0] 
    multiarray.layout.dim.append(dim)

    
    multiarray.data = []
    for n in np.nditer(np_array, order="F"):
        multiarray.data.append(n)

    return multiarray

def get_index(point, resolution, length):
    len_x = length[0]
    len_y = length[1]

    i = int((len_x / 2 - point[0]) // resolution)
    j = int((len_y / 2 - point[1]) // resolution)

    return i, j


def get_height_in_point(full_map, point, resolution):
    x = point[0]
    y = point[1]

    len_x = resolution * full_map.shape[0]
    len_y = resolution * full_map.shape[1]

    if abs(x) >= len_x / 2 or abs(y) >= len_y / 2:
        return BIG_HEIGHT_VAL
    i, j = get_index(point, resolution, (len_x, len_y))
    
    return full_map[i, j]


def get_map_piece(full_map, point, new_len, resolution):
    half_new_len_x = new_len[0] / 2
    half_new_len_y = new_len[1] / 2

    x_range = np.arange(point[0] + half_new_len_x, point[0] - half_new_len_x, -resolution)
    y_range = np.arange(point[1] + half_new_len_y, point[1] - half_new_len_y, -resolution)

    new_map = np.full((len(x_range), len(y_range)), BIG_HEIGHT_VAL, dtype=np.float32)
    for i, x in enumerate(x_range):
        for j, y in enumerate(y_range):
            z = get_height_in_point(full_map, (x, y), resolution)
            new_map[i, j] = z

    return new_map
            



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('grid_map_publisher')
        self.publisher_ = self.create_publisher(GridMap, 'elevation_map', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.full_map = np.float32(np.load('/home/user/ros2_ws/src/algorithms/local_planners/mppic/grid_map_publisher/grid_map_publisher/elevation_map_filtered.npz')['arr_0'])[0].transpose()
        self.resolution = 0.05
        self.to_frame = "odom"
        self.from_frame = "base_link"

        self.new_len = (2, 2)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('grid_map_publisher configured. Full map loaded')
        print(self.full_map.shape)

    def timer_callback(self):
        trans = None
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.to_frame, self.from_frame, now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.from_frame} to {self.self.to_frame}: {ex}')
            return


        center_x = trans.transform.translation.x
        center_y = trans.transform.translation.y
        
        self.get_logger().info('grid_map updated')
        new_map = get_map_piece(self.full_map, (center_x, center_y), self.new_len, self.resolution)
        
        msg = GridMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.resolution
        msg.info.length_x = new_map.shape[0] * msg.info.resolution
        msg.info.length_y = new_map.shape[1] * msg.info.resolution

        msg.info.pose.position.x = center_x
        msg.info.pose.position.y = center_y
        msg.info.pose.position.z = 0.0
        msg.layers.append("elevation")

        msg.data.append(numpy_to_multiarray(new_map))
    
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()