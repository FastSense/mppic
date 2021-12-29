from numpy.core.defchararray import center
import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from grid_map_msgs.msg import GridMap

BIG_HEIGHT_VAL = 20

def numpy_to_multiarray(np_array):
    
    multiarray = Float32MultiArray()
    # for i in range(np_array.ndim):

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


    # print(multiarray.layout.data_offset)
    
    multiarray.data = []
    for n in np.nditer(np_array, order="F"):
        # print(num_array,end=' ')
        multiarray.data.append(n)
    # .reshape([1, -1])[0].tolist()

    return multiarray

def get_index(point, resolution, length):
    len_x = length[0]
    len_y = length[1]

    i = (len_x / 2 - point[0]) // resolution
    j = (len_y / 2 - point[1]) // resolution

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
    y_range = np.arange(point[0] + half_new_len_y, point[0] - half_new_len_y, -resolution)

    new_map = np.full((len(x_range), len(y_range)), BIG_HEIGHT_VAL, dtype=np.float32)
    for i, x in enumerate(x_range):
        for j, y in enumerate(y_range):
            z = get_height_in_point(full_map, (x, y), resolution)
            new_map[i, j] = z

    return new_map
            






class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(GridMap, 'elevation_map', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # full_grid = np.load('/home/user/ros2_ws/src/algorithms/local_planners/mppic/grid_map_publisher/grid_map_publisher/elevation_map_filtered.npz')['arr_0']
        # self.original_grid = np.float32(original_grid[0, 50:200, 150:250])
        self.full_map = np.float32(np.load('/home/user/ros2_ws/src/algorithms/local_planners/mppic/grid_map_publisher/grid_map_publisher/elevation_map_filtered.npz')['arr_0'])[0]
        self.resolution = 0.05
        # plt.imshow(self.original_grid)
        # plt.show()
        print(self.full_map.shape)

    def timer_callback(self):
        len_x = self.resolution * self.full_map.shape[0]
        len_y = self.resolution * self.full_map.shape[1]

        center_x = np.random.uniform(-len_x/2, len_x/2, 1)
        center_y = np.random.uniform(-len_y/2, len_y/2, 1)
        new_len = (2, 2)
        new_map = get_map_piece(self.full_map, (center_x, center_y), new_len, self.resolution)
        
        
        msg = GridMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.length_x = self.full_map.shape[0] * msg.info.resolution
        msg.info.length_y = self.full_map.shape[1] * msg.info.resolution
        # msg.info.pose.position.x = -0.5
        # msg.info.pose.position.y = 4.85
        # msg.info.pose.position.z = 0.0
        msg.info.pose.position.x = center_x
        msg.info.pose.position.y = center_y
        msg.info.pose.position.z = 0.0
        msg.layers.append("elevation")
        # print(self.original_grid.dtype)
        msg.data.append(numpy_to_multiarray(new_map))
        self.publisher_.publish(msg)


# # Resolution of the grid [m/cell].
# float64 resolution

# # Length in x-direction [m].
# float64 length_x

# # Length in y-direction [m].
# float64 length_y

# # Pose of the grid map center in the frame defined in `header` [m].
# geometry_msgs/Pose pose
        # Header (time and frame)
# std_msgs/Header header

# # Grid map header
# GridMapInfo info

# # Grid map layer names.
# string[] layers

# # Grid map basic layer names (optional). The basic layers
# # determine which layers from `layers` need to be valid
# # in order for a cell of the grid map to be valid.
# string[] basic_layers

# # Grid map data.
# std_msgs/Float32MultiArray[] data

# # Row start index (default 0).
# uint16 outer_start_index

# # Column start index (default 0).
# uint16 inner_start_index
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()