# import sys
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
# from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
import open3d


class PCDPublisher(Node):

    rate = 5  # frame/seconds
    FRAME_START_NUM = 69
    FRAME_END_NUM = 221
    frame_num = FRAME_START_NUM

    header = Header()
    header.frame_id = 'LiDAR'
    points = None

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('pcd_path', '/home/thu/Downloads/2021_08_23_21_47_19/225')
        self.declare_parameter('rate', '5')

        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 100)
        rate = self.get_parameter('rate').get_parameter_value().integer_value
        # rate = int(rate_str)
        timer_period = 1/rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        pcd_path = self.get_parameter('pcd_path').get_parameter_value().string_value
        assert os.path.exists(pcd_path), "路径不存在：" + pcd_path

        num_str = str(self.frame_num).zfill(6)
        pcd_file = pcd_path + '/' + num_str + '.pcd'
        assert os.path.isfile(pcd_file), 'PCD文件不存在：' + pcd_file
        pcd = open3d.io.read_point_cloud(pcd_file)  # 返回open3d.geometry.PointCloud
        self.points = np.asarray(pcd.points)  # 返回numpy.ndarray

        self.get_logger().info('PCD file: ' + pcd_file)

        self.header.stamp = self.get_clock().now().to_msg()

        pc = create_cloud(self.header, self.points)  # 返回PointCloud2
        self.publisher.publish(pc)

        if self.frame_num+2 > self.FRAME_END_NUM:
            self.frame_num = self.FRAME_START_NUM
        else:
            self.frame_num += 2
        self.count += 1


def create_cloud(header, points: np.ndarray):
    """ Creates a point cloud message.
    Args:
        header: PointCloud2 header
        points: Nx3 array of xyz positions.
    Returns:
        sensor_msgs/PointCloud2 message
    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes()

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PCDPublisher('point_cloud_publisher')
    rclpy.spin(point_cloud_publisher)

    point_cloud_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
