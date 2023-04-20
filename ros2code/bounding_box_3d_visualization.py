import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection3DArray
# from numpy import quaternion

import numpy as np
import open3d


class Detection3DArraySubscriber(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window(window_name='Bounding box')

        self.crt = self.vis.get_view_control()

        render_opt = self.vis.get_render_option()
        render_opt.point_size = 1
        render_opt.background_color = np.asarray([0, 0, 0])
        self.open3d_pcd = open3d.geometry.PointCloud()  # 创建空点云

        self.pcd_subscriber = self.create_subscription(
            Detection3DArray,  # Msg type
            'bbox',  # topic
            self.listener_callback,  # Function to call
            100  # QoS
        )

    def listener_callback(self, msg):
        objs = msg.detections  # Detection3D[]->list
        obj_num = len(objs)

        self.vis.clear_geometries()

        # objs[i]->Detection3D objs[i].bbox->BoundingBox3D
        for i in range(obj_num):
            size = objs[i].bbox.size  # l,w,h
            center = objs[i].bbox.center.position  # x,y,z
            quaternion = objs[i].bbox.center.orientation  # x,y,z,w

            v = np.array([0, 0, 0, 1])  # 指向ego车辆前方的向量：w, x, y, z。实部为0
            q = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])  # 四元数
            # print('quaternion: ')
            # print(q)
            q_conj = np.array([q[0], -1 * q[1], -1 * q[2], -1 * q[3]])  # 四元数的共轭
            # q * v * q_conj 旋转后的向量：w, x, y, z
            v_new = quaternion_inner_product(quaternion_inner_product(q, v), q_conj)
            v_obj = v_new[1:]

            l_x = size.x / 2
            w_y = size.y / 2
            h_z = size.z / 2
            # 中心位于原点的框的顶点
            a_points = np.array([[-l_x, -w_y, -h_z], [-l_x, -w_y, h_z], [-l_x, w_y, h_z], [-l_x, w_y, -h_z],
                                 [l_x, -w_y, -h_z], [l_x, -w_y, h_z], [l_x, w_y, h_z], [l_x, w_y, -h_z]])
            center_point = np.array([center.x, center.y, center.z])

            b_points = np.zeros((8, 3))  # 旋转后的框
            for j in range(8):
                a_points_j = np.zeros(4)
                a_points_j[1:] = a_points[j, :]  # w x y z
                # q * b_points_j * q_conj  旋转后的向量：w, x, y, z
                a_points_j_new = quaternion_inner_product(quaternion_inner_product(q, a_points_j), q_conj)
                b_points[j, :] = a_points_j_new[1:]
            # print('points_obj: ')
            # print(points_obj)
            points_obj = b_points + center_point  # 平移整个框

            # 框顶点之间的连接线
            box_lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                                  [4, 5], [5, 6], [6, 7], [7, 4],
                                  [0, 4], [1, 5], [2, 6], [3, 7]])
            # 线的颜色
            colors = np.array([[0, 1, 0] for k in range(12)])
            line_set = open3d.geometry.LineSet()
            line_set.lines = open3d.utility.Vector2iVector(box_lines)
            line_set.colors = open3d.utility.Vector3dVector(colors)
            line_set.points = open3d.utility.Vector3dVector(points_obj)

            self.vis.add_geometry(line_set)

        # self.crt.set_up((0, 0, 1))  # 设置垂直指向屏幕外的向量
        # self.crt.set_front((-1, -1, 1))  # 设置垂直指向屏幕上方的向量
        # self.crt.set_zoom(0.2)  # 设置视角放大比例

        self.vis.poll_events()
        self.vis.update_renderer()


def quaternion_inner_product(q1: np.ndarray, q2: np.ndarray):
    """
    四元数内积
    :param q1: w x y z
    :param q2: w x y z
    :return: w x y z
    """
    r1 = q1[0]
    r2 = q2[0]
    v1 = np.array([q1[1], q1[2], q1[3]])
    v2 = np.array([q2[1], q2[2], q2[3]])
    r = r1 * r2 - np.dot(v1, v2)
    v = r1 * v2 + r2 * v1 + np.cross(v1, v2)
    q = np.array([r, v[0], v[1], v[2]])
    return q


def main(args=None):
    rclpy.init(args=args)
    bounding_box_3d_visualization = Detection3DArraySubscriber('bounding_box_3d_visualization')
    rclpy.spin(bounding_box_3d_visualization)

    bounding_box_3d_visualization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
