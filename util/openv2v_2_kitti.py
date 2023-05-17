##################################################################################
# 注意，OpenV2V是左手坐标系！！！！！！！！！
# open3d是右手坐标系
##################################################################################

import math

import yaml
import open3d
import numpy as np
import os


def show_openv2v_3d_bbox(pcd_file_name='/home/thu/Downloads/2021_08_23_21_47_19/243/000069'):
    """
    根据对应的配置文件，在点云中绘制识别车辆的3D边界框
    :param: 路径和帧名
    :return:
    """

    with open(pcd_file_name+'.yaml', 'r', encoding='utf-8') as f:
        result = yaml.load(f.read(), Loader=yaml.FullLoader)

    visualizer = open3d.visualization.Visualizer()
    visualizer.create_window()
    render_opt = visualizer.get_render_option()
    render_opt.point_size = 1
    render_opt.background_color = np.asarray([1, 1, 1])

    # crt = visualizer.get_view_control()
    # crt.set_front((-1, -1, 1))
    # crt.set_up((0, 0, 1))
    # crt.set_zoom(0.2)

    pcd = open3d.io.read_point_cloud(pcd_file_name+'.pcd',
                                     print_progress=False)
    pcd.paint_uniform_color([0, 0, 1.0])

    lidar_pose = result['lidar_pose']  # x y z roll yaw pitch，雷达在地图坐标系中的位置和姿态
    # ego车辆的雷达在地图中的坐标，注意，该坐标对应点云坐标系的原点
    lidar_location = np.array([lidar_pose[0], lidar_pose[1], lidar_pose[2]])
    lidar_rotation = np.array([lidar_pose[3], lidar_pose[5], lidar_pose[4]])  # roll, pitch, yaw
    pcd_rotation = (lidar_rotation / 180) * np.pi
    pcd_rotate_matrix = euler_angle_2_rotation_matrix(pcd_rotation)  # 点云旋转矩阵

    pcd.translate(lidar_location, relative=True)  # 移动点云，但点云中心点不移动
    pcd.rotate(pcd_rotate_matrix, center=lidar_location)  # 旋转点云与地图坐标系对齐
    # print(pcd.get_center())  # 点云中心

    visualizer.add_geometry(pcd)

    # 地图坐标原点
    axit_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
    visualizer.add_geometry(axit_pcd)

    vehicle_objects = result['vehicles']
    for vehicle in vehicle_objects.items():
        # print(vehicle[1], type(vehicle[1]))  # vehicle是一个元组
        angle_list = vehicle[1]['angle']  # roll, yaw, pitch
        center_list = vehicle[1]['center']  # 从边界框中心到车前轴中心的相对位置
        extent_list = vehicle[1]['extent']
        location_list = vehicle[1]['location']  # 车前轴中心在地图中的坐标
        # 半长、半宽、半高
        half_l, half_w, half_h = extent_list
        # 中心位于地图坐标原点的框的顶点
        a_points = np.array([[-half_l, -half_w, -half_h], [-half_l, -half_w, half_h],
                             [-half_l, half_w, half_h], [-half_l, half_w, -half_h],
                             [half_l, -half_w, -half_h], [half_l, -half_w, half_h],
                             [half_l, half_w, half_h], [half_l, half_w, -half_h]])
        # 框的中心点
        center_point = (np.asarray(location_list) +
                        np.asarray(center_list))
        # print(center_point)

        angle = np.array([angle_list[0], angle_list[2], angle_list[1]])  # roll, pitch, yaw
        angle = (angle / 180) * np.pi
        rotate_matrix = euler_angle_2_rotation_matrix(angle)  # 旋转矩阵

        points_obj = a_points + center_point  # 平移整个框

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
        line_set.rotate(rotate_matrix, center=center_point)  # 旋转边界框
        visualizer.add_geometry(line_set)

    # 画裁剪范围
    # 半长、半宽、半高
    lidar = np.array([lidar_location[0], lidar_location[1], 0])
    half_l, half_w, half_h = 140, 60, 3
    a, b, c, d = np.array([half_l, half_w, -half_h]) + lidar, \
                 np.array([half_l, -half_w, -half_h]) + lidar, \
                 np.array([-half_l, -half_w, -half_h]) + lidar, \
                 np.array([-half_l, half_w, -half_h]) + lidar
    e, f, g, h = np.array([half_l, half_w, half_h]) + lidar, \
                 np.array([half_l, -half_w, half_h]) + lidar, \
                 np.array([-half_l, -half_w, half_h]) + lidar, \
                 np.array([-half_l, half_w, half_h]) + lidar
    ploy_points = np.array([g, c,
                            d, h,
                            f, b,
                            a, e])
    box_lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                          [4, 5], [5, 6], [6, 7], [7, 4],
                          [0, 4], [1, 5], [2, 6], [3, 7]])
    colors = np.array([[0, 1, 0] for k in range(12)])
    line_set = open3d.geometry.LineSet()
    line_set.lines = open3d.utility.Vector2iVector(box_lines)
    line_set.colors = open3d.utility.Vector3dVector(colors)
    line_set.points = open3d.utility.Vector3dVector(ploy_points)
    visualizer.add_geometry(line_set)

    visualizer.run()


def show_openv2v_3d_bbox_v2(pcd_file_name='/home/thu/Downloads/2021_08_23_21_47_19/243/000069'):
    """
    根据对应的配置文件，在点云中绘制识别车辆的3D边界框。将点云数据和地图坐标转换到右手坐标系。
    所有坐标，y轴取反，其它轴保持不变。所有姿态，翻滚roll=-roll，倾斜pitch=pitch，偏航yaw=-yaw。
    :param: 路径和帧名
    :return:
    """

    with open(pcd_file_name+'.yaml', 'r', encoding='utf-8') as f:
        result = yaml.load(f.read(), Loader=yaml.FullLoader)

    visualizer = open3d.visualization.Visualizer()
    visualizer.create_window()
    render_opt = visualizer.get_render_option()
    render_opt.point_size = 1
    render_opt.background_color = np.asarray([1, 1, 1])

    # crt = visualizer.get_view_control()
    # crt.set_front((-1, -1, 1))
    # crt.set_up((0, 0, 1))
    # crt.set_zoom(0.2)

    pcd = open3d.io.read_point_cloud(pcd_file_name+'.pcd',
                                     print_progress=False)
    pcd.paint_uniform_color([0, 0, 1.0])

    lidar_pose_left = result['lidar_pose']  # x y z roll yaw pitch，雷达在地图坐标系中的位置和姿态
    # ego_pose = result['true_ego_pos']  # x y z roll yaw pitch，ego车辆在地图坐标系中的位置和姿态
    # ego车辆的前轴中心在地图中的坐标
    # ego_location = np.array([ego_pose[0], -ego_pose[1], ego_pose[2]])
    # ego_rotation = np.array([-ego_pose[3], ego_pose[5], -ego_pose[4]])  # roll, pitch, yaw
    # ego车辆的雷达在地图中的坐标，注意，该坐标对应点云坐标系的原点。转换到右手坐标系
    lidar_location = np.array([lidar_pose_left[0], -lidar_pose_left[1], lidar_pose_left[2]])
    lidar_rotation = np.array([-lidar_pose_left[3], lidar_pose_left[5], -lidar_pose_left[4]])  # roll, pitch, yaw
    lidar_rotation = (lidar_rotation / 180) * np.pi
    lidar_rotate_matrix = euler_angle_2_rotation_matrix(-lidar_rotation)  # 点云旋转矩阵，反方向旋转

    right_hand = np.array([[1, 0, 0],
                           [0, -1, 0],
                           [0, 0, 1]])
    pcd.rotate(right_hand, center=[0, 0, 0])  # 旋转点云到右手坐标系，y轴取反

    # pcd.translate(lidar_location, relative=True)  # 移动点云，但点云中心点不移动
    # pcd.rotate(pcd_rotate_matrix, center=[0, 0, 0])  # 旋转点云与地图坐标系对齐
    # print(pcd.get_center())  # 点云中心

    visualizer.add_geometry(pcd)

    # 地图坐标原点
    axit_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
    visualizer.add_geometry(axit_pcd)

    vehicle_objects = result['vehicles']
    for vehicle in vehicle_objects.items():
        # print(vehicle[1], type(vehicle[1]))  # vehicle是一个元组
        angle_list = vehicle[1]['angle']  # roll, yaw, pitch，左手坐标系
        center_list = vehicle[1]['center']  # 从边界框中心到车前轴中心的相对位置，左手坐标系
        extent_list = vehicle[1]['extent']
        obj_location_list = vehicle[1]['location']  # 车前轴中心在地图中的坐标，左手坐标系
        # 半长、半宽、半高
        half_l, half_w, half_h = extent_list
        # 中心位于地图坐标原点的框的顶点
        a_points = np.array([[-half_l, -half_w, -half_h], [-half_l, -half_w, half_h],
                             [-half_l, half_w, half_h], [-half_l, half_w, -half_h],
                             [half_l, -half_w, -half_h], [half_l, -half_w, half_h],
                             [half_l, half_w, half_h], [half_l, half_w, -half_h]])
        # 框的中心点，点云坐标系
        center_point = (np.array([obj_location_list[0], -obj_location_list[1], obj_location_list[2]]) -
                        lidar_location +
                        np.array([center_list[0], -center_list[1], center_list[2]]))
        # print(center_point)

        angle = np.array([-angle_list[0], angle_list[2], -angle_list[1]])  # roll, pitch, yaw,转换到右手坐标系
        angle = (angle / 180) * np.pi
        rotate_matrix = euler_angle_2_rotation_matrix(angle)  # 旋转矩阵

        points_obj = a_points + center_point  # 平移整个框

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
        line_set.rotate(rotate_matrix, center=center_point)  # 以框的中心为旋转中心，旋转边界框
        line_set.rotate(lidar_rotate_matrix, center=[0, 0, 0])  # 以点云的原点为旋转中心，旋转边界框
        visualizer.add_geometry(line_set)

    # 画裁剪范围
    # 半长、半宽、半高
    half_l, half_w, half_h = 140.8, 40, 3
    a, b, c, d = np.array([half_l, half_w, -half_h]), \
                 np.array([half_l, -half_w, -half_h]), \
                 np.array([-half_l, -half_w, -half_h]), \
                 np.array([-half_l, half_w, -half_h])
    e, f, g, h = np.array([half_l, half_w, half_h]), \
                 np.array([half_l, -half_w, half_h]), \
                 np.array([-half_l, -half_w, half_h]), \
                 np.array([-half_l, half_w, half_h])
    ploy_points = np.array([g, c,
                            d, h,
                            f, b,
                            a, e])
    box_lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                          [4, 5], [5, 6], [6, 7], [7, 4],
                          [0, 4], [1, 5], [2, 6], [3, 7]])
    colors = np.array([[0, 1, 0] for k in range(12)])
    line_set = open3d.geometry.LineSet()
    line_set.lines = open3d.utility.Vector2iVector(box_lines)
    line_set.colors = open3d.utility.Vector3dVector(colors)
    line_set.points = open3d.utility.Vector3dVector(ploy_points)
    line_set.rotate(lidar_rotate_matrix, center=[0, 0, 0])
    visualizer.add_geometry(line_set)

    visualizer.run()


def pcd2bin(pcd_dir_path: str, bin_dir_path: str, txt_dir_path: str, output_num: int = 0, frame_num: int = 69):
    pcd_file = pcd_dir_path + '/' + str(frame_num).zfill(6) + '.pcd'
    yaml_file = pcd_dir_path + '/' + str(frame_num).zfill(6) + '.yaml'
    assert os.path.isfile(pcd_file), 'PCD文件不存在：' + pcd_file
    assert os.path.isfile(yaml_file), 'yaml配置文件不存在：' + yaml_file
    assert os.path.exists(bin_dir_path), "路径不存在：" + bin_dir_path
    assert os.path.exists(txt_dir_path), "路径不存在：" + txt_dir_path
    pcd = open3d.io.read_point_cloud(pcd_file)
    right_hand = np.array([[1, 0, 0],
                           [0, -1, 0],
                           [0, 0, 1]])
    pcd.rotate(right_hand, center=[0, 0, 0])  # 旋转点云到右手坐标系，y轴取反

    with open(yaml_file, 'r', encoding='utf-8') as f:
        result = yaml.load(f.read(), Loader=yaml.FullLoader)
        f.close()
    lidar_pose = result['lidar_pose']  # x y z roll yaw pitch，雷达（即ego车辆）在地图坐标系中的位置和姿态
    ego_pose = result['true_ego_pos']  # x y z roll yaw pitch，ego车辆在地图坐标系中的位置和姿态
    # ego车辆的前轴中心在地图中的坐标
    ego_location = np.array([ego_pose[0], -ego_pose[1], ego_pose[2]])
    ego_rotation = np.array([-ego_pose[3], ego_pose[5], -ego_pose[4]])  # roll, pitch, yaw
    # ego车辆的雷达在地图中的坐标，注意，该坐标对应点云坐标系的原点
    lidar_location = np.array([lidar_pose[0], -lidar_pose[1], lidar_pose[2]])
    lidar_rotation = np.array([-lidar_pose[3], lidar_pose[5], -lidar_pose[4]])  # roll, pitch, yaw
    lidar_rotation = (lidar_rotation / 180) * np.pi
    lidar_rotate_matrix = euler_angle_2_rotation_matrix(-lidar_rotation)  # 点云旋转矩阵
    # 不移动点云
    # pcd.translate(lidar_location, relative=True)  # 移动点云，但点云中心点不移动
    # 不旋转点云
    # pcd.rotate(pcd_rotate_matrix, center=lidar_location)  # 旋转点云与地图坐标系对齐

    # 裁剪点云数据
    vol = open3d.visualization.SelectionPolygonVolume()
    vol.orthogonal_axis = 'z'
    vol.axis_max = 1
    vol.axis_min = -3
    bounding_ploy = np.array([[140.8, 140.8, 0],  # x-axis和y-axis区域
                              [140.8, -140.8, 0],
                              [-140.8, -140.8, 0],
                              [-140.8, 140.8, 0]])
    bounding_ploy_pcd = open3d.geometry.PointCloud()
    bounding_ploy_pcd.points = open3d.utility.Vector3dVector(bounding_ploy)
    bounding_ploy_pcd.rotate(lidar_rotate_matrix, center=[0, 0, 0])
    vol.bounding_polygon = bounding_ploy_pcd.points
    pcd = vol.crop_point_cloud(pcd)

    points = np.asarray(pcd.points)  # 点云数据，不包含强度信息
    intensity = np.asarray(pcd.colors)[:, 0:1]
    pcd_array = np.concatenate((points, intensity), axis=1)
    pcd_array = pcd_array.astype(np.float32)

    bin_file = bin_dir_path + '/' + str(output_num).zfill(4) + str(frame_num).zfill(6) + '.bin'
    pcd_array.tofile(bin_file)

    # 测试
    # bin_file_array = np.fromfile(bin_file, dtype=np.float32)
    # bin_file_array = bin_file_array.reshape(-1, 4)
    # print(bin_file_array.shape)
    # point_cloud = open3d.geometry.PointCloud()
    # point_cloud.points = open3d.utility.Vector3dVector(bin_file_array[:, 0:3])
    # open3d.visualization.draw_geometries([point_cloud])

    # .yaml to .txt
    txt_file = txt_dir_path + '/' + str(output_num).zfill(4) + str(frame_num).zfill(6) + '.txt'
    with open(txt_file, 'w', encoding='utf-8') as f:
        vehicle_objects = result['vehicles']
        for vehicle in vehicle_objects.items():
            f.write('Car 0.00 0.00 0.00 0.00 0.00 0.00 0.00 ')
            # print(vehicle[1], type(vehicle[1]))  # vehicle是一个元组
            angle_list = vehicle[1]['angle']  # roll, yaw, pitch ，以地图坐标系为参考
            center_list = vehicle[1]['center']  # 从边界框中心到车前轴中心的相对位置
            extent_list = vehicle[1]['extent']
            obj_location_list = vehicle[1]['location']  # 车前轴中心在地图中的坐标
            obj_location = np.array([obj_location_list[0], -obj_location_list[1], obj_location_list[2]])
            # 长、宽、高
            l, w, h = (v*2 for v in extent_list)
            f.write(f'{h:.2f}' + ' ' + f'{w:.2f}' + ' ' + f'{l:.2f}' + ' ')
            # 框的中心点，减去ego的地图坐标，使得框中心点以点云坐标系为参考
            center_point = (obj_location - lidar_location +
                            np.array([center_list[0], -center_list[1], center_list[2]]))
            # 坐标向量反方向旋转，旋转角度是雷达的yaw
            box_rotate_matrix = euler_angle_2_rotation_matrix(-np.array([0, 0, lidar_rotation[2]]))
            center_point = np.dot(box_rotate_matrix, center_point)
            f.write(f'{center_point[0]:.2f}' + ' ' + f'{center_point[1]:.2f}' + ' ' + f'{center_point[2]:.2f}' + ' ')

            # obj在的方位yaw
            yaw = -(angle_list[1] + ego_rotation[2]) / 180 * np.pi
            f.write(f'{yaw:.2f}')
            f.write('\n')
        f.close()


def euler_angle_2_rotation_matrix(theta):
    """
    欧拉角转旋转矩阵
    :param theta: roll, pitch, yaw 弧度单位
    :return:
    """
    r_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]])
    r_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])
    r_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]])
    return np.dot(r_z, np.dot(r_y, r_x))


def openv2v_2_kitti(openv2v_dir_path: str, kitti_dir_path: str, output_num: int = 0):
    """
    OpenV2V数据集格式转KITTI数据集格式
    :param output_num: 0-9999,KITTI数据集文件名唯一标记，为了使输出的文件文件名不重复，建议对该值累加
    :param openv2v_dir_path: 该路径下应该包含pcd点云文件和对应的yaml配置文件
    :param kitti_dir_path: 该路径下应该包含lidar和label文件夹，否则会自动创建这两个文件夹。lidar保存bin文件，label保存txt文件
    :return:
    """
    frame_start_num = 69
    frame_end_num = 221

    assert os.path.exists(openv2v_dir_path), "路径不存在：" + openv2v_dir_path
    assert os.path.exists(kitti_dir_path), "路径不存在：" + kitti_dir_path
    if not os.path.exists(kitti_dir_path + '/lidar/'):
        print('创建文件夹：' + kitti_dir_path + '/lidar')
        os.mkdir(kitti_dir_path + '/lidar/')
    if not os.path.exists(kitti_dir_path + '/label'):
        print('创建文件夹：' + kitti_dir_path + '/label')
        os.mkdir(kitti_dir_path + '/label/')
    bin_dir_path = kitti_dir_path + '/lidar'
    txt_dir_path = kitti_dir_path + '/label'

    frame_num = frame_start_num
    for _ in range(int((frame_end_num-frame_start_num)/2)):
        frame_num = frame_num + 2
        pcd2bin(openv2v_dir_path, bin_dir_path, txt_dir_path, output_num=output_num, frame_num=frame_num)


def t_kitti(bin_file: str, txt_file: str):
    assert os.path.isfile(bin_file), 'bin文件不存在：' + bin_file
    assert os.path.isfile(txt_file), 'txt文件不存在：' + txt_file

    visualizer = open3d.visualization.Visualizer()
    visualizer.create_window()
    render_opt = visualizer.get_render_option()
    render_opt.point_size = 1
    render_opt.background_color = np.asarray([1, 1, 1])

    # with np.fromfile(bin_file, dtype=np.float32) as pcd_array:
    pcd_array = np.fromfile(bin_file, dtype=np.float32)
    pcd_array = pcd_array.reshape(-1, 4)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(pcd_array[:, 0:3])
    visualizer.add_geometry(pcd)

    # cars_str = []
    with open(txt_file, encoding='utf-8') as f:
        for line in f:
            # cars_str.append((line.split()))
            car = line.split()

            # 半长、半宽、半高
            half_l, half_w, half_h = float(car[10])/2, float(car[9])/2, float(car[8])/2,
            # 中心位于地图坐标原点的框的顶点
            a_points = np.array([[-half_l, -half_w, -half_h], [-half_l, -half_w, half_h],
                                 [-half_l, half_w, half_h], [-half_l, half_w, -half_h],
                                 [half_l, -half_w, -half_h], [half_l, -half_w, half_h],
                                 [half_l, half_w, half_h], [half_l, half_w, -half_h]])
            # 框的中心点，点云坐标系
            center_point = (np.array([float(car[11]), float(car[12]), float(car[13])]))
            # print(center_point)

            angle = np.array([0, 0, float(car[14])])  # roll, pitch, yaw,转换到右手坐标系
            rotate_matrix = euler_angle_2_rotation_matrix(angle)  # 旋转矩阵

            points_obj = a_points + center_point  # 平移整个框

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
            line_set.rotate(rotate_matrix, center=center_point)  # 以框的中心为旋转中心，旋转边界框
            visualizer.add_geometry(line_set)

    visualizer.run()


if __name__ == '__main__':
    # openv2v_2_kitti('/home/thu/Downloads/2021_08_23_21_47_19/225',
    #                 '/home/thu/Downloads/openv2v_2_kitti/train',
    #                 output_num=0)
    # show_openv2v_3d_bbox_v2(pcd_file_name='/home/thu/Downloads/2021_08_23_21_47_19/243/000131')
    t_kitti('/home/thu/Downloads/openv2v_2_kitti/train/lidar/0000000151.bin',
            '/home/thu/Downloads/openv2v_2_kitti/train/label/0000000151.txt')

