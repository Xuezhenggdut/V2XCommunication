import open3d
import numpy as np


visualizer = open3d.visualization.Visualizer()
visualizer.create_window()

render_opt = visualizer.get_render_option()
render_opt.point_size = 1
render_opt.background_color = np.asarray([0, 0, 0])
# render_opt.show_coordinate_frame = True

crt = visualizer.get_view_control()
# crt.set_lookat(np.array([0, 0, 0]))
# crt.set_up((0, -1, 0))
# crt.set_front((-1, 0, 0))

pcd = open3d.io.read_point_cloud('/home/thu/Downloads/2021_08_23_21_47_19/225/000069.pcd',
                                 print_progress=False)
visualizer.add_geometry(pcd)

for i in range(70):
    num = 71+2*i
    if num > 99:
        pcd = open3d.io.read_point_cloud('/home/thu/Downloads/2021_08_23_21_47_19/225/000%d.pcd' % num,
                                         print_progress=False)
    else:
        pcd = open3d.io.read_point_cloud('/home/thu/Downloads/2021_08_23_21_47_19/225/0000%d.pcd' % num,
                                         print_progress=False)
    visualizer.clear_geometries()
    visualizer.add_geometry(pcd)
    # crt.set_front((-1, -1, 1))  # 斜视角
    # crt.set_up((0, 0, 1))
    # crt.set_zoom(0.2)
    crt.set_up((1, 0, 1))  # 设置垂直指向屏幕上方的向量
    crt.set_front((-1, 0, 1))  # 设置垂直指向屏幕外的向量
    crt.set_zoom(0.2)  # 设置视角放大比例
    visualizer.poll_events()
    visualizer.update_renderer()
# visualizer.destroy_window()
visualizer.run()
