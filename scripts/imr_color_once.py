'''
注释？
'''
import open3d as o3d
import numpy as np
import cv2 
import color_cloud
import copy

# Init the file name
pointcloud_file = 'data/3.pcd'
img_file = 'data/1.png'
key_frame = 'data/key_frame.txt'

# Init the parameters
camera_intrinsic = np.float64([
    [911.362, 0.0, 652.81],
    [0.0, 911.002, 364.706],
    [0.0, 0.0, 1.0]
])
distCoeffs = np.float64([0.0, 0.0, 0.0, 0.0, 0.0])

lidar_to_camera_extrinsic = np.float64([
    [-0.3187587, -0.9473149, -0.0314238,-0.0132929],
    [ 0.3149248, -0.0745818, -0.9461818,-0.538881],
    [ 0.8939884, -0.3114998,  0.3221065,-0.0140522],
    [ 0, 0, 0, 1]
])

lidar_to_camera_extrinsic = np.float64([
    [-0.314745, -0.948059, -0.0460449, -0.00389479],
    [ 0.259136, -0.0391605, -0.965047, -0.25794],
    [ 0.913117, -0.315676,  0.258001, -0.0854201],
    [ 0, 0, 0, 1]
])
a = np.linalg.inv(lidar_to_camera_extrinsic)
print(a)
if __name__ == "__main__":
    # Load the files with given parameters
    pcd = color_cloud.load_pcd(pointcloud_file)
    points = np.asarray(pcd.points)
    num_points = points.shape[0]
    point_colors_bgr = np.ones((num_points, 3), dtype=np.uint8)
    Trans, Stamps= color_cloud.key_frame_extract(key_frame)
    _,rvec,tvec = color_cloud.get_R_and_T(lidar_to_camera_extrinsic)

    # Load the pose info to matrix for loop
    
    # pcd = o3d.io.read_point_cloud(pointcloud_file)
    # pose = Trans[100]
    # print(Stamps[100])
    # pose_inverse = color_cloud.get_inverse_T(pose)
    # pcd.transform(pose_inverse)
    
    # Choose function
    if_pic = 0

    # Project pcd onto the image, project only once
    if if_pic:
        result = color_cloud.color_pic(pcd,img_file,rvec, tvec, camera_intrinsic, distCoeffs)
        cv2.imshow('Projected Points', result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    # Transform the color matrix from bgr to rgb so that pcd can be viewed normally
    else:
        point_colors_bgr= color_cloud.colorCloud(pcd,img_file,rvec, tvec, camera_intrinsic, distCoeffs,point_colors_bgr)
        point_colors_rgb = point_colors_bgr[:, [2, 1, 0]]
        point_colors_rgb = point_colors_rgb / 255
        # # pcd.transform(pose)
        pcd.colors = o3d.utility.Vector3dVector(point_colors_rgb)
        FOR1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=15, origin=[0, 0, 0])
        pcd2 = color_cloud.load_pcd("F:/data/mid360/2.pcd")
        o3d.visualization.draw_geometries([pcd])
        
    o3d.io.write_point_cloud("data/result/copy_of_fragment.pcd", pcd)
    
    # # View coordinate of pointcloud
    # FOR1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=15, origin=[0, 0, 0])
    # pcd_T = copy.deepcopy(pcd)
    # pcd_T.transform(pose_inverse)
    # o3d.visualization.draw_geometries([FOR1,pcd_T], window_name="常规变换",
    #                           width=800,  # 窗口宽度
    #                           height=600)