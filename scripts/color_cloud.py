import open3d as o3d
import numpy as np
import warnings
import cv2 
import os
from scipy.spatial.transform import Rotation as R
warnings.filterwarnings("ignore",category=RuntimeWarning)

def load_pcd(pointcloud_file):
    pcd_origin = o3d.io.read_point_cloud(pointcloud_file)
    pcd = o3d.geometry.PointCloud()
    points = np.asarray(pcd_origin.points)
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def get_R_and_T(Trans_matrix):
    if Trans_matrix.shape[0]!=4:
        print("False")
    R = np.zeros((3,3))
    T = np.zeros((3,1))
    R = Trans_matrix[0:3,0:3]
    T = Trans_matrix[0:3,3:4]
    rvec = cv2.Rodrigues(R)[0]
    rvec = np.float64([rvec[0,0],rvec[1,0], rvec[2,0]])
    tvec = np.float64([T[0,0],T[1,0], T[2,0]])
    return R,rvec, tvec


def get_inverse_T(Trans_origin):
    R = Trans_origin[:3, :3]
    T = Trans_origin[:3, 3]
    R_transpose = R.T
    T_inv = -np.dot(R_transpose, T)
    Trans_inverse = np.eye(4)
    Trans_inverse[:3, :3] = R_transpose
    Trans_inverse[:3, 3] = T_inv
    return Trans_inverse


def colorCloud(pcd, img_file, rvec, tvec, camera_intrinsic, distCoeffs,point_colors_bgr):
    image = cv2.imread(img_file) # load the image
    points = np.asarray(pcd.points)
    num_points = points.shape[0]
    point_matrix = np.ones((num_points, 4))
    point_matrix[:, :3] = points
    point_2d, _ = cv2.projectPoints(points, rvec, tvec, camera_intrinsic, distCoeffs)
    point_pixel_coords = point_2d.astype(int)
    
    distance = []
    for point in points:
        dis = np.sqrt(np.power(point[0],2)+np.power(point[1],2)+np.power(point[2],2))
        distance.append(dis)

    result_image = np.copy(image)
    i = 0
    repeat_matrix = np.empty((result_image.shape[0],result_image.shape[1]),dtype=object)
    for i in range(result_image.shape[0]):
        for j in range(result_image.shape[1]):
            repeat_matrix[i][j] = []
    i = 0
    repeat_index = []
    for point in point_pixel_coords:
        x = point[0][0]
        y = point[0][1]
        if 0 <= y < result_image.shape[0] and 0 <= x < result_image.shape[1] and point_matrix[i,0]>=0 and point_matrix[i,2]>=0 and distance[i]<=25:
            # if [y,x] not in repeat_index:
            #     repeat_index.append([y,x])
            #     repeat_matrix[y,x].append(i)
            # else:
            #     if distance[repeat_matrix[y,x][0]]>distance[i]:
            #         repeat_matrix[y,x][0] = i
            #     # print("repeat",[y,x])
            #     # print(repeat_matrix[y,x])
            #     pass
            if sum(point_colors_bgr[i]) <= 3:
                point_colors_bgr[i] = image[y, x]
        i += 1
    # for y in range(result_image.shape[0]):
    #     for x in range(result_image.shape[1]):
    #         if repeat_matrix[y][x]:
    #            point_colors_bgr[repeat_matrix[y][x][0]] = image[y, x]
        
    # for y in range(result_image.shape[0]):
    #     for x in range(result_image.shape[1]):
    #         if repeat_matrix[y][x]:
    #             index_min = repeat_matrix[y][x][0]
    #             if len(repeat_matrix[y][x])-1:
    #                 distance_min = 1000
    #                 for index in repeat_matrix[y][x]:
    #                     if distance[index]<distance_min:
    #                         distance_min = distance[index]
    #                         index_min = index
    #             point_colors_bgr[index_min] = image[y, x]
                
    

    return point_colors_bgr


def color_pic(pcd, img_file, rvec, tvec, camera_intrinsic, distCoeffs):
    image = cv2.imread(img_file) # load the image
    points = np.asarray(pcd.points)
    num_points = points.shape[0]
    point_matrix = np.ones((num_points, 4))
    point_matrix[:, :3] = points  
    point_2d, _ = cv2.projectPoints(points, rvec, tvec, camera_intrinsic, distCoeffs)
    point_pixel_coords = point_2d.astype(int)

    result_image = np.copy(image)
    point_colors_bgr = np.zeros((point_pixel_coords.shape[0], 3), dtype=np.uint8)
    i = 0
    for point in point_pixel_coords:
        x = point[0][0]
        y = point[0][1]
        if 0 <= y < result_image.shape[0] and 0 <= x < result_image.shape[1] and point_matrix[i,0]>=0:
            result_image[y, x] = [255, 0, 0]
        i += 1
    return result_image


def key_frame_extract(key_frame_file):
    rotation_matrixs = []
    T = []
    Transformations = []
    Times = []
    with open(key_frame_file, 'r' ) as file:
        lines = file.readlines()
    for line in lines:
        elements = line.strip().split()
        Time = float(elements[0])
        q = np.array([elements[1], elements[2], elements[3], elements[4]])
        t = np.array([[elements[5]], [elements[6]], [elements[7]]]).astype(float)
        RT = R.from_quat(q)
        RT = RT.as_matrix()
        Transformation = np.zeros((4,4))
        Transformation[0:3,0:3] = RT
        Transformation[:,3:4] = [[float(elements[5])], [float(elements[6])], [float(elements[7])],[1.0]]
        rotation_matrixs.append(RT)
        T.append(t)
        Transformations.append(Transformation)
        Times.append(Time)
    return Transformations,Times

def get_img_list(Img_file):
    all_objects = os.listdir(Img_file)
    img_list = [file for file in all_objects if file.lower().endswith(('.jpg', '.jpeg', '.png', '.gif'))]
    img_name = []
    for img in img_list:
        name = os.path.splitext(img)[0]
        name = float(name)
        img_name.append(name)
    return img_list,img_name

def get_img_name(img_list,img_name,stamp):
    i=0
    count = 0
    former_delta = img_name[0]+stamp
    for name in img_name:
        delta = abs(name - stamp)
        if delta < former_delta:
            former_delta = delta
            count = i
        i += 1
    return img_list[count]


if __name__ == "__main__":
    pointcloud_file = 'F:/data/calibration/calibration_in/1.pcd'
    img_file = 'F:/data/calibration/1.jpg'
    Img_path = 'F:/data/0814/pic_imr/pictures'
    
    img_list,img_name = get_img_list(Img_path)
    
    filename = Img_path+'/'+get_img_name(img_list,img_name,1691140902.9809)
    print(filename)
    

    # camera_intrinsic = np.float64([
    #     [918.6080, 0.0, 631.37719],
    #     [0.0, 915.973022, 356.666],
    #     [0.0, 0.0, 1.0]
    # ])
    # distCoeffs = np.float64([0.0, 0.0, 0.0, 0.0, 0.0])

    # RT = np.float64([
    #     [-0.346889,-0.937895,0.00461545],
    #     [0.251673,-0.0978218,-0.962856],
    #     [0.903509,-0.332842,0.269976],
    # ])
    # tvec = np.float64([0.180297, 0.240911, -0.0592223])

    # rvec = cv2.Rodrigues(RT)[0]
    # rvec = np.float64([rvec[0,0],rvec[1,0], rvec[2,0]])
    # pcd = colorCloud(pointcloud_file,img_file,rvec, tvec, camera_intrinsic, distCoeffs)
    # o3d.visualization.draw_geometries([pcd])