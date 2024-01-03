import open3d as o3d
import numpy as np
import cv2
import color_cloud

visible_path = 'data/result/colored_centroids.pcd'
origin_pcd = 'F:/data/visible_map.pcd'
img_path = 'data/1.png'
img_file = 'data/project.png'
pcd = o3d.io.read_point_cloud(visible_path)

camera_intrinsic = np.float64([
    [918.608, 0.0, 631.377],
    [0.0, 915.973, 356.666],
    [0.0, 0.0, 1.0]
])
distCoeffs = np.float64([0.0, 0.0, 0.0, 0.0, 0.0])

lidar_to_camera_extrinsic = np.float64([
    [ 0.0, -1.0,  0.0, 0.0],
    [ 0.0,  0.0, -1.0, 0.0],
    [ 1.0,  0.0,  0.0, 0.0],
    [ 0, 0, 0, 1]
])

x_seg = np.float64([371.4732028923862, 434.35545722713863, 526.4869325997249, 399.53954268932245, 130.56895193065407, 224.96265761396702, 309.23314285714287, 193.47372262773723, 533.9596321645204, 578.0236220472441, 294.5707019328586, 588.9181494661922, 478.50955414012736, 60.874051593323216, 681.3039499771237, 323.63803680981596, 334.68268434134217, 639.3005780346821, 453.621812150114, 614.2206910596701, 592.0335160727635, 532.9299418121363, 297.5577708383377, 616.0700876095119, 606.2694524495678, 415.44662921348316, 599.2978468899521, 217.12834223186192, 459.5808988764045, 633.8805278174037, 355.45197947214075, 447.90075784680437, 608.2995066948555, 488.2990384615385, 506.66052631578947, 537.5042307342371, 319.34630498162, 599.7222222222222, 269.25219683655536, 211.38333702636828, 631.7748701973001, 146.0434862385321, 588.4094594594594, 469.5, 585.1089139987445, 653.546307151231, 689.3393486385478, 642.9788888888888, 589.4076923076923, 618.0003571428572, 294.2920353982301, 499.65, 226.77833065810594, 696.2281553398059, 649.568710359408, 686.7234322563664, 323.69607843137254, 602.5557620817843, 649.2697368421053, 602.4061123715468, 611.4887372013652, 620.927927927928, 565.457938820102, 54.32295482295482, 195.78352083775562, 604.487980173482, 135.81393534002228, 615.0371674491392, 142.39474338483396, 564.5, 305.5131578947368, 716.5523012552301, 636.3091149273448])
y_seg = np.float64([181.66269672479797, 662.1297935103245, 448.9105914718019, 736.2485840151038, 15.38849487785658, 820.2720659553831, 85.408, 965.6240875912408, 565.0955593759123, 754.6098612673416, 965.4923702950152, 669.2115460656386, 464.4668789808917, 218.00758725341427, 442.0626811041635, 1043.0955302366344, 420.26636288318144, 216.97398843930637, 440.1702260004147, 367.4407711628538, 25.47844505357588, 523.4193516209476, 721.3552726399231, 160.54443053817272, 111.69020172910663, 132.33988764044943, 772.0538277511962, 395.80572852134725, 162.93314606741572, 230.36126961483595, 1198.098973607038, 990.7141909253537, 119.71634954193094, 741.2278846153846, 431.2684210526316, 602.8518484820914, 703.0235771327165, 740.0416666666666, 1219.8580843585237, 985.4298692665633, 1060.1426791277258, 727.8014678899083, 1031.3783783783783, 159.5, 1035.392969240427, 1079.21101992966, 951.6805125467165, 92.67444444444445, 509.8, 320.5628571428571, 1008.3407079646017, 172.52, 932.8747993579454, 227.35061782877318, 899.6215644820296, 878.9613958895396, 223.69607843137254, 132.9182156133829, 859.7302631578947, 77.05402375550514, 443.79419795221844, 178.0930930930931, 445.31218135469777, 1267.95115995116, 1086.7739333474847, 393.705824039653, 989.833110367893, 1058.3016431924882, 798.7431362102647, 444.5, 969.4912280701755, 369.22384937238496, 966.3642085632139])
points = np.asarray(pcd.points)


# 获取点云质心的投影结果并画图检查
result_image = cv2.imread(img_file)
_,rvec,tvec = color_cloud.get_R_and_T(lidar_to_camera_extrinsic)
point_2d, _ = cv2.projectPoints(points, rvec, tvec, camera_intrinsic, distCoeffs)
point_2d = point_2d.astype(int)
print(point_2d)
i = 0
color = [255, 0, 0]
count = 0
cloud_centro = []
for point in point_2d:
    x = point[0][0]
    y = point[0][1]
    cloud_centro.append([x,y])
    if 0 <= y < result_image.shape[0] and 0 <= x < result_image.shape[1] :
        result_image[y, x] = [255, 0, 0]
        result_image = cv2.circle(result_image, (x ,y), 5, color, -1)
        count += 1
    i += 1
cv2.imshow("0", result_image)
cv2.waitKey(0)

# 获取分割结果，并投影检查
if (len(point_2d)==count):print("yes")
if (len(x_seg)==len(y_seg)):print("load seg success")
coords = []
for x0, y0 in zip(x_seg, y_seg):
    coord = [y0, x0]
    coords.append(coord)
    
img = cv2.imread('F:/data/mask.png')
color = [255, 0, 0]
for coord in coords:
    coord_x = int(coord[0])
    coord_y = int(coord[1])
    img =  cv2.circle(img, (coord_x ,coord_y), 5, color, -1)
cv2.imshow("0", img)
cv2.waitKey(0)

# 归一化处理
center = np.mean(coords, axis=0)
# center = np.array([0,0])
centered_points = coords - center

center_cloud = np.mean(cloud_centro, axis=0)
# center_cloud = np.array([0,0])
point_2d_centered = cloud_centro -center_cloud


# 寻找最近点，以点云质心为基础，在分割质心中寻找对应点
closet_points = []
for point in point_2d_centered:
    min_distance = 100000.0
    closet_point = None
    
    for search_point in centered_points:
        point = np.array(point)  
        search_point = np.array(search_point)
        distance = np.linalg.norm(point - search_point)
        
        if distance < min_distance:
            min_distance = distance
            closet_point = search_point
    
    closet_points.append(closet_point)   

# 找到所有对应的分割质心，记得从归一化中恢复，并在点云质心投影结果中，检查
closet_points = closet_points + center
color = [0, 255, 0]
color_line = [0, 0, 255]
for closet_point, origin_point in zip(closet_points,cloud_centro):
    x_piexl = int(closet_point[0])
    y_piexl = int(closet_point[1])
    result_image = cv2.circle(result_image, [x_piexl,y_piexl], 5, color, -1)
    result_image = cv2.line(result_image, [x_piexl,y_piexl], origin_point, color_line, 2 )
cv2.imshow("0",result_image)
cv2.waitKey(0)
if len(closet_points) == len(points):print('ok')

# PNP求解，并用结果投影检查
success, rotation_vector, translation_vector = cv2.solvePnP(points, closet_points, camera_intrinsic, distCoeffs)

pcd = o3d.io.read_point_cloud(origin_pcd)
points = np.asarray(pcd.points)
point_2d, _ = cv2.projectPoints(points, rotation_vector, translation_vector, camera_intrinsic, distCoeffs)
# point_2d, _ = cv2.projectPoints(points, rvec, tvec, camera_intrinsic, distCoeffs)
point_2d = point_2d.astype(int)
i = 0
color = [255, 0, 0]
count = 0
result_image = cv2.imread(img_path)
for point in point_2d:
    x = point[0][0]
    y = point[0][1]
    if 0 <= y < result_image.shape[0] and 0 <= x < result_image.shape[1] :
        result_image[y, x] = [255, 0, 0]
        result_image = cv2.circle(result_image, [x,y], 5, color, -1)
        count += 1
    i += 1
cv2.imshow("0",result_image)
cv2.waitKey(0)