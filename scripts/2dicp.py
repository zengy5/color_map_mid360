import numpy as np
import math


# 不以科学计数显示
np.set_printoptions(suppress=True)


def GetDistOfPoints2D(p1, p2):
    '''
    求出两个点的距离
    '''
    return math.sqrt(math.pow(p2[0] - p1[0], 2) + math.pow(p2[1] - p1[1], 2))


def GetClosestID(p, p_set):
    '''
    在p_set点集中找到距p最近点的id
    '''
    id = 0
    min = float("inf")  # 初始化取最大值
    for i in range(p_set.shape[1]):
        dist = GetDistOfPoints2D(p, p_set[:, i])
        if dist < min:
            id = i
            min = dist
    return id


def GetDistOf2DPointsSet(set1, set2):
    '''
    求两个点集之间的平均点距
    '''
    loss = 0
    for i in range(set1.shape[1]):
        id = GetClosestID(set1[:, i], set2)
        dist = GetDistOfPoints2D(set1[:, i], set2[:, id])
        loss += dist
    return loss/set1.shape[1]


def ICP_2D(targetPoints, sourcePoints):
    '''
    二维 ICP 配准算法
    '''
    A = targetPoints  # A是目标点云（地图值）
    B = sourcePoints  # B是源点云（感知值）

    # 初始化迭代参数
    iteration_times = 0  # 迭代次数
    dist_now = 1  # A,B两点集之间初始化距离
    dist_improve = 1  # A,B两点集之间初始化距离提升
    dist_before = GetDistOf2DPointsSet(A, B)  # A,B两点集之间距离

    # 初始化 R，T
    R = np.identity(2)
    T = np.zeros((2, 1))

    # 均一化点云
    A_mean = np.mean(A, axis=1).reshape((2, 1))
    A_ = A - A_mean

    # 迭代次数小于10并且距离大于0.01时，继续迭代
    while iteration_times < 10 and dist_now > 0.01:

        # 均一化点云
        B_mean = np.mean(B, axis=1).reshape((2, 1))
        B_ = B - B_mean

        # t_nume表示角度公式中的分子 t_deno表示分母
        t_nume, t_deno = 0, 0

        # 对源点云B中每个点进行循环
        for i in range(B_.shape[1]):
            j = GetClosestID(B_[:, i], A_)  # 找到距离最近的目标点云A点id
            t_nume += A_[1][j] * B_[0][i] - A_[0][j] * B_[1][i]  # 获得求和公式，分子的一项
            t_deno += A_[0][j] * B_[0][i] + A_[1][j] * B_[1][i]  # 获得求和公式，分母的一项

        # 计算旋转弧度θ
        theta = math.atan2(t_nume, t_deno)

        # 由旋转弧度θ得到旋转矩阵Ｒ
        delta_R = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])

        # 计算平移矩阵Ｔ
        delta_T = A_mean - np.matmul(delta_R, B_mean)

        # 更新最新的点云
        B = np.matmul(delta_R, B) + delta_T

        # 更新旋转矩阵Ｒ和平移矩阵Ｔ
        R = np.matmul(delta_R, R)
        T = np.matmul(delta_R, T) + delta_T

        # 更新迭代
        iteration_times += 1  # 迭代次数+1
        dist_now = GetDistOf2DPointsSet(A, B)  # 更新两个点云之间的距离
        dist_improve = dist_before - dist_now  # 计算这一次迭代两个点云之间缩短的距离
        dist_before = dist_now  # 将"现在距离"赋值给"以前距离"

        # 打印迭代次数、损失距离、损失提升
        print("迭代：第{}次，距离：{:.2f}，缩短：{:.2f}".format(iteration_times, dist_now, dist_improve))

    return R, T


# 目标点云（地图上的数据）
a = np.array([[1, 2, 2], [1, 2, 3]])

# 设置转换量
c_theta = math.radians(30)  # 逆时针旋转30° 求解R旋转角度应为顺时针旋转30°
c_r = np.array([[math.cos(c_theta), -math.sin(c_theta)], [math.sin(c_theta), math.cos(c_theta)]])
c_t = np.array([[6], [-0.6]])  # 偏移（6，-0.6） 求解T应为（-6，0.6）

# 源点云（假设检测到的数据）
b = np.matmul(c_r, a + c_t)

# ICP 配准
r, t = ICP_2D(a, b)

# 显示参数
print("A:\n", a)
print("\nB:\n", b)
print("\nR:\n", r)
print("\nT:\n", t)
print("\nNew B:\n", np.matmul(r, b) + t)
input("\n输入任意键退出")