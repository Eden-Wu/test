import numpy as np
import transformations as tf
import calculating_C5
import math

# 计算旋转矩阵
def rot_mat(angle, vector):
    # 定义旋转角度和旋转轴
    theta = np.deg2rad(angle)
    axis = np.array(vector)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    rot_mat = np.array([[cos_theta + (1 - cos_theta) * axis[0] ** 2,
                         (1 - cos_theta) * axis[0] * axis[1] - sin_theta * axis[2],
                         (1 - cos_theta) * axis[0] * axis[2] + sin_theta * axis[1],
                         0],
                        [(1 - cos_theta) * axis[0] * axis[1] + sin_theta * axis[2],
                         cos_theta + (1 - cos_theta) * axis[1] ** 2,
                         (1 - cos_theta) * axis[1] * axis[2] - sin_theta * axis[0],
                         0],
                        [(1 - cos_theta) * axis[0] * axis[2] - sin_theta * axis[1],
                         (1 - cos_theta) * axis[1] * axis[2] + sin_theta * axis[0],
                         cos_theta + (1 - cos_theta) * axis[2] ** 2,
                         0],
                        [0, 0, 0, 1]])
    return rot_mat

# 计算4,6 2,84个点的位姿
def get_O_CX_pose(angle,vector):
    global O_B_matrics
    global O_C_matrics
    # 从urbasic 那里得到的tcp位姿
    x, y, z, rx, ry, rz = C5
    O_C_matrics = tf.euler_matrix(rx, ry, rz, 'sxyz')
    O_C_matrics[:3, 3] = [x, y, z]
    # 计算B_C的4*4矩阵
    x3, y3, z3, rx3, ry3, rz3 = 0, 0, -0.1, 0, 0, 0
    B_C_matrics = tf.euler_matrix(rx3, ry3, rz3, 'sxyz')
    B_C_matrics[:3, 3] = [x3, y3, z3]
    # print(f'B_C_matrics\n:{B_C_matrics}')
    C_B_matrics = np.linalg.inv(B_C_matrics)
    # print(f'C_B_matrics\n:{C_B_matrics}')
    O_B_matrics = np.dot(O_C_matrics, C_B_matrics)
    # print(f'O_B_matrics\n:{O_B_matrics}')
    B_CX_matrics = np.dot(rot_mat(angle, vector),B_C_matrics)   #  ？
    # print(f'B_CX_matrics\n:{B_CX_matrics}')
    O_CX_matrics = np.dot(O_B_matrics, B_CX_matrics)
    # print(f'O_CX_matrics\n{O_CX_matrics}')
    x1,y1,z1 = O_CX_matrics[:3, 3]
    rx1,ry1,rz1 = tf.euler_from_matrix(O_CX_matrics, 'rxyz')
    O_CX_pose1 = [x1,y1,z1,rx1,ry1,rz1]
    # return O_CX_pose1
    return O_CX_pose1

#计算1，3，7，9四个点的位姿
def get_O_CX_pose2(angle1,vector1,angle2,vector2):
    x, y, z, rx, ry, rz = C5
    O_C_matrics = tf.euler_matrix(rx, ry, rz, 'sxyz')
    O_C_matrics[:3, 3] = [x, y, z]
    # 计算B_C的4*4矩阵
    x3, y3, z3, rx3, ry3, rz3 = 0,0, -0.1, 0, 0, 0
    B_C_matrics = tf.euler_matrix(rx3, ry3, rz3, 'sxyz')
    B_C_matrics[:3, 3] = [x3, y3, z3]
    # print(f'B_C_matrics\n:{B_C_matrics}')
    C_B_matrics = np.linalg.inv(B_C_matrics)
    O_B_matrics = np.dot(O_C_matrics, C_B_matrics)
    B_CX_matrics = np.dot(rot_mat(angle1, vector1),B_C_matrics)   #  ？
    B_CX_matrics = np.dot(rot_mat(angle2, vector2),B_CX_matrics)
    O_CX_matrics = np.dot(O_B_matrics, B_CX_matrics)
    x1,y1,z1 = O_CX_matrics[:3, 3]
    rx1,ry1,rz1 = tf.euler_from_matrix(O_CX_matrics, 'rxyz')
    O_CX_pose2 = [x1,y1,z1,rx1,ry1,rz1]
    return O_CX_pose2

def check_max_pose(CP):
    x = CP[0]
    y = CP[1]
    z = CP[2]
    distance = math.sqrt(x**2+y**2+z**2)
    print(distance)
    if distance < 0.49:
        print('True')
    else:
        print('False')

C5= calculating_C5.C5
C4 =get_O_CX_pose(30, [0, 0, 1])
C6 =get_O_CX_pose(-30, [0, 0, 1])
C2 =get_O_CX_pose(30, [0, 1, 0])
C8 = get_O_CX_pose(-30, [0, 1, 0])

C1 =get_O_CX_pose2(30, [0, 1, 0],15,[0, 0, 1])
C3 =get_O_CX_pose2(30, [0, 1, 0],-15,[0, 0, 1])
C7 =get_O_CX_pose2(-30, [0, 1, 0],15,[0, 0, 1])
C9 =get_O_CX_pose2(-30, [0, 1, 0],-15,[0, 0, 1])


C = (C4,C1,C3,C9,C7,C1,C9,C7,C3)
C_N = ['C4','C1','C3','C9','C7','C1','C9','C7','C3']
print(C_N[0])

tmp = 0
for i in C:
    print(C_N[tmp])

    tmp+=1
    check_max_pose(i)


# if __name__ == '__main__':
#     pass
