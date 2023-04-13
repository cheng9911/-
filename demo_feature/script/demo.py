import numpy as np
from scipy.spatial.transform import Rotation as Ro
import math


def rot2T(pose):
    r = Ro.from_rotvec([pose[3], pose[4], pose[5]])
    rotation = r.as_matrix()
    translation = np.array([pose[0], pose[1], pose[2]])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def T2rot(T):
    end_rotation = T[0:3, 0:3]
    end_translation = T[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    rv = r.as_rotvec()
    rv_end = [end_translation[0], end_translation[1], end_translation[2], rv[0], rv[1], rv[2]]
    return rv_end


def T2rot_2(T):
    end_rotation = T[0:3, 0:3]
    end_translation = T[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    rot = r.as_rotvec()
    return end_translation, rot


def euler2T(pose):
    r = Ro.from_euler('XYZ', [pose[3], pose[4], pose[5]])
    rotation = r.as_matrix()
    translation = np.array([pose[0], pose[1], pose[2]])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def T2euler(T):
    end_rotation = T[0:3, 0:3]
    end_translation = T[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    rv = r.as_euler('XYZ')
    rv_end = [end_translation[0], end_translation[1], end_translation[2], rv[0], rv[1], rv[2]]
    return rv_end


def T2euler_2(matrix):
    end_rotation = matrix[0:3, 0:3]
    end_translation = matrix[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    end_rotation = r.as_euler('XYZ')
    return end_translation, end_rotation


def quat2T(a, b, c, d, e, f, g):
    r = Ro.from_quat([d, e, f, g])
    rotation = r.as_matrix()
    translation = np.array([a, b, c])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def T2quat(matrix):
    end_rotation = matrix[0:3, 0:3]
    end_translation = matrix[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    end_rotation = r.as_quat()
    qvec_nvm = np.array(end_rotation)
    return end_translation, qvec_nvm


def quat2euler(data):
    a = Ro.from_quat(data)
    rot = a.as_euler('ZYX')
    return rot


def euler2quat(data):
    a = Ro.from_euler('ZYX', data)
    quart = a.as_quat()
    return quart


def euler2rot(rotation):
    a = Ro.from_euler('ZYX', rotation)
    rot = a.as_rotvec()
    return rot


def send_matrix(tr_matrix, ro_matrix):
    str_my = ""
    for x in tr_matrix:
        str_my += str('%.4f' % x) + " "
    for y in ro_matrix:
        str_my += str('%.4f' % y) + " "
    # str_my += "end"
    str_my.encode('utf-8')
    return str_my

def rpy2T(pose):
    r = Ro.from_euler('xyz', [pose[3], pose[4], pose[5]])
    rotation = r.as_matrix()
    translation = np.array([pose[0], pose[1], pose[2]])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def T2rpy(T):
    end_rotation = T[0:3, 0:3]
    end_translation = T[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    rv = r.as_euler('xyz')
    rv_end = [end_translation[0], end_translation[1], end_translation[2], rv[0], rv[1], rv[2]]
    return rv_end


def T2rpy_2(matrix):
    end_rotation = matrix[0:3, 0:3]
    end_translation = matrix[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    end_rotation = r.as_euler('xyz')
    return end_translation, end_rotation

def get_T(pose1, pose2):
    """
    获得过渡矩阵
    pose1:视觉定位点位姿
    pose2:目标点位姿
    返回 过渡矩阵T
    """
    pose1_T = rpy2T(pose1)  # 视觉定位点位姿
    pose2_T = rpy2T(pose2)  # 目标点位姿
    offset = np.dot(np.linalg.inv(pose1_T), pose2_T)
    # transition=T.T2rot(transition_T)
    return offset
tcp_pose_1=[-0.31518831849098206, -0.15450476109981537, 0.6491077542304993, 1.5690081119537354, -0.024433109909296036, -1.6696906089782715]
tcp_pose_2= [-0.36855289340019226, -0.05543673783540726, 0.6340771317481995, 1.5683705806732178, 0.003705112263560295, -1.5785548686981201]
T_pose1_base=rpy2T(tcp_pose_1)
T_pose2_base=rpy2T(tcp_pose_2)
T_camera_eelink= euler2T([0.02, -0.09, 0.03, 0, 0, math.pi])
T_camera1_base=np.dot(T_pose1_base,T_camera_eelink)
T_camera2_base=np.dot(T_pose2_base,T_camera_eelink)
T_camera2_camera1=np.dot(np.linalg.inv(T_camera1_base),T_camera2_base)
print("T_camera2_camera1",T_camera2_camera1)
euler1=T2euler(T_camera2_camera1)
print("euler1",euler1)


R=np.array([[0.996497827174435, 0.0626404894046952, -0.0553917820959913,0.04048902968427485],
 [-0.06685686567969559, 0.9947197930306126, -0.07786329600410435,0.5408975444430671],
 [0.05022190705376148, 0.08129392622008022, 0.9954240591886491,-0.8401133762122147],
 [0,0,0,1]])

euler2=T2euler(R)
print("euler2",euler2)

# T_arm=np.array([[9.95455425e-01 , 2.78983548e-02 , 9.10504128e-02 ,-9.36544744e-02],
# [-2.78703345e-02 , 9.99610299e-01 ,-1.57942265e-03, -1.28588162e-02],
# [-9.10589936e-02 ,-9.65360609e-04 , 9.95845032e-01  ,6.28622380e-02],
# [ 0.00000000e+00 , 0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]])
# euler2=T2euler(T_arm)
# print("euler2",euler2)

# [[ 9.95455425e-01  2.78983548e-02 -9.10504128e-02  9.35247054e-02]
#  [-2.78703345e-02  9.99610299e-01  1.57942265e-03  1.34285325e-02]
#  [ 9.10589936e-02  9.65360609e-04  9.95845032e-01  6.10032916e-02]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]