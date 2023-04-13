# 获取对齐的rgb和深度图

import math
import time
import pyrealsense2 as rs
import numpy as np
import cv2
# 提示没有aruco的看问题汇总
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)


def get_aligned_images():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    # 获取intelrealsense参数
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    # 内参矩阵，转ndarray方便后续opencv直接使用
    intr_matrix = np.array([
        [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
    ])
    # 深度图-16位
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    # 深度图-8位
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
    pos = np.where(depth_image_8bit == 0)
    depth_image_8bit[pos] = 255
    # rgb图
    color_image = np.asanyarray(color_frame.get_data())
    # return: rgb图，深度图，相机内参，相机畸变系数(intr.coeffs)
    return np.array(intr.coeffs)
print(get_aligned_images())