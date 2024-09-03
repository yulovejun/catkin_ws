#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
from std_msgs.msg import String, Bool, Empty
import rospy, sys
from rm_msgs.msg import MoveJ_P, Arm_Current_State, Gripper_Set, Gripper_Pick, ArmState, Hand_Angle, MoveJ, MoveC
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R
from vi_grab.msg import ObjectInfo
from geometry_msgs.msg import TransformStamped, PointStamped
from geometry_msgs.msg import Point, Quaternion


def convert(x, y, z, x1, y1, z1, rx, ry, rz):
    """
    我们需要将旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体坐标（x, y, z）和
    机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体相对于机械臂基座的位姿（x, y, z, rx, ry, rz）
    """
    # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量#[[ 0.28  0.01  0.96]
    #  [ 0.96 -0.06 -0.28]
    #  [ 0.06  1.   -0.03]]

    #[[-0.17]
    #[-0.06]
    # [ 0.11]]
    rotation_matrix = np.array([[0.42, -0.01, 0.91],
                                [0.91, -0.07, -0.42],
                                [0.07, 1.00, -0.02]])
    translation_vector = np.array([0.29, -0.14, -0.04])
    # 深度相机识别物体返回的坐标
    obj_camera_coordinates = np.array([x, y, z])

    # 机械臂末端的位姿，单位为弧度
    end_effector_pose = np.array([x1, y1, z1,
                                  rx, ry, rz])
    # 将旋转矩阵和平移向量转换为齐次变换矩阵
    T_camera_to_end_effector = np.eye(4)
    T_camera_to_end_effector[:3, :3] = rotation_matrix
    T_camera_to_end_effector[:3, 3] = translation_vector
    # 机械臂末端的位姿转换为齐次变换矩阵
    position = end_effector_pose[:3]
    orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
    T_base_to_end_effector = np.eye(4)
    T_base_to_end_effector[:3, :3] = orientation
    T_base_to_end_effector[:3, 3] = position
    # 计算物体相对于机械臂基座的位姿
    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # 将物体坐标转换为齐次坐标
    # obj_end_effector_coordinates_homo = np.linalg.inv(T_camera_to_end_effector).dot(obj_camera_coordinates_homo)
    obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
    obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)
    obj_base_coordinates = obj_base_coordinates_homo[:3]  # 从齐次坐标中提取物体的x, y, z坐标
    # 计算物体的旋转
    obj_orientation_matrix = T_base_to_end_effector[:3, :3].dot(rotation_matrix)
    obj_orientation_euler = R.from_matrix(obj_orientation_matrix).as_euler('xyz', degrees=False)
    # 组合结果
    obj_base_pose = np.hstack((obj_base_coordinates, obj_orientation_euler))
    obj_base_pose[3:] = rx, ry, rz
    return obj_base_pose


def object_pose_callback(data):
    # 在这里执行回调函数callback
    if data.object_class == object_msg.data:   #判断传入的 data 消息中的 object_class 属性是否等于 object_msg.data
        arm_pose_msg = rospy.wait_for_message("/rm_driver/Arm_Current_State", Arm_Current_State, timeout=None)
        arm_orientation_msg = rospy.wait_for_message("/rm_driver/ArmCurrentState", ArmState, timeout=None)
        print("+++++++++++++++++++++++")
        print(arm_orientation_msg)
        print("+++++++++++++++++++++++")
        result = convert(data.x, data.y, data.z, arm_pose_msg.Pose[0], arm_pose_msg.Pose[1], arm_pose_msg.Pose[2],
                         arm_pose_msg.Pose[3], arm_pose_msg.Pose[4], arm_pose_msg.Pose[5])
        print("***********************")
        print(result)
        print("***********************")
        #before_catch()
        hand_opentask()
        rospy.sleep(3)
        catch(result, arm_orientation_msg)
        rospy.sleep(5)
        hand_picktask()
        rospy.sleep(5)
        hand_opentask()
        #before_catch()
        sub_object_pose.unregister()

    pass


def catch(result, arm_orientation_msg):
    moveJ_P_pub = rospy.Publisher("rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    zero_pose = MoveJ_P()
    #到达需要抓取物体的位置
    zero_pose.Pose.position.x = 0.2246440052986145
    zero_pose.Pose.position.y = -0.7021329998970032
    zero_pose.Pose.position.z = -0.18496200203895569
    zero_pose.Pose.orientation.x = 0.6751851585222391
    zero_pose.Pose.orientation.y = -0.2999736832796051
    zero_pose.Pose.orientation.z = 0.0843225000558658
    zero_pose.Pose.orientation.w = 0.6686034004066196
    zero_pose.speed = 0.09
    moveJ_P_pub.publish(zero_pose)
    rate.sleep()


def before_catch():
    moveJ_pub = rospy.Publisher("/rm_driver/MoveJ_Cmd", MoveJ, queue_size=10)
    rospy.sleep(1)
    pic_joint = MoveJ()
    pic_joint.joint = [2.5242297649383545/57.3, 0.5825507640838623/57.3, -2.151550054550171/57.3, 2.0903878211975098/57.3, 1.2464709281921387/57.3, -3.53777813911438/57.3]
    pic_joint.speed = 0.05
    moveJ_pub.publish(pic_joint)


# def settask():
#     set_pub = rospy.Publisher("rm_driver/Gripper_Set", Gripper_Set, queue_size=10)
#     rospy.sleep(1)
#     rate = rospy.Rate(10)
#     set = Gripper_Set()
#     set.position = 1000
#     set_pub.publish(set)


# def picktask():  #夹爪抓取
#     pick_pub = rospy.Publisher("rm_driver/Gripper_Pick_On", Gripper_Pick, queue_size=10)
#     rospy.sleep(1)
#     rate = rospy.Rate(10)
#     pick1 = Gripper_Pick()
#     pick1.speed = 200
#     pick1.force = 1000
#     pick_pub.publish(pick1)
#     rate.sleep()


def hand_opentask():
    open_pub = rospy.Publisher("rm_driver/Hand_SetAngle", Hand_Angle, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10) #设置频率为10Hz
    open1 = Hand_Angle()
    # 设置灵巧手角度，灵巧手有 6 个自由度，从 1~6 分别为小拇指，无名指，中指，食指，大拇指弯曲，大拇指旋转
    # int16[6] hand_angle  手指角度数组，范围：0~1000.另外，-1 代表该自由度不执行任何操作，保持当前状态
    open1.hand_angle = [990, 990, 990, 990, 660, 100]
    open_pub.publish(open1)


def hand_picktask():
    pick_pub = rospy.Publisher("rm_driver/Hand_SetAngle", Hand_Angle, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10) #设置频率为10Hz
    pick1 = Hand_Angle()
    pick1.hand_angle = [800, 800, 800, 650, 650, 100]
    pick_pub.publish(pick1)
    rate.sleep()


if __name__ == '__main__':
    rospy.init_node('object_catch') #初始化节点
    rospy.sleep(0.5)
    hand_picktask()
    #before_catch()
    object_msg = rospy.wait_for_message('choice_object', String, timeout=None)  #等待选取抓取的物体
    #before_catch()
    sub_object_pose = rospy.Subscriber("object_pose", ObjectInfo, object_pose_callback, queue_size=10)  #订阅yolov5的类型识别信息

    rospy.spin()



