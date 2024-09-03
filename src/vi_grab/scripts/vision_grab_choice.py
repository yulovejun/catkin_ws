#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
from std_msgs.msg import String, Bool,Empty
import rospy, sys
from rm_msgs.msg import MoveJ_P,Arm_Current_State,Gripper_Set, Gripper_Pick,ArmState
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R
from vi_grab.msg import ObjectInfo
from geometry_msgs.msg import TransformStamped,PointStamped
from geometry_msgs.msg import Point, Quaternion

def convert(x,y,z,x1,y1,z1,rx,ry,rz):
    """
    我们需要将旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体坐标（x, y, z）和
    机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体相对于机械臂基座的位姿（x, y, z, rx, ry, rz）
    """
    # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量
    rotation_matrix = np.array([[ 0.01206237 , 0.99929647,  0.03551135],
 [-0.99988374 , 0.01172294 , 0.00975125],
 [ 0.00932809, -0.03562485 , 0.9993217 ]])
    translation_vector = np.array([-0.08039019, 0.03225555, -0.08256825])
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
    #obj_end_effector_coordinates_homo = np.linalg.inv(T_camera_to_end_effector).dot(obj_camera_coordinates_homo)
    obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
    obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)
    obj_base_coordinates = obj_base_coordinates_homo[:3]  # 从齐次坐标中提取物体的x, y, z坐标
    # 计算物体的旋转
    obj_orientation_matrix = T_base_to_end_effector[:3, :3].dot(rotation_matrix)
    obj_orientation_euler = R.from_matrix(obj_orientation_matrix).as_euler('xyz', degrees=False)
    # 组合结果
    obj_base_pose = np.hstack((obj_base_coordinates, obj_orientation_euler))
    obj_base_pose[3:] = rx,ry,rz
    return obj_base_pose

def object_pose_callback(data):
    # 在这里执行回调函数callback
    if data.object_class == object_msg.data:
 
        arm_pose_msg = rospy.wait_for_message("/rm_driver/Arm_Current_State", Arm_Current_State, timeout=None)
        arm_orientation_msg = rospy.wait_for_message("/rm_driver/ArmCurrentState", ArmState, timeout=None)

        result = convert(data.x,data.y,data.z,arm_pose_msg.Pose[0],arm_pose_msg.Pose[1],arm_pose_msg.Pose[2],arm_pose_msg.Pose[3],arm_pose_msg.Pose[4],arm_pose_msg.Pose[5])
        print(result)
        catch(result,arm_orientation_msg)
        rospy.sleep(2)
        picktask()
        rospy.sleep(2)
        settask()
        sub_object_pose.unregister()

    pass
def catch(result,arm_orientation_msg):
    moveJ_P_pub = rospy.Publisher("rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    zero_pose = MoveJ_P()
    zero_pose.Pose.position.x = result[0]
    zero_pose.Pose.position.y = result[1]
    zero_pose.Pose.position.z = result[2]
    zero_pose.Pose.orientation.x = arm_orientation_msg.Pose.orientation.x
    zero_pose.Pose.orientation.y = arm_orientation_msg.Pose.orientation.y
    zero_pose.Pose.orientation.z = arm_orientation_msg.Pose.orientation.z
    zero_pose.Pose.orientation.w = arm_orientation_msg.Pose.orientation.w
    zero_pose.speed = 0.2
    moveJ_P_pub.publish(zero_pose)
    rate.sleep()
def settask():
    set_pub = rospy.Publisher("rm_driver/Gripper_Set", Gripper_Set, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    set = Gripper_Set()
    set.position = 1000
    set_pub.publish(set)

def picktask():
    pick_pub = rospy.Publisher("rm_driver/Gripper_Pick_On", Gripper_Pick, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    pick1 = Gripper_Pick()
    pick1.speed = 200
    pick1.force = 1000
    pick_pub.publish(pick1)
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('object_catch')
    rospy.sleep(0.5)
    object_msg = rospy.wait_for_message('choice_object', String, timeout=None)

    sub_object_pose = rospy.Subscriber("object_pose", ObjectInfo, object_pose_callback, queue_size=10)

    rospy.spin()


    
