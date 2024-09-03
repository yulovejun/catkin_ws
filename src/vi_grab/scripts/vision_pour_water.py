#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
from std_msgs.msg import String, Bool,Empty
import rospy, sys
from rm_msgs.msg import MoveJ_P,Arm_Current_State,Gripper_Set, Gripper_Pick,ArmState,MoveL,MoveJ
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
    # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量，通过手眼标定得到
    rotation_matrix = np.array([[ 0.01206237 , 0.99929647 , 0.03551135],
 [-0.99988374 , 0.01172294 , 0.00975125],
 [ 0.00932809, -0.03562485 , 0.9993217 ]])
    translation_vector = np.array([-0.08039019,0.03225555,-0.08256825])
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
catch_flag = True
def object_pose_callback(data):        #接收到YOLOV5识别的类型信息，进入回调函数
    global catch_flag                  #初始化抓取标志
    settask()                          #打开夹爪
    if data.object_class == object_msg.data and catch_flag :  #如果接收到的YOLOV5识别类型信息与发布choice_object信息一致，则进入以下
        print(data)
        pub_arm_pose.publish()                               #发布一次获取机械臂位姿的请求
        arm_pose_msg = rospy.wait_for_message("/rm_driver/Arm_Current_State", Arm_Current_State, timeout=None)       #等待位姿信息的接收，x,yz,rx,ry,rz
        print(arm_pose_msg)
        rospy.sleep(1)
        pub_arm_pose.publish()                             #发布一次获取机械臂位姿的请求
        arm_orientation_msg = rospy.wait_for_message("/rm_driver/ArmCurrentState", ArmState, timeout=None)           #等待位姿信息的接收，pose:x,y,z    orientation:x,y,z,w
        print(arm_orientation_msg)
        result = convert(data.x,data.y,data.z,arm_pose_msg.Pose[0],arm_pose_msg.Pose[1],arm_pose_msg.Pose[2],arm_pose_msg.Pose[3],arm_pose_msg.Pose[4],arm_pose_msg.Pose[5])   #将物体三维坐标转化到机械臂基坐标系下
        print(data.object_class,':',result)
        #before_catch()                              #如果途中有奇异区域，就先通过movej规划机械臂绕过奇异区域
        rospy.sleep(1)
        catch(result,arm_orientation_msg)            #进行抓取
        print('---------------------------ok------------------------------')
        catch_flag = False                           #标志位false，只进行一次抓取
        #rospy.Duration(2)
        rospy.signal_shutdown("****************catch completed****************")

def before_catch():
    moveJ_pub = rospy.Publisher("/rm_driver/MoveJ_Cmd", MoveJ, queue_size=10)
    rospy.sleep(1)

    pic_joint = MoveJ()
    pic_joint.joint = [-0.09342730045318604, -0.8248963952064514, 1.5183943510055542, 0.06789795309305191, 0.8130478262901306, 0.015879500657320023]
    pic_joint.speed = 0.3
    moveJ_pub.publish(pic_joint)


def catch(result,arm_orientation_msg):
    moveJ_P_pub = rospy.Publisher("rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=1)       #movej_p操作机械臂运动到物体位置前20cm处
    rospy.sleep(0.5)
    zero_pose = MoveJ_P()
    zero_pose.Pose.position.x = result[0]+0.15          #先运动到距离末端15cm处
    zero_pose.Pose.position.y = result[1]
    zero_pose.Pose.position.z = result[2]+0.03          #+0.03是由于手眼标定结果不太准，Z轴加了补偿，手眼标定结果准确不需要加
    zero_pose.Pose.orientation.x = arm_orientation_msg.Pose.orientation.x
    zero_pose.Pose.orientation.y = arm_orientation_msg.Pose.orientation.y
    zero_pose.Pose.orientation.z = arm_orientation_msg.Pose.orientation.z
    zero_pose.Pose.orientation.w = arm_orientation_msg.Pose.orientation.w
    zero_pose.speed = 0.3
    moveJ_P_pub.publish(zero_pose)

    print('*************************catching*************************')
    rospy.sleep(5.0)
    moveL_pub = rospy.Publisher("rm_driver/MoveL_Cmd", MoveL, queue_size=1)            #机械臂直线运动到物体坐标
    rospy.sleep(0.5)
    first_pose = MoveL()
    first_pose.Pose.position.x = result[0]            #运动到物体处
    first_pose.Pose.position.y = result[1]
    first_pose.Pose.position.z = result[2]+0.03
    first_pose.Pose.orientation.x = arm_orientation_msg.Pose.orientation.x
    first_pose.Pose.orientation.y = arm_orientation_msg.Pose.orientation.y
    first_pose.Pose.orientation.z = arm_orientation_msg.Pose.orientation.z
    first_pose.Pose.orientation.w = arm_orientation_msg.Pose.orientation.w
    first_pose.speed = 0.2
    moveL_pub.publish(first_pose)
    rospy.sleep(3)
    picktask()                                                  #夹爪闭合
    rospy.sleep(3)              
    up_pose = MoveL()                                           #抬起物体4cm
    up_pose.Pose.position.x = result[0]
    up_pose.Pose.position.y = result[1]
    up_pose.Pose.position.z = result[2]+0.08
    up_pose.Pose.orientation.x = arm_orientation_msg.Pose.orientation.x
    up_pose.Pose.orientation.y = arm_orientation_msg.Pose.orientation.y
    up_pose.Pose.orientation.z = arm_orientation_msg.Pose.orientation.z
    up_pose.Pose.orientation.w = arm_orientation_msg.Pose.orientation.w
    up_pose.speed = 0.2
    moveL_pub.publish(up_pose)
    print('*************************moving*************************')
    rospy.sleep(5.0)

    rospy.sleep(1)
    second_pose = MoveL()                                      #直线运动到倒水的地方
    second_pose.Pose.position.x = -0.312
    second_pose.Pose.position.y = 0.178
    second_pose.Pose.position.z = 0.56
    second_pose.Pose.orientation.x = arm_orientation_msg.Pose.orientation.x
    second_pose.Pose.orientation.y = arm_orientation_msg.Pose.orientation.y
    second_pose.Pose.orientation.z = arm_orientation_msg.Pose.orientation.z
    second_pose.Pose.orientation.w = arm_orientation_msg.Pose.orientation.w
    second_pose.speed = 0.2
    moveL_pub.publish(second_pose)
    rospy.sleep(3)
    print('*************************waiting*************************')

    rospy.sleep(5.0)
    movej_pub = rospy.Publisher("rm_driver/MoveJ_Cmd", MoveJ, queue_size=1)
    rospy.sleep(1)
    pub_arm_pose.publish()
    arm_joint_msg = rospy.wait_for_message("/rm_driver/ArmCurrentState", ArmState, timeout=None)        #获取当前关节角度
    third_pose = MoveJ()
    third_pose.joint = [arm_joint_msg.joint[0],arm_joint_msg.joint[1],arm_joint_msg.joint[2],arm_joint_msg.joint[3],arm_joint_msg.joint[4],arm_joint_msg.joint[5]-1]        #运动关节6进行倒水
    third_pose.speed = 0.2
    movej_pub.publish(third_pose)
    rospy.sleep(3)
    end_pose = MoveJ()
    end_pose.joint = [arm_joint_msg.joint[0],arm_joint_msg.joint[1],arm_joint_msg.joint[2],arm_joint_msg.joint[3],arm_joint_msg.joint[4],arm_joint_msg.joint[5]]       #倒水完成，关节6恢复
    end_pose.speed = 0.2
    movej_pub.publish(end_pose)
    rospy.sleep(3)
    print('*************************pouring*************************')

def settask():
    set_pub = rospy.Publisher("rm_driver/Gripper_Set", Gripper_Set, queue_size=1)
    rospy.Duration(1)
    set = Gripper_Set()
    set.position = 1000
    set_pub.publish(set)

def picktask():
    pick_pub = rospy.Publisher("rm_driver/Gripper_Pick_On", Gripper_Pick, queue_size=1)
    rospy.sleep(1)
    pick1 = Gripper_Pick()
    pick1.speed = 200
    pick1.force = 500
    pick_pub.publish(pick1)


def voice_callback(msg):
    rospy.loginfo(msg.data)
    if msg.data.find("杯") >-1:
      target_pub = rospy.Publisher("/choice_object",String,queue_size=1)
      msg1 = 'cup'
      rospy.Duration(1)
      target_pub.publish(msg1)
      print(msg1)
    elif msg.data.find("水瓶") >-1:
      target_pub2 = rospy.Publisher("/choice_object",String,queue_size=1)
      msg2 = 'bottle'
      rospy.Duration(1)
      target_pub2.publish(msg2)
      print(msg2)
    else:
      print('no match')

if __name__ == '__main__':
    rospy.init_node('object_catch')  #初始化节点

    pub_arm_pose = rospy.Publisher("/rm_driver/GetCurrentArmState",Empty,queue_size=1) #创建发布机械臂位姿的pub
    object_msg = rospy.wait_for_message('/choice_object', String, timeout=None)  #等待选取抓取的物体
    #rospy.Subscriber("xfspeech", String, voice_callback)     #如果是语音控制，则通过xfspeech话题获取需要抓取物体的类型
    sub_object_pose = rospy.Subscriber("/object_pose", ObjectInfo, object_pose_callback, queue_size=1)   #订阅yolov5的类型识别信息
    rospy.spin()   
    print('---------------------------final----------------------------')
    



    
