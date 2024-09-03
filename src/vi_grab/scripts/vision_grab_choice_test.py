#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
from std_msgs.msg import String, Bool, Empty
import rospy, sys
from rm_msgs.msg import MoveJ_P, Arm_Current_State, Gripper_Set, Gripper_Pick, ArmState, Hand_Angle, MoveJ, MoveC ,MoveL ,Hand_Speed ,ChangeWorkFrame_Name ,ChangeTool_Name ,Hand_Force
from geometry_msgs.msg import Pose
import numpy as np
import json
import time
import socket
from scipy.spatial.transform import Rotation as R
from vi_grab.msg import ObjectInfo
from geometry_msgs.msg import TransformStamped, PointStamped
from geometry_msgs.msg import Point, Quaternion
import ctypes

move_to_a3_completed = False
selected_object = None

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
    rotation_matrix = np.array([[0.01, 1.00, 0.02],
                                [-1.00, 0.01, -0.00],
                                [-0.01, -0.02, 1.00]])
    translation_vector = np.array([-0.08, 0.03, 0.01])#base
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
    print("+++++++*************************************************************************++++++++++++++++")
    print("Object orientation in Euler angles (radians):", obj_orientation_euler)
    print("+++++++*************************************************************************++++++++++++++++")
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
        hand_pick_one()
        rospy.sleep(0.5)
        catch(result, arm_orientation_msg)
        rospy.sleep(3)
        hand_pick_two()
        #rospy.sleep(5)
        #hand_pick_one()#
        rospy.sleep(3)
        #before_catch()  ######
        catch1(result, arm_orientation_msg)
        rospy.sleep(3)
        move_chassis(chassis_client, 'a4')
        rospy.sleep(2)
        catch2(result, arm_orientation_msg)
        #hand_pick_one()
        #rospy.sleep(5)
        before_catch()
        rospy.sleep(5)
        move_chassis(chassis_client, 'a1')
        sub_object_pose.unregister()

    pass


def catch(result, arm_orientation_msg):
    rm_x = 0.045
    rm_y = -0.08
    rm_z = -0.25
    moveL_pub = rospy.Publisher("rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    zero_pose = MoveJ_P()
    #dll = ctypes.cdll.LoadLibrary('./libRM_Service.so')
    ##res = dll
    #到达需要抓取物体的位置
    zero_pose.Pose.position.x = result[0]+rm_x
    zero_pose.Pose.position.y = result[1]+rm_y
    zero_pose.Pose.position.z = result[2]+rm_z
    zero_pose.Pose.orientation.x = 0
    zero_pose.Pose.orientation.y = 0
    zero_pose.Pose.orientation.z = 0.724
    zero_pose.Pose.orientation.w = 0.690
    zero_pose.speed = 0.40
    moveL_pub.publish(zero_pose)
    rate.sleep()

    print('*************************catching*************************')  # 转动灵巧手
    rospy.sleep(2)
    moveL_pub2 = rospy.Publisher("rm_driver/MoveL_Cmd", MoveL, queue_size=1)  # 机械臂直线运动到物体坐标
    # rospy.sleep(0.5)
    # hand_pose = MoveL()
    # hand_pose.Pose.position.x = result[0]+rm_x
    # hand_pose.Pose.position.y = result[1]+rm_y
    # hand_pose.Pose.position.z = result[2]+rm_z
    # hand_pose.Pose.orientation.x = 0
    # hand_pose.Pose.orientation.y = 0
    # hand_pose.Pose.orientation.z = 0.724
    # hand_pose.Pose.orientation.w = 0.690
    # hand_pose.speed = 0.50
    # moveL_pub.publish(hand_pose)
    # rospy.sleep(3)
# #############################
    rospy.sleep(3)
    up_pose = MoveL()
    up_pose.Pose.position.x = result[0]+rm_x
    up_pose.Pose.position.y = result[1]+rm_y
    up_pose.Pose.position.z = result[2]-0.09   #前进到物体
    up_pose.Pose.orientation.x = 0
    up_pose.Pose.orientation.y = 0
    up_pose.Pose.orientation.z = 0.724
    up_pose.Pose.orientation.w = 0.690
    up_pose.speed = 0.40
    moveL_pub2.publish(up_pose)
    print('*************************moving*************************')
    rospy.sleep(1)
    #hand_pick_two()


def catch1(result, arm_orientation_msg):
    rm_x = 0.02
    rm_y = -0.18
    rm_z = -0.15
    #*******************************************************************************
    #抓取到物体后，收回手臂
    moveL_pub1 = rospy.Publisher("rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    zero_pose1 = MoveJ_P()
    zero_pose1.Pose.position.x = result[0]-0.20
    zero_pose1.Pose.position.y = result[1]-0.08
    zero_pose1.Pose.position.z = result[2]-0.30
    zero_pose1.Pose.orientation.x = 0
    zero_pose1.Pose.orientation.y = 0
    zero_pose1.Pose.orientation.z = 0.724
    zero_pose1.Pose.orientation.w = 0.690
    zero_pose1.speed = 0.40
    moveL_pub1.publish(zero_pose1)
    rate.sleep()
    rospy.sleep(2)


def catch2(result, arm_orientation_msg):
    rm_x = 0.02
    rm_y = -0.18
    rm_z = -0.15
    moveL_pub = rospy.Publisher("rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    zero_pose = MoveJ_P()
    #dll = ctypes.cdll.LoadLibrary('./libRM_Service.so')
    ##res = dll
    #到达需要抓取物体的位置
    zero_pose.Pose.position.x = result[0]+rm_x
    zero_pose.Pose.position.y = result[1]+rm_y
    zero_pose.Pose.position.z = result[2]+rm_z
    zero_pose.Pose.orientation.x = 0
    zero_pose.Pose.orientation.y = 0
    zero_pose.Pose.orientation.z = 0.724
    zero_pose.Pose.orientation.w = 0.690
    zero_pose.speed = 0.35
    moveL_pub.publish(zero_pose)
    rate.sleep()
    rospy.sleep(2)
    #********************************
    hand_pick_one()
    rospy.sleep(5)
    moveL_pub2 = rospy.Publisher("rm_driver/MoveL_Cmd", MoveL, queue_size=1)
    up_pose = MoveL()
    up_pose.Pose.position.x = result[0]+rm_x-0.09
    up_pose.Pose.position.y = result[1]+rm_y
    up_pose.Pose.position.z = result[2]+rm_z-0.08
    up_pose.Pose.orientation.x = 0
    up_pose.Pose.orientation.y = 0
    up_pose.Pose.orientation.z = 0.724
    up_pose.Pose.orientation.w = 0.690
    up_pose.speed = 0.30
    moveL_pub2.publish(up_pose)
    rospy.sleep(1)


def before_catch():
    moveJ_pub = rospy.Publisher("/rm_driver/MoveJ_Cmd", MoveJ, queue_size=10)
    rospy.sleep(1)
    pic_joint = MoveJ()
    #pic_joint.joint = [-171.157/57.3, -124.670/57.3, 70.857/57.3, -0.652/57.3, 82.494/57.3, -121.658/57.3]
    pic_joint.joint = [-171.157/57.3, -114.554/57.3, 82.685/57.3, -0.652/57.3, 69.748/57.3, -121.658/57.3]
    pic_joint.speed = 0.15
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

def hand_speed():
    pick_pub = rospy.Publisher("rm_driver/Hand_SetSpeed", Hand_Speed, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10) #设置频率为10Hz
    hand_speed1 = Hand_Speed()
    hand_speed1.hand_speed = 260  #150..
    pick_pub.publish(hand_speed1)
    rate.sleep()


def hand_force():
    pick_pub = rospy.Publisher("rm_driver/Hand_SetForce", Hand_Force, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10)  # 设置频率为10Hz
    hand_force1 = Hand_Force()
    hand_force1.hand_force = 1000
    pick_pub.publish(hand_force1)
    rate.sleep()


def hand_opentask():
    open_pub = rospy.Publisher("rm_driver/Hand_SetAngle", Hand_Angle, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10) #设置频率为10Hz
    open1 = Hand_Angle()
    # 设置灵巧手角度，灵巧手有 6 个自由度，从 1~6 分别为小拇指，无名指，中指，食指，大拇指弯曲，大拇指旋转
    # int16[6] hand_angle  手指角度数组，范围：0~1000.另外，-1 代表该自由度不执行任何操作，保持当前状态
    open1.hand_angle = [990, 990, 990, 990, 800, 100]
    open_pub.publish(open1)


def hand_picktask():
    pick_pub = rospy.Publisher("rm_driver/Hand_SetAngle", Hand_Angle, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(10) #设置频率为10Hz
    pick1 = Hand_Angle()
    pick1.hand_angle = [800, 800, 800, 650, 650, 100]
    pick_pub.publish(pick1)
    rate.sleep()


    #*****************************************************************
def hand_oktask():  # ok手势
        ok_pub = rospy.Publisher("rm_driver/Hand_SetAngle", Hand_Angle, queue_size=10)
        rospy.sleep(1)
        rate = rospy.Rate(10)  # 设置频率为10Hz
        ok = Hand_Angle()
        ok.hand_angle = [1000, 1000, 1000, 490, 550, 120]
        ok_pub.publish(ok)
        rate.sleep()


def hand_pick_open():  # 手张开
        open_pub = rospy.Publisher("rm_driver/Hand_SetAngle", Hand_Angle, queue_size=10)
        rospy.sleep(1)
        rate = rospy.Rate(10)  # 设置频率为10Hz
        open1 = Hand_Angle()
        open1.hand_angle = [1000, 1000, 1000, 1000, 1000, 900]
        open_pub.publish(open1)
        rate.sleep()


def hand_pick_one():  # 准备抓取预动作，大拇指旋转
        pick_pub1 = rospy.Publisher("rm_driver/Hand_SetAngle", Hand_Angle, queue_size=10)
        rospy.sleep(1)
        rate = rospy.Rate(10)  # 设置频率为10Hz
        pick1 = Hand_Angle()
        pick1.hand_angle = [950, 950, 950, 950, 950, 120]
        pick_pub1.publish(pick1)
        rate.sleep()

def hand_pick_two():  # 进行抓取，5指都弯曲
        pick_pub2 = rospy.Publisher("rm_driver/Hand_SetAngle", Hand_Angle, queue_size=10)
        rospy.sleep(1)
        rate = rospy.Rate(10)  # 设置频率为10Hz
        pick2 = Hand_Angle()
        pick2.hand_angle = [650, 650, 650, 650, 650, 0]
        pick_pub2.publish(pick2)
        rate.sleep()


def move_chassis(client, marker):
    global move_to_a3_completed
    move_command = f'/api/move?marker={marker}'
    client.send(move_command.encode('utf-8'))
    chassis_receive_data = client.recv(1024)
    print("********")
    #time.sleep(1)
    response = client.recv(1024).decode()
    # print("Response:", response)
    # data = json.loads(response)
    # move_code = data.get("code")
    #time.sleep(10)
    while True:
        response = client.recv(1024).decode()
        print("Response:", response)
        data = json.loads(response)
        move_code = data.get("code")
        if move_code == "01002":
            print("Move completed successfully.")
            print("********")
            print("                            ")
            time.sleep(1)
            move_to_a3_completed = True
        break


# def get_current_arm_state():
#     # 创建发布者对象，发布Empty消息到/rm_driver/GetCurrentArmState话题
#     pub = rospy.Publisher('/rm_driver/GetCurrentArmState', Empty, queue_size=1)
#     # 等待rospy和发布者准备好
#     rospy.sleep(1)
#     # 发布Empty消息以获取机器臂的当前状态
#     rospy.loginfo("Publishing request to get current arm state.")
#     state = Empty()
#     pub.publish(state)


# def select_grasp_object(object_name):
#     # 创建发布者对象，发布String消息到/choice_object话题
#     pub = rospy.Publisher('/choice_object', String, queue_size=10)
#     rate = rospy.Rate(10)  # 设置频率为10Hz
#     # 等待rospy和发布者准备好
#     rospy.sleep(1)
#     # 创建String消息实例
#     object1 = String()
#     object1.data = object_name  #设置要抓取的物体名称
#     # 发布消息
#     rospy.loginfo("Publishing object selection: {}".format(object_name))
#     pub.publish(object1)
#     rospy.sleep(1)
#     rate.sleep()


# 回调函数，处理接收到的消息
def object_selection_callback(obj):
    global selected_object  # 声明全局变量
    # 打印接收到的消息
    print("I heard the object to grab is: %s", obj)
    if obj.data == 'bottle':
        selected_object = obj.data
        print("selected_object:", obj.data)
        print("selected_object:**", selected_object)
    else:
        print("The selected object is not the desired type.")
    #rospy.spin()

def switch_tool_coordinate_system(tool_name):
    # 初始化节点
    #rospy.init_node('tool_coordinate_switcher', anonymous=True)

    # 创建发布者对象，发布ChangeTool_Name消息到/rm_driver/ChangeToolName_Cmd话题
    pub = rospy.Publisher('/rm_driver/ChangeToolName_Cmd', ChangeTool_Name, queue_size=1)

    # 等待rospy和发布者准备好
    rospy.sleep(1)

    # 创建ChangeTool_Name消息实例
    msg1 = ChangeTool_Name()
    msg1.toolname = tool_name  # 设置工具坐标系名称

    # 发布消息
    pub.publish(msg1)
    print("***************************************")
    # 打印确认信息
    rospy.loginfo("Published tool coordinate switch command for: {}".format(tool_name))
    print("***************************************")
    # 保持节点运行一段时间，以确保消息被发送和处理
    #rospy.spin()


def switch_work_coordinate_system(work_frame_name):
    # 初始化节点
    #rospy.init_node('work_coordinate_system_switcher', anonymous=True)

    # 创建发布者对象，发布ChangeWorkFrame_Name消息到/rm_driver/ChangeWorkFrame_Cmd话题
    pub = rospy.Publisher('/rm_driver/ChangeWorkFrame_Cmd', ChangeWorkFrame_Name, queue_size=1)

    # 等待rospy和发布者准备好
    rospy.sleep(1)

    # 创建ChangeWorkFrame_Name消息实例
    msg2 = ChangeWorkFrame_Name()
    msg2.WorkFrame_name = work_frame_name  # 设置工作坐标系名称

    # 发布消息
    pub.publish(msg2)
    print("***************************************")
    # 打印确认信息
    rospy.loginfo("Published work coordinate system switch command for: {}".format(work_frame_name))
    print("***************************************")
    # 保持节点运行一段时间，以确保消息被发送和处理
    #rospy.spin()


if __name__ == '__main__':
    rospy.init_node('object_catch') #初始化节点
    rospy.sleep(0.5)
    print("*************************************************")
    try:
        # 调用函数，切换到名为'Base'的工作坐标系
        switch_work_coordinate_system('Base')
        rospy.sleep(1)
        # 调用函数，切换到名为'RM'的工作坐标系
        switch_tool_coordinate_system('RM')
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Switching work coordinate system interrupted or completed.")
    # switch_tool_coordinate_system('RM')
    # switch_work_coordinate_system('Base')
    rospy.sleep(1)
    before_catch()  # 初始位姿
    print("*************************************************")

    # **********与底盘建立通信***********
    chassis_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    chassis_host = '192.168.10.10'
    chassis_port = 31001
    chassis_client.connect((chassis_host, chassis_port))
    print("-------------")
    # **********与底盘建立通信***********

    #move_chassis(chassis_client, 'a1')  # 运动到a1点
    move_chassis(chassis_client, 'a2')  # 运动到a2点
    move_chassis(chassis_client, 'a3')  # 运动到a3点
    #rospy.sleep(3)

    if move_to_a3_completed:
        #select_grasp_object('bottle')
        rospy.sleep(1)
        #print("Waiting for messages on 'choice_object' topic...")
        #$sub_choice_object = rospy.Subscriber('choice_object', String, object_selection_callback, queue_size=10)
        #print("*******Received message:*******", selected_object)
        #rospy.sleep(10)     ##等待回调函数处理
        #if selected_object == 'bottle':
        object_msg = rospy.wait_for_message('choice_object', String, timeout=None)  # 等待选取抓取的物体
        #print("Received message: %s", selected_object)  # 打印接收到的消息
        hand_speed()
        hand_force()
        rospy.sleep(1)
        hand_oktask()
            #rospy.sleep(1)
            #rospy.sleep(2)
        sub_object_pose = rospy.Subscriber("object_pose", ObjectInfo, object_pose_callback, queue_size=10)  #订阅yolov5的类型识别信息
        #else:
            #print("Selection error")
    #print("*******AfterReceived message:*******", selected_object)
    rospy.spin()



