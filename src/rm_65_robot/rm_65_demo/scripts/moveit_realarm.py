#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from std_msgs.msg import String, Bool,Empty
from rm_msgs.msg import MoveJ_P,Arm_Current_State,Gripper_Set, Gripper_Pick,ArmState,MoveL,MoveJ
import numpy as np
from scipy.spatial.transform import Rotation as R
from vi_grab.msg import ObjectInfo
from geometry_msgs.msg import PoseStamped, Pose
import geometry_msgs.msg
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import Quaternion
catch_flag = True
class moveit_grab:
    
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_grab')
        print('1')
        self.moveit_control()

   
    # 设置场景物体的颜色
    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)
        
    def pour_water():
        moveJ_pub = rospy.Publisher("/rm_driver/MoveJ_Cmd", MoveJ, queue_size=10)
        rospy.sleep(1)
        rate = rospy.Rate(10)
        pic_joint = MoveJ()
        pic_joint.joint = [-0.06339585036039352, -0.6514782905578613, 1.9803133010864258, 0.0842486023902893, 0.28349271416664124, -0.011517000384628773]
        pic_joint.speed = 0.1
        moveJ_pub.publish(pic_joint)
        rate.sleep()

    def settask():
        set_pub = rospy.Publisher("rm_driver/Gripper_Set", Gripper_Set, queue_size=1)
        rospy.Duration(1)
        rate = rospy.Rate(10)
        set = Gripper_Set()
        set.position = 1000
        set_pub.publish(set)

    def picktask():
        pick_pub = rospy.Publisher("rm_driver/Gripper_Pick_On", Gripper_Pick, queue_size=1)
        rospy.Duration(1)
        rate = rospy.Rate(10)
        pick1 = Gripper_Pick()
        pick1.speed = 200
        pick1.force = 1000
        pick_pub.publish(pick1)
        rate.Duration()

    def convert(self,x,y,z,x1,y1,z1,rx,ry,rz):
        """
        我们需要将旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体坐标（x, y, z）和
        机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体相对于机械臂基座的位姿（x, y, z, rx, ry, rz）
        """
        # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量
        rotation_matrix = np.array([[ 0.01206237 , 0.99929647  ,0.03551135],
    [-0.99988374 , 0.01172294 , 0.00975125],
    [ 0.00932809 ,-0.03562485 , 0.9993217 ]])
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

    def moveit_control(self):
        self.pub_arm_pose = rospy.Publisher("/rm_driver/GetCurrentArmState",Empty,queue_size=1)
        print('self.pub_arm_pose')
        self.object_msg = rospy.wait_for_message('/choice_object', String, timeout=None)
        #rospy.Subscriber("xfspeech", String, voice_callback)
        print('xfspeech')
        self.sub_object_pose = rospy.Subscriber("/object_pose", ObjectInfo, self.object_pose_callback, queue_size=1)
        rospy.spin()

    def object_pose_callback(self,data):
        global catch_flag
        if data.object_class == self.object_msg.data and catch_flag :
            print(data)
            self.pub_arm_pose.publish()
            arm_pose_msg = rospy.wait_for_message("/rm_driver/Arm_Current_State", Arm_Current_State, timeout=None)
            print(arm_pose_msg)
            rospy.Duration(1)
            self.pub_arm_pose.publish()
            arm_orientation_msg = rospy.wait_for_message("/rm_driver/ArmCurrentState", ArmState, timeout=None)
            print(arm_orientation_msg)
            result = self.convert(data.x,data.y,data.z,arm_pose_msg.Pose[0],arm_pose_msg.Pose[1],arm_pose_msg.Pose[2],arm_pose_msg.Pose[3],arm_pose_msg.Pose[4],arm_pose_msg.Pose[5])
            print(data.object_class,':',result)
            scene = PlanningSceneInterface()
            
            # 创建一个发布场景变化信息的发布者
            self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
            
            # 创建一个存储物体颜色的字典对象
            self.colors = dict()
            
            # 等待场景准备就绪
            rospy.sleep(1)
                            
            # 初始化需要使用move group控制的机械臂中的arm group
            arm = MoveGroupCommander('arm')
            # 获取终端link的名称
            end_effector_link = arm.get_end_effector_link()
            oc = OrientationConstraint()
            oc.link_name = "link6"
            oc.header.frame_id = "base_link"
            oc.orientation = Quaternion(0.712, -0.004, 0.702, -0.004)  # specify the desired orientation here
            oc.absolute_x_axis_tolerance = 0.01
            oc.absolute_y_axis_tolerance = 0.01
            oc.absolute_z_axis_tolerance = 0.01
            oc.weight = 1.0

            constraints = moveit_commander.Constraints()
            constraints.orientation_constraints.append(oc)
            arm.set_path_constraints(constraints)           
            # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
            arm.set_goal_position_tolerance(0.01)
            arm.set_goal_orientation_tolerance(0.01)
        
            # 当运动规划失败后，允许重新规划
            arm.allow_replanning(True)
            
            # 设置目标位置所使用的参考坐标系
            reference_frame = 'base_link'
            arm.set_pose_reference_frame(reference_frame)
            
            # 设置每次运动规划的时间限制：5s
            arm.set_planning_time(10)
            
            # 设置场景物体的名称
            table_id = 'table'
            box1_id = 'box1'
            box2_id = 'box2'
            
            # 移除场景中之前运行残留的物体
            scene.remove_world_object(table_id)
            scene.remove_world_object(box1_id)
            scene.remove_world_object(box2_id)    
            rospy.sleep(1)
            
            # 设置桌面的高度
            table_ground = 0.35

            # 设置table、box1和box2的三维尺寸
            table_size = [0.2, 0.8, 0.30]
            box1_size = [0.05, 0.05, 0.05]
            box2_size = [0.05, 0.05, 0.05]

            # 将三个物体加入场景当中
            table_pose = PoseStamped()
            table_pose.header.frame_id = reference_frame
            table_pose.pose.position.x = -0.33
            table_pose.pose.position.y = 0.0
            # table_pose.pose.position.z = table_ground + table_size[2] / 2.0
            table_pose.pose.position.z = 0.0
            table_pose.pose.orientation.w = 1.0
            scene.add_box(table_id, table_pose, table_size)
            # rospy.sleep(2)

            box1_pose = PoseStamped()
            box1_pose.header.frame_id = reference_frame
            box1_pose.pose.position.x = -0.35
            box1_pose.pose.position.y = 0
            box1_pose.pose.position.z = 0
            box1_pose.pose.orientation.w = 1.0
            scene.add_box(box1_id, box1_pose, box1_size)
            # rospy.sleep(2)
            
            box2_pose = PoseStamped()
            box2_pose.header.frame_id = reference_frame
            box2_pose.pose.position.x = -0.35
            box2_pose.pose.position.y = 0.02
            box2_pose.pose.position.z = 0
            box2_pose.pose.orientation.w = 1.0
            scene.add_box(box2_id, box2_pose, box2_size)
            rospy.sleep(2)

            #将桌子设置成红色，两个box设置成橙色
            self.setColor(table_id, 0.8, 0, 0, 1.0)
            self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
            self.setColor(box2_id, 0.8, 0.4, 0, 1.0)

            #将场景中的颜色设置发布
            self.sendColors()
            rospy.sleep(2)
            #preper pose
            moveJ_pub = rospy.Publisher("/rm_driver/MoveJ_Cmd", MoveJ, queue_size=10)
            rospy.sleep(1)

            # pic_joint = MoveJ()
            # pic_joint.joint = [-0.06339585036039352, -0.6514782905578613, 1.9803133010864258, 0.0842486023902893, 0.28349271416664124, -0.011517000384628773]
            # pic_joint.speed = 0.1
            # moveJ_pub.publish(pic_joint)
            # rospy.sleep(3)

            #设置机械臂的运动目标位置，位于桌面之上两个盒子之间
            target_pose = PoseStamped()
            target_pose.header.frame_id = reference_frame
            target_pose.pose.position.x = result[0]
            target_pose.pose.position.y = result[1]
            target_pose.pose.position.z = result[2]

            target_pose.pose.orientation.w = 0.0026202021745442778
            target_pose.pose.orientation.x = 0.7220693066153941#arm_orientation_msg.Pose.orientation.x
            target_pose.pose.orientation.y = 0.0027474012636504236#arm_orientation_msg.Pose.orientation.y
            target_pose.pose.orientation.z = -0.6918103083727684#arm_orientation_msg.Pose.orientation.z
    
            # 控制机械臂运动到目标位置
            arm.set_pose_target(target_pose, end_effector_link)
            arm.go(wait=True)
            print('target1')
            rospy.sleep(2)

            # 设置机械臂的运动目标位置，进行避障规划
            # target_pose2 = PoseStamped()
            # target_pose2.header.frame_id = reference_frame
            # target_pose2.pose.position.x = 0.2
            # target_pose2.pose.position.y = -0.25
            # target_pose2.pose.position.z = table_pose.pose.position.z + table_size[2] + 0.05
            # target_pose2.pose.orientation.w = 1.0
            
            # # 控制机械臂运动到目标位置
            # arm.set_pose_target(target_pose2, end_effector_link)
            # arm.go()
            # print('target2')
            # rospy.sleep(2)
            #
            # # 控制机械臂回到初始化位置
            # arm.set_named_target('zero')
            # arm.go()
            
            # 关闭并退出moveit
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)
            catch_flag = False
        else:
            rospy.is_shutdown

if __name__ == "__main__":
    try:
        moveit_grab()
    except KeyboardInterrupt:
        raise
