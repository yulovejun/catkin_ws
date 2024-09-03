#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from std_msgs.msg import String, Bool,Empty
from rm_msgs.msg import MoveJ_P,Arm_Current_State,Gripper_Set, Gripper_Pick,ArmState,MoveL,MoveJ
import numpy as np
from scipy.spatial.transform import Rotation as R
from vi_grab.msg import ObjectInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import OrientationConstraint, CollisionObject
import geometry_msgs.msg
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import AttachedCollisionObject


def add_collision_objects(planning_scene_interface):
    table_id = 'table1'
    box1_id = 'object'
    box2_id = 'beaker'
    planning_scene_interface.remove_world_object(table_id)
    planning_scene_interface.remove_world_object(box1_id)
    planning_scene_interface.remove_world_object(box2_id) 
    # 创建桌子
    table = CollisionObject()
    table.id = "table1"
    table.header.frame_id = "base_link"
    table_primitive = SolidPrimitive()
    table_primitive.type = SolidPrimitive.BOX
    table_primitive.dimensions = [0.6, 0.8, 0.4]
    table.primitives.append(table_primitive)
    table_pose = Pose()
    table_pose.position.x = 0.45
    table_pose.position.y = 0
    table_pose.position.z = 0.2
    table.primitive_poses.append(table_pose)
    table.operation = table.ADD

    #创建水瓶
    cylinder = CollisionObject()
    cylinder.id = "object"
    cylinder.header.frame_id = "base_link"
    cylinder_primitive = SolidPrimitive()
    cylinder_primitive.type = SolidPrimitive.CYLINDER
    cylinder_primitive.dimensions = [0.2, 0.04]  # Height,Radius
    cylinder.primitives.append(cylinder_primitive)
    cylinder_pose = Pose()
    cylinder_pose.position.x = 0.34
    cylinder_pose.position.y = -0.2
    cylinder_pose.position.z = 0.5
    cylinder.primitive_poses.append(cylinder_pose)
    cylinder.operation = cylinder.ADD


    #创建烧杯
    beaker = CollisionObject()
    beaker.id = "beaker"
    beaker.header.frame_id = "base_link"
    beaker_primitive = SolidPrimitive()
    beaker_primitive.type = SolidPrimitive.CYLINDER
    beaker_primitive.dimensions = [0.1, 0.1]  # 高度,半径
    beaker.primitives.append(beaker_primitive)
    beaker_pose = Pose()
    beaker_pose.position.x = 0.4
    beaker_pose.position.y = 0.2
    beaker_pose.position.z = 0.45
    beaker.primitive_poses.append(beaker_pose)
    beaker.operation = beaker.ADD
    planning_scene_interface.add_object(table)
    planning_scene_interface.add_object(cylinder)
    planning_scene_interface.add_object(beaker)
   
    rospy.sleep(1)

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("rm65_pick_place")
    moveit_commander.roscpp_initialize(sys.argv)

    # 创建运动规划的情景，等待创建完成
    planning_scene_interface = PlanningSceneInterface()
    # 初始化需要使用move group控制的机械臂中的arm group
    group = moveit_commander.MoveGroupCommander("arm")
    robot = moveit_commander.RobotCommander()
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.set_goal_position_tolerance(0.02)
    group.set_goal_orientation_tolerance(0.05)
    rospy.sleep(2.0)

    # 在场景中创建相关物体
    add_collision_objects(planning_scene_interface)  

    # 等待一段时间来初始化ROS
    rospy.sleep(1.0)
    #水瓶所在点，控制机械臂运动规划到此处
    target_pose1 = geometry_msgs.msg.Pose()
    target_pose1.orientation.w = 1.0
    # target_pose1.orientation.x = 1.42057e-05
    # target_pose1.orientation.y = 0.711007
    # target_pose1.orientation.z = -1.912e-05
    target_pose1.position.x = -0.34
    target_pose1.position.y = -0.2
    target_pose1.position.z = 0.478058

    # Set the pose target for the group
    group.set_pose_target(target_pose1)

    # Plan the motion
    my_plan = group.go()
    eef_link = group.get_end_effector_link()
    # Visualize the plan
    if my_plan:
        rospy.loginfo("Visualizing plan 1 (pose goal)")
        my_plan
        rospy.sleep(1.0)
        object_shape = SolidPrimitive()
        object_shape.type = SolidPrimitive.CYLINDER
        object_shape.dimensions = [0.1, 0.1]  # 比如，圆柱体直径为0.1米，高度0.1米

        # 定义物体的位姿
        object_pose = Pose()
        object_pose.position = Point(0.0, 0.0, 0.05)  # 与末端链接的相对位置
        object_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # 朝向

        # 创建 CollisionObject 对象并将其添加到规划场景中
        object_id = "attached_object"
        collision_object = CollisionObject(header="Link6", id=object_id, primitives=[object_shape], primitive_poses=[object_pose], operation=CollisionObject.ADD)
        planning_scene_interface.addCollisionObjects([collision_object])

        # 创建 AttachedCollisionObject 对象，并指定要附着的链接
        attached_object = AttachedCollisionObject(link_name="Link6", object=collision_object)

        # 将附着对象添加到规划场景中
        planning_scene_interface.addAttachedObject(attached_object)
        # box_pose = geometry_msgs.msg.PoseStamped()
        # box_pose.header.frame_id = "Link6"
        # box_pose.pose.orientation.w = 1.0
        # box_name = "box"
        # planning_scene_interface.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        # planning_scene_interface.attach_box(eef_link, box_name, touch_links=eef_link)

        # planning_scene_interface.remove_attached_object(eef_link, name=box_name)

        # # 删去水瓶原来位置
        # planning_scene_interface.remove_world_object(box_name)


        # # 烧杯固定位置
        target_pose2 = geometry_msgs.msg.Pose()
        target_pose2.orientation.w = 0.501046
        # target_pose2.orientation.x = -0.49893
        # target_pose2.orientation.y = 0.49938
        # target_pose2.orientation.z = 0.500641
        target_pose2.position.x = -0.034
        target_pose2.position.y = -0.2
        target_pose2.position.z = 0.504879
        group.set_pose_target(target_pose2)
        group.go()
        moveit_commander.roscpp_shutdown()
