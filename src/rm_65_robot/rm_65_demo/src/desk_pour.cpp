//
// Created by lamont on 23-11-9.
//
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <tf2/LinearMath/Quaternion.h>

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // 创建环境中的物体(一个桌子和一个目标物体)
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";

    // 设置物体table1的形状及相关尺寸
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.6;
    collision_objects[0].primitives[0].dimensions[1] = 0.6;
    collision_objects[0].primitives[0].dimensions[2] = 0.3;

    // 设置物体table1的位置
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.65;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;      

    collision_objects[0].operation = collision_objects[0].ADD;

    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "beaker";
    collision_objects[1].header.frame_id = "base_link";

    // 设置物体的形状及相关尺寸
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.2;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.1;

    // 设置物体的位置
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.45;
    collision_objects[1].primitive_poses[0].position.y = 0.3;
    collision_objects[1].primitive_poses[0].position.z = 0.3;

    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rm65_pour_water");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();

//    ros::WallDuration(1.0).sleep();

    //创建运动规划的情景，等待创建完成
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // 初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface group("arm");
    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.setGoalPositionTolerance(0.02);
    group.setGoalOrientationTolerance(0.05);
    sleep(5.0);

    // 在场景中创建相关物体
    addCollisionObjects(planning_scene_interface);

    // 等待一段时间来初始化ROS
    ros::WallDuration(1.0).sleep();

    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose1;

    target_pose1.orientation.w = 0.703184;
    target_pose1.orientation.x= 1.42057e-05;
    target_pose1.orientation.y = 0.711007;
    target_pose1.orientation.z = -1.912e-05;

    target_pose1.position.x = 0.406138;
    target_pose1.position.y = 0.00100714;
    target_pose1.position.z = 0.478058;
    group.setPoseTarget(target_pose1, "Link6");

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"target_pose1":"FAILED");

    //让机械臂按照规划的轨迹开始运动。
    if(success)
    {
        group.execute(my_plan);

        ros::WallDuration(1.0).sleep();

        // 定义附加对象消息
        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = "Link6";
        /* The header must contain a valid TF frame*/
        attached_object.object.header.frame_id = "Link6";
        /* The id of the object */
        attached_object.object.id = "box";

        /* A default pose */
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;

        /* 定义物体形状及尺寸大小 */
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1;
        primitive.dimensions[1] = 0.1;
        primitive.dimensions[2] = 0.1;

        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(pose);

        //附加对象需要有一个添加操作
        attached_object.object.operation = attached_object.object.ADD;

        //因为要将物体连接到机器人的手上以模拟拿起物体，所以设置碰撞检查器忽略物体和机器人手之间的碰撞
        attached_object.touch_links = std::vector<std::string>{ "Link4", "Link5", "Link6" };

        // 添加物体到场景中
        ROS_INFO("Adding the object into the world at the location of the hand.");
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);
        /**
         * 同步更新与异步更新
         * 1.一种方式是通过rosservice调用发送一个diff,直到diff被应用(同步机制)
         * 2.另一种方式是通过话题发送diff,无论是否应用都继续进行
         */
        ros::ServiceClient planning_scene_diff_client =
            nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();
        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
          /* First, define the REMOVE object message*/
        moveit_msgs::CollisionObject remove_object;
        remove_object.id = "box";
        remove_object.header.frame_id = "Link1";
        remove_object.operation = remove_object.REMOVE;

        ROS_INFO("Attaching the object to the hand and removing it from the world.");
        // 确定diff消息没有包含其他对象
        planning_scene.world.collision_objects.clear();
        planning_scene.world.collision_objects.push_back(remove_object);
        planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
        planning_scene_diff_publisher.publish(planning_scene);
//    ros::WallDuration(1.0).sleep();

        // 设置放置点的目标位置
        geometry_msgs::Pose target_pose2;


        target_pose2.position.x = 0.35;
        target_pose2.position.y = -0.2;
        target_pose2.position.z = 0.478058;

        group.setPoseTarget(target_pose2);

        // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
        moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
        moveit::planning_interface::MoveItErrorCode success2 = group.plan(my_plan2);

        ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"target_pose2":"FAILED");

        //让机械臂按照规划的轨迹开始运动。

    }

    ros::waitForShutdown();

    return 0;
}
