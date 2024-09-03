/**
 * 该例程将订阅chatter话题，消息类型String
 */
 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vi_grab/ObjectInfo.h"
// 接收到订阅的消息后，会进入消息回调函数
bool data_received = false;
void chatterCallback(const vi_grab::ObjectInfo::ConstPtr& msg)
{
  // 将接收到的消息打印出来
  char * str1;
  str1 = "tv";
  if (0 == strcmp(str1, msg->object_class.c_str()))
  {
    ROS_INFO("I heard: [%s],[%f][%f][%f]", msg->object_class.c_str(),msg->x,msg->y,msg->z);
    data_received = true;
  }
  
}

int main(int argc, char **argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "listener");

  // 创建节点句柄
  ros::NodeHandle n;
  //此处是执行机械臂到拍照的姿态函数
  // 创建一个Subscriber，订阅名为chatter的topic，注册回调函数chatterCallback
  ros::Subscriber sub = n.subscribe("object_pose", 1, chatterCallback);


  //ros::Duration(1).sleep();  // 等待稳定数据到来，可以根据需要调整休眠时间
  ros::spinOnce();
  //接下来执行抓取

  return 0;
}