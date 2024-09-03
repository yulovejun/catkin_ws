#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
import rospy
from std_msgs.msg import String, Bool,Empty

if __name__ == '__main__':
    # 初始化ROS节点，节点名为“物体选择”
    rospy.init_node('object_selection', anonymous=True)
    # 创建一个发布者，发布到'/choice_object'主题，消息类型为std_msgs/String
    pub = rospy.Publisher('/choice_object', String, queue_size=1)
    rospy.sleep(2) # 给ROS系统一点时间来建立连接
    rate = rospy.Rate(5)
    obj = String()

    obj.data = 'bottle'  # 设置要抓取的物体名称

    print("**Publishing object selection: {}".format(obj.data))
    while not rospy.is_shutdown():
    # 发布消息
        pub.publish(obj)
        rate.sleep()
    #rospy.sleep(0.5)
    #rospy.spin()
