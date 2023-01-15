#!/usr/bin/env python3

import tf2_ros
import rospy
from threading import Thread

from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Fault, Health_Monitor_Control, Health_Monitor_Status

def ros_func():

    control_sub = BufferedROSMsgHandlerPy(health_monitor_node)
    control_sub.register_for_updates("healthmonitorcontrol")
    status_pub = rospy.Publisher(name="HealthMonitorStatus", data_class=Arm_Status, queue_size=50, tcp_nodelay=True)

    rate = rospy.Rate(20)
    rospy.loginfo("Health monitoring")
    faultlist = []
    while not rospy.is_shutdown():
        faults = control_sub.get()
        
        if control_sub.get() is not None:
            if control_sub.get.faults not in faultlist:
                faultlist.append(control_sub.get.faults)


        pubmsg = Health_Monitor_Status()
        pubmsg = faultlist.sort(reverse = True, key = ['priority'])

        status_pub.publish(pubmsg)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)