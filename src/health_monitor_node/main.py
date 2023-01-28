#!/usr/bin/env python3

import rospy
from threading import Thread

from ck_ros_msgs_node.msg import Fault, Health_Monitor_Control, Health_Monitor_Status
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Arm_Status, Intake_Status


    
def ros_func():
    control_subscriber = BufferedROSMsgHandlerPy(Health_Monitor_Control)
    control_subscriber.register_for_updates("HealthMonitorControl")

    arm_subscriber = BufferedROSMsgHandlerPy(Arm_Status)
    arm_subscriber.register_for_updates("ArmStatus")

    intake_subscriber = BufferedROSMsgHandlerPy(Intake_Status)
    intake_subscriber.register_for_updates("IntakeStatus")

    status_publisher = rospy.Publisher(name="HealthMonitorStatus", data_class=Health_Monitor_Status, queue_size=50, tcp_nodelay=True)

    rate = rospy.Rate(20)

    global fault_list 
    fault_list = []

    while not rospy.is_shutdown():


        if control_subscriber.get() is not None:
            if control_subscriber.get().acknowledge:
                fault_list.clear()

        if arm_subscriber.get() is not None:
            pass

        if intake_subscriber.get() is not None:
            pass
        
        status = Health_Monitor_Status()
        status.faults = sorted(fault_list, key=lambda fault: fault.priority, reverse=True)
        status_publisher.publish(status)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    #register_for_robot_updates()

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)