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
            arm_status_message = arm_subscriber.get()

            if not arm_status_message.left_arm_base_remote_loss_of_signal:
                left_arm_base_remote_loss_of_signal = Fault()
                left_arm_base_remote_loss_of_signal.code = "Left Arm Base Remote Loss of Signal"
                left_arm_base_remote_loss_of_signal.priority = 1
                if left_arm_base_remote_loss_of_signal not in fault_list:
                    fault_list.append(left_arm_base_remote_loss_of_signal)
            
            if not arm_status_message.right_arm_base_remote_loss_of_signal:
                right_arm_base_remote_loss_of_signal = Fault()
                right_arm_base_remote_loss_of_signal.code = "Rightt Arm Base Remote Loss of Signal"
                right_arm_base_remote_loss_of_signal.priority = 1
                if right_arm_base_remote_loss_of_signal not in fault_list:
                    fault_list.append(right_arm_base_remote_loss_of_signal)
           
            
            if not arm_status_message.left_arm_base_reset_during_en:
                left_arm_base_reset_during_en = Fault()
                left_arm_base_reset_during_en.code = "Left Arm Base Reset During En"
                left_arm_base_reset_during_en.priority = 1
                if left_arm_base_reset_during_en not in fault_list:
                    fault_list.append(left_arm_base_reset_during_en)
            
            if not arm_status_message.right_arm_base_reset_during_en:
                right_arm_base_reset_during_en = Fault()
                right_arm_base_reset_during_en.code = "Right Arm Base Reset During En"
                right_arm_base_reset_during_en.priority = 1
                if right_arm_base_reset_during_en not in fault_list:
                    fault_list.append(right_arm_base_reset_during_en)

            if not arm_status_message.left_arm_base_hardware_ESD_reset:
                left_arm_base_hardware_ESD_reset = Fault()
                left_arm_base_hardware_ESD_reset.code = "Left Arm Base Hardware ESD Reset"
                left_arm_base_hardware_ESD_reset.priority = 1
                if left_arm_base_hardware_ESD_reset not in fault_list:
                    fault_list.append(left_arm_base_hardware_ESD_reset)

            if not arm_status_message.right_arm_base_hardware_ESD_reset:
                right_arm_base_hardware_ESD_reset = Fault()
                right_arm_base_hardware_ESD_reset.code = "Right Arm Base Hardware ESD Reset"
                right_arm_base_hardware_ESD_reset.priority = 1
                if right_arm_base_hardware_ESD_reset not in fault_list:
                    fault_list.append(right_arm_base_hardware_ESD_reset)

            if not arm_status_message.left_arm_base_supply_unstable:
                left_arm_base_supply_unstable = Fault()
                left_arm_base_supply_unstable.code = "Left Arm Base Supply Unstable"
                left_arm_base_supply_unstable.priority = 2
                if left_arm_base_supply_unstable not in fault_list:
                    fault_list.append(left_arm_base_supply_unstable)

            if not arm_status_message.right_arm_base_supply_unstable:
                right_arm_base_supply_unstable = Fault()
                right_arm_base_supply_unstable.code = "Right Arm Base Supply Unstable"
                right_arm_base_supply_unstable.priority = 2
                if right_arm_base_supply_unstable not in fault_list:
                    fault_list.append(right_arm_base_supply_unstable)
            
          

            

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