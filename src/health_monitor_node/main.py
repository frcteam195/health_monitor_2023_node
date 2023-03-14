#!/usr/bin/env python3

from threading import Thread
import rospy

from ck_ros_msgs_node.msg import Arm_Status, Fault, Health_Monitor_Control, Health_Monitor_Status, Intake_Status
from ck_ros_base_msgs_node.msg import Motor_Status, Motor_Info
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import BufferedROSMsgHandlerPy
from ck_utilities_py_node.rosparam_helper import load_parameter_class
import rosnode
from dataclasses import dataclass, field
from typing import List

@dataclass
class HealthMonitorParams():
    node_checklist : List[str] = field(default_factory=list) 
    motor_id_checklist : List[int] = field(default_factory=list) 

class HealthMonitorNode():
    """
    The health monitor node.
    """

    def __init__(self) -> None:

        self.fault_list = []

        self.control_subscriber = BufferedROSMsgHandlerPy(Health_Monitor_Control)
        self.control_subscriber.register_for_updates("HealthMonitorControl")

        self.arm_subscriber = BufferedROSMsgHandlerPy(Arm_Status)
        self.arm_subscriber.register_for_updates("ArmStatus")

        self.intake_subscriber = BufferedROSMsgHandlerPy(Intake_Status)
        self.intake_subscriber.register_for_updates("IntakeStatus")

        self.motor_status_subscriber = BufferedROSMsgHandlerPy(Motor_Status)
        self.motor_status_subscriber.register_for_updates("MotorStatus")

        self.status_publisher = rospy.Publisher(name="HealthMonitorStatus", data_class=Health_Monitor_Status, queue_size=50, tcp_nodelay=True)

        self.params = HealthMonitorParams()
        load_parameter_class(self.params)

        t1 = Thread(target=self.loop)
        t1.start()

        rospy.spin()

        t1.join(5)


    def loop(self) -> None:
        """
        Periodic function for the health monitor node.
        """

        rate = rospy.Rate(20)
        frame_counter = 0
        ros_fully_booted = False
        while not rospy.is_shutdown():
            if frame_counter % 10 == 0:
                ros_fully_booted = True
                node_list = rosnode.get_node_names()
                for s in self.params.node_checklist:
                    ros_fully_booted &= s in node_list

                motor_status : Motor_Status = self.motor_status_subscriber.get()
                if motor_status is not None:
                    motor_list : List[Motor_Info] = motor_status.motors
                    motor_id_list : List[int] = []

                    for m in motor_list:
                        motor_id_list.append(m.id)

                    for m_id in self.params.motor_id_checklist:
                        ros_fully_booted &= m_id in motor_id_list
                else:
                    ros_fully_booted = False

                frame_counter = 0
            frame_counter += 1

            if self.control_subscriber.get() is not None:
                if self.control_subscriber.get().acknowledge:
                    self.fault_list.clear()

            if self.arm_subscriber.get() is not None:
                arm_status_message : Arm_Status = self.arm_subscriber.get()
                ros_fully_booted &= arm_status_message.is_node_alive
                if arm_status_message.left_arm_base_remote_loss_of_signal:
                    left_arm_base_remote_loss_of_signal = Fault()
                    left_arm_base_remote_loss_of_signal.code = "Left Arm Base Remote Loss of Signal"
                    left_arm_base_remote_loss_of_signal.priority = 1
                    if left_arm_base_remote_loss_of_signal not in self.fault_list:
                        self.fault_list.append(left_arm_base_remote_loss_of_signal)

                if arm_status_message.right_arm_base_remote_loss_of_signal:
                    right_arm_base_remote_loss_of_signal = Fault()
                    right_arm_base_remote_loss_of_signal.code = "Right Arm Base Remote Loss of Signal"
                    right_arm_base_remote_loss_of_signal.priority = 1
                    if right_arm_base_remote_loss_of_signal not in self.fault_list:
                        self.fault_list.append(right_arm_base_remote_loss_of_signal)

                if arm_status_message.left_arm_base_reset_during_en:
                    left_arm_base_reset_during_en = Fault()
                    left_arm_base_reset_during_en.code = "Left Arm Base Reset During En"
                    left_arm_base_reset_during_en.priority = 1
                    if left_arm_base_reset_during_en not in self.fault_list:
                        self.fault_list.append(left_arm_base_reset_during_en)

                if arm_status_message.right_arm_base_reset_during_en:
                    right_arm_base_reset_during_en = Fault()
                    right_arm_base_reset_during_en.code = "Right Arm Base Reset During En"
                    right_arm_base_reset_during_en.priority = 1
                    if right_arm_base_reset_during_en not in self.fault_list:
                        self.fault_list.append(right_arm_base_reset_during_en)

                if arm_status_message.left_arm_base_hardware_ESD_reset:
                    left_arm_base_hardware_ESD_reset = Fault()
                    left_arm_base_hardware_ESD_reset.code = "Left Arm Base Hardware ESD Reset"
                    left_arm_base_hardware_ESD_reset.priority = 1
                    if left_arm_base_hardware_ESD_reset not in self.fault_list:
                        self.fault_list.append(left_arm_base_hardware_ESD_reset)

                if arm_status_message.right_arm_base_hardware_ESD_reset:
                    right_arm_base_hardware_ESD_reset = Fault()
                    right_arm_base_hardware_ESD_reset.code = "Right Arm Base Hardware ESD Reset"
                    right_arm_base_hardware_ESD_reset.priority = 1
                    if right_arm_base_hardware_ESD_reset not in self.fault_list:
                        self.fault_list.append(right_arm_base_hardware_ESD_reset)

                if arm_status_message.left_arm_base_supply_unstable:
                    left_arm_base_supply_unstable = Fault()
                    left_arm_base_supply_unstable.code = "Left Arm Base Supply Unstable"
                    left_arm_base_supply_unstable.priority = 2
                    if left_arm_base_supply_unstable not in self.fault_list:
                        self.fault_list.append(left_arm_base_supply_unstable)

                if arm_status_message.right_arm_base_supply_unstable:
                    right_arm_base_supply_unstable = Fault()
                    right_arm_base_supply_unstable.code = "Right Arm Base Supply Unstable"
                    right_arm_base_supply_unstable.priority = 2
                    if right_arm_base_supply_unstable not in self.fault_list:
                        self.fault_list.append(right_arm_base_supply_unstable)
            else:
                ros_fully_booted = False

            if self.intake_subscriber.get() is not None:
                intake_status : Intake_Status = self.intake_subscriber.get()
                ros_fully_booted &= intake_status.is_node_alive
                pass
            else:
                ros_fully_booted = False

            status = Health_Monitor_Status()
            status.faults = sorted(self.fault_list, key=lambda fault: fault.priority, reverse=True)
            status.is_ros_fully_booted = ros_fully_booted
            self.status_publisher.publish(status)

            rate.sleep()
