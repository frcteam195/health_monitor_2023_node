#!/usr/bin/env python3

from threading import Thread
import rospy

from ck_ros_msgs_node.msg import Arm_Status, Fault, Health_Monitor_Control, Health_Monitor_Status, Intake_Status
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import BufferedROSMsgHandlerPy

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

        self.status_publisher = rospy.Publisher(name="HealthMonitorStatus", data_class=Health_Monitor_Status, queue_size=50, tcp_nodelay=True)

        t1 = Thread(target=self.loop)
        t1.start()

        rospy.spin()

        t1.join(5)


    def loop(self) -> None:
        """
        Periodic function for the health monitor node.
        """

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.control_subscriber.get() is not None:
                if self.control_subscriber.get().acknowledge:
                    self.fault_list.clear()

            if self.arm_subscriber.get() is not None:
                arm_status_message = self.arm_subscriber.get()

                if not arm_status_message.left_arm_base_remote_loss_of_signal:
                    left_arm_base_remote_loss_of_signal = Fault()
                    left_arm_base_remote_loss_of_signal.code = "Left Arm Base Remote Loss of Signal"
                    left_arm_base_remote_loss_of_signal.priority = 1
                    if left_arm_base_remote_loss_of_signal not in self.fault_list:
                        self.fault_list.append(left_arm_base_remote_loss_of_signal)

                if not arm_status_message.right_arm_base_remote_loss_of_signal:
                    right_arm_base_remote_loss_of_signal = Fault()
                    right_arm_base_remote_loss_of_signal.code = "Rightt Arm Base Remote Loss of Signal"
                    right_arm_base_remote_loss_of_signal.priority = 1
                    if right_arm_base_remote_loss_of_signal not in self.fault_list:
                        self.fault_list.append(right_arm_base_remote_loss_of_signal)

                if not arm_status_message.left_arm_base_reset_during_en:
                    left_arm_base_reset_during_en = Fault()
                    left_arm_base_reset_during_en.code = "Left Arm Base Reset During En"
                    left_arm_base_reset_during_en.priority = 1
                    if left_arm_base_reset_during_en not in self.fault_list:
                        self.fault_list.append(left_arm_base_reset_during_en)

                if not arm_status_message.right_arm_base_reset_during_en:
                    right_arm_base_reset_during_en = Fault()
                    right_arm_base_reset_during_en.code = "Right Arm Base Reset During En"
                    right_arm_base_reset_during_en.priority = 1
                    if right_arm_base_reset_during_en not in self.fault_list:
                        self.fault_list.append(right_arm_base_reset_during_en)

                if not arm_status_message.left_arm_base_hardware_ESD_reset:
                    left_arm_base_hardware_ESD_reset = Fault()
                    left_arm_base_hardware_ESD_reset.code = "Left Arm Base Hardware ESD Reset"
                    left_arm_base_hardware_ESD_reset.priority = 1
                    if left_arm_base_hardware_ESD_reset not in self.fault_list:
                        self.fault_list.append(left_arm_base_hardware_ESD_reset)

                if not arm_status_message.right_arm_base_hardware_ESD_reset:
                    right_arm_base_hardware_ESD_reset = Fault()
                    right_arm_base_hardware_ESD_reset.code = "Right Arm Base Hardware ESD Reset"
                    right_arm_base_hardware_ESD_reset.priority = 1
                    if right_arm_base_hardware_ESD_reset not in self.fault_list:
                        self.fault_list.append(right_arm_base_hardware_ESD_reset)

                if not arm_status_message.left_arm_base_supply_unstable:
                    left_arm_base_supply_unstable = Fault()
                    left_arm_base_supply_unstable.code = "Left Arm Base Supply Unstable"
                    left_arm_base_supply_unstable.priority = 2
                    if left_arm_base_supply_unstable not in self.fault_list:
                        self.fault_list.append(left_arm_base_supply_unstable)

                if not arm_status_message.right_arm_base_supply_unstable:
                    right_arm_base_supply_unstable = Fault()
                    right_arm_base_supply_unstable.code = "Right Arm Base Supply Unstable"
                    right_arm_base_supply_unstable.priority = 2
                    if right_arm_base_supply_unstable not in self.fault_list:
                        self.fault_list.append(right_arm_base_supply_unstable)

            if self.intake_subscriber.get() is not None:
                pass

            status = Health_Monitor_Status()
            status.faults = sorted(self.fault_list, key=lambda fault: fault.priority, reverse=True)
            self.status_publisher.publish(status)

            rate.sleep()
