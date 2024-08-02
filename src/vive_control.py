#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import tf_transformations
import math
from vive_ros2.msg import VRControllerData

def deg2rad(deg):
    return round(deg * np.pi / 180.0, 3)

class ViveControl(Node):
    def __init__(self):
        super().__init__('vive_control')
        self.subscription = self.create_subscription(
            TransformStamped,
            'vive_pose_rel',
            self.tf_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.controller_subscription = self.create_subscription(
            VRControllerData,
            'controller_data',
            self.controller_callback,
            10
        )

        self.bot = InterbotixManipulatorXS(
            robot_model='wx250s',
            group_name='arm',
            gripper_name='gripper',
        )

        robot_startup()

        if self.bot.arm.group_info.num_joints < 5:
            self.get_logger().fatal(
                'This demo requires the robot to have at least 5 joints!'
            )
            robot_shutdown()
            sys.exit()

        self.bot.arm.set_trajectory_time(moving_time=2)
        self.bot.arm.go_to_sleep_pose(moving_time=2, blocking=True)
        self.bot.arm.go_to_home_pose(moving_time=2, blocking=True)
        self.bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.3, moving_time=2, blocking=True)

        self.grip_button_pressed = False

        self.current_x = 0.3
        self.current_y = 0
        self.current_z = 0.3
        self.current_roll = 0
        self.current_pitch = 0
        self.current_yaw = 0

    def tf_callback(self, msg):
        if not self.grip_button_pressed:
            return

        delta_x = msg.transform.translation.x
        delta_y = msg.transform.translation.y
        delta_z = msg.transform.translation.z

        # Convert the quaternion to Euler angles
        qx = msg.transform.rotation.x
        qy = msg.transform.rotation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w

        euler = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
        delta_roll, delta_pitch, delta_yaw = euler
        
        # Calculate new EE pose
        new_x = self.current_x + delta_x
        new_y = self.current_y + delta_y
        new_z = self.current_z + delta_z
        new_roll = self.current_roll + delta_roll
        new_pitch = self.current_pitch + delta_pitch
        new_yaw = self.current_yaw + delta_yaw

        self.get_logger().info(f'New Pose - x: {new_x:.3f}, y: {new_y:.3f}, z: {new_z:.3f}, roll: {math.degrees(new_roll):.3f}, pitch: {math.degrees(new_pitch):.3f}, yaw: {math.degrees(new_yaw):.3f}')

        # Control the robot arm
        self.bot.arm.set_ee_pose_components(x=new_x, y=new_y, z=new_z, roll=new_roll, pitch=new_pitch, yaw=new_yaw, blocking=True, moving_time=0.03, accel_time=0.01)

    def register_pose(self):
        current_pose_matrix = self.bot.arm.get_ee_pose()

        # Extract position from the transformation matrix
        self.current_x = current_pose_matrix[0, 3]
        self.current_y = current_pose_matrix[1, 3]
        self.current_z = current_pose_matrix[2, 3]

        # Extract orientation (rotation matrix) and convert to Euler angles
        current_rotation_matrix = current_pose_matrix[:3, :3]
        current_euler = tf_transformations.euler_from_matrix(current_rotation_matrix)
        self.current_roll, self.current_pitch, self.current_yaw = current_euler

        self.get_logger().info(f'Registered Pose - x: {self.current_x:.3f}, y: {self.current_y:.3f}, z: {self.current_z:.3f}, roll: {math.degrees(self.current_roll):.3f}, pitch: {math.degrees(self.current_pitch):.3f}, yaw: {math.degrees(self.current_yaw):.3f}')

    def controller_callback(self, msg):
        # self.get_logger().info(f'Grip Button: {msg.grip_button}')
        if msg.grip_button == 1 and not self.grip_button_pressed:
            self.get_logger().info('Grip Button Pressed')
            self.grip_button_pressed = True
            self.register_pose()
        elif msg.grip_button == 0 and self.grip_button_pressed:
            self.get_logger().info('Grip Button Released')
            self.grip_button_pressed = False
        if msg.trackpad_button == 1:
            self.get_logger().info('Trackpad Button Pressed!')
            # self.bot.arm.go_to_home_pose(moving_time=1, blocking=True)
            self.bot.arm.set_trajectory_time(moving_time=1)
            self.bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.2, moving_time=1, blocking=True)
            self.grip_button_pressed = False
        if msg.trigger_button == 1:
            self.get_logger().info('Trigger Button Pressed')
            self.bot.gripper.grasp()
        else:
            self.get_logger().info('Trigger Button Released')
            self.bot.gripper.release()
        
        # # Extract pose data from VRControllerData message
        # new_x = msg.abs_pose.transform.translation.x
        # new_y = msg.abs_pose.transform.translation.y
        # new_z = msg.abs_pose.transform.translation.z

        # # Extract orientation (quaternion) and convert to Euler angles
        # new_qx = msg.abs_pose.transform.rotation.x
        # new_qy = msg.abs_pose.transform.rotation.y
        # new_qz = msg.abs_pose.transform.rotation.z
        # new_qw = msg.abs_pose.transform.rotation.w

        # # Convert quaternion to Euler angles
        # new_roll, new_pitch, new_yaw = tf_transformations.euler_from_quaternion([new_qx, new_qy, new_qz, new_qw])
        # self.get_logger().info(f'New Pose - x: {new_x:.3f}, y: {new_y:.3f}, z: {new_z:.3f}, roll: {math.degrees(new_roll):.3f}, pitch: {math.degrees(new_pitch):.3f}, yaw: {math.degrees(new_yaw):.3f}')


def main(args=None):
    rclpy.init(args=args)
    vive_control = ViveControl()
    rclpy.spin(vive_control)
    vive_control.destroy_node()
    rclpy.shutdown()
    robot_shutdown()

if __name__ == '__main__':
    main()
