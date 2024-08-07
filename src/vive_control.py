#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import tf_transformations
import math
import copy
from vive_ros2.msg import VRControllerData

def deg2rad(deg):
    return round(deg * np.pi / 180.0, 3)

class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class ViveControl(Node):
    def __init__(self):
        super().__init__('vive_control')

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
        # self.bot.arm.go_to_home_pose(moving_time=2, blocking=True)
        self.initial_pose = Pose(x=0.25, y=0, z=0.15, roll=0, pitch=math.radians(35), yaw=0)
        self.current_pose = copy.deepcopy(self.initial_pose)
        self.bot.arm.set_ee_pose_components(x=self.initial_pose.x, y=self.initial_pose.y, z=self.initial_pose.z, roll=self.initial_pose.roll, pitch=self.initial_pose.pitch, yaw=self.initial_pose.yaw, moving_time=2, blocking=True)

        self.grip_button_pressed = False



    def register_pose(self):
        current_pose_matrix = self.bot.arm.get_ee_pose()

        # Extract position from the transformation matrix
        self.current_pose.x = current_pose_matrix[0, 3]
        self.current_pose.y = current_pose_matrix[1, 3]
        self.current_pose.z = current_pose_matrix[2, 3]

        # Extract orientation (rotation matrix) and convert to Euler angles
        current_rotation_matrix = current_pose_matrix[:3, :3]
        current_euler = tf_transformations.euler_from_matrix(current_rotation_matrix)
        self.current_pose.roll, self.current_pose.pitch, self.current_pose.yaw = current_euler

        self.get_logger().info(f'Registered Pose - x: {self.current_pose.x:.3f}, y: {self.current_pose.y:.3f}, z: {self.current_pose.z:.3f}, roll: {math.degrees(self.current_pose.roll)}, pitch: {math.degrees(self.current_pose.pitch)}, yaw: {math.degrees(self.current_pose.yaw)}')

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
            print('Initial Pose:', self.initial_pose.x, self.initial_pose.y, self.initial_pose.z, self.initial_pose.roll, self.initial_pose.pitch, self.initial_pose.yaw)
            self.bot.arm.set_ee_pose_components(x=self.initial_pose.x, y=self.initial_pose.y, z=self.initial_pose.z, roll=self.initial_pose.roll, pitch=self.initial_pose.pitch, yaw=self.initial_pose.yaw, moving_time=1, blocking=True)
            self.grip_button_pressed = False
        if msg.trigger_button == 1:
            self.get_logger().info('Trigger Button Pressed')
            self.bot.gripper.grasp()
        else:
            self.get_logger().info('Trigger Button Released')
            self.bot.gripper.release()
        
        if self.grip_button_pressed:
            # Extract relative pose data from VRControllerData message
            delta_x = -msg.rel_pose.transform.translation.z
            delta_y = -msg.rel_pose.transform.translation.x
            delta_z = msg.rel_pose.transform.translation.y
            qx = -msg.rel_pose.transform.rotation.z
            qy = -msg.rel_pose.transform.rotation.x
            qz = msg.rel_pose.transform.rotation.y
            qw = msg.rel_pose.transform.rotation.w
            euler = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
            delta_roll, delta_pitch, delta_yaw = euler

            # Calculate new EE pose
            new_x = self.current_pose.x + delta_x
            new_y = self.current_pose.y + delta_y
            new_z = self.current_pose.z + delta_z
            new_roll = self.current_pose.roll + delta_roll
            new_pitch = self.current_pose.pitch + delta_pitch
            new_yaw = self.current_pose.yaw + delta_yaw

            self.get_logger().info(f'New Pose - x: {new_x:.3f}, y: {new_y:.3f}, z: {new_z:.3f}, roll: {math.degrees(new_roll):.3f}, pitch: {math.degrees(new_pitch):.3f}, yaw: {math.degrees(new_yaw):.3f}')
            new_z = max(new_z, 0.06)
            self.bot.arm.set_ee_pose_components(x=new_x, y=new_y, z=new_z, pitch=self.current_pose.pitch, blocking=True, moving_time=0.02, accel_time=0.01)


def main(args=None):
    rclpy.init(args=args)
    vive_control = ViveControl()
    rclpy.spin(vive_control)
    vive_control.destroy_node()
    rclpy.shutdown()
    robot_shutdown()

if __name__ == '__main__':
    main()
