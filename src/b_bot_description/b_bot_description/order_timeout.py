#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import sys
import select
from tf_transformations import quaternion_from_euler


class b_bot(Node):
    def __init__(self):
        super().__init__('b_bot')

        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.declare_parameter('timeout_sec', 10)
        self.timeout = self.get_parameter('timeout_sec').value

        self.create_subscription(String, '/order', self.order_callback, 10)

        self.poses = self.define_waypoints()
        self.set_initial_pose()

        self.cancel_requested = False
        self.canceled_tables = set()
        self.missed_confirmation = False

        self.get_logger().info(" b_bot is ready for orders!")

      

    def define_waypoints(self):
        poses = {}

        def make_pose(x, y, z, yaw):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

        # Convert yaw (radians) to quaternion
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            return pose

        poses['home'] = make_pose(2.499, -0.008, 0.038716, -3.130)
        poses['kitchen'] = make_pose(0.039, -0.035, 0.038716, 3.130)
        poses['table1'] = make_pose(2.191, 1.622, 0.038716, 1.54)
        poses['table2'] = make_pose(1.17, 3.057, 0.038716, -3.128)
        poses['table3'] = make_pose(0.040, 1.707, 0.038716, 0.013)

        return poses


    def set_initial_pose(self):
        pose = self.poses['home']
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.nav.setInitialPose(pose)
        time.sleep(1)

    def order_callback(self, msg):
        data = msg.data.strip().lower()

        if data.startswith('cancel:'):
            table_to_cancel = data.split(':')[1].strip()
            self.get_logger().warn(f" Cancellation received for table: {table_to_cancel}")
            self.canceled_tables.add(table_to_cancel)
            return

        if data == 'cancel':
            self.get_logger().warn(" Cancel order received!")
            self.cancel_requested = True
            self.nav.cancelTask()
            return

        table_names = [name.strip().lower() for name in data.split(',')]
        self.get_logger().info(f"\n New Order Received: Deliver to {', '.join(table_names)}")

        valid_table_names = [name for name in table_names if name in self.poses and name not in self.canceled_tables]
        for table_name in table_names:
            if table_name not in self.poses:
                self.get_logger().warn(f" Invalid table name: {table_name}")
            elif table_name in self.canceled_tables:
                self.get_logger().warn(f" Skipping canceled table: {table_name}")

        self.run_delivery(valid_table_names)

    def confirm_interactively(self, step: str) -> bool:
        self.get_logger().info(f"\n Confirm '{step}'? Type 'yes' or 'no' within {self.timeout} seconds:")
        print(f"[{step.capitalize()}] > ", end='', flush=True)
        rlist, _, _ = select.select([sys.stdin], [], [], self.timeout)

        if rlist:
            user_input = sys.stdin.readline().strip().lower()
            if user_input == 'yes':
                self.get_logger().info(f" {step.capitalize()} confirmed.")
                return True
            elif user_input == 'no':
                self.get_logger().warn(f" {step.capitalize()} canceled.")
                return False
            else:
                self.get_logger().warn(f" Invalid input '{user_input}'. Treating as cancellation.")
                return False
        else:
            self.get_logger().warn(f" {step.capitalize()} confirmation timed out (no input).")
            return False

    def go_to(self, pose_name):
        self.cancel_requested = False
        pose = self.poses[pose_name]
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.nav.goToPose(pose)

        print(f"Going to {pose_name}... Type 'c' to cancel, 'c1', 'c2', 'c3' to skip tables.")

        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.5)

            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                user_input = sys.stdin.readline().strip().lower()
                if user_input == 'c':
                    self.get_logger().warn(f"Navigation to {pose_name} canceled by user.")
                    self.cancel_requested = True
                    self.nav.cancelTask()
                    if pose_name == 'kitchen':
                        self.go_to('home')
                    else:
                        self.go_to('kitchen')
                        self.go_to('home')
                    return False
                elif user_input in ['c1', 'c2', 'c3']:
                    table_map = {'c1': 'table1', 'c2': 'table2', 'c3': 'table3'}
                    table_to_cancel = table_map[user_input]
                    self.get_logger().warn(f"Cancel received: Skipping {table_to_cancel}")
                    self.canceled_tables.add(table_to_cancel)

            feedback = self.nav.getFeedback()
            if feedback:
                self.get_logger().info(f"Moving to {pose_name}...")

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached {pose_name}.")
            return True
        else:
            self.get_logger().error(f"Failed to reach {pose_name}.")
            return False

    def run_delivery(self, table_names):
        self.missed_confirmation = False
        canceled_during_delivery = False

        if self.cancel_requested:
            self.get_logger().warn(" Order was canceled before starting delivery.")
            self.go_to('home')
            return

        if not self.go_to('kitchen'):
            self.get_logger().warn(" Couldn't reach kitchen. Returning to home.")
            self.go_to('home')
            return

        if self.cancel_requested:
            self.get_logger().warn(" Order was canceled after reaching kitchen.")
            self.go_to('home')
            return

        if not self.confirm_interactively('pickup'):
            self.get_logger().warn(" Pickup not confirmed. Returning to home.")
            self.go_to('home')
            return

        delivery_success = False

        for table_name in table_names:
            if table_name in self.canceled_tables:
                self.get_logger().info(f" Skipping canceled table {table_name}.")
                canceled_during_delivery = True
                continue

            if not self.confirm_delivery_to_table(table_name):
                canceled_during_delivery = True
                continue

            delivery_success = True

        if canceled_during_delivery or self.missed_confirmation:
            self.get_logger().info(" Some deliveries were canceled or missed. Returning to kitchen.")
            self.go_to('kitchen')

        self.get_logger().info(" Returning to home.")
        self.go_to('home')

    def confirm_delivery_to_table(self, table_name):
        if not self.go_to(table_name):
            self.get_logger().warn(f" Couldn't reach {table_name}. Skipping delivery.")
            return False

        if not self.confirm_interactively(f'delivery to {table_name}'):
            self.get_logger().warn(f" Delivery to {table_name} not confirmed. Skipping.")
            self.missed_confirmation = True
            return False

        self.get_logger().info(f" Delivered to {table_name}.")
        return True


def main(args=None):
    rclpy.init(args=args)
    bot = b_bot()
    try:
        rclpy.spin(bot)
    except KeyboardInterrupt:
        bot.get_logger().info(" Shutdown requested.")
    finally:
        bot.nav.cancelTask()
        bot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()