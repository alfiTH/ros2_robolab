#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('joystick_topic_node')

        # Initialize PyGame and the joystick
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # Publishers
        self.sync_command_pub = self.create_publisher(String, '/sync_command', 10)
        self.speech_command_pub = self.create_publisher(String, '/speech_command', 10)

        # Timer for reading the controller state
        self.timer = self.create_timer(0.05, self.read_controller)  # 10 Hz

        # Variables to track button states
        self.prev_view_button = False
        self.prev_turbo_button = False
        self.prev_menu_button = False
        self.prev_dpad_up = False
        self.prev_dpad_down = False
        self.prev_dpad_left = False
        self.prev_dpad_right = False

        self.last_topic_published = None

    def read_controller(self):
        pygame.event.pump()  # Process PyGame events

        # Get button states
        view_button = self.joystick.get_button(6)  # View button
        turbo_button = self.joystick.get_button(7)  # Turbo button
        menu_button = self.joystick.get_button(8)  # Menu button
        dpad_up = self.joystick.get_hat(0)[1] == 1  # D-pad up
        dpad_down = self.joystick.get_hat(0)[1] == -1  # D-pad down
        dpad_left = self.joystick.get_hat(0)[0] == -1  # D-pad left
        dpad_right = self.joystick.get_hat(0)[0] == 1  # D-pad right

        # Publish sync commands
        # if view_button and not self.prev_view_button:
        #     self.publish_sync_command("goal")
        # if turbo_button and not self.prev_turbo_button:
        #     self.publish_sync_command("start")
        # if menu_button and not self.prev_menu_button:
        #     self.publish_sync_command("discard")

        # Publish sync commands
        if view_button and not self.prev_view_button:
            if self.last_topic_published == "goal":
                self.get_logger().info(f"Last sent command is the same. Start before stopping again.")
            else:
                self.publish_sync_command("goal")
        if turbo_button and not self.prev_turbo_button:
            if self.last_topic_published == "start":
                self.get_logger().info(f"Last sent command is the same. Stop before start again.")
            else:
                self.publish_sync_command("start")
        if menu_button and not self.prev_menu_button:
            if self.last_topic_published != "start":
                self.get_logger().info(f"Nothing to discard for now.")
            else:
                self.publish_sync_command("discard")

        # Publish speech commands
        if dpad_up and not self.prev_dpad_up:
            self.publish_speech_command("Excuse me")
        if dpad_down and not self.prev_dpad_down:
            self.publish_speech_command("Please give way")
        if dpad_left and not self.prev_dpad_left:
            self.publish_speech_command("Attention, there’s a robot here")
        if dpad_right and not self.prev_dpad_right:
            self.publish_speech_command("Attention, there’s a robot here")

        # Update previous button states
        self.prev_view_button = view_button
        self.prev_turbo_button = turbo_button
        self.prev_menu_button = menu_button
        self.prev_dpad_up = dpad_up
        self.prev_dpad_down = dpad_down
        self.prev_dpad_left = dpad_left
        self.prev_dpad_right = dpad_right

    def publish_sync_command(self, command):
        msg = String()
        msg.data = command
        self.sync_command_pub.publish(msg)
        self.get_logger().info(f"Published to /sync_command: {command}")
        self.last_topic_published = command

    def publish_speech_command(self, command):
        msg = String()
        msg.data = command
        self.speech_command_pub.publish(msg)
        self.get_logger().info(f"Published to /speech_command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()