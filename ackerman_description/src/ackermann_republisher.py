#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import os
import termios
import tty
from getkey import getkey, keys

# --- Configuration (Unchanged) ---
MAX_STEERING_ANGLE = 0.5
MAX_VELOCITY = 10.0
STEP_VELOCITY = 0.5
STEP_ANGLE = 0.05

# --- Instructions (Unchanged) ---
MSG = """
Teleop Dual Ackerman Robot 🤖
---------------------------
Steering (Position Command for Hinge):
    'a': Steer Left (Positive Angle)
    'd': Steer Right (Negative Angle)
    's': Center Steering (0.0 rad)

Driving (Velocity Command for Wheels):
    'w': Increase Forward Velocity
    'x': Increase Reverse Velocity
    'q': Emergency Stop / Zero Velocity (0.0 rad/s)

Quit:
    'e' or Ctrl+C to exit.

Current State:
    Steering Angle: {angle:.2f} rad
    Velocity: {vel:.2f} rad/s
"""

# --- Teleop Node ---

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.get_logger().info('Teleop Node Initialized.')

        # Publishers
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_position_controller/commands',
            10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )

        # State Variables
        self.current_steering = 0.0
        self.current_velocity = 0.0
        
        # Save original terminal settings
        self.settings = termios.tcgetattr(sys.stdin)


    def __del__(self):
        # Restore terminal settings on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.get_logger().info('Teleop Node Shutdown and terminal restored.')


    def print_status(self):
        """Clears the screen and prints the current status."""
        os.system('clear')
        print(MSG.format(angle=self.current_steering, vel=self.current_velocity))


    def publish_commands(self):
        """Publishes the current velocity and steering commands."""
        
        # 1. Steering Command (Position) - UPDATED TO SEND 4 VALUES
        # Array must be ordered according to your YAML: [LF, RF, RL, RR]
        steering_msg = Float64MultiArray()
        
        # LF and RF turn the same way (multiplier 1.0)
        # RL and RR turn the opposite way (multiplier -1.0)
        steering_msg.data = [
            self.current_steering,          # left_steering_hinge_joint (Master/LF)
            self.current_steering * 1.0,    # right_steering_hinge_joint (RF)
            self.current_steering * -1.0,   # rear_left_steering_hinge_joint (RL)
            self.current_steering * -1.0    # rear_right_steering_hinge_joint (RR)
        ] 
        self.steering_pub.publish(steering_msg)

        # 2. Velocity Command (All 4 wheels)
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [
            self.current_velocity, 
            self.current_velocity, 
            self.current_velocity, 
            self.current_velocity
        ]
        self.velocity_pub.publish(velocity_msg)


    def run(self):
        """The main blocking loop for reading keyboard input."""
        
        # Set terminal to cbreak mode (non-canonical, characters available immediately)
        tty.setcbreak(sys.stdin.fileno())
        self.print_status()

        try:
            while rclpy.ok():
                # Read a single key, blocking until a key is pressed
                key = getkey()
                
                # --- Steering Control (Position) ---
                if key == 'a': # Steer Left (Positive Angle)
                    self.current_steering = min(MAX_STEERING_ANGLE, self.current_steering + STEP_ANGLE)
                elif key == 'd': # Steer Right (Negative Angle)
                    self.current_steering = max(-MAX_STEERING_ANGLE, self.current_steering - STEP_ANGLE)
                elif key == 's': # Center Steering
                    self.current_steering = 0.0
                
                # --- Velocity Control ---
                elif key == 'w': # Forward
                    self.current_velocity = min(MAX_VELOCITY, self.current_velocity + STEP_VELOCITY)
                elif key == 'x': # Backward
                    self.current_velocity = max(-MAX_VELOCITY, self.current_velocity - STEP_VELOCITY)
                elif key == 'q': # Stop (Zero Velocity)
                    self.current_velocity = 0.0
                
                # --- Quit ---
                elif key == 'e' or key == keys.ETX: # 'e' or Ctrl+C
                    break

                # Publish commands and update status display
                self.publish_commands()
                self.print_status()
                
                # Allow ROS 2 to process callbacks
                rclpy.spin_once(self, timeout_sec=0)


        except Exception as e:
            self.get_logger().error(f"Error during teleop loop: {e}")
        
        finally:
            # Stop the robot on exit
            self.current_velocity = 0.0
            self.current_steering = 0.0
            self.publish_commands()
            
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info('Teleop finished.')
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    
    # Start the main teleop loop
    teleop_node.run()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()