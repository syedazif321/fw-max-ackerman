#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import os
import termios
import tty
from getkey import getkey, keys

# --- Configuration (UPDATED) ---
MAX_STEERING_ANGLE = 1.57  # Allowing 90 degrees
MAX_VELOCITY = 10.0
STEP_VELOCITY = 0.5
STEP_ANGLE = 0.05

# Define Steering Modes
MODE_NORMAL = 0
MODE_CRAB = 1
MODE_ROTATE = 2

# --- Instructions (UPDATED) ---
MSG = """
Teleop Dual Swerve/Crab Robot 🤖
---------------------------
Steering (Position Command for Hinge):
    'a': Steer Left (Positive Angle)
    'd': Steer Right (Negative Angle)
    's': Center Steering (0.0 rad)

Driving (Velocity Command for Wheels):
    'w': Increase Forward Velocity
    'x': Increase Reverse Velocity
    'q': Emergency Stop / Zero Velocity (0.0 rad/s)

Mode Control:
    'r': Cycle Steering Mode (Normal -> Crab -> Rotate)
    'e' or Ctrl+C to exit.

Current State:
    Mode: {mode_name}
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

        # State Variables (UPDATED)
        self.current_steering = 0.0
        self.current_velocity = 0.0
        self.current_mode = MODE_NORMAL  # Start in Normal mode
        
        # Save original terminal settings
        self.settings = termios.tcgetattr(sys.stdin)


    def __del__(self):
        # Restore terminal settings on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.get_logger().info('Teleop Node Shutdown and terminal restored.')


    def get_mode_name(self):
        """Helper to return the current mode name for display."""
        if self.current_mode == MODE_NORMAL:
            return "NORMAL (Front Steer/Ackerman)"
        elif self.current_mode == MODE_CRAB:
            return "CRAB (Sideways Motion)"
        elif self.current_mode == MODE_ROTATE:
            return "ROTATE (Spin In Place)"
        return "UNKNOWN"

    def print_status(self):
        """Clears the screen and prints the current status."""
        os.system('clear')
        mode_name = self.get_mode_name()
        print(MSG.format(mode_name=mode_name, angle=self.current_steering, vel=self.current_velocity))


    def publish_commands(self):
        """Publishes the current velocity and steering commands based on the current mode."""
        
        steering_msg = Float64MultiArray()
        
        # 1. Determine Steering Multipliers based on MODE
        if self.current_mode == MODE_NORMAL:
            # Dual-Ackermann (Front and Rear steer opposite, limited to <90 deg for stability)
            # We enforce a smaller limit here, but use the full MAX_STEERING_ANGLE for the other modes
            max_angle = min(MAX_STEERING_ANGLE, 0.78) # Limit to approx 45 degrees for normal
            
            # Clamp steering angle for this mode only
            steer_cmd = max(-max_angle, min(max_angle, self.current_steering))

            # LF, RF steer forward. RL, RR steer backward (or opposite direction)
            steering_msg.data = [
                steer_cmd,      # LF (+1.0)
                steer_cmd,      # RF (+1.0)
                -steer_cmd,     # RL (-1.0)
                -steer_cmd      # RR (-1.0)
            ]

        elif self.current_mode == MODE_CRAB:
            # All four wheels must point in the same direction (90 degrees for pure sideways)
            # The full MAX_STEERING_ANGLE (1.57 rad) is available.
            steer_cmd = self.current_steering
            steering_msg.data = [
                steer_cmd,      # LF (+1.0)
                steer_cmd,      # RF (+1.0)
                steer_cmd,      # RL (+1.0)
                steer_cmd       # RR (+1.0)
            ]

        elif self.current_mode == MODE_ROTATE:
            # Front wheels steer one way, rear wheels steer opposite (zero-radius turn)
            # The full MAX_STEERING_ANGLE (1.57 rad) is available.
            steer_cmd = self.current_steering
            steering_msg.data = [
                steer_cmd,      # LF (+1.0)
                steer_cmd,      # RF (+1.0)
                -steer_cmd,     # RL (-1.0)
                -steer_cmd      # RR (-1.0)
            ]

        self.steering_pub.publish(steering_msg)

        # 2. Velocity Command (All 4 wheels) - velocity is applied equally to all
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
        
        tty.setcbreak(sys.stdin.fileno())
        self.print_status()

        try:
            while rclpy.ok():
                key = getkey()
                
                # --- Steering Control ---
                if key == 'a': # Steer Left (Positive Angle)
                    # Use MAX_STEERING_ANGLE for all modes, the actual command is clamped in publish_commands
                    self.current_steering = min(MAX_STEERING_ANGLE, self.current_steering + STEP_ANGLE)
                elif key == 'd': # Steer Right (Negative Angle)
                    self.current_steering = max(-MAX_STEERING_ANGLE, self.current_steering - STEP_ANGLE)
                elif key == 's': # Center Steering
                    self.current_steering = 0.0
                
                # --- Mode Control (NEW) ---
                elif key == 'r': # Cycle Mode
                    self.current_mode = (self.current_mode + 1) % 3
                    self.current_steering = 0.0 # Reset steering angle on mode change
                    self.current_velocity = 0.0 # Stop movement on mode change

                # --- Velocity Control ---
                elif key == 'w': # Forward
                    self.current_velocity = min(MAX_VELOCITY, self.current_velocity + STEP_VELOCITY)
                elif key == 'x': # Backward
                    self.current_velocity = max(-MAX_VELOCITY, self.current_velocity - STEP_VELOCITY)
                elif key == 'q': # Stop (Zero Velocity)
                    self.current_velocity = 0.0
                
                # --- Quit ---
                elif key == 'e' or key == keys.ETX: 
                    break

                # Publish commands and update status display
                self.publish_commands()
                self.print_status()
                
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
    
    teleop_node.run()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()