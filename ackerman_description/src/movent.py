#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math
import sys

# ==========================================================
# --- USER-ADJUSTABLE CONFIGURATION PARAMETERS ---
# ==========================================================

# ⚠️ CHANGE THESE TWO PARAMETERS TO ADJUST ALL MOTION ⚠️

# 1. SPEED CONTROL (Adjusts velocity for all maneuvers)
MAX_SPEED = 9.0      # Target velocity for full motion (rad/s). HIGH SPEED.

# 2. BASE DISTANCE/DURATION CONTROL (Used for all moving maneuvers)
BASE_MOVE_TIME = 6.0  # Time in seconds for velocity application (6.0s)

# 3. ROTATION DISTANCE/DURATION CONTROL (Reset to BASE_MOVE_TIME)
ROTATION_MOVE_TIME = BASE_MOVE_TIME 

# ==========================================================
# --- CONSTANTS & DERIVED PARAMETERS (UPDATED FOR FASTER TRANSITIONS) ---
# ==========================================================
NODE_NAME = 'demonstration_pipeline_node'
MAX_ANGLE = 1.57     # 90 degrees in radians
ACK_ANGLE = math.radians(20) # 20 degrees for Ackerman turn
CRAB_ANGLE = MAX_ANGLE
TURN_ANGLE = MAX_ANGLE

# NEW DIAGONAL ANGLE
DIAG_ANGLE = math.radians(45) # 45 degrees for diagonal movement

# Derived durations
# REDUCED TIME for snappier transitions
STEERING_STABILIZATION_TIME = 0.5 # Time to wait for steering to complete (Reduced from 1.0s)

# --- Command Definition (Steering Angles: [LF, RF, RL, RR]) ---

# Base Motion
CMD_STRAIGHT = ([0.0] * 4) # Zero steering angle
CMD_STOP_VEL = 0.0

# Rotation (Spin)
CMD_ROTATE_CW_STEER = ([TURN_ANGLE, TURN_ANGLE, -TURN_ANGLE, -TURN_ANGLE])
CMD_ROTATE_CCW_STEER = ([-TURN_ANGLE, -TURN_ANGLE, TURN_ANGLE, TURN_ANGLE])

# Crab Motion (Sideways)
CMD_CRAB_LEFT_STEER = ([CRAB_ANGLE] * 4)
CMD_CRAB_RIGHT_STEER = ([-CRAB_ANGLE] * 4)

# NEW: Diagonal/Intermediate Crab Motion (45 degrees, all wheels parallel)
CMD_DIAG_FWD_LEFT = ([DIAG_ANGLE] * 4)
CMD_DIAG_FWD_RIGHT = ([-DIAG_ANGLE] * 4)

# Ackerman-Style Turn (Rotation + Translation)
CMD_ACK_LEFT_STEER = ([ACK_ANGLE, ACK_ANGLE, -ACK_ANGLE, -ACK_ANGLE])
CMD_ACK_RIGHT_STEER = ([-ACK_ANGLE, -ACK_ANGLE, ACK_ANGLE, ACK_ANGLE])


class DemonstrationNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.get_logger().info('Demonstration Pipeline Initialized.')

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

    def send_velocity(self, velocity_value):
        """Publishes the current velocity command."""
        velocity_msg = Float64MultiArray()
        # Apply the same velocity to all 4 wheels
        velocity_msg.data = [velocity_value] * 4
        self.velocity_pub.publish(velocity_msg)

    def send_steering(self, steering_angles):
        """Publishes the current steering command."""
        steering_msg = Float64MultiArray()
        # Ensure the array has 4 values in the correct order: [LF, RF, RL, RR]
        steering_msg.data = steering_angles
        self.steering_pub.publish(steering_msg)

    def ramp_velocity(self, target_speed, duration):
        """Gradually ramps the velocity up to target_speed and down to 0."""

        # Determine the sign for ramping (forward or backward)
        direction = 1 if target_speed >= 0 else -1
        target_abs_speed = abs(target_speed)

        # Total duration split into ramp up, steady, and ramp down
        segment_time = duration / 3.0

        # 1. Ramp Up
        start_time = time.time()
        while time.time() - start_time < segment_time and rclpy.ok():
            current_speed = (time.time() - start_time) / segment_time * target_abs_speed
            self.send_velocity(current_speed * direction)
            time.sleep(0.05)
            rclpy.spin_once(self, timeout_sec=0)

        # 2. Steady State
        self.send_velocity(target_speed)
            # Use reduced wait time for centering
        time.sleep(segment_time)

        # 3. Ramp Down
        start_time = time.time()
        while time.time() - start_time < segment_time and rclpy.ok():
            current_speed = target_abs_speed - (time.time() - start_time) / segment_time * target_abs_speed
            self.send_velocity(max(0.0, current_speed) * direction)
            time.sleep(0.05)
            rclpy.spin_once(self, timeout_sec=0)

        # Ensure final stop
        self.send_velocity(CMD_STOP_VEL)
        rclpy.spin_once(self, timeout_sec=0)

    def execute_maneuver(self, description, steer_cmd, vel_cmd, duration, is_90_degree_maneuver=False):
        """
        Executes a maneuver. is_90_degree_maneuver uses the 3-step logic.
        Other maneuvers use the 2-step (steer + ramp) logic.
        """
        self.get_logger().info(f"--- Executing: {description} ---")

        if is_90_degree_maneuver:
            # 1. Steer Wheels to 90 degrees (Zero velocity)
            self.get_logger().info(f"   (STEP 1/3: Steering to {round(steer_cmd[0], 2)} rad, waiting {STEERING_STABILIZATION_TIME}s)")
            self.send_steering(steer_cmd)
            self.send_velocity(CMD_STOP_VEL)
            time.sleep(STEERING_STABILIZATION_TIME)
            rclpy.spin_once(self, timeout_sec=0)

            # 2. Move (Apply Ramped Velocity)
            self.get_logger().info(f"   (STEP 2/3: Applying ramped velocity of {vel_cmd} rad/s for {duration}s)")
            self.ramp_velocity(vel_cmd, duration)

            # 3. Return to Base State (Straight Steering and Stop)
            self.get_logger().info("   (STEP 3/3: Returning steering to center)")
            self.send_steering(CMD_STRAIGHT)
            time.sleep(STEERING_STABILIZATION_TIME / 2.0)
            rclpy.spin_once(self, timeout_sec=0)

        else:
            # Standard maneuver (Forward, Backward, Ackerman, 45-degree Diagonal)
            self.get_logger().info(f"   (Applying steering and ramped velocity for {duration}s)")
            self.send_steering(steer_cmd)
            self.ramp_velocity(vel_cmd, duration)

            # Explicitly center steering after movement finishes if the steering was non-zero
            if steer_cmd != CMD_STRAIGHT:
                 self.get_logger().info("   (Centering steering)")
                 self.send_steering(CMD_STRAIGHT)
                 time.sleep(STEERING_STABILIZATION_TIME / 2.0)
                 rclpy.spin_once(self, timeout_sec=0)

        # Small mandatory pause between maneuvers to ensure command processing
        time.sleep(0.1)


    def run_demo(self):
            """The main pipeline execution."""

            # Clear steering and velocity before starting
            self.send_steering(CMD_STRAIGHT)
            self.send_velocity(CMD_STOP_VEL)
            time.sleep(STEERING_STABILIZATION_TIME)

            # Define the exact sequence requested by the user
            # Sequence format: (Description, Steering_Cmd, Velocity_Cmd, Duration, is_90_degree_maneuver)
            sequence = [
                # 1. Forward with gradual speed increase/decrease
                ("Move Forward (Ramped)", CMD_STRAIGHT, MAX_SPEED, BASE_MOVE_TIME, False),

                # 2. Same Backwards with gradual speed increase/decrease
                ("Move Backward (Ramped)", CMD_STRAIGHT, -MAX_SPEED, BASE_MOVE_TIME, False),

                # 3. Full Rotation (Spin Clockwise)
                ("Rotation Clockwise (Spin)", CMD_ROTATE_CW_STEER, MAX_SPEED, ROTATION_MOVE_TIME, True), 

                # 4. Opposite Rotation (Spin Counter-Clockwise)
                ("Rotation Counter-Clockwise", CMD_ROTATE_CCW_STEER, MAX_SPEED, ROTATION_MOVE_TIME, True),

                # 5. Crab Movement one side (Left) - 90-degree
                ("Crab Move Left (Sideways)", CMD_CRAB_LEFT_STEER, MAX_SPEED, BASE_MOVE_TIME, True),

                # 6. Crab Movement opposite side (Right) - 90-degree
                ("Crab Move Right (Sideways)", CMD_CRAB_RIGHT_STEER, MAX_SPEED, BASE_MOVE_TIME, True),
                
                # --- NEW DIAGONAL MOVEMENTS ---
                # 7. Diagonal Movement FWD-Left (45-degree, all wheels parallel)
                ("Diagonal Move FWD-Left (45 deg)", CMD_DIAG_FWD_LEFT, MAX_SPEED, BASE_MOVE_TIME, False),

                # 8. Diagonal Movement FWD-Right (Negative 45-degree, all wheels parallel)
                ("Diagonal Move FWD-Right (-45 deg)", CMD_DIAG_FWD_RIGHT, MAX_SPEED, BASE_MOVE_TIME, False),
                # --- END NEW DIAGONAL MOVEMENTS ---

                # 9. Ackerman Turn Left (Uses 2-step logic)
                ("Ackerman Turn Left", CMD_ACK_LEFT_STEER, MAX_SPEED, BASE_MOVE_TIME, False),

                # 10. Ackerman Turn Right (Uses 2-step logic)
                ("Ackerman Turn Right", CMD_ACK_RIGHT_STEER, MAX_SPEED, BASE_MOVE_TIME, False),
            ]

            try:
                self.get_logger().info('--- Starting Full Movement Demonstration (Fast Transitions) ---')
                for description, steer_cmd, vel_cmd, duration, is_90_degree_maneuver in sequence:
                    if not rclpy.ok():
                        break

                    # Execute the maneuver using its specific duration
                    self.execute_maneuver(description, steer_cmd, vel_cmd, duration, is_90_degree_maneuver)

            except Exception as e:
                self.get_logger().error(f"Demonstration interrupted by error: {e}")

            finally:
                # Final stop and cleanup
                self.get_logger().info("Demonstration Finished. Sending final STOP command.")
                self.send_velocity(CMD_STOP_VEL)
                self.send_steering(CMD_STRAIGHT)
                self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    demo_node = DemonstrationNode()

    # Run the demo sequence
    demo_node.run_demo()

    rclpy.shutdown()

if __name__ == '__main__':
    main()