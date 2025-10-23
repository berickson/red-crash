#!/usr/bin/env python3
"""
Test script to analyze velocity response behavior.
Sends step inputs to cmd_vel and monitors RoboClaw status to identify plateaus.

Usage:
    python3 test_velocity_response.py
    
The robot should be on blocks with wheels free to spin.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros2_roboclaw_driver.msg import RoboClawStatus
import time
import sys


class VelocityResponseTest(Node):
    def __init__(self):
        super().__init__('velocity_response_test')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_sub = self.create_subscription(
            RoboClawStatus, '/roboclaw_status', self.status_callback, 10)
        
        # Data tracking
        self.last_status = None
        self.test_start_time = None
        self.commanded_velocity = 0.0
        
        self.get_logger().info('Velocity Response Test Node Started')
        self.get_logger().info('Waiting for RoboClaw status messages...')
        
    def status_callback(self, msg):
        """Track status messages for analysis"""
        self.last_status = msg
        
        if self.test_start_time is not None:
            elapsed = time.time() - self.test_start_time
            
            # Calculate errors
            m1_error = abs(msg.m1_commanded_speed - msg.m1_current_speed)
            m2_error = abs(msg.m2_commanded_speed - msg.m2_current_speed)
            
            # Track for summary (only if commanded speed > 0.05)
            if hasattr(self, 'velocity_errors') and msg.m1_commanded_speed > 0.05:
                m1_pct_error = (m1_error / msg.m1_commanded_speed) * 100 if msg.m1_commanded_speed > 0 else 0
                m2_pct_error = (m2_error / msg.m2_commanded_speed) * 100 if msg.m2_commanded_speed > 0 else 0
                self.velocity_errors.append({
                    'sp': msg.m1_commanded_speed,
                    'pv_m1': msg.m1_current_speed,
                    'pv_m2': msg.m2_current_speed,
                    'error_pct': (m1_pct_error + m2_pct_error) / 2.0
                })
            
            # Log SP vs PV
            self.get_logger().info(
                f't={elapsed:.2f}s | '
                f'SP: M1={msg.m1_commanded_speed:.3f} M2={msg.m2_commanded_speed:.3f} m/s | '
                f'PV: M1={msg.m1_current_speed:.3f} M2={msg.m2_current_speed:.3f} m/s | '
                f'Error: M1={msg.m1_commanded_speed - msg.m1_current_speed:.3f} '
                f'M2={msg.m2_commanded_speed - msg.m2_current_speed:.3f} m/s'
            )
    
    def send_cmd_vel(self, linear_x, angular_z=0.0):
        """Send a velocity command"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
        self.commanded_velocity = linear_x
        self.get_logger().info(f'>>> SENT cmd_vel: linear.x={linear_x:.3f} angular.z={angular_z:.3f}')
    
    def run_step_test(self, target_velocity=0.3, duration=5.0):
        """Run a step input test"""
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'STARTING STEP TEST: Target={target_velocity:.3f} m/s, Duration={duration}s')
        self.get_logger().info('=' * 80)
        
        # Start from zero
        self.send_cmd_vel(0.0)
        time.sleep(1.0)
        
        # Step input
        self.test_start_time = time.time()
        self.send_cmd_vel(target_velocity)
        
        # Monitor for duration
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stop
        self.send_cmd_vel(0.0)
        self.get_logger().info('>>> SENT cmd_vel: STOP')
        time.sleep(1.0)
        
        self.test_start_time = None
        self.get_logger().info('=' * 80)
        self.get_logger().info('STEP TEST COMPLETE')
        self.get_logger().info('=' * 80)
    
    def run_ramp_test(self, max_velocity=0.3, step=0.05, step_duration=2.0):
        """Run a ramping velocity test"""
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'STARTING RAMP TEST: Max={max_velocity:.3f} m/s, Step={step:.3f}, Duration={step_duration:.1f}s')
        self.get_logger().info('=' * 80)
        
        # Track errors for summary
        self.velocity_errors = []
        
        # Start from zero
        self.send_cmd_vel(0.0)
        time.sleep(1.0)
        
        # Ramp up
        self.test_start_time = time.time()
        velocity = step
        while velocity <= max_velocity:
            self.send_cmd_vel(velocity)
            start_time = time.time()
            while (time.time() - start_time) < step_duration:
                rclpy.spin_once(self, timeout_sec=0.05)
            velocity += step
        
        # Hold at max
        self.get_logger().info('>>> Holding at max velocity')
        start_time = time.time()
        while (time.time() - start_time) < step_duration:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Ramp down
        velocity = max_velocity - step
        while velocity >= 0:
            self.send_cmd_vel(velocity)
            start_time = time.time()
            while (time.time() - start_time) < step_duration:
                rclpy.spin_once(self, timeout_sec=0.05)
            velocity -= step
        
        # Stop
        self.send_cmd_vel(0.0)
        self.get_logger().info('>>> SENT cmd_vel: STOP')
        time.sleep(1.0)
        
        self.test_start_time = None
        
        # Print summary
        if hasattr(self, 'velocity_errors') and len(self.velocity_errors) > 0:
            self.get_logger().info('=' * 80)
            self.get_logger().info('TEST SUMMARY:')
            
            # Find absolute maximum velocity achieved
            max_m1 = max(s['pv_m1'] for s in self.velocity_errors)
            max_m2 = max(s['pv_m2'] for s in self.velocity_errors)
            self.get_logger().info(f'Maximum velocity achieved: M1={max_m1:.3f} m/s, M2={max_m2:.3f} m/s')
            self.get_logger().info('')
            
            # Group by commanded velocity
            sp_groups = {}
            for sample in self.velocity_errors:
                sp_key = round(sample['sp'], 1)
                if sp_key not in sp_groups:
                    sp_groups[sp_key] = []
                sp_groups[sp_key].append(sample)
            
            # Calculate average error for each setpoint
            for sp in sorted(sp_groups.keys()):
                samples = sp_groups[sp]
                avg_error = sum(s['error_pct'] for s in samples) / len(samples)
                avg_pv_m1 = sum(s['pv_m1'] for s in samples) / len(samples)
                avg_pv_m2 = sum(s['pv_m2'] for s in samples) / len(samples)
                max_pv_m1 = max(s['pv_m1'] for s in samples)
                max_pv_m2 = max(s['pv_m2'] for s in samples)
                
                status = '✓' if avg_error < 5 else '⚠' if avg_error < 15 else '✗'
                self.get_logger().info(
                    f'  {status} SP={sp:.1f} m/s: Avg PV M1={avg_pv_m1:.3f} M2={avg_pv_m2:.3f} | '
                    f'Max PV M1={max_pv_m1:.3f} M2={max_pv_m2:.3f} | Avg Error={avg_error:.1f}%'
                )
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('RAMP TEST COMPLETE')
        self.get_logger().info('=' * 80)


def main(args=None):
    rclpy.init(args=args)
    
    test_node = VelocityResponseTest()
    
    # Wait for initial status
    print('\nWaiting for RoboClaw status messages...')
    for _ in range(50):  # Wait up to 5 seconds
        rclpy.spin_once(test_node, timeout_sec=0.1)
        if test_node.last_status is not None:
            break
    
    if test_node.last_status is None:
        print('ERROR: No RoboClaw status received. Is the motor driver running?')
        test_node.destroy_node()
        rclpy.shutdown()
        return 1
    
    print('\n' + '=' * 80)
    print('VELOCITY RESPONSE TEST')
    print('=' * 80)
    print('Robot should be on blocks with wheels free to spin.')
    print('This test will send velocity commands and monitor the response.')
    print('')
    print('Tests available:')
    print('  1) Step input (0 -> 1.0 m/s)')
    print('  2) Medium step input (0 -> 0.5 m/s)')
    print('  3) Small step input (0 -> 0.3 m/s)')
    print('  4) Ramp test (0 -> 0.9 m/s in steps)')
    print('  5) Rotation test (in-place rotation)')
    print('  6) Custom step input')
    print('  q) Quit')
    print('=' * 80)
    
    try:
        while True:
            choice = input('\nSelect test (1-6, q to quit): ').strip()
            
            if choice == 'q':
                break
            elif choice == '1':
                test_node.run_step_test(target_velocity=1.0, duration=5.0)
            elif choice == '2':
                test_node.run_step_test(target_velocity=0.5, duration=5.0)
            elif choice == '3':
                test_node.run_step_test(target_velocity=0.3, duration=5.0)
            elif choice == '4':
                test_node.run_ramp_test(max_velocity=0.9, step=0.1, step_duration=0.2)
            elif choice == '5':
                # Rotation test
                test_node.get_logger().info('ROTATION TEST')
                test_node.test_start_time = time.time()
                test_node.send_cmd_vel(0.0, angular_z=0.5)
                time.sleep(3.0)
                test_node.send_cmd_vel(0.0, angular_z=0.0)
                test_node.test_start_time = None
            elif choice == '6':
                target = float(input('Enter target velocity (m/s): '))
                duration = float(input('Enter duration (seconds): '))
                test_node.run_step_test(target_velocity=target, duration=duration)
            else:
                print('Invalid choice')
            
            # Keep spinning to process messages
            for _ in range(10):
                rclpy.spin_once(test_node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        print('\nTest interrupted by user')
    except Exception as e:
        print(f'\nError: {e}')
    finally:
        # Make sure robot stops
        test_node.send_cmd_vel(0.0)
        time.sleep(0.5)
        test_node.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
