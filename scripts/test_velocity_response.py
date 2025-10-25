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
                self.velocity_errors.append({
                    'sp': msg.m1_commanded_speed,
                    'pv_m1': msg.m1_current_speed,
                    'pv_m2': msg.m2_current_speed,
                    'error_m1': m1_error,
                    'error_m2': m2_error
                })

    
    def send_cmd_vel(self, linear_x, angular_z=0.0, log=False):
        """Send a velocity command"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
        self.commanded_velocity = linear_x
        if log:
            self.get_logger().info(f'>>> SENT cmd_vel: linear.x={linear_x:.3f} angular.z={angular_z:.3f}')
    
    def run_step_test(self, target_velocity=0.3, duration=5.0):
        """Run a step input test"""
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'STEP TEST: Target={target_velocity:.3f} m/s, Duration={duration}s')
        self.get_logger().info('=' * 80)
        
        # Confirmation prompt
        response = input('Press ENTER to start motors, any other key to abort: ')
        if response != '':
            self.get_logger().info('Test aborted by user')
            return
        
        self.get_logger().info('STARTING TEST...')
        
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
        time.sleep(1.0)
        
        self.test_start_time = None
        self.get_logger().info('=' * 80)
        self.get_logger().info('STEP TEST COMPLETE')
        self.get_logger().info('=' * 80)
    
    def run_ramp_test(self, max_velocity=0.3, ramp_up_time=2.0, flat_time=2.0, ramp_down_time=2.0):
        """Run a ramping velocity test with custom timing
        
        Args:
            max_velocity: Maximum velocity in m/s
            ramp_up_time: Time to ramp from 0 to max_velocity (seconds)
            flat_time: Time to hold at max_velocity (seconds)
            ramp_down_time: Time to ramp from max_velocity to 0 (seconds)
        """
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'RAMP TEST: Max={max_velocity:.3f} m/s')
        self.get_logger().info(f'  Ramp up: {ramp_up_time:.1f}s, Flat: {flat_time:.1f}s, Ramp down: {ramp_down_time:.1f}s')
        self.get_logger().info('=' * 80)
        
        # Confirmation prompt
        response = input('Press ENTER to start motors, any other key to abort: ')
        if response != '':
            self.get_logger().info('Test aborted by user')
            return
        
        self.get_logger().info('STARTING TEST...')
        
        # Track errors for summary
        self.velocity_errors = []
        
        # 50 Hz update rate
        dt = 0.02  # 20ms = 50Hz
        
        # Start from zero
        self.send_cmd_vel(0.0)
        time.sleep(1.0)
        
        self.test_start_time = time.time()
        
        # Ramp up phase
        start_time = time.time()
        while (time.time() - start_time) < ramp_up_time:
            elapsed = time.time() - start_time
            velocity = (elapsed / ramp_up_time) * max_velocity
            velocity = min(velocity, max_velocity)  # Clamp to max
            self.send_cmd_vel(velocity)
            rclpy.spin_once(self, timeout_sec=dt)
            time.sleep(max(0, dt - 0.001))  # Account for processing time
        
        # Hold at max phase
        start_time = time.time()
        while (time.time() - start_time) < flat_time:
            self.send_cmd_vel(max_velocity)
            rclpy.spin_once(self, timeout_sec=dt)
            time.sleep(max(0, dt - 0.001))
        
        # Ramp down phase
        start_time = time.time()
        while (time.time() - start_time) < ramp_down_time:
            elapsed = time.time() - start_time
            velocity = max_velocity * (1.0 - elapsed / ramp_down_time)
            velocity = max(velocity, 0.0)  # Clamp to zero
            self.send_cmd_vel(velocity)
            rclpy.spin_once(self, timeout_sec=dt)
            time.sleep(max(0, dt - 0.001))
        
        # Stop
        self.send_cmd_vel(0.0)
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
                avg_error_m1 = sum(s['error_m1'] for s in samples) / len(samples)
                avg_error_m2 = sum(s['error_m2'] for s in samples) / len(samples)
                avg_error = (avg_error_m1 + avg_error_m2) / 2.0
                avg_pv_m1 = sum(s['pv_m1'] for s in samples) / len(samples)
                avg_pv_m2 = sum(s['pv_m2'] for s in samples) / len(samples)
                max_pv_m1 = max(s['pv_m1'] for s in samples)
                max_pv_m2 = max(s['pv_m2'] for s in samples)
                
                status = '✓' if avg_error < 0.05 else '⚠' if avg_error < 0.15 else '✗'
                self.get_logger().info(
                    f'  {status} SP={sp:.1f} m/s: Avg PV M1={avg_pv_m1:.3f} M2={avg_pv_m2:.3f} | '
                    f'Max PV M1={max_pv_m1:.3f} M2={max_pv_m2:.3f} | Avg Error={avg_error:.3f} m/s'
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
    
    def print_menu():
        print('\n' + '=' * 80)
        print('VELOCITY RESPONSE TEST')
        print('=' * 80)
        print('Robot should be on blocks with wheels free to spin.')
        print('This test will send velocity commands and monitor the response.')
        print('')
        print('Tests available:')
        print('  1) Custom step input')
        print('  2) Custom ramp test')
        print('  q) Quit')
        print('=' * 80)
    
    print_menu()
    
    try:
        while True:
            choice = input('\nSelect test (1-2, q to quit): ').strip()
            
            if choice == 'q':
                break
            elif choice == '1':
                target = float(input('Enter target velocity (m/s): '))
                duration = float(input('Enter duration (seconds): '))
                test_node.run_step_test(target_velocity=target, duration=duration)
            elif choice == '2':
                # Custom ramp test
                max_vel = float(input('Enter maximum velocity (m/s): '))
                ramp_up = float(input('Enter ramp up time (seconds): '))
                flat = float(input('Enter flat time (seconds): '))
                ramp_down = float(input('Enter ramp down time (seconds): '))
                test_node.run_ramp_test(max_velocity=max_vel, ramp_up_time=ramp_up, 
                                       flat_time=flat, ramp_down_time=ramp_down)
            else:
                print('Invalid choice')
            
            # Keep spinning to process messages
            for _ in range(10):
                rclpy.spin_once(test_node, timeout_sec=0.1)
            
            # Print menu again after test completes
            print_menu()
    
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
