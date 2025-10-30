#!/usr/bin/env python3
"""
Test script for TTS voice consistency and exploration.
Usage:
  # Test consistency (repeat same phrase 3 times)
  ./test_voices.py --repeat 3
  
  # Try different speaker IDs
  ./test_voices.py --speaker-ids 0 10 20 30 40 50 60 65 70 80 90 100
  
  # Try ALL speaker IDs (0-903, will take a while!)
  ./test_voices.py --all-speakers
  
  # Try all speakers with shorter delay
  ./test_voices.py --all-speakers --delay 1.5
  
  # Custom phrase
  ./test_voices.py --phrase "Your custom phrase here"
  
  # Change voice parameters
  ./test_voices.py --speaker-id 65 --length-scale 1.0 --noise-scale 0.667

voices to consider: 4 13 17 25 40 54 62 85 94 125 131 141 144 150 162

"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse
import time


class VoiceTester(Node):
    def __init__(self):
        super().__init__('voice_tester')
        self.publisher = self.create_publisher(String, 'speech/say', 10)
        # Give time for publisher to be ready
        time.sleep(0.5)
    
    def say(self, text):
        """Publish text to TTS"""
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{text}"')
    
    def set_parameter(self, param_name, value):
        """Set a parameter on the speech node"""
        from rclpy.parameter import Parameter
        
        # Create parameter client for speech node
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType
        
        client = self.create_client(SetParameters, '/speech/set_parameters')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Parameter service not available')
            return False
        
        # Create parameter message
        param = ParamMsg()
        param.name = param_name
        param.value = ParameterValue()
        
        if isinstance(value, int):
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = value
        elif isinstance(value, float):
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = value
        else:
            self.get_logger().error(f'Unsupported parameter type: {type(value)}')
            return False
        
        # Call service
        request = SetParameters.Request()
        request.parameters = [param]
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result().results[0]
            if result.successful:
                self.get_logger().info(f'Set {param_name} = {value}')
                return True
            else:
                self.get_logger().error(f'Failed to set {param_name}: {result.reason}')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False


def main():
    parser = argparse.ArgumentParser(description='Test TTS voices')
    parser.add_argument('--phrase', type=str, 
                       default="Hi, my name is Ivy! I'm the halloween pumpkin robot.",
                       help='Phrase to say')
    parser.add_argument('--repeat', type=int, default=0,
                       help='Repeat the phrase N times to test consistency')
    parser.add_argument('--speaker-ids', type=int, nargs='+',
                       help='Try multiple speaker IDs')
    parser.add_argument('--all-speakers', action='store_true',
                       help='Try all speaker IDs (0-903 for libritts_r)')
    parser.add_argument('--speaker-id', type=int,
                       help='Set specific speaker ID')
    parser.add_argument('--length-scale', type=float,
                       help='Set length scale (speed, default 1.0)')
    parser.add_argument('--noise-scale', type=float,
                       help='Set noise scale (variability, default 0.667)')
    parser.add_argument('--noise-w-scale', type=float,
                       help='Set noise w scale (default 0.8)')
    parser.add_argument('--delay', type=float, default=3.0,
                       help='Delay between phrases in seconds')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = VoiceTester()
    
    try:
        # Set single parameters if specified
        if args.speaker_id is not None:
            node.set_parameter('speaker_id', args.speaker_id)
            time.sleep(0.5)
        
        if args.length_scale is not None:
            node.set_parameter('length_scale', args.length_scale)
            time.sleep(0.5)
        
        if args.noise_scale is not None:
            node.set_parameter('noise_scale', args.noise_scale)
            time.sleep(0.5)
        
        if args.noise_w_scale is not None:
            node.set_parameter('noise_w_scale', args.noise_w_scale)
            time.sleep(0.5)
        
        # Test consistency mode
        if args.repeat > 0:
            node.get_logger().info(f'Testing consistency: repeating phrase {args.repeat} times')
            for i in range(args.repeat):
                node.get_logger().info(f'Repetition {i+1}/{args.repeat}')
                node.say(args.phrase)
                time.sleep(args.delay)
        
        # Test all speaker IDs
        elif args.all_speakers:
            node.get_logger().info('Testing all speaker IDs (0-903 for libritts_r model)')
            for speaker_id in range(904):
                node.get_logger().info(f'--- Speaker ID: {speaker_id} ---')
                node.set_parameter('speaker_id', speaker_id)
                time.sleep(0.5)
                node.say(args.phrase)
                time.sleep(args.delay)
        
        # Test multiple speaker IDs
        elif args.speaker_ids:
            node.get_logger().info(f'Testing {len(args.speaker_ids)} different speaker IDs')
            for speaker_id in args.speaker_ids:
                node.get_logger().info(f'--- Speaker ID: {speaker_id} ---')
                node.set_parameter('speaker_id', speaker_id)
                time.sleep(0.5)
                node.say(args.phrase)
                time.sleep(args.delay)
        
        # Just say the phrase once
        else:
            node.say(args.phrase)
            time.sleep(args.delay)
        
        node.get_logger().info('Done')
    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
