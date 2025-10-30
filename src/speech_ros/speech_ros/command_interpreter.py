#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import re
import random
import os
from datetime import datetime
from zoneinfo import ZoneInfo

from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False


class CommandInterpreterNode(Node):
    def __init__(self):
        super().__init__('command_interpreter')
        
        # Publisher for text-to-speech
        self.say_publisher = self.create_publisher(String, '/speech/say', 10)
        
        # Subscriber for speech utterances
        self.utterances_subscriber = self.create_subscription(
            String,
            'speech/utterances',
            self.utterances_callback,
            10)
        
        # Subscriber for PTT utterances (no wake word needed)
        self.ptt_utterances_subscriber = self.create_subscription(
            String,
            'speech/ptt_utterances',
            self.ptt_utterances_callback,
            10)
        
        # Subscriber for diagnostics
        self.diagnostics_subscriber = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10)
        
        # Store latest diagnostics
        self.latest_diagnostics = dict()
        
        # Initialize OpenAI client if available
        self.openai_client = None
        if OPENAI_AVAILABLE:
            api_key = os.environ.get('OPENAI_API_KEY')
            if api_key:
                self.openai_client = OpenAI(api_key=api_key)
                self.get_logger().info("OpenAI client initialized")
            else:
                self.get_logger().warn("OPENAI_API_KEY not found in environment")
        else:
            self.get_logger().info("OpenAI not available - install with: pip install openai")
        
        self.get_logger().info("Command interpreter started")
    
    def say(self, text):
        """Send text to speech node"""
        self.get_logger().info(f"reply: {text}")
        self.say_publisher.publish(String(data=text))
    
    def ask_openai(self, question):
        """Ask OpenAI for a response"""
        if not self.openai_client:
            return None
        
        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-4.1",  
                messages=[
                    {"role": "system", "content": f"You are Ivy the Halloween pumpkin robot, a friendly autonomous robot with six wheels and a plastic pumpkin head filled with candy. Today is {datetime.now(ZoneInfo('America/Los_Angeles')).strftime('%A, %B %d, %Y at %I:%M %p Pacific')}. You were built by the genius Brian Erickson. Keep responses brief and conversational (1-2 sentences). You have a playful personality. Optimize output for use with piper text to speech"},
                    {"role": "user", "content": question}
                ],
                max_tokens=100,
                temperature=0.7
            )
            return response.choices[0].message.content
        except Exception as e:
            self.get_logger().error(f"OpenAI API error: {e}")
            return None
    
    def text_to_float(self, text):
        """Convert text representation of numbers to float"""
        text = text.lower()
        text_to_num = {
            "zero": 0.0, "one": 1.0, "two": 2.0, "three": 3.0, "four": 4.0,
            "five": 5.0, "six": 6.0, "seven": 7.0, "eight": 8.0, "nine": 9.0, "ten": 10.0
        }
        return text_to_num.get(text, float(text))
    
    def diagnostics_callback(self, diagnostic_array):
        return
        """Handle diagnostic messages and announce status changes"""
        OK = 0
        WARN = 1
        ERROR = 2
        STALE = 3
        
        for d in diagnostic_array.status:
            if d.name not in self.latest_diagnostics or self.latest_diagnostics[d.name].level != d.level:
                level_string = "invalid"
                if d.level == OK:
                    level_string = "OK"
                elif d.level == WARN:
                    level_string = "Warning"
                elif d.level == ERROR:
                    level_string = "Error"
                elif d.level == STALE:
                    level_string = "Error"
                
                if d.level == OK:
                    message = f"{d.name} {level_string}"
                else:
                    message = f"{d.name} {level_string}. {d.message}"
                
                self.get_logger().info(message)
                self.say(message)
            
            self.latest_diagnostics[d.name] = d
    
    def ptt_utterances_callback(self, utterance_ros):
        """Process PTT voice commands (no wake word needed)"""
        request = utterance_ros.data.lower()
        self.get_logger().info(f"PTT utterance: {request}")
        self.process_command(request)
    
    def utterances_callback(self, utterance_ros):
        """Process voice commands"""
        utterance = utterance_ros.data.lower()
        self.get_logger().info(f"utterance: {utterance}")
        
        # Look for wake words
        wake_words = ["hey robot", "he robot", "arabic", "hear about", "hero but", 
                     "hero bike", "your robot", "you're about", "a robot"]
        found_wake = False
        wake_word = ""
        
        for wake_word in wake_words:
            if utterance.startswith(wake_word):
                found_wake = True
                break
        
        if not found_wake:
            self.get_logger().info(f"wake word not heard")
            return
        
        # Remove wake word to find request
        request = utterance[len(wake_word):].lstrip()
        self.process_command(request)
    
    def process_command(self, request):
        """Process a command request (after wake word or from PTT)"""
        self.get_logger().info(f"processing command: {request}")
        
        # # Process different command patterns
        # p = re.compile('.*your name.*', re.IGNORECASE)
        # if p.match(request) is not None:
        #     self.say("my name is red, crash")
        #     return
        
        p = re.compile('.*(?:old|born|age).*', re.IGNORECASE)
        if p.match(request) is not None:
            self.say("I was born on April 17th 2021")
            return
        
        p = re.compile('.*(?:marry|married).*', re.IGNORECASE)
        if p.match(request) is not None:
            retorts = [
                "Sorry, I'm sure you're very nice, but you're a little too squishy for me.",
                "Would that be legal?",
                "I would, but I don't think you could keep up with me.",
                "Sure, but you'll have to sign a pre-nup and name our kids after star wars robots."
            ]
            self.say(random.choice(retorts))
            return
        
        p = re.compile('.*(?:volume|audio|speaker).* (zero|one|two|three|four|five|six|seven|eight|nine|ten|\\d+\\.?\\d*).*', re.IGNORECASE)
        m = p.match(request)
        if m is not None:
            g = m.groups()
            self.get_logger().info(str(g))
            p = self.text_to_float(g[0])
            p = int(p)
            if 0 <= p <= 100:
                # Set parameter on speech node
                self.declare_parameter('speaker_volume_percent', p)
                self.say(f"request to set volume to {p}")
            else:
                self.say("volume must be a number between zero and one hundred")
            return
        
        p = re.compile('.*(status|how are).*', re.IGNORECASE)
        m = p.match(request)
        if m is not None:
            self.say("I'm ok")
            return
        
        # If no pattern matched, try OpenAI as fallback
        if self.openai_client:
            self.get_logger().info(f"Asking OpenAI: {request}")
            ai_response = self.ask_openai(request)
            if ai_response:
                self.say(ai_response)
                return
        
        # Final fallback
        self.say(f"Sorry, I don't know how to respond to {request}")


def main(args=None):
    rclpy.init(args=args)
    node = CommandInterpreterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
