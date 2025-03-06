#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from melo.api import TTS
import os

EN2ES = {"Excuse me" : "Disculpa", 
         "Please give way" : "Déjame pasar un momento por favor",
         "Attention, there’s a robot here" : "¡Atención! Hay un robot aquí"
         }

class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')

        # Speed is adjustable
        self.speed = 1.0

        # CPU is sufficient for real-time inference.
        # You can also change to cuda:0
        self.device = 'cuda:0'

        self.model = TTS(language='ES', device=self.device)
        self.speaker_ids = self.model.hps.data.spk2id

        self.output_path = 'es.wav'


        # Subscriber to /speech_command topic
        self.subscription = self.create_subscription(
            String,
            '/speech_command',
            self.speech_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def speech_callback(self, msg):
        # Get the command from the message
        command = msg.data
        self.get_logger().info(f"Received speech command: {command}")
        self.model.tts_to_file(EN2ES[command], self.speaker_ids['ES'], self.output_path, speed=self.speed)
        os.system(f"ffplay -nodisp -autoexit {self.output_path}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()