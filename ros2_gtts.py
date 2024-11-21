import socket
import os
import pygame
from gtts import gTTS
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class TextToSpeech(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')
        self.user_name = None
        self.conversation_over = False
        self.sr_active = False

        pygame.init()
        pygame.mixer.init()

        self.get_logger().info("TTS Node Initialized")
        
        self.subscription = self.create_subscription(
            String, '/user_input', self.speak_callback, 10)

        self.activate_sr_publisher = self.create_publisher(Bool, '/activate_sr', 10)

    def check_internet_connection(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False

    def speak(self, text):
        tts = gTTS(text=text, lang='en')
        audio_file = "temp_audio.mp3"
        tts.save(audio_file)
        pygame.mixer.music.load(audio_file)
        time.sleep(0.1)  
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue
        os.remove(audio_file)

    def speak_callback(self, msg):
        text = msg.data
        if text.strip().lower() == "goodbye": 
            self.speak("Goodbye, see you next time")
            self.get_logger().info("Received 'goodbye', shutting down...") 
            rclpy.shutdown() 
            return 
        self.get_logger().info(f"Received input: {text}")

        self.speak(text)

        activate_msg = Bool()
        activate_msg.data = True
        self.activate_sr_publisher.publish(activate_msg)
        self.get_logger().info("Published activate_sr signal")


def main():
    rclpy.init()
    tts_node = TextToSpeech()

    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        tts_node.get_logger().info("Shutting down TTS Node...")
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
