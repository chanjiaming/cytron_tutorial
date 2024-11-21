import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import speech_recognition as sr


class SpeechRecog(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        self.listening_over = False
        self.listening_active = True  
        self.get_logger().info("Speech Recognition Node Initialized")

        self.user_input_publisher = self.create_publisher(String, '/user_input', 10)

        self.activate_sr_subscription = self.create_subscription(
            Bool, '/activate_sr', self.activate_sr_callback, 10)

    def check_internet_connection(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False

    def activate_sr_callback(self, msg):
        self.get_logger().info(f"Received message on /activate_sr: {msg.data}")
        if msg.data == True:
            self.listening_active = True
            self.get_logger().info("Resumed Speech Recognition")
        else:
            self.get_logger().warning(f"Unexpected message on /activate_sr: {msg.data}")


    def listen(self):
        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        with microphone as source:
            self.get_logger().info("Listening for your response...")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            audio = recognizer.listen(source)

        try:
            self.get_logger().info("Recognizing speech...")
            user_input = recognizer.recognize_google(audio)
            self.get_logger().info(f"You: {user_input}")


            msg = String()
            msg.data = user_input

            self.user_input_publisher.publish(msg)
            self.get_logger().info(f"Published: {user_input}")

      
            self.listening_active = False 

        except sr.UnknownValueError:
            self.get_logger().warning("Sorry, I didn't catch that.")

        except sr.RequestError:
            self.get_logger().error("Could not request results from the speech recognition service.")


    def run(self):
        if not self.check_internet_connection():
            self.get_logger().error("No internet connection available.")
            return

        self.get_logger().info("Internet connection available.")

        while not self.listening_over:
            if self.listening_active:
                self.listen()
            else: 
                rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    sr_node = SpeechRecog()

    try:
        sr_node.run()
    except KeyboardInterrupt:
        sr_node.get_logger().info("Shutting down SR Node...")
    finally:
        sr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
