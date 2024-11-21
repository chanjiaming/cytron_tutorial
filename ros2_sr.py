import socket
import speech_recognition as sr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeechRecog(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        self.publisher = self.create_publisher(String, '/user_input', 10)
        self.listening_over = False
        self.get_logger().info("Speech recognition node is initialized")

    def check_internet_connection(self):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False

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
            return user_input

        except sr.UnknownValueError:
            self.get_logger().warning("Sorry, I didn't catch that.")
            return None
        except sr.RequestError:
            self.get_logger().error("Could not request results from the speech recognition service.")
            return None

    def run(self):
        if not self.check_internet_connection():
            self.get_logger().error("No internet connection available.")
            return
        else:
            self.get_logger().info("Internet connection available.")

        while not self.listening_over:
            user_input = self.listen()

            if user_input is None:
                continue

            # Publish the user input to the ROS 2 topic
            msg = String()
            msg.data = user_input
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: {user_input}")

            if "goodbye" in user_input.lower():
                self.listening_over = True
                self.get_logger().info("Goodbye! It was nice talking to you.")
                break


def main():
    rclpy.init()  # Initialize ROS 2
    speech_recog = SpeechRecog()

    try:
        speech_recog.run()
    except KeyboardInterrupt:
        speech_recog.get_logger().info("Shutting down...")
    finally:
        speech_recog.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
