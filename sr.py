import socket
import speech_recognition as sr



class SpeechRecog:
    def __init__(self):
        self.listening_over = False
        print("Speech recognition is initialized")

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
            print("Listening for your response...")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            audio = recognizer.listen(source)

        try:
            print("Recognizing speech...")
            user_input = recognizer.recognize_google(audio)
            print(f"You: {user_input}")
            return user_input
        
        except sr.UnknownValueError:
            print("Sorry, I didn't catch that.")
            return None
        except sr.RequestError:
            print("Could not request results from the speech recognition service.")
            return None

    def run(self):
        if not self.check_internet_connection():
            print("No internet connection available.")
            return
        else:
            print("Internet connection available.")

        while not self.listening_over:
            user_input = self.listen()

            if user_input is None:
                continue

            if "goodbye" in user_input.lower():
                self.conversation_over = True
                print(f"Goodbye! It was nice talking to you.")
                break

def main():
    speech_recog = SpeechRecog()
    speech_recog.run()

if __name__ == "__main__":
    main()


