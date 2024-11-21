import socket
import os
import pygame
from gtts import gTTS
import time

class text2speech:
    def __init__(self):
        self.user_name = None
        self.conversation_over = False
        pygame.init()
        pygame.mixer.init()
        print("Gemini Chat Initialized")

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
        time.sleep(0.1)  # Small delay for smooth play
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue
        os.remove(audio_file)


    def run(self):
        if not self.check_internet_connection():
            print("No internet connection available.")
            self.speak("No internet connection available. Please connect to the internet and restart.")
            return
        else:
            print("Internet connection available.")

        welcome_message = "Welcome! How can I assist you today?"
        self.speak(welcome_message)
        print("Welcome! How can I assist you today?")



def main():
    tts = text2speech()
    tts.run()

if __name__ == "__main__":
    main()
