#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
import vosk
import json
import queue
import Jetson.GPIO as GPIO
import pyttsx3
import time

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')

        # === GPIO Setup ===
        GPIO.setmode(GPIO.BOARD)
        self.BUTTON_PIN = 29
        GPIO.setup(self.BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # === Vosk Setup ===
        self.model = vosk.Model("/home/coler/AutonomousTrike/trike/voice_assist/vosk-model-small-en-us-0.15")
        self.q = queue.Queue()

        # === TTS Setup ===
        self.engine = pyttsx3.init()

        # === ROS2 Publisher ===
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)

        # === Button State Tracking ===
        self.previous_state = GPIO.input(self.BUTTON_PIN)
        self.debounce_time = 0.2
        self.last_pressed = time.time()

        # === Sounddevice Setup ===
        self.desired_mic_name = "Mic"
        self.mic_index = self.find_input_device(self.desired_mic_name)

        # === Timer to Check Button ===
        self.create_timer(0.01, self.check_button)

        self.speak("Press the button to speak.")

    def find_input_device(self, name):
        devices = sd.query_devices()
        for idx, dev in enumerate(devices):
            if name in dev['name']:
                return idx
        raise RuntimeError(f"Input device with name '{name}' not found.")

    def callback(self, indata, frames, time_info, status):
        self.q.put(bytes(indata))

    def speak(self, message):
        self.get_logger().info(f"TTS: {message}")
        self.engine.say(message)
        self.engine.runAndWait()

    def listen_and_recognize(self):
        with sd.RawInputStream(device=self.mic_index,
                               samplerate=16000,
                               blocksize=8000,
                               dtype='int16',
                               channels=1,
                               callback=self.callback):
            rec = vosk.KaldiRecognizer(self.model, 16000)
            self.get_logger().info("Listening...")

            while True:
                data = self.q.get()
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    return result.get('text', '')

    def check_button(self):
        input_state = GPIO.input(self.BUTTON_PIN)

        if input_state == GPIO.LOW and self.previous_state == GPIO.HIGH:
            now = time.time()
            if now - self.last_pressed > self.debounce_time:
                self.last_pressed = now
                self.speak("Recording!")
                spoken_text = self.listen_and_recognize()
                if spoken_text:
                    self.get_logger().info(f"Recognized: {spoken_text}")
                    msg = String()
                    msg.data = spoken_text
                    self.publisher_.publish(msg)
                    self.speak(f"Setting destination to {spoken_text}")
                else:
                    self.speak("I didn't catch that. Try again.")

        self.previous_state = input_state

    def destroy_node(self):
        GPIO.cleanup()
        self.engine.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
