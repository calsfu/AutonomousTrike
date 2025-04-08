#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import speech_recognition as sr
import Jetson.GPIO as GPIO

# --------------------------
# Configuration & Setup
# --------------------------

# GPIO setup for arcade button (GPIO 13, physical pin 33)
BUTTON_PIN = 13  
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Internal pull-up resistor

# Predefined destinations (example coordinates or identifiers)
DESTINATIONS = {
    "photonics": (42.3492878608647, -71.10668764629143),
    "gym": (42.35159958704134, -71.11704581309559),
    "media lab": (42.36031475739776, -71.08726905801085),
}

class VoiceNavigation(Node):
    def __init__(self):
        super().__init__('voice_navigation')
        self.get_logger().info("Voice Navigation Node Started.")
        self.run_navigation()
    
    def recognize_speech(self):
        """
        Uses PocketSphinx via SpeechRecognition to capture and process voice commands.
        """
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        with mic as source:
            self.get_logger().info("Listening for destination (using PocketSphinx)...")
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source)
        try:
            command = recognizer.recognize_sphinx(audio).lower()
            self.get_logger().info(f"Recognized command: {command}")
            return command
        except sr.UnknownValueError:
            self.get_logger().warn("PocketSphinx could not understand the audio")
        except sr.RequestError as e:
            self.get_logger().error(f"PocketSphinx error: {e}")
        return None

    def get_destination(self, command):
        """
        Checks if the recognized command contains one of the predefined destination keywords.
        """
        for place in DESTINATIONS:
            if place in command:
                return DESTINATIONS[place]
        return None

    def wait_for_confirmation(self):
        """
        Waits for the arcade button to be pressed.
        The button is wired such that when pressed, GPIO input goes LOW.
        """
        self.get_logger().info("Waiting for button press to confirm destination...")
        while GPIO.input(BUTTON_PIN):  # Wait until button is pressed (GPIO goes LOW)
            time.sleep(0.1)
        self.get_logger().info("Destination confirmed via button press!")
        return True

    def run_navigation(self):
        # Step 1: Recognize voice command using PocketSphinx
        command = self.recognize_speech()
        if command:
            # Step 2: Identify destination from the command
            destination = self.get_destination(command)
            if destination:
                self.get_logger().info(f"Destination '{destination}' selected based on command.")
                # Step 3: Wait for user confirmation via the arcade button
                if self.wait_for_confirmation():
                    self.get_logger().info(f"Navigating to {destination}...")
            else:
                self.get_logger().warn("No valid destination found in the voice command. Please try again.")
        else:
            self.get_logger().warn("Voice command recognition failed.")

# --------------------------
# Entry Point
# --------------------------

def main():
    rclpy.init()
    node = VoiceNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
