#!/usr/bin/env python3
import rclpy
import sounddevice as sd
import os
import soundfile as sf
from std_msgs.msg import Int8
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from enum import Enum

class AudioFile(Enum):
    AUTONOMOUS_ON = 0
    BRAKES_OFF = 1
    BRAKES_ON = 2
    DESTINATION_CONFIRMED = 3
    DESTINATION_SET = 4
    MANUAL_ON = 5
    NEUTRAL_ON = 6
    PARK_ON = 7
    NO_GPS_SIGNAL = 8
    ENTER_DESTINATION = 9
    SYSTEM_READY = 10
    TURNING_LEFT = 11
    TURNING_RIGHT = 12
    CONFIRM_DESTINATION = 13

    _filename_map = {
        AUTONOMOUS_ON: "autonomous_on.wav",
        BRAKES_OFF: "brakes_off.wav",
        BRAKES_ON: "brakes_on.wav",
        DESTINATION_CONFIRMED: "destination_confirmed.wav",
        DESTINATION_SET: "destination_set.wav",
        MANUAL_ON: "manual_on.wav",
        NEUTRAL_ON: "neutral_on.wav",
        PARK_ON: "park_on.wav",
        NO_GPS_SIGNAL: "no_gps_signal_found.wav",
        ENTER_DESTINATION: "please_enter_a_destination.wav",
        SYSTEM_READY: "system_ready.wav",
        TURNING_LEFT: "turning_left.wav",
        TURNING_RIGHT: "turning_right.wav",
        CONFIRM_DESTINATION: "would_you_like_to_confirm_this_destination.wav",
        NO_GPS_SIGNAL: "no_gps_signal_found.wav"
    }

    def filename(self):
        return self._filename_map[self]

class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        self.audio_dir = os.path.join(get_package_share_directory('trike'), 'audio')
        if not os.path.exists(self.audio_dir):
            self.get_logger().error(f"Audio directory {self.audio_dir} does not exist.")
            return
        self.create_subscription(Int8, 'audio_command', self.listener_callback, 10)
        self.get_logger().info('Audio Player Initialized')

    def listener_callback(self, msg):
        command = msg.data
        if command in AudioFile._value2member_map_:
            audio_file = AudioFile(command).name + '.wav'
            self.play_audio(audio_file)
        else:
            self.get_logger().error(f"Invalid audio command: {command}")

    def play_audio(self, file_name):
        try:
            file_path = os.path.join(self.audio_dir, file_name)
            data, samplerate = sf.read(file_path, dtype='float32')
            sd.play(data, samplerate)
            self.get_logger().info(f"Playing audio for command: {file_name}")
            sd.wait()
        except Exception as e:
            self.get_logger().error(f"Error playing audio file {file_name}: {e}")

    
def main(args=None):
    rclpy.init(args=args)
    audio_player = AudioPlayer()
    rclpy.spin(audio_player)
    audio_player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()