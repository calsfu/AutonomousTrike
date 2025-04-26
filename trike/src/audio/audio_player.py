#!/usr/bin/env python3
import rclpy
import sounddevice as sd
import os
import soundfile as sf
from std_msgs.msg import Int8
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from enum import Enum

audio_files = {
    0: "autonomous_mode.wav",
    1: "brakes_off.wav",
    2: "brakes_on.wav",
    3: "destination_confirmed.wav",
    4: "destination_set_to.wav",
    5: "manual_mode.wav",
    6: "neutral_mode.wav",
    7: "park_mode.wav",
    8: "no_gps_signal_found.wav",
    9: "please_enter_a_destination.wav",
    10: "system_ready.wav",
    11: "turning_left.wav",
    12: "turning_right.wav",
    13: "would_you_like_to_confirm_this_destination.wav"
}

class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        self.audio_dir = os.path.join(get_package_share_directory('trike'), 'audio')
        if not os.path.exists(self.audio_dir):
            self.get_logger().error(f"Audio directory {self.audio_dir} does not exist.")
            return
        self.create_subscription(Int8, 'audio_command', self.listener_callback, 10)
        self.audio_data = {}
        self.preload_audio_files()
        self.get_logger().info('Audio Player Initialized')

    def listener_callback(self, msg):
        command = msg.data
        self.play_audio(command)
        self.get_logger().info(f"Received audio command: {command}")
        # if command in AudioFile._value2member_map_:
        #     audio_file = AudioFile._filename_map[AudioFile(command)]
        #     self.play_audio(audio_file)
        # else:
        #     self.get_logger().error(f"Invalid audio command: {command}")

    def play_audio(self, command):
        try:
            data, samplerate = self.audio_data[command]
            sd.play(data, samplerate)
            self.get_logger().info(f"Playing audio for command: {audio_files[command]}")
            # sd.wait()
        except Exception as e:
            self.get_logger().error(f"Error playing audio file {audio_files[command]}: {e}")

    def preload_audio_files(self):
        for command, file_name in audio_files.items():
            try:
                file_path = os.path.join(self.audio_dir, file_name)
                data, samplerate = sf.read(file_path, dtype='float32')
                self.audio_data[command] = (data, samplerate)
                self.get_logger().info(f"Preloaded audio file for command: {file_name}")
            except Exception as e:
                self.get_logger().error(f"Error preloading audio file {file_name}: {e}")
           

    
def main(args=None):
    rclpy.init(args=args)
    audio_player = AudioPlayer()
    rclpy.spin(audio_player)
    audio_player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()