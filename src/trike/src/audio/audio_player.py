import rclpy
import sounddevice as sd
import os
import soundfile as sf
from std_msgs.msg import String
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        self.audio_dir = os.path.join(get_package_share_directory('trike'), 'audio')
        if not os.path.exists(self.audio_dir):
            self.get_logger().error(f"Audio directory {self.audio_dir} does not exist.")
            return
        self.create_subscription(String, 'audio_command', self.listener_callback, 10)
        self.get_logger().info('Audio Player Initialized')

    def listener_callback(self, msg):
        command = msg.data
        if command == 'system_ready':
            self.play_audio('system_ready.wav')
        elif command == 'left':
            self.play_audio('turning_left.wav')
        elif command == 'right':
            self.play_audio('turning_right.wav')
        elif command == 'brake_applied':
            self.play_audio('brakes_on.wav')
        elif command == 'brake_released':
            self.play_audio('brakes_off.wav')
        elif command == 'park':
            self.play_audio('park_mode.wav')
        elif command == 'neutral':
            self.play_audio('neutral_mode.wav')
        elif command == 'manual':
            self.play_audio('manual_mode.wav')
        elif command == 'autonomous':
            self.play_audio('autonomous_mode.wav')
        elif command == 'left':
            self.play_audio('turning_left.wav')
        elif command == 'enter_destination':
            self.play_audio('please_enter_a_destination.wav')
        elif command == 'destination_set':
            self.play_audio('destination_set_to.wav')
        elif command == 'would_you_like_to_confirm':
            self.play_audio('would_you_like_to_confirm_this_destination.wav')
        elif command == 'confirmed':
            self.play_audio('destination_confirmed.wav')
        elif command == 'no_gps':
            self.play_audio('no_gps_signal_found.wav')
        else:
            self.get_logger().info(f"Unknown command: {command}")
            return

    def play_audio(self, file_name):
        try:
            file_path = os.path.join(self.audio_dir, file_name)
            data, samplerate = sf.read(file_path, dtype='float32')
            sd.play(data, samplerate)
            self.get_logger().info(f"Playing audio for command: {file_name}")
            sd.wait()
        except Exception as e:
            self.get_logger().error(f"Error playing audio file {file_name}: {e}")

    