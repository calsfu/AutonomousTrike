import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
from scipy.io.wavfile import read
import os

class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        self.subscription = self.create_subscription(
            String,
            'play_audio',
            self.listener_callback,
            10
        )
        # Set audio directory relative to this file
        self.audio_dir = os.path.join(os.path.dirname(__file__), '..', 'audio')

    def listener_callback(self, msg):
        command = msg.data.lower()
        if command == 'system_on':
            self.play_audio('system_on.wav')
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
            self.play_audio('neutral-mode.wav')
        elif command == 'manual':
            self.play_audio('manual-mode.wav')
        elif command == 'autonomous':
            self.play_audio('autonomous-mode.wav')
        elif command == 'left':
            self.play_audio('turning_left.wav')
        elif command == 'enter_destination':
            self.play_audio('please-enter-a-destination.wav')
        elif command == 'destination_set':
            self.play_audio('destination-set-to.wav')
        elif command == 'would_you_like_to_confirm':
            self.play_audio('would-you-like-to-confirm-this-destination.wav')
        elif command == 'confirmed':
            self.play_audio('destination-confirmed.wav')
        elif command == 'no_gps':
            self.play_audio('no-gps-signal-found.wav')
        else:
            self.get_logger().warn(f'Unknown audio command: {command}')

    def play_audio(self, file_name):
        try:
            file_path = os.path.join(self.audio_dir, file_name)
            rate, data = read(file_path)
            sd.play(data, rate)
            sd.wait()
            self.get_logger().info(f'Played: {file_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to play {file_name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    audio_player = AudioPlayer()
    rclpy.spin(audio_player)
    audio_player.destroy_node()
    rclpy.shutdown()
