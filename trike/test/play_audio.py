import sounddevice as sd
import os
import soundfile as sf

audio_dir = './audio'

def listener_callback(command):
    if command == 'system_ready':
        play_audio('system_ready.wav')
    elif command == 'left':
        play_audio('turning_left.wav')
    elif command == 'right':
        play_audio('turning_right.wav')
    elif command == 'brake_applied':
        play_audio('brakes_on.wav')
    elif command == 'brake_released':
        play_audio('brakes_off.wav')
    elif command == 'park':
        play_audio('park_mode.wav')
    elif command == 'neutral':
        play_audio('neutral_mode.wav')
    elif command == 'manual':
        play_audio('manual_mode.wav')
    elif command == 'autonomous':
        play_audio('autonomous_mode.wav')
    elif command == 'left':
        play_audio('turning_left.wav')
    elif command == 'enter_destination':
        play_audio('please_enter_a_destination.wav')
    elif command == 'destination_set':
        play_audio('destination_set_to.wav')
    elif command == 'would_you_like_to_confirm':
        play_audio('would_you_like_to_confirm_this_destination.wav')
    elif command == 'confirmed':
        play_audio('destination_confirmed.wav')
    elif command == 'no_gps':
        play_audio('no_gps_signal_found.wav')
    else:
        print(f"Unknown command: {command}")

def play_audio(file_name):
    try:
        file_path = os.path.join(audio_dir, file_name)
        data, samplerate = sf.read(file_path, dtype='float32')
        print(f"Playing {file_path}...")
        sd.play(data, samplerate)
        sd.wait()
    except Exception as e:
        print(f"Error playing audio file {file_name}: {e}")

def main(args=None):
    commands = [
        'system_ready',
        'left',
        'right',
        'brake_applied',
        'brake_released',
        'park',
        'neutral',
        'manual',
        'autonomous',
        'enter_destination',
        'destination_set',
        'would_you_like_to_confirm',
        'confirmed',
        'no_gps'
    ]
    for command in commands:
        print(f"Simulating command: {command}")
        listener_callback(command)

if __name__ == "__main__":
    main()