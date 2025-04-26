#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32
from evdev import InputDevice, categorize, ecodes
import sys

class RawKeyboardNode(Node):
	def __init__(self):
		super().__init__('raw_keyboard_listener')
		self.brake_publisher_ = self.create_publisher(Int8, 'control/brake', 10)  # publisher for braking
		self.steer_publisher_ = self.create_publisher(Int8, 'control/steer', 10) # publisher for steering
		self.dev_path = '/dev/input/by-id/usb-LiteOn_Lenovo_Traditional_USB_Keyboard-event-kbd'
		self.dev = None
		self.last_steer = 0
		self.last_brake = 0

		try:
			self.dev = InputDevice(self.dev_path)
			self.get_logger().info(f'Listening to device: {self.dev.name} {self.dev.phys}')
		except FileNotFoundError:
			self.get_logger().error(f'Device not found at {self.dev_path}')
			sys.exit(1)
		except PermissionError:
			self.get_logger().error(f'Permission denied to access {self.dev_path}. You might need root or specific group permissions.')
			sys.exit(1)
		except Exception as e:
			self.get_logger().error(f'Error opening device: {e}')
			sys.exit(1)

	def read_events(self):
		if self.dev:
			for event in self.dev.read_loop():
				if event.type == ecodes.EV_KEY:
					key_event = categorize(event)
					key = key_event.keycode
					event_value = event.value
					# if event_value != 1:
					# 	return
					# brake message
					brake_msg = Int8()
					brake_msg.data = self.last_brake

					# steer message
					# steer_msg = Float32()
					# steer_msg.data = 0.0

					steer_msg = Int8()
					steer_msg.data = self.last_steer

					# check for quit
					if(key == 'KEY_Q'):
						self.get_logger().info(f'Shutdown initiated by {self.get_name()}')
						self.destroy_node()
					# check for activate brake
					# 0 for activate brake, 1 for release brake
					if(key == 'KEY_W'):
						brake_msg.data = 0
					elif(key == 'KEY_S'):
						brake_msg.data = 1


					# check for steer
					# if(key == 'a'):
					#     steer_msg.data = -1.0
					# elif(key == 'd'):
					#     steer_msg.data = 1.0

					# j for left, k for straight, l for right
					if(key == 'KEY_J'):
						steer_msg.data = -1
					elif(key == 'KEY_K'):
						steer_msg.data = 0
					elif(key == 'KEY_L'):
						steer_msg.data = 1

					self.last_steer = steer_msg.data
					self.last_brake = brake_msg.data

					# publish messages
					self.brake_publisher_.publish(brake_msg)
					self.steer_publisher_.publish(steer_msg)

					# log info
					self.get_logger().info(f'Publishing: {brake_msg.data} to /control/brake')
					self.get_logger().info(f'Publishing: {steer_msg.data} to /control/steer')
				
		

def main(args=None):
	rclpy.init(args=args)
	raw_keyboard_node = RawKeyboardNode()
	raw_keyboard_node.read_events()
	raw_keyboard_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
