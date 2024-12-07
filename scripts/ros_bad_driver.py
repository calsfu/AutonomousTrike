import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('driving_declaring', String, queue_size=10)
    rospy.init_node('driving_control', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    throttle = 0.
    steering = -.5
    while not rospy.is_shutdown():
        command = str(throttle) + "," + str(steering) + ",0"
        rospy.loginfo(command)
        pub.publish(command)
        throttle = throttle + .1
        if throttle == .4:
            throttle = 0.
        steering = steering + .1
        if steering == .5:
            steering = -.5
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass