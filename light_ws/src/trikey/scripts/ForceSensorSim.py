#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

# Low-pass filter coefficient (0 < ALPHA <= 1). Smaller means slower response.
ALPHA = 0.05

# Global variables to store the filtered torques from each sensor
filtered_torques = [0.0, 0.0, 0.0]

def callback_sensor_1(msg):
    global filtered_torques
    new_val = msg.wrench.torque.z
    filtered_torques[0] = (1 - ALPHA) * filtered_torques[0] + ALPHA * new_val

def callback_sensor_2(msg):
    global filtered_torques
    new_val = msg.wrench.torque.z
    filtered_torques[1] = (1 - ALPHA) * filtered_torques[1] + ALPHA * new_val

def callback_sensor_3(msg):
    global filtered_torques
    new_val = msg.wrench.torque.z
    filtered_torques[2] = (1 - ALPHA) * filtered_torques[2] + ALPHA * new_val

def timer_callback(event):
    """ 
    Timer callback to publish JointState messages with nearly identical timestamps.
    """
    js_msg = JointState()
    # Use a single timestamp for the entire message.
    js_msg.header.stamp = rospy.Time.now()
    js_msg.name = ["wheel_joint_1", "wheel_joint_2", "wheel_joint_3"]
    js_msg.position = filtered_torques.copy()
    pub.publish(js_msg)

if __name__ == '__main__':
    rospy.init_node("ForceSensorSim", anonymous=True)

    # Create the publisher for JointState messages.
    pub = rospy.Publisher("/torque_sensor_data", JointState, queue_size=10)

    # Subscribe to the three sensor topics.
    rospy.Subscriber("/ft_sensor_1_topic", WrenchStamped, callback_sensor_1)
    rospy.Subscriber("/ft_sensor_2_topic", WrenchStamped, callback_sensor_2)
    rospy.Subscriber("/ft_sensor_3_topic", WrenchStamped, callback_sensor_3)

    # Create a Timer that calls timer_callback at 100 Hz.
    # This ensures that each published message uses a single, tightly synchronized timestamp.
    rospy.Timer(rospy.Duration(0.01), timer_callback)

    rospy.spin()
