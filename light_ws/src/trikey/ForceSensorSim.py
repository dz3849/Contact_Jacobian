#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

# Global array to hold torque values from three sensors
sensed_torques = [0.0, 0.0, 0.0]

def callback_sensor_1(msg):
    # In a WrenchStamped, the actual torque is in msg.wrench.torque
    sensed_torques[0] = msg.wrench.torque.z

def callback_sensor_2(msg):
    sensed_torques[1] = msg.wrench.torque.z

def callback_sensor_3(msg):
    sensed_torques[2] = msg.wrench.torque.z

def main():
    rospy.init_node("ForceSensorSim", anonymous=True)

    # Create a publisher for JointState messages
    pub = rospy.Publisher("/torque_sensor_data", JointState, queue_size=10)

    # Create subscribers for the three WrenchStamped topics
    rospy.Subscriber("/ft_sensor_1_topic", WrenchStamped, callback_sensor_1)
    rospy.Subscriber("/ft_sensor_2_topic", WrenchStamped, callback_sensor_2)
    rospy.Subscriber("/ft_sensor_3_topic", WrenchStamped, callback_sensor_3)

    # We will publish continuously at, for example, 10 Hz
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Create and populate the JointState message
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        
        # Name your joints (one name per wheel)
        js_msg.name = ["wheel_link_1", "wheel_link_2", "wheel_link_3"]
        
        # Put torque values in the position array
        js_msg.position = [sensed_torques[0], 
                           sensed_torques[1], 
                           sensed_torques[2]]
        # You could also store torque in effort[] if you prefer,
        # but you specifically asked to store torque in position.

        # Publish the JointState
        pub.publish(js_msg)

        # Sleep to maintain the loop at 10 Hz
        rate.sleep()

if __name__ == '__main__':
    main()
