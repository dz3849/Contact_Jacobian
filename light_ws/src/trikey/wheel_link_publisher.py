#! /usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetLinkProperties

def get_link_properties(link_name="wheel_link"):
    rospy.wait_for_service('/gazebo/get_link_properties')
    try:
        get_link_properties = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
        response = get_link_properties(link_name)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node("get_link_properties_example")
    get_link_properties()