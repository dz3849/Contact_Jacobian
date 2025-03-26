#!/usr/bin/env python3

from typing import Optional
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from dynamic_reconfigure.server import Server
from trikey.cfg import JacobianConfig
import message_filters
import math
import time
class ContactJacobian:
    def __init__(
        self,
        rw: float,
        rr: float,
        M: float,
        Br: float,
        alpha: float,
        Iw: float,
        Ir: float,
        Ib: float,
        TractionTorque: float
    ) -> None:
        # ---- Dynamic Reconfigure Server ----
        self.server = Server(JacobianConfig, self.dynamic_reconfig_callback)

        # ---- Robot parameters ----
        self.rw: float = rw
        self.rr: float = rr
        self.M: float = M
        self.Br: float = Br 
        self.alpha: float = alpha
        self.Iw: float = Iw
        self.Ir: float = Ir
        self.Ib: float = Ib
        self.TractionTorque: float = TractionTorque
        self.visualize_threshold: float = 40.0

        # ---- Internal state (initialized to None or placeholders) ----
        self.theta: Optional[float] = None                 # yaw
        self.velocity: Optional[np.ndarray] = None         # vx, vy, wz
        self.torque_sensed: Optional[np.ndarray] = None    # from /filtered_torque_data
        self.Jcw: Optional[np.ndarray] = None
        self.Jcwdot: Optional[np.ndarray] = None
        self.Jcwinv: Optional[np.ndarray] = None
        self.Jcwdot_inv: Optional[np.ndarray] = None
        self.Jcr: Optional[np.ndarray] = None
        self.Jcrdot: Optional[np.ndarray] = None
        self.acceleration: Optional[np.ndarray] = None
        self.x_ddot: Optional[float] = None
        self.y_ddot: Optional[float] = None

        self.Br_dynamic: Optional[np.ndarray] = None

        self.t_now: Optional[float] = None
        self.t_last: Optional[float] = None
        self.delta_t: float = 0.0

        self.t_last_torque: Optional[rospy.Time] = None

        self.angular_vel_wheels_now: Optional[np.ndarray] = None
        self.angular_vel_wheels_prev: Optional[np.ndarray] = None
        self.wheel_angular_acceleration: Optional[np.ndarray] = None

        self.theta_dot_now: Optional[float] = None
        self.theta_dot_prev: Optional[float] = None
        self.theta_ddot: Optional[float] = None


        # ---- Marker Publishers ----
        self.pub_sphere = rospy.Publisher("contact_point", Marker,queue_size=10)
        self.pub_arrow = rospy.Publisher("force_arrow", Marker, queue_size=10)
        self.force_value_pub = rospy.Publisher("external_force_values", Float64MultiArray, queue_size=100)
        self.pub_lina = rospy.Publisher("self_acceleration", Float64MultiArray, queue_size=100)

        self.marker = self._init_markers()

        # ---- TF for wheel positions ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.robot_vertices = self.lookup_wheel_positions()


        #wheel0
        #x: -0.127
        #y:  0.220


         #wheel1
        # x: -0.127
        # y: -0.220



        #wheel2 
        # x:  0.25410432670066835
        # y:  0.0


        
        self.robot_vertices = [(-0.127, 0.220), (-0.127, -0.220), (0.25410432670066835, 0.0)]
        self.R = 0.25410432670066835




        if self.robot_vertices is None:
            rospy.logerr("Could not find wheel positions from TF, waiting 0.2 seconds and trying again.")
            rospy.sleep(0.2)
            self.robot_vertices = self.lookup_wheel_positions()
            if self.robot_vertices is None:
                rospy.logerr("Still could not find wheel positions from TF, aborting.")
                rospy.signal_shutdown("TF lookup failed")
                exit(1)
        else:
            rospy.loginfo("Found wheel positions from TF: {}".format(self.robot_vertices))

        # ---- ROS Subscribers ----
        # modelstates_sub=message_filters.Subscriber("/modelstatesetry/filtered", Odometry, queue_size=10000)
        modelstates_sub=message_filters.Subscriber("/gazebo/model_states", ModelStates, queue_size=10000)
        torque_sub=message_filters.Subscriber("/torque_sensor_data", JointState, queue_size=10000)
        joint_sub=message_filters.Subscriber("/joint_states", JointState, queue_size=10000)



        modelstates_stamp_cache = message_filters.Cache(modelstates_sub, 10) # used to check if newest msg is older by 5 seconds than oldest msg in the 10 msg cache, indicating time jump
        torque_stamp_cache = message_filters.Cache(torque_sub, 10)
        joint_stamp_cache = message_filters.Cache(joint_sub, 10)



        # ApproximateTimeSynchronizer:
        # - queue_size=20 means we buffer up to 20 messages per topic
        # - slop=0.1 means we consider messages within 0.1s of each other as "synchronized"
        sync = message_filters.ApproximateTimeSynchronizer(
            [modelstates_stamp_cache, torque_stamp_cache, joint_stamp_cache],
            queue_size=20,
            slop=0.05,
            allow_headerless=True  # if some messages don't have a header stamp
        )
        sync.registerCallback(self.synchronized_callback)

        rospy.loginfo("Contact Jacobian node initialized.")
        self.freq = 100.0
        # self.timer = rospy.Timer(rospy.Duration(0.01), self.update_callback)
        r = rospy.Rate(self.freq)  # 100 Hz in sim time
        while not rospy.is_shutdown():
            try:
                # self.update_callback(None)
                new_modelstates_stamp = modelstates_stamp_cache.getLatestTime()
                new_torque_stamp = torque_stamp_cache.getLatestTime()
                new_joint_stamp = joint_stamp_cache.getLatestTime()

                oldest_modelstates_stamp = modelstates_stamp_cache.getOldestTime()
                oldest_torque_stamp = torque_stamp_cache.getOldestTime()
                oldest_joint_stamp = joint_stamp_cache.getOldestTime()

                if new_modelstates_stamp is not None and new_torque_stamp is not None and new_joint_stamp is not None:                 #dont check this if there are no messages in the cache
                    if new_modelstates_stamp < oldest_modelstates_stamp or new_torque_stamp < oldest_torque_stamp or new_joint_stamp < oldest_joint_stamp: #check if the newest message is older than the oldest message in the cache
                        rospy.logwarn("Time jump detected, resetting sync.")

                        # reset cache and sync
                        modelstates_stamp_cache = None
                        torque_stamp_cache = None
                        joint_stamp_cache = None

                        modelstates_stamp_cache = message_filters.Cache(modelstates_sub, 10)
                        torque_stamp_cache = message_filters.Cache(torque_sub, 10)
                        joint_stamp_cache = message_filters.Cache(joint_sub, 10)

                        sync = None
                        sync = message_filters.ApproximateTimeSynchronizer(
                            [modelstates_stamp_cache, torque_stamp_cache, joint_stamp_cache],
                            queue_size=10000,
                            slop=0.05,
                            allow_headerless=True  # if some messages don't have a header stamp
                        )
                        sync.registerCallback(self.synchronized_callback)


                r.sleep()
                
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("Time jumped backwards—ignoring. Sleeping real time.")
                if new_modelstates_stamp < oldest_modelstates_stamp or new_torque_stamp < oldest_torque_stamp or new_joint_stamp < oldest_joint_stamp:
                    pass


                
                # Fallback to wall-clock sleep so we don’t spin at 100% CPU
                # time.sleep(1/self.freq)





    def dynamic_reconfig_callback(self, config, level):
        self.M = config.mass
        self.Br = np.array([[config.roller_damping_Br]] * 3)
        self.Iw = config.wheel_inertia_iw
        self.Ir = config.roller_inertia_ir
        self.Ib = config.body_inertia_ib
        self.TractionTorque = config.TractionTorque
        rospy.loginfo(
            "Reconfigure Request: mass=%.2f, Br=%.2f, Iw=%.2f, Ir=%.2f, Ib=%.2f, TractionTorque=%.2f",
            self.M, config.roller_damping_Br, self.Iw, self.Ir, self.Ib, self.TractionTorque
        )
        return config

    def _init_markers(self) -> None:
        # Arrow marker for the force vector
        self.arrow_marker = Marker()
        self.arrow_marker.header.frame_id = "base_link"
        self.arrow_marker.type = Marker.ARROW
        self.arrow_marker.scale.x = 0.1
        self.arrow_marker.scale.y = 0.1
        self.arrow_marker.scale.z = 0.1
        self.arrow_marker.color.a = 1.0
        self.threshold = self.visualize_threshold

        # Sphere marker for the contact point
        self.sphere_marker = Marker()
        self.sphere_marker.header.frame_id = "base_link"
        self.sphere_marker.type = Marker.SPHERE
        self.sphere_marker.scale.x = 0.025
        self.sphere_marker.scale.y = 0.025
        self.sphere_marker.scale.z = 0.025
        self.sphere_marker.color.r = 1.0
        self.sphere_marker.color.g = 0.0
        self.sphere_marker.color.b = 0.0
        self.sphere_marker.color.a = 1.0

        # Orientations
        self.arrow_marker.pose.orientation.x = 0.0
        self.arrow_marker.pose.orientation.y = 0.0
        self.arrow_marker.pose.orientation.z = 0.0
        self.arrow_marker.pose.orientation.w = 1.0
        self.sphere_marker.pose.orientation.x = 0.0
        self.sphere_marker.pose.orientation.y = 0.0
        self.sphere_marker.pose.orientation.z = 0.0
        self.sphere_marker.pose.orientation.w = 1.0

        #Lifetime
        self.arrow_marker.lifetime = rospy.Duration(1)
        self.sphere_marker.lifetime = rospy.Duration(1)

    def publish_wheel_markers(self, vertices):
        marker_arr = MarkerArray()
        now = rospy.Time.now()
        for i, (x, y) in enumerate(vertices):
            mk = Marker()
            mk.header.frame_id = "base_link"
            mk.header.stamp = now
            mk.ns = "wheel_positions"
            mk.id = i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = x
            mk.pose.position.y = y
            mk.pose.position.z = 0.0
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.1
            mk.color.a = 1.0
            mk.color.r = 1.0
            mk.color.g = 0.0
            mk.color.b = 0.0
            marker_arr.markers.append(mk)
        # self.wheel_marker_pub.publish(marker_arr)

    def lookup_wheel_positions(self):
        wheel_frames = ["wheel0", "wheel1", "wheel2"]
        base_frame = "base_link"
        vertices = []
        for wheel_frame in wheel_frames:
            try:
                trans = self.tf_buffer.lookup_transform(
                    base_frame, wheel_frame, rospy.Time(0), rospy.Duration(0.1)
                )
            except:
                rospy.logerr("TF transform to {} not found.".format(wheel_frame))
                return None
            else:
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                vertices.append((x, y))
                # store R as distance from center
                self.R = math.sqrt(x**2 + y**2)
        self.publish_wheel_markers(vertices)
        return vertices

    # def modelstates_callback(self, msg: Odometry) -> None:
    def modelstates_callback(self, msg: Imu) -> None:
        # Get yaw from orientation
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.theta = yaw

        theta_dot = msg.angular_velocity.z
        


        # Get Linear Accelerations x_ddot, y_ddot
        self.x_ddot = msg.linear_acceleration.x
        self.y_ddot = msg.linear_acceleration.y
       


        # differentiate angular velocity (theta_dot) to get angular acceleration (theta_ddot)
        if self.theta_dot_now is None:
            self.theta_dot_now = theta_dot
            self.theta_dot_prev = self.theta_dot_now
            return
        else:
            self.theta_dot_prev = self.theta_dot_now
            self.theta_dot_now = theta_dot


        if self.theta is not None:
            self.build_jacobians(self.theta)

    def torque_callback(self, joint_msg: JointState) -> None:
        """ /filtered_torque_data is sensor_msgs/JointState,
            'position' stores the 3 torque values (one per wheel).
        """
        self.torque_sensed = np.array(joint_msg.position).reshape(3, 1)
        self.t_last_torque = rospy.Time.now()

    def wheelcallback(self, joint_msg: JointState) -> None:
        """Subscribe to /joint_states to track wheel velocities and approximate \dot{omega}."""
        if len(joint_msg.velocity) < 3:
            return

    #diffentiate wheel's angular velocity to get their angular acceleration
        if self.angular_vel_wheels_now is None:
            self.angular_vel_wheels_now = np.array(joint_msg.velocity).reshape(3, 1)
            self.angular_vel_wheels_prev = self.angular_vel_wheels_now.copy()
            return
        else:
            self.angular_vel_wheels_prev = self.angular_vel_wheels_now.copy()
            self.angular_vel_wheels_now = np.array(joint_msg.velocity).reshape(3, 1)

        self.wheel_angular_acceleration = ( (self.angular_vel_wheels_now - self.angular_vel_wheels_prev) / self.delta_t)


    def build_jacobians(self, theta: float) -> None:
        self.Jcw = (1.0 / self.rw) * np.array([
            [-math.sin(theta),                      math.cos(theta),                      self.R],
            [-math.sin(theta + 2.0/3.0*math.pi),    math.cos(theta + 2.0/3.0*math.pi),    self.R],
            [-math.sin(theta + 4.0/3.0*math.pi),    math.cos(theta + 4.0/3.0*math.pi),    self.R]
        ])

        self.Jcwdot = (1.0 / self.rw) * np.array([
            [-math.cos(theta),                     -math.sin(theta),                      0.0],
            [-math.cos(theta + 2.0/3.0*math.pi),   -math.sin(theta + 2.0/3.0*math.pi),    0.0],
            [-math.cos(theta + 4.0/3.0*math.pi),   -math.sin(theta + 4.0/3.0*math.pi),    0.0]
        ])

        self.Jcwinv = np.linalg.inv(self.Jcw)
        print("Jcwinv: ", self.Jcwinv)
        self.Jcwdot_inv = np.linalg.pinv(self.Jcwdot) # pseudo-inverse because may become rank-deficient
        print("Jcwdot: ", self.Jcwdot)   
        print("Jcwdot_inv: ", self.Jcwdot_inv)  
        self.Jcr = (1.0 / self.rr) * np.array([
            [math.cos(theta),                      math.sin(theta),                      0.0],
            [math.cos(theta + 2.0/3.0*math.pi),    math.sin(theta + 2.0/3.0*math.pi),    0.0],
            [math.cos(theta + 4.0/3.0*math.pi),    math.sin(theta + 4.0/3.0*math.pi),    0.0]
        ])

        self.Jcrdot = (1.0 / self.rr) * np.array([
            [-math.sin(theta),                     math.cos(theta),                      0.0],
            [-math.sin(theta + 2.0/3.0*math.pi),   math.cos(theta + 2.0/3.0*math.pi),    0.0],
            [-math.sin(theta + 4.0/3.0*math.pi),   math.cos(theta + 4.0/3.0*math.pi),    0.0]
        ])

    def synchronized_callback(self, modelstates_msg, torque_msg, joint_msg) -> None:
        self.modelstates_callback(modelstates_msg)
        self.torque_callback(torque_msg)
        self.wheelcallback(joint_msg)

        if self.theta is None:
            rospy.logwarn_throttle(0.1, "Waiting for Theta")
            return
        if self.torque_sensed is None:
            rospy.logwarn_throttle(0.1, "Waiting for Torque")
            return
        if self.angular_vel_wheels_now is None or self.wheel_angular_acceleration is None:
            rospy.logwarn_throttle(0.1, "Waiting for /joint_states data.")
            return
        


        if self.t_last is None:
            self.t_last = rospy.Time.now().to_sec()
            return
        self.t_now = rospy.Time.now().to_sec()
        self.delta_t = self.t_now - self.t_last
        self.t_last = self.t_now

        if self.t_last_torque is None or (rospy.Time.now() - self.t_last_torque) > rospy.Duration(1):
            self.torque_sensed = None
        if self.delta_t == 0:
            rospy.logwarn_throttle(0.1, "Delta time is zero.")
            return
        self.theta_ddot = (self.theta_dot_now - self.theta_dot_prev) / self.delta_t

        self.acceleration = np.array([[self.x_ddot], [self.y_ddot], [self.theta_ddot]])

        #use wheel velocities to approximate LINEAR velocity of the robot
        self.velocity = np.matmul(self.Jcwdot_inv, self.angular_vel_wheels_now)
        
        self.velocity = np.append(self.velocity[:2], self.theta_dot_now) #use the angular velocity from the IMU instead of the wheel velocities


        #roller velocity q_r_dot = Jcr * self.velocity
        q_r_dot = np.matmul(self.Jcr, self.velocity) 
        #ensure shape is (3,1)
        q_r_dot = q_r_dot.reshape(3,1)

        # solve for Br_dynamic
        self.Br_dynamic = self.Br * np.tanh(self.alpha * q_r_dot)
        


        # self.acceleration = ( np.matmul(self.Jcwinv, self.wheel_angular_acceleration)  + np.matmul(self.Jcwdot_inv, self.angular_vel_wheels_now))





        output_nominal = self.external_forces()
        self.visualize(output_nominal)

        msg = Float64MultiArray()
        msg.data = output_nominal
        self.force_value_pub.publish(msg)

    def external_forces(self):
        """
        Compute external force in body frame from the difference between
        expected torque (no external contact) and sensed torque.
        """
        msg = Float64MultiArray()
        msg.data = self.acceleration.flatten()
        self.pub_lina.publish(msg)

        # T_noFext = Jcwinv^T * [ M*a + Jcr^T * Br ]
        T_noFext = np.matmul(
            self.Jcwinv.T,
            self.M * self.acceleration + np.matmul(self.Jcr.T, self.Br_dynamic)
        )
        print("T_noFext: ", T_noFext)
        if self.torque_sensed is None:
            diff_torque = 0
        else:
            diff_torque = T_noFext - self.torque_sensed
        # print("diff torque: ", diff_torque)

        # F_body = Jcw^T * (T_noFext - T_sensed)
        F_body = np.matmul(self.Jcw.T, diff_torque)
        Fextx = F_body[0, 0]
        Fexty = F_body[1, 0]

        tf_Fext = self.vector_transform([Fextx, Fexty])
        contact_pt = self.force_line_intersection(self.robot_vertices, tf_Fext)

        # rospy.loginfo(
            # "Contact=(%.3f, %.3f), Fext=(%.3f, %.3f)",
            # contact_pt[0], contact_pt[1], Fextx, Fexty
        # )
        return [contact_pt[0], contact_pt[1], Fextx, Fexty]

    def vector_transform(self, Fext):
        """Rotate global force -> local frame by -theta."""
        x_global, y_global = Fext
        cosT = math.cos(self.theta)
        sinT = math.sin(self.theta)
        x_n = cosT * x_global + sinT * y_global
        y_n = -sinT * x_global + cosT * y_global
        return [x_n, y_n]

    def force_line_intersection(self, robot_vertices, Fext):
        """
        Intersect lines of the triangular robot frame with the direction of Fext.
        """
        top_left, bottom_tip, top_right = robot_vertices
        edges = [
            (top_left, bottom_tip),
            (top_left, top_right),
            (bottom_tip, top_right),
        ]

        intersections = []
        edge_flag = [False, False, False]
        contact_point = [0, 0]
        edge_count = -1

        for edge_start, edge_end in edges:
            edge_count += 1
            denom = (Fext[1] * (edge_end[0] - edge_start[0])
                     - Fext[0] * (edge_end[1] - edge_start[1]))
            if abs(denom) < 1e-12:
                continue
            s = ((Fext[0] * edge_start[1]) - (Fext[1] * edge_start[0])) / denom
            if 0 <= s <= 1:
                x = edge_start[0] + s * (edge_end[0] - edge_start[0])
                y = edge_start[1] + s * (edge_end[1] - edge_start[1])
                edge_flag[edge_count] = True
                intersections.append((x, y))

        if (Fext[0] == 0 and Fext[1] == 0):
            rospy.loginfo("No intersection, zero force.")
            return [0, 0]

        elif Fext[0] != 0:
            if edge_flag[0] and edge_flag[1]:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            elif edge_flag[0] and edge_flag[2]:
                if Fext[0] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            elif edge_flag[1] and edge_flag[2]:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            else:
                if len(intersections) == 1:
                    contact_point = intersections[0]
        else:
            # purely vertical Fext
            if len(intersections) == 1:
                contact_point = intersections[0]
            elif len(intersections) >= 2:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[-1]

        return contact_point

    def visualize(self, output_nominal) -> None:
        contact_x, contact_y, Fextx, Fexty = output_nominal
        norm = math.hypot(Fextx, Fexty)

        # Sphere marker
        self.sphere_marker.header.stamp = rospy.Time.now()
        self.sphere_marker.pose.position.x = contact_x
        self.sphere_marker.pose.position.y = contact_y
        self.sphere_marker.pose.position.z = 0.0

        if norm < self.threshold:
            # Below threshold, hide arrow, reset sphere to origin
            self.sphere_marker.pose.position.x = 0
            self.sphere_marker.pose.position.y = 0
            self.sphere_marker.pose.position.z = 0
            self.pub_sphere.publish(self.sphere_marker)
            self.arrow_marker.scale.x = 0.0
            self.arrow_marker.scale.y = 0.0
            self.arrow_marker.scale.z = 0.0
            self.arrow_marker.points = [Point(x=0.0,y=0.0,z=0.0), Point(x=0.0,y=0.0,z=-9.8)]
            self.pub_arrow.publish(self.arrow_marker)

            # rospy.loginfo("Force magnitude below threshold, not visualizing arrow.")
            return

        self.pub_sphere.publish(self.sphere_marker)

        arrow_length = 0.5
        start_pt = Point(x=contact_x, y=contact_y, z=0.0)
        end_pt = Point(
            x=contact_x + arrow_length * Fextx / norm,
            y=contact_y + arrow_length * Fexty / norm,
            z=0.0
        )
        # rospy.loginfo("arrow end_pt=(%.3f, %.3f)", end_pt.x, end_pt.y)

        # For the arrow color
        self.arrow_marker.color.r = 0.0
        self.arrow_marker.color.g = 1.0
        self.arrow_marker.color.b = 0.0
        self.arrow_marker.scale.x = 0.15
        self.arrow_marker.scale.y = 0.1
        self.arrow_marker.scale.z = 0.05
        self.arrow_marker.header.stamp = rospy.Time.now()
        self.arrow_marker.points = [start_pt, end_pt]
        self.pub_arrow.publish(self.arrow_marker)

def main():
    rospy.init_node("Contact_Jacobian", anonymous=True)

    # Get params from ROS param server
    rw = rospy.get_param("~wheel_radius", 0.1)
    rr = rospy.get_param("~roller_radius", 0.00918135)
    BotMass = rospy.get_param("~mass", 30)
    Br = rospy.get_param("~roller_damping_Br", 0.2)
    Iw = rospy.get_param("~wheel_inertia_iw", 1)
    Ir = rospy.get_param("~roller_inertia_ir", 1)
    Ib = rospy.get_param("~body_inertia_ib", 1)
    TractionTorque = rospy.get_param("~TractionTorque", 1)

    external_torque = ContactJacobian(rw, rr, BotMass, Br, 0.4, Iw, Ir, Ib, TractionTorque)
    rospy.spin()

if __name__ == "__main__":
    main()
