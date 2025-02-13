#! /usr/bin/env python3
import numpy as np
import rospy 
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
#Fix name space issue and i hopefully sdhould be good to go :(
# and this error
#[WARN] [1732923937.001813, 377.757000]: Controller Spawner couldn't find the expected controller_manager ROS interface.
class ContactJacobian():
    def __init__(self, R, rw, L, M, Br, Iw, Ir, Ib, TractionTorque):

        self.R = R
        self.rw = rw #wheel radius
        self.rr = 1 #roller radius
        self.M = M
        self.Br = [[Br], [Br], [Br]]
        self.Iw = Iw
        self.Ir = Ir
        self.Ib = Ib
        self.TractionTorque = TractionTorque
        self.torque_values=[]
        self.Jcw = None
        self.Jcwdot = None
        self.Jcr = None
        self.Jcrdot = None

        rospy.init_node("BumpyBotNode", anonymous=True)

        self.sub_body = rospy.Subscriber("/gazebo/model_states", ModelStates, callback = self.position_callback) 
        self.theta = None
        rospy.Subscriber("/gazebo/model_states", ModelStates , callback = self.thetacallback) # absolute orientation of the wheel
        self.velocity = None
        rospy.Subscriber("/gazebo/model_states", Twist, callback = self.vel_callback) # this is a vector
        self.base_angular_acceleration = None
        rospy.Subscriber("/torque_sensor_data", JointState, callback = self.torquecallback) #switch to subsribing to /trikey_light/joint_states
        self.t  = rospy.get_rostime()
        #rospy.Subscriber("/clock", Time, self.time_callback)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_callback)

        #create basic marker to visualize the external force
        self.pub = rospy.Publisher("external_force", Marker, queue_size=10)
        self.marker = Marker() 
        self.marker.header.frame_id = "center_link"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "external_force"
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0


        self.Ts = None
        self.x = None
        self.y = None
        #self.robot_vertices =[(0.250216,-0.144463), (0, 0.288925), (-0.250216,-0.144463)]
        # self.robot_vertices = [(-0.250216,0.144463),(0,-0.288925),(0.250216,0.144463)]


        # self.robot_vertices = [(2.4, 3.36),(-3.4175, 0),(2.4, -3.36)]

        self.robot_vertices = [(2.4, 3.36),(-3.4175, 0),(2.4, -3.36)]

        #Jacobian matrix for frame conversion all angles in radians
        
        self.wheel_angular_acceleration = None
        self.acceleration = None
        # get robot values from old paper or frank
        #chmod +x the file to make it executable
        #dig into the messages i need to receive for each variable defined
        # TODO: fix the math in equation 42 here and figure out how to display jointstates correctly. no tpublishin genough values for tf? set joint_states to only one link?



    def update_callback(self, event):
        # Make sure required variables (like self.theta) are set before updating.
        if self.theta is None:
            rospy.logwarn("Theta not yet set.")
            return

        output_nominal = self.external_forces()
        self.visualize(output_nominal)

    def position_callback(self, data):
       robot_index = data.name.index('trikey_light')

       robot_pose = data.pose[robot_index].orientation
       self.x = robot_pose.x
       self.y = robot_pose.y
       
    def vel_callback(self, data):
        try:
            robot_index = data.name.index('trikey_light')
            
            robot_twist = data.twist[robot_index]

            self.velocity = [[robot_twist.linear.x], [robot_twist.linear.y], [robot_twist.angular.z]]
            self.wheel_angular_velocity = np.matmul(self.Jcw, self.velocity)
            self.roller_angular_velocity = np.matmul(self.Jcr, self.velocity)


        except ValueError:
            rospy.logerr("Robot model 'trikey_light' not found in /gazebo/model_states")
        except Exception as e:
            rospy.logerr(f"Error in vel_callback: {str(e)}")

    def thetacallback(self, data):
        try:
            robot_index = data.name.index('trikey_light')
            
            # Extract the quaternion from the pose
            quaternion = data.pose[robot_index].orientation
            q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            
            # Convert the quaternion to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = euler_from_quaternion(q)
            
            self.theta= yaw
            self.Jcw = 1/self.rw*np.array([[-np.sin(self.theta), np.cos(self.theta), self.R ], [-np.sin(self.theta + 2/3*np.pi),
                np.cos(self.theta + 2/3*np.pi), self.R ], [-np.sin(self.theta + 4/3*np.pi), np.cos(self.theta + 4/3*np.pi), self.R]])
            self.Jcwdot = 1/self.rw*np.array([[-np.cos(self.theta), -np.sin(self.theta), 0.0], [-np.cos(self.theta+2/3*np.pi), 
                -np.sin(self.theta + 2/3*np.pi), 0.0], [-np.cos(self.theta + 4/3*np.pi), -np.sin(self.theta + 4/3*np.pi,), 0.0]])
            self.Jcwinv = np.linalg.inv(self.Jcw)
            self.Jcwdot_inv = np.linalg.pinv(self.Jcwdot)

            self.Jcr = 1/self.rr*np.array([[np.cos(self.theta), np.sin(self.theta), 0], [np.cos(self.theta + 2/3*np.pi),
                np.sin(self.theta + 2/3*np.pi), 0], [np.cos(self.theta + 4/3*np.pi), np.sin(self.theta + 4/3*np.pi), 0]])#equation 16
            self.Jcrdot = 1/self.rr*np.array([[-np.sin(self.theta), np.cos(self.theta), self.R ], [-np.sin(self.theta + 2/3*np.pi),
                np.cos(self.theta + 2/3*np.pi), self.R ], [-np.sin(self.theta + 4/3*np.pi), np.cos(self.theta + 4/3*np.pi), self.R]])
            #rospy.loginfo (f"inverse jacobian {self.Jcwdot_inv}")
            #rospy.loginfo(f"Robot yaw angle: {self.theta} radians")
        except ValueError:
            rospy.logerr(f"Robot trikey_light not found in model_states")

    def torquecallback(self, data):   
        try:
            self.Ts = data.position
            # rospy.loginfo(f"Instantaneous torque for the wheels: {self.Ts} Nm")
            
        except ValueError:
            rospy.logwarn(f"Joints maybe not found in JointState message")



    def NominalTorque(self):
        transpose_wheel = np.transpose(self.Jcw)
        self.wheel_angular_acceleration = [[6*(np.pi**2)*((self.theta)**2)*np.cos(2*np.pi*self.theta*self.t.to_sec())], [0], [0]]#given before equation 29 #assuming wheel trajectory angle is theta since it's wheel 0
        self.angular_vel_wheel= [[3*np.pi*self.theta*np.sin(2*np.pi*self.theta*self.t.to_sec())], [0], [0]]
        # self.acceleration = np.linalg.inv(self.Jcw)*self.wheel_angular_acceleration + np.matmul(np.linalg.inv(self.Jcwdot), self.wheel_angular_velocity) #equation 29
        # self.acceleration = np.linalg.inv(self.Jcw)*self.wheel_angular_acceleration + np.linalg.pinv(self.Jcwdot)*self.scalar_wheel_qdot
        self.acceleration = np.matmul(self.Jcwinv, self.wheel_angular_acceleration) + np.matmul(self.Jcwdot_inv, self.angular_vel_wheel) #FOR THE LOVE OF GOD CHANGE THIS SHIT PLEASE I CANNOT FORGET IT AGAIN
        self.TNom = np.linalg.inv(transpose_wheel)*(self.M*self.acceleration+self.Br) + self.Ir*self.wheel_angular_acceleration #THIS LINE IS NOT NEEDED
    
    def torque_no_fext(self):
        return np.matmul(np.linalg.inv(np.transpose(self.Jcw)), (self.M*self.acceleration) + np.matmul(np.transpose(self.Jcr), self.Br))#equation 38 without transpose(Jcr)
            
    # need help on hte last row of equation 42
    def external_forces(self): 
        self.NominalTorque()
        self.T_ext_not = self.torque_no_fext()
        rospy.loginfo(f"nominal toruqe:{self.T_ext_not}")
        
        rospy.loginfo(f"Sensed Torque: {self.Ts}")
        RH_Matrix = np.matmul(np.linalg.inv(np.transpose(self.Jcw)), self.T_ext_not - self.Ts) # self.Ts
        rospy.loginfo(f"RH Matrix: {RH_Matrix}")
        self.Fextx = RH_Matrix[0][0]
        self.Fexty = RH_Matrix[1][0]
        Fext = [self.Fextx, self.Fexty]
        tf_Fext = self.vector_transform(Fext)
        #self.find_robot_vertices(self.theta)
        intersection = self.force_line_intersection(self.robot_vertices, tf_Fext)
    
        return [intersection[0], intersection[1], self.Fextx, self.Fexty] #Final Output
    
    def vector_transform(self, Fext):
        """ 
        Take the force line from the global perspective and transform it to the local frame of the robot

        Parameters: 
        self: instance of class
        Fext: the external force vector

        Return:
        new_vector: The transformed force vector on the local frame

        """
        x_n = np.cos(self.theta)*Fext[0] + np.sin(self.theta)* Fext[1]
        y_n = -np.sin(self.theta)*Fext[0] + np.cos(self.theta)* Fext[1]
        new_vector = [x_n, y_n]
        return new_vector


    def force_line_intersection(self, robot_vertices, Fext):
        """ 
        Solves parametric parameter s separately for two lines that it intersects
        Finds the first intersection of the force vector
        Need to transform the force vector to a local frame where the centroid of the robot body is (0,0)
        *only rotational motion is considered

        Parameters: 
        self: instance of class
        robot_vertices: the vertices of the robot

        Return:
        contact_point: The point where the external force first intersects the robot
        """
        top_left, bottom_tip, top_right = robot_vertices[0], robot_vertices[1], robot_vertices[2]

        # Edges of the triangle
        edges = [(top_left, bottom_tip), (top_left, top_right), (bottom_tip, top_right)]

        intersections = [] #parametric parameter along the edge. s in order will be point one edge 1, 2, then 3
        edge_flag = [False, False, False]
        edge_count = -1
        contact_point = [0, 0]
        # Loop through each edge of the triangle
        #Centroid on the local frame is (0,0)
        for edge_start, edge_end in edges: #checks for edge 1 to edge 2 to edge 3
            edge_count += 1
            # s = (Fext[0]*(edge_start[1] - 0) - Fext[1]*(edge_start[0] - 0))/(Fext[1]*(edge_end[0] - edge_start[0]) - Fext[0]*(edge_end[1] - edge_end[1]))
            # Calculate the denominator correctly
            denom = (Fext[1] * (edge_end[0] - edge_start[0])
                    - Fext[0] * (edge_end[1] - edge_start[1]))

            # Avoid division by zero if denom == 0
            if abs(denom) < 1e-12:
                continue

            # Parametric "s" for the edge
            s = ((Fext[0] * edge_start[1]) - (Fext[1] * edge_start[0])) / denom
            if s >= 0 and s <= 1:
            
                x = edge_start[0] + s * (edge_end[0] - edge_start[0])
                y = edge_start[1] + s * (edge_end[1] - edge_start[0])
                edge_flag[edge_count] = True
                intersections.append((x, y))

        #The stuff for first contact point    

        if Fext[0] != 0:
            if edge_flag[0] == True and edge_flag[1] == True:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            if edge_flag[0] == True and edge_flag[2] ==True:
                if Fext[0] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point[1] = intersections[1]

            if edge_flag[1] == True and edge_flag[2] ==True:
                if Fext[1] > 0:
                    contact_point = intersections[0]
                else:
                    contact_point = intersections[1]
            else: #this is when m = 0, a horizontal line
                if Fext[0] > 0:
                    contact_point = intersections[0]

                else:
                    contact_point = intersections[1]
        else: # slope in x is 0, vertical line
            if Fext[1] > 0: 
                if edge_flag[0] == True and edge_flag[1] == True: # edge1 and edge2
                    contact_point = intersections[1]
                else:
                    contact_point = intersections[0]
            else: # edge2 awnd edge 3
                 contact_point = intersections[0]
 
        return contact_point



    def global_point_transform(self, cp):
        """ 
        Take the contact point from the local perspective and transform it to the global frame of the robot

        Parameters: 
        self: instance of class
        contact point: the external foce contact point on the local frame

        Return:
        cp_global: The transformed contact point back to the global frame

        """
        x_n = self.x + np.cos(self.theta)*cp[0] - np.sin(self.theta)* cp[1]
        y_n = self.y + np.sin(self.theta)*cp[0] + np.cos(self.theta)* cp[1]
        cp_global = [x_n, y_n]
        return cp_global

    # This shows where the vertices are in the global frame aka top view of the whole world
    # No need to use this atm
    def find_robot_vertices(self, theta, distances = [0.288925, 0.288925, 0.288925], angles = [0, 2/3*np.pi, 4/3*np.pi]):
        index = 0
        for d, phi in zip(distances, angles):
            # Calculate the new coordinates with rotation
            x = self.x + d * np.cos(phi + theta)
            y = self.y + d * np.sin(phi + theta)
            self.robot_vertices[index] = (x, y)
            index += 1


    def visualize(self, output_nominal):
        rospy.loginfo("Visualizing external force...")
        contact_x, contact_y, Fextx, Fexty = output_nominal

        end_point = Point(x=contact_x, y=contact_y, z=0.0)
        scale = 0.1
        start_point = Point( x=contact_x *-Fextx,  y=contact_y *-Fexty,z=0.0)

        self.marker.header.stamp = rospy.Time.now()
        self.marker.points = [start_point, end_point]
        self.pub.publish(self.marker)
        rospy.loginfo("External force visualization complete.")



def main():
    rr = 1  # roller radius
    rw = 1  # wheel radius
    R = 1
    BotMass = 1.5
    RollerMass = 0
    Br = 0.2  # roller damping, Nm
    Iw = 1    # wheel inertia
    Ir = 1    # roller inertia
    Ib = 1    # body inertia
    TractionTorque = 1  # modeled value
    
    ExternalTorque = ContactJacobian(R, rw, rr, BotMass, Br, Iw, Ir, Ib, TractionTorque)
    
    # Wait until self.theta is set by the callback.
    rate = rospy.Rate(10)  # 10 Hz
    while ExternalTorque.theta is None and not rospy.is_shutdown():
        rospy.logwarn("Waiting for theta to be set...")
        rate.sleep()
    
    output_nominal = ExternalTorque.external_forces()
    print(f"position of external force: {output_nominal}")    
    rospy.spin()

if __name__ == "__main__":
    main()