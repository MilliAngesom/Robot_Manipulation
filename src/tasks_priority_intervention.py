#!/usr/bin/env python3
from kinematic_intervention import * 
import tf.transformations
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Float64MultiArray, Bool, String
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from std_srvs.srv import SetBool 
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import copy

class Intervention:
    def __init__(self) -> None:
        self.placed = True
        self.picked = True
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.theta4 = 0.0
        self.wheel_radius = 0.035
        self.wheel_base = 0.23
        self.revolute = [True, True, True, True] # 4 joints of robot arm 
        self.flag = True
        self.temp = []
        self.eta = np.zeros((3,1))
        self.desired_pose = np.zeros((1,2))
        self.priority = False
        self.change_state = False
        self.stop = False 
        self.error = float('inf')
        self.robot = MobileManipulator(self.revolute, self.theta1, self.theta2, self.theta3, self.theta4) # Manipulator object
        # task definition
        self.task1 = Position3D("End-effector position", np.array([2.0, 0.0, -0.1616]).reshape(3,1))
        self.task2 = Configuration3D("End-effector configuration", np.array([2.0, 0.0, -0.17, np.pi * 0.4 ]).reshape(4,1))
        self.task3 = Orientation("End-effector orientation", np.array([0.1 * np.pi ]).reshape(1,1))

        joint1_limit = np.array([-1.571, 1.571]).reshape(2,1)
        joint2_limit = np.array([-1.571, 0.05]).reshape(2,1)
        joint3_limit = np.array([-1.571, 0.05]).reshape(2,1)
        joint4_limit = np.array([-1.571, 1.571]).reshape(2,1)

        self.task5 = JointLimit("Joint Limit", joint1_limit, np.array([0.05, 0.05]),0)
        self.task6 = JointLimit("Joint Limit", joint2_limit, np.array([0.05, 0.05]),1)
        self.task7 = JointLimit("Joint Limit", joint3_limit, np.array([0.05, 0.05]),2)
        self.task8 = JointLimit("Joint Limit", joint4_limit, np.array([0.05, 0.05]),3)
        self.tasks = [ ]
        self.count = 0

        # joint state subscriber and publishers
        self.js_sub = rospy.Subscriber("/turtlebot/joint_states", JointState, self.joint_state_callback) #/swiftpro/joint_states /turtlebot/joint_states
        self.pub = rospy.Publisher("/current_EE_pose",PoseStamped,queue_size=1)
        self.pub_pose = rospy.Publisher("/desired_pose",PoseStamped, queue_size=1)
        self.pub_robot = rospy.Publisher("/robot_odom", Odometry, queue_size=1)
        self.pub_velocity = rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)
        self.pub_base_velocity = rospy.Publisher("/turtlebot/kobuki/commands/wheel_velocities", Float64MultiArray, queue_size=10)
        self.pub_pump = rospy.Publisher('/turtlebot/swiftpro/vacuum_gripper/pump_state', Bool, queue_size=10)
        # Create a ROS service clientz
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        self.set_pump_client = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
        self.pub_trajectory = rospy.Publisher("/trajectory", Marker, queue_size=1)
        self.tf_br = TransformBroadcaster()
        self.ground_truth = rospy.Subscriber("/turtlebot/kobuki/ground_truth", Odometry, self.update_odom)
        rospy.Subscriber("end_effector_pose", PoseStamped, self.get_pose)
        self.pub_intervention = rospy.Publisher('/intervention_feedback', String,  queue_size=10)
        self.pub_error = rospy.Publisher("/control_error", Float64MultiArray, queue_size=10)
        self.joint_velocily = rospy.Publisher("/joint_velocity", Float64MultiArray,queue_size=10)
    '''
    Function updates the joint states from robot.
    '''
    def joint_state_callback(self,msg):

        if msg.name[0] == 'turtlebot/swiftpro/joint1':  

            self.robot.theta1 = np.copy(msg.position[0]) #theta1
            self.robot.theta2 = np.copy(msg.position[1]) #theta2
            self.robot.theta3 = np.copy(msg.position[2]) #theta3
            self.robot.theta4 = np.copy(msg.position[3]) #theta4
    '''
    Function gets the desired point from behavior tree and executes the task priority
    '''
    def get_pose(self, pos):
            
            self.task1 = Position3D("End-effector position", np.array([pos.pose.position.x, pos.pose.position.y, pos.pose.position.z]).reshape(3,1))
            # print("self.desired_pose",self.desired_pose)
            self.tasks = [ self.task5, self.task6, self.task7, self.task8, self.task1 ]
            if self.count == 0:
                self.task_priority(0.008)
                self.pub_intervention.publish(str("success"))
            elif self.count == 1:
                self.task_priority(0.15)
                self.pub_intervention.publish(str("success"))
            elif self.count == 2:
                self.task_priority(0.09)
                self.pub_intervention.publish(str("success"))
            elif self.count == 3:
                self.task_priority(0.009)
                self.pub_intervention.publish(str("success"))
    '''
    Function gets odometry from groundtruth
    '''
    def update_odom(self,odom):
        
        quaternion = [
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w]

        _, _, self.eta[2] = copy.copy(euler_from_quaternion(quaternion))
        self.eta[0] = copy.copy(odom.pose.pose.position.x)
        self.eta[1] = copy.copy(odom.pose.pose.position.y)
        self.robot.update(self.eta)
        # Calculate dt
        current_time = rospy.Time.from_sec(odom.header.stamp.secs + odom.header.stamp.nsecs * 1e-9)
        q = quaternion_from_euler(0, 0, self.eta[2])
        # Publish odom

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "world"
        odom.child_frame_id = "turtlebot/kobuki/base_footprint"

        odom.pose.pose.position.x = self.eta[0]
        odom.pose.pose.position.y = self.eta[1] 

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2] 
        odom.pose.pose.orientation.w = q[3]

        self.pub_robot.publish(odom)                      
        
        self.tf_br.sendTransform((self.eta[0], self.eta[1], 0.0), q, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id)
    '''
    Function implements the task priority 
    '''
    def task_priority(self, thershold):
        self.count += 1
        self.error = float('inf')
        ## Recursive Task-Priority algorithm starts
        while np.linalg.norm(self.error)  > thershold:
            self.pub_intervention.publish(str("running"))
            # Initialize number of dof
            num_dof = self.robot.getDOF()
            # Initialize null-space projector
            P = np.eye(num_dof)

            # Initialize output vector (joint velocity)
            dq = np.zeros((num_dof,1)).reshape(num_dof,1)

            # Loop over tasks
            for i in range(len(self.tasks)):
                # Update task state
                self.tasks[i].update(self.robot)
                # check if the task is active
                if self.tasks[i].isActive():
                    # Compute augmented Jacobian
                    Jbar = self.tasks[i].getJacobian() @ P
                    # Compute task velocity
                    if i==4 and np.linalg.norm(self.tasks[4].getDesired()[:2] - np.array([self.eta[0],self.eta[1]]).reshape(2,1)) < 0.3:
                        weight = np.array([[1000,0,0,0,0,0],[0,1000,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0], [0,0,0,0,0,1]])
                    elif i==4 and np.linalg.norm(self.tasks[4].getDesired()[:2] - np.array([self.eta[0],self.eta[1]]).reshape(2,1)) >= 0.3:
                        weight = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1000,0,0,0],[0,0,0,1000,0,0],[0,0,0,0,1000,0], [0,0,0,0,0,1000]])
                    else:
                        weight = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0], [0,0,0,0,0,1]])
                    dq = dq +  DLS(Jbar,0.05,weight) @ ((self.tasks[i].getGain() @ self.tasks[i].getError() + self.tasks[i].getFeedForward()) - self.tasks[i].getJacobian() @ dq)
                    # Accumulate velocity ----- it is done in the above line of code
                    # Update null-space projector
                    P = P - np.linalg.pinv(Jbar)@ Jbar
            
            if np.linalg.norm(self.tasks[4].getDesired()[:2] - np.array([self.eta[0],self.eta[1]]).reshape(2,1)) > 0.4:
                dq[0]= dq[0]
                dq[1]= dq[1] 
            else:
                dq[2:]= 0.2*dq[2:]

            # Publish joint velocities
            vel = Float64MultiArray()
            vel.data = np.array([dq[2],dq[3],dq[4],dq[5]])
            self.pub_velocity.publish(vel)
            # Publish wheel velocities
            velocity = Float64MultiArray()
            velocity.data = np.array([dq[0],dq[1],dq[2],dq[3],dq[4],dq[5]])
            self.joint_velocily.publish(velocity)

            # Convert the velocity of the end effector to the velocity of the Turtlebot
            v_x =dq[1]
            w = dq[0]

            # Calculate the velocity of the left and right wheels
            left_lin_vel = (2*v_x + w*self.wheel_base) / 2
            right_lin_vel = (2*v_x - w*self.wheel_base) / 2
            
            # Publish velocities for base
            vel_wheel = Float64MultiArray()
            vel_wheel.data = [left_lin_vel,right_lin_vel]
            self.pub_base_velocity.publish(vel_wheel)
            # Publish the end-effector
            pose = self.robot.getEETransform()
            EEyaw = self.robot.getEEOrientation()
            self.publish_EE(pose, EEyaw)
            # Publish the desired possition
            des_pose = self.tasks[4].getDesired()
            # Set the desired orientation of the end effector
            yaw = copy.copy(self.tasks[4].getDesired()[-1][0])
            self.publish_desiredEE(des_pose,yaw)
            # Publish the control errors
            self.error = self.tasks[4].getError()
            error = Float64MultiArray()
            error.data = np.array(self.tasks[4].getError())
            self.pub_error.publish(error)

    '''
    Function publishes the end-effector to Rviz
    '''
    def publish_EE(self, pose, EEyaw):
        # Publish Current End-Effector Position
        ee_pose = PoseStamped()
        ee_pose.header.stamp= rospy.Time.now()
        ee_pose.header.frame_id = 'world'
        # ee_pose.header.frame_id = 'turtlebot/kobuki/base_footprint'
        ee_pose.pose.position.x = pose[0]
        ee_pose.pose.position.y = pose[1]
        ee_pose.pose.position.z = pose[2]
        
        # Set the orientation of the end effector
        quaternion = tf.transformations.quaternion_from_euler(0, 0, EEyaw)
        ee_pose.pose.orientation.x = quaternion[0]
        ee_pose.pose.orientation.y = quaternion[1]
        ee_pose.pose.orientation.z = quaternion[2]
        ee_pose.pose.orientation.w = quaternion[3]
        self.pub.publish(ee_pose)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()  # Set the timestamp
        marker.header.frame_id = 'world'  # Set the frame ID
        marker.ns = 'end_effector'  # Set the namespace
        marker.id = 0  # Set the marker ID
        marker.type = Marker.LINE_STRIP  # Set the marker type to LINE_STRIP
        marker.action = Marker.ADD  # Set the marker action to ADD
        marker.scale.x = 0.05  # Set the point size (diameter)
        marker.scale.y = 0.05  # Set the point size (diameter)
        marker.scale.z = 0.0  # Set the point size (diameter)
        marker.color.a = 1.0  # Set the color alpha value
        marker.color.r = 0.0  # Set the color red value
        marker.color.g = 1.0  # Set the color green value
        marker.color.b = 0.0  # Set the color blue value

        # Create the point data
        point = Point()
        point.x = pose[0]  # Set the point position (x-coordinate)
        point.y = pose[1]  # Set the point position (y-coordinate)
        point.z = pose[2]  # Set the point position (z-coordinate)

        self.temp.append(point)
        # Add the point to the marker
        for k in self.temp:
            marker.points.append(k)
        self.pub_trajectory.publish(marker)
    '''
    Function publishes the desired position to Rviz
    '''
    def publish_desiredEE(self, des_pose, yaw):
        # Publish the Desired End-Effector Position
        desired_pose = PoseStamped()
        desired_pose.header.stamp= rospy.Time.now()
        desired_pose.header.frame_id = 'world'
        # ee_pose.header.frame_id = 'turtlebot/kobuki/base_footprint'
        desired_pose.pose.position.x = des_pose[0]
        desired_pose.pose.position.y = des_pose[1]
        desired_pose.pose.position.z = des_pose[2]

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        desired_pose.pose.orientation.x = quaternion[0]
        desired_pose.pose.orientation.y = quaternion[1]
        desired_pose.pose.orientation.z = quaternion[2]
        desired_pose.pose.orientation.w = quaternion[3]
        
        self.pub_pose.publish(desired_pose)


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node("node_name")
    handsonP = Intervention()
    rospy.spin()