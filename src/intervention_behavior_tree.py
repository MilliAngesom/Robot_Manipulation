#!/usr/bin/env python3
import py_trees
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Float64MultiArray
import tf
import time
from std_srvs.srv import SetBool
import tf.transformations 


'''
    Subclass of behavior tree, representing the go-to the first point from aruco.
'''
class go_to1(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(go_to1, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.blackboard.register_key("aruco_position", access=py_trees.common.Access.WRITE)
        self.pub = rospy.Publisher('end_effector_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/intervention_feedback', String, self.callback)
        self.end_effector_pose_reached = False
        rospy.Subscriber("/aruco/pose", Float64MultiArray, self.get_pose)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.aruco_detect = False

    def setup(self):
        self.logger.debug("  %s [Pick and Place::setup()]" % self.name)
        
    def initialise(self):
        
        self.publish_once = True
        self.logger.debug("  %s [Pick and Place::initialise()]" % self.name)
        
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    def get_pose(self, pose):
        self.pose_x = pose.data[0] + 0.1
        self.pose_y = pose.data[1] -0.05
        self.aruco_detect = True

    def update(self):
    
        if self.publish_once and self.aruco_detect:
            # read location from BB and publish it 
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world"
            msg.pose.position.x = self.pose_x
            msg.pose.position.y = self.pose_y 
            msg.pose.position.z = -0.147
            # convert yaw angle to quaternion and set the orientation
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            # publish the message
            self.pub.publish(msg)
            self.publish_once = False
            self.aruco_detect = False
            self.blackboard.set("aruco_position",[self.pose_x,self.pose_y])

        
        if self.end_effector_pose_reached=="failure":
            return py_trees.common.Status.FAILURE
        elif self.end_effector_pose_reached=="success":
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

'''
    Subclass of behavior tree, representing the go-to the second point for safety.
'''
class go_to2(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(go_to2, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.blackboard.register_key("aruco_position", access=py_trees.common.Access.READ)
        self.pub = rospy.Publisher('end_effector_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/intervention_feedback', String, self.callback)
        self.pub_pump = rospy.Publisher('/turtlebot/swiftpro/vacuum_gripper/pump_state', Bool, queue_size=10)
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        self.set_pump_client = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
        self.end_effector_pose_reached = False

    def setup(self):
        self.logger.debug("  %s [Pick and Place2::setup()]" % self.name)

    def initialise(self):
        
        self.publish_once = True
        self.logger.debug("  %s [Pick and Place2::initialise()]" % self.name)
        
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    def update(self):
        
        if self.publish_once:
            pump_state = True  # Set the desired pump state (True for on, False for off)
            response = self.set_pump_client(pump_state)
            # read location from BB and publish it 
            aruco_position = self.blackboard.get("aruco_position")
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world"
            msg.pose.position.x = aruco_position[0]
            msg.pose.position.y = aruco_position[1] - 0.5
            msg.pose.position.z = -0.35
            # convert yaw angle to quaternion and set the orientation
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            # publish the message
            self.pub.publish(msg)
            self.publish_once = False

        if self.end_effector_pose_reached=="failure":
            return py_trees.common.Status.FAILURE
        elif self.end_effector_pose_reached=="success":
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
'''
    Subclass of behavior tree, representing the go-to the third point for placing.
'''
class go_to3(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(go_to3, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.pub = rospy.Publisher('end_effector_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/intervention_feedback', String, self.callback)
        self.pub_pump = rospy.Publisher('/turtlebot/swiftpro/vacuum_gripper/pump_state', Bool, queue_size=10)
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        self.set_pump_client = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
        self.end_effector_pose_reached = False

    def setup(self):
        self.logger.debug("  %s [Pick and Place2::setup()]" % self.name)
        
    def initialise(self):
        
        self.publish_once = True
        self.logger.debug("  %s [Pick and Place2::initialise()]" % self.name)
        
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    def update(self):
        if self.publish_once:
            pump_state = True  # Set the desired pump state (True for on, False for off)
            response = self.set_pump_client(pump_state)
            # read location from BB and publish it 
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world"
            msg.pose.position.x = 2.0
            msg.pose.position.y = -3.0
            msg.pose.position.z = -0.25
            # convert yaw angle to quaternion and set the orientation
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            # publish the message
            self.pub.publish(msg)
            self.publish_once = False

        if self.end_effector_pose_reached=="failure":
            return py_trees.common.Status.FAILURE
        elif self.end_effector_pose_reached=="success":
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
'''
    Subclass of behavior tree, representing the go-to the home position.
'''
class go_to4(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(go_to4, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.pub = rospy.Publisher('end_effector_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/intervention_feedback', String, self.callback)
        self.end_effector_pose_reached = False

    def setup(self):
        self.logger.debug("  %s [Pick and Place2::setup()]" % self.name)
        
    def initialise(self):
        
        self.publish_once = True
        self.logger.debug("  %s [Pick and Place2::initialise()]" % self.name)
        
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    def update(self):
        if self.publish_once:
            # read location from BB and publish it 
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world"
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = -0.3
            # convert yaw angle to quaternion and set the orientation
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]
            # publish the message
            self.pub.publish(msg)
            self.publish_once = False
        
        if self.end_effector_pose_reached=="failure":
            return py_trees.common.Status.FAILURE
        elif self.end_effector_pose_reached=="success":
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
'''
    Subclass of behavior tree, representing the picking task.
'''
class pick(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(pick, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.pub_pump = rospy.Publisher('/turtlebot/swiftpro/vacuum_gripper/pump_state', Bool, queue_size=10)
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        self.set_pump_client = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)

    def setup(self):
        self.logger.debug("  %s [Pick and Place2::setup()]" % self.name)
        
    def initialise(self):
        
        self.publish_once = True
        self.logger.debug("  %s [Pick and Place2::initialise()]" % self.name)
        
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    def update(self):
        # Call the service to set the pump state
        pump_state = True  # Set the desired pump state (True for on, False for off)
        response = self.set_pump_client(pump_state)
        if response.success:
            rospy.loginfo('Pump state changed successfully')
            
        if response.success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
'''
    Subclass of behavior tree, representing the placing task.
'''
class place(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(place, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("location", access=py_trees.common.Access.READ)
        self.pub_pump = rospy.Publisher('/turtlebot/swiftpro/vacuum_gripper/pump_state', Bool, queue_size=10)
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        self.set_pump_client = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
        
    def setup(self):
        self.logger.debug("  %s [Pick and Place2::setup()]" % self.name)

    def initialise(self):
        self.publish_once = True
        self.logger.debug("  %s [Pick and Place2::initialise()]" % self.name)
        
    def callback(self, data):
        self.end_effector_pose_reached = data.data
        
    def update(self):
    
         # Call the service to set the pump state
        pump_state = False  # Set the desired pump state (True for on, False for off)
        response = self.set_pump_client(pump_state)
        if response.success:
            rospy.loginfo('Pump state changed successfully')

        if response.success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
              
    def terminate(self, new_status):
        self.logger.debug("  %s [Pick and Place::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
'''
    Class for creating behavior tree.
'''
def create_tree():
    # Create Behaviors
    go1 = go_to1("Go_To_1")
    go2 = go_to2("Go_To_2")
    go3 = go_to3("Go_To_3")
    go4 = go_to4("Go_To_4")
    place_ = place("Place")
    pick_ = pick("Pick")
    
    seq = py_trees.composites.Sequence(name="Sequence", memory=True)

    seq.add_child(go1)   
    seq.add_child(pick_)  
    seq.add_child(go2)
    seq.add_child(go3)
    seq.add_child(place_)
    seq.add_child(go4)

    return seq

def run(it=1):
        
        print("Call setup for all tree children")
        root.setup_with_descendants() 
        print("Setup done!\n\n")
        py_trees.display.ascii_tree(root)
        root.tick_once()
        time.sleep(1)
        
if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")
    root = create_tree()
    py_trees.display.render_dot_tree(root)
    try:
        while not rospy.is_shutdown():
            run(10)
    except KeyboardInterrupt:
        pass
    # Run forever
    rospy.spin()