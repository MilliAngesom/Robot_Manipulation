import numpy as np
from math import cos, sin, atan2

'''
    Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

    Arguments:
    A (Numpy array): matrix to be inverted
    damping (double): damping factor
    weights (np.array): weights for the implementation of "weighted DLS implementation"

    Returns:
    (Numpy array): inversion of the input matrix
'''
def DLS(A, damping, weights):

    A = A.astype(np.float64)
    weights_inverse = np.linalg.inv(weights.astype(np.float64))

    return weights_inverse @ A.T @ np.linalg.inv(A @ weights_inverse @ A.T + damping**2 * np.eye(A.shape[0])) 

'''
    Class representing a robotic manipulator.
'''
class MobileManipulator:
    '''
        Constructor.

        Arguments:
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
        theta1: angle of joint 1 robot arm 
        theta2: angle of joint 2 robot arm
        theta3: angle of joint 3 robot arm
        theta4: angle of joint 4 robot arm

    '''
    def __init__(self, revolute, theta1, theta2, theta3, theta4):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.dt = 0.0
        self.T = np.zeros((3,1), dtype=np.float64)
        self.revolute = revolute
        self.J_base = np.zeros((6,2), dtype=np.float64)
        self.J = np.zeros((6,6), dtype=np.float64)
        self.q = np.array([self.theta1,self.theta2,self.theta3,self.theta4]).reshape(-1, 1)
        self.world_T_endeffector = np.zeros((4,4), dtype=np.float64)
        self.world_T_mobileBase = np.zeros((4,4), dtype=np.float64)
        # just move x and y and rotate in yaw
        self.revoluteExt = [True, False] + self.revolute  # List of joint types extended with base joints
        self.dof = len(self.revoluteExt) # Number of DOF of the system
        self.hist = np.array([0,0])
    
    '''
        Method that updates the state of the robot.

        Arguments:
        eta (Numpy array): the robot pose from navigation
    '''
    def update(self, eta): 
        # Update manipulator
        self.T = self.kinematics(self.theta1, self.theta2, self.theta3, self.theta4, eta[0], eta[1], eta[2])
        self.q = np.array([self.theta1,self.theta2,self.theta3,self.theta4]).reshape(-1, 1)
        d = np.linalg.norm(np.array([eta[0], eta[1]]) - self.hist)
        self.jacobian(eta[2], d)
        self.hist= np.array([eta[0], eta[1]])
    
    
    def kinematics(self, theta1, theta2, theta3, theta4, x, y, theta):
    
        L1 = 0.108
        L2 = 0.142 
        L3 = 0.1588
        L4 = 0.056
        d = 0.0722
        r = 0.0132

        x_R = np.cos(theta1)*(-L2*np.sin(theta2) + L3*np.cos(theta3) + L4 + r) # x of end effector in the manipulator base frame
        y_R = np.sin(theta1)*(-L2*np.sin(theta2) + L3*np.cos(theta3) + L4 + r) # y of end effector in the manipulator base frame
        z_R = -L2*np.cos(theta2) - L3*np.sin(theta3) - L1 + d  # z of end effector in the manipulator base frame

        # compute the end effector position in the mobile base frame
        a = np.array([[1, 0, 0, 0.041], 
                      [0, 1, 0, 0], 
                      [0, 0, 1, 0], 
                      [0, 0, 0, 1]]).reshape(4,4).astype(np.float64)
        
        b = np.array([[np.cos(-0.5*np.pi), -np.sin(-0.5*np.pi), 0, 0], 
                      [np.sin(-0.5*np.pi), np.cos(-0.5*np.pi), 0, 0], 
                      [0, 0, 1, 0], 
                      [0, 0, 0, 1]]).reshape(4,4).astype(np.float64)

        mobileBase_T_Arm = a @ b  # transformation matrix between the manipulator base and the mobile base frame

        endEffector_position = np.array([[np.cos(theta4 + theta1), -np.sin(theta4 + theta1), 0, 0], 
                                         [np.sin(theta4 + theta1), np.cos(theta4 + theta1), 0, 0], 
                                         [0, 0, 1, 0], 
                                         [0, 0, 0, 1]]).reshape(4,4).astype(np.float64)

        mobileBase_T_endeffector = mobileBase_T_Arm @ endEffector_position

        # compute the end effector position in the world frame
        self.world_T_mobileBase = np.array([[np.cos(theta), -np.sin(theta), 0, x], 
                                       [np.sin(theta), np.cos(theta), 0, y], 
                                       [0, 0, 1, -0.198], 
                                       [0, 0, 0, 1]], dtype=object).reshape(4,4)

        self.world_T_endeffector = self.world_T_mobileBase @ mobileBase_T_endeffector

        x_w = np.cos(theta)*(y_R+0.041) + np.sin(theta)*x_R + x  # x of end effector in the world frame
        y_w = np.sin(theta)*(y_R+0.041) - np.cos(theta)*x_R + y # y of end effector in the world frame
        z_w = z_R - 0.198 # z of end effector in the world frame

        
        T = np.array([x_w,y_w,z_w], dtype=np.float64).reshape(3,1)

        return T
    '''
        Function builds a Jacobian for the end-effector of a robot,
        described by a list of kinematic transformations and a list of joint types.

        Arguments:
        theta: the yaw angle of the mobile base
        d: the translation of the mobile base

        Returns:
        (Numpy array): the end-effector Jacobian
    '''
    def jacobian(self,theta,d):
        L1 = 0.108
        L2 = 0.142 
        L3 = 0.1588
        L4 = 0.056
        r = 0.0132

        # x_w = cos(theta)*(sin(self.theta1)*(-L2*sin(self.theta2) + L3*cos(self.theta3) + L4 + r)+0.041) + sin(theta)*(cos(self.theta1)*(-L2*sin(self.theta2) + L3*cos(self.theta3) + L4 + r)) + x + cos(theta) * d 
        
        x0 = -(sin(self.theta1)*(L3*cos(self.theta3)-L2*sin(self.theta2)+r+L4)+41/1000)*sin(theta)-d*sin(theta)+cos(self.theta1)*(L3*cos(self.theta3)-L2*sin(self.theta2)+r+L4)*cos(theta)
        x1 = cos(theta)
        x2 = -(L3*np.cos(self.theta3)-L2*np.sin(self.theta2)+r+L4)*(np.sin(theta)*np.sin(self.theta1)-np.cos(theta)*np.cos(self.theta1))
        x3= -L2*(np.cos(theta)*np.sin(self.theta1)+np.sin(theta)*np.cos(self.theta1))*np.cos(self.theta2)
        x4=  -L3*(np.cos(theta)*np.sin(self.theta1)+np.sin(theta)*np.cos(self.theta1))*np.sin(self.theta3)
        x5= 0

        # y_w = sin(theta)*(sin(self.theta1)*(-L2*sin(self.theta2) + L3*cos(self.theta3) + L4 + r)+0.041) - cos(theta)*(cos(self.theta1)*(-L2*sin(self.theta2) + L3*cos(self.theta3) + L4 + r)) + y + sin(theta) * d 

        
        y0 =cos(self.theta1)*(L3*cos(self.theta3)-L2*sin(self.theta2)+r+L4)*sin(theta)+(sin(self.theta1)*(L3*cos(self.theta3)-L2*sin(self.theta2)+r+L4)+41/1000)*cos(theta)+d*cos(theta)
        y1= sin(theta)
        y2= (L3*np.cos(self.theta3)-L2*np.sin(self.theta2)+r+L4)*(np.cos(theta)*np.sin(self.theta1)+np.sin(theta)*np.cos(self.theta1))
        y3= -L2*(np.sin(theta)*np.sin(self.theta1)-np.cos(theta)*np.cos(self.theta1))*np.cos(self.theta2)
        y4= -L3*(np.sin(theta)*np.sin(self.theta1)-np.cos(theta)*np.cos(self.theta1))*np.sin(self.theta3)
        y5= 0

        # z_w = -L2*cos(self.theta2) - L3*sin(self.theta3) - L1 + d - 0.198

        z0= 0
        z1= 0
        z2= 0
        z3= L2*np.sin(self.theta2)
        z4= -L3*np.cos(self.theta3)
        z5= 0

        self.J = np.array([[x0,x1,x2,x3,x4,x5],
                           [y0,y1,y2,y3,y4,y5],
                           [z0,z1,z2,z3,z4,z5],
                           [0,0,0,0,0,0],
                           [0,0,0,0,0,0],
                           [1,0,1,0,0,1]],dtype=np.float64)

    '''
        Method that returns the end-effector Jacobian.
    '''
    def getEEJacobian(self):
        return self.J
    
    '''
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        return self.T
    
    '''
        Method that returns the end-effector orientation.
    '''
    def getEEOrientation(self):
        return atan2(self.world_T_endeffector[1,0], self.world_T_endeffector[0,0])
    '''
        Method that returns the mobile base orientation.
    '''
    def getMBOrientation(self):
        return atan2(self.world_T_mobileBase[1,0], self.world_T_mobileBase[0,0])  
    '''
        Method that returns the link transformation.
    '''
    def getLinkTransform(self, link):
        return self.T[link]
    '''
        Method that returns the position of a selected joint.

        Argument:
        joint (integer): index of the joint

        Returns:
        (double): position of the joint
    '''
    def getJointPos(self, joint):
        return self.q[joint]
    
    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof
    

'''
    Base class representing an abstract Task.
'''
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)

    '''
    def __init__(self, name, desired):
        self.name = name 
        self.sigma_d = desired 

    ''' 
        Method setting the desired sigma.
    
        Arguments:
        value(Numpy array): value of the desired sigma (goal)
    '''
    def setDesired(self, value):
        self.sigma_d = value

    '''
        Method returning the desired sigma.
    '''
    def getDesired(self):
        return self.sigma_d

    '''
        Method returning the task Jacobian swift pro.
    '''
    def getJacobian(self):
        return self.J
    
    '''
        Method returning the task error (tilde sigma).
    '''    
    def getError(self):
        return self.err
    ''' 
        Method setting the gain matrix K.
    
        Arguments:
        K(Numpy array): matrix of the gain matrix K
    '''
    def setKgain(self, K):
        self.gain = K
    ''' 
        Method returning the gain matrix K.
    '''
    def getGain(self):
        return self.gain
    ''' 
        Method setting the feedforward velocity vector.
    
        Arguments:
        vel(list): value of the feedforward velocity vector
    '''
    def setFeedForward(self, vel):
        self.feedforward = vel

    ''' 
        Method setting the feedforward velocity vector.
    
        Arguments:
        vel(list): value of the feedforward velocity vector
    '''
    def getFeedForward(self):
        return self.feedforward
    

'''
    Subclass of Task, representing the 3D position task.
'''
class Position3D(Task):
    def __init__(self, name, desired):
        super().__init__(name, desired)
        self.feedforward = np.zeros((3,1)).reshape(3,1)  # the feedforward velocity vector
        self.gain = np.diag([7, 7, 7])   # the gain matrix K [4,1,1]
        self.J = np.zeros((3,6))# Initialize with proper dimensions 
        self.err = np.zeros((3,1))# Initialize with proper dimensions
        self.active = True 
        
    def isActive(self):
        return self.active
    
    def update(self, robot):
        self.J = robot.getEEJacobian()[0:3,:]# Update task Jacobian 
        self.err = self.getDesired() - robot.getEETransform()# Update task error
        

'''
    Subclass of Task, representing the 3D configuration task.
'''
class Configuration3D(Task):
    def __init__(self, name, desired):
        super().__init__(name, desired)
        
        self.J = np.zeros((4,6))# Initialize with proper dimensions 
        self.err = np.zeros((4,1))# Initialize with proper dimensions
        self.feedforward = np.zeros((4,1)).reshape(4,1)  # the feedforward velocity vector
        self.gain = np.diag([3, 3, 3, 2])   # the gain matrix K  [5, 5, 5, 2]
        self.active = True # true if task is active, default value is True

    def isActive(self):
        return self.active
        
    def update(self, robot):
        j_position = robot.getEEJacobian() [0:3,:] # jacobian of the position task
        j_orientation = robot.getEEJacobian() [-1,:].reshape(1,len(robot.revolute)+2)
        self.J = np.concatenate((j_position, j_orientation), axis=0)# Update task Jacobian
        EE_orientation = robot.getEEOrientation() # current End Effector orientation   
        self.err = self.getDesired() - np.append(robot.T[0:3,-1], EE_orientation).reshape(4,1) 
        
'''
    Subclass of Task, representing the orientation task.
'''
class Orientation(Task):
    def __init__(self, name, desired):
        super().__init__(name, desired)
        self.feedforward = np.zeros(1)   # Initialize feedforward
        self.gain = 10*np.eye(1)   # Initialize gain
        self.active = True # true if task is active, default value is True

    def isActive(self):
        return self.active
        
    def update(self, robot):
        EE_orientation = robot.getEEOrientation() # current End Effector orientation
        self.J = robot.getEEJacobian() [-1,:].reshape(1,len(robot.revolute)+2)
        self.err = self.getDesired()[0][0] - EE_orientation # Update task error
        self.err = np.array(self.err).reshape(1,1)

'''
    Subclass of Task, representing base orientation task.
'''
class BaseOrientation(Task):
    def __init__(self, name, desired):
        super().__init__(name, desired)
        self.feedforward = np.zeros(1)   # Initialize feedforward
        self.gain = 10*np.eye(1)   # Initialize gain
        self.active = True # true if task is active, default value is True

    def isActive(self):
        return self.active
        
    def update(self, robot):
        MB_orientation = robot.getMBOrientation() # current mobile base orientation
        self.J = robot.getEEJacobian() [-1,:].reshape(1,len(robot.revolute)+2)
        self.err = self.getDesired()[0][0] - MB_orientation # Update task error
        self.err = np.array(self.err).reshape(1,1)
        
''' 
    Subclass of Task, representing the the joint limits task.
'''
class JointLimits3D(Task):
    def __init__(self, name, joint, safe_range):
        super().__init__(name, 0.0)
        self.J = np.zeros((1,3))# Initialize with proper dimensions 
        self.err = np.zeros((1,1))# Initialize with proper dimensions
        self.threshold = np.array([0.01, 0.02])
        self.safe_range =  safe_range
        self.active = False
        self.joint = joint
        self.feedforward = np.array([0])  # the feedforward velocity vector
        self.gain = np.array([1])   # the gain matrix K [4,1,1]
    
    def Activation(self,position):
        if self.active == 0 and position >= (self.safe_range[1] - self.threshold[0]): 
            self.active = 1
        elif self.active == 0 and position <= (self.safe_range[0] + self.threshold[0]):
            self.active = 1
        elif self.active == 1 and position <= (self.safe_range[1] - self.threshold[1]): 
            self.active = 0
        elif self.active == 1 and position >= (self.safe_range[0] + self.threshold[1]): 
            self.active = 0

    def isActive(self):
        if self.active != 0:
            return True
        else: 
            return False
        
    def update(self, robot):
        self.J = np.zeros((1,robot.getDOF()))# Update task Jacobian
        self.J[0][self.joint] = 1
        self.dis = robot.getJointPos(self.joint)# Update task error
        self.Activation(robot.getJointPos(self.joint))
        self.err = np.array([self.active]).reshape(1,1)

''' 
    Subclass of Task, representing the joint position task.
'''
class JointPosition3D(Task):
    def __init__(self, name, desired):
        super().__init__(name, desired)
        self.feedforward = np.zeros((3,1)).reshape(3,1)  # the feedforward velocity vector
        self.gain = np.diag([4, 1, 1])   # the gain matrix K
        self.J = np.zeros((3,6))# Initialize with proper dimensions 
        self.err = np.zeros((3,1))# Initialize with proper dimensions
        self.active = True 

    def isActive(self):
        return self.active
        
    def update(self, robot):
        self.J = np.array([1,0,0]).reshape(1,3)# Update task Jacobian 
        self.err = self.getDesired() - robot.getJointPos(0)# Update task error

'''
    Subclass of Task, representing the Joint Limit task.
    Arguments:
        name (string): title of the task
        desired (Numpy array): the lower and upper limit joint angle 
        joint (int): joint index
        limits (Numpy array): task deactivates if current q >=  desired + limits
'''
class JointLimit(Task):
    def __init__(self, name, desired,limits,joint):
        super().__init__(name, desired)
        self.joint = joint # joint index starts from 0
        self.feedforward = np.zeros(1)
        self.gain = np.eye(1)
        self.limit = limits # angle to activate or deactivate this task 
        self.active = False  # true if task is active, default value is False
        
    def isActive(self):
        return self.active
    
    def update(self, robot):  
        joint_position = robot.getJointPos(self.joint)
        self.J = np.zeros((1,len(robot.revolute)+2)).reshape(1,len(robot.revolute)+2)
        self.J[0,self.joint+2] = 1 # Update task Jacobian
        if joint_position <= self.getDesired()[0]:
            self.active = True
            self.err = np.array([1])    # Update task error
        elif  joint_position > self.getDesired()[0] and joint_position <= (self.getDesired()[0] + self.limit[0]) and self.isActive():
            self.active = True
            self.err = np.array([1])    # Update task error
        elif joint_position >= self.getDesired()[1] :
            self.active = True
            self.err = np.array([-1])  # Update task error
        elif joint_position < self.getDesired()[1] and joint_position >= (self.getDesired()[1] - self.limit[0]) and self.isActive():
            self.active = True
            self.err = np.array([-1])   # Update task error
        else:
            self.active = False

