#!/usr/bin/env python

import rospy
import rospkg
import numpy
from numpy.linalg.linalg import dot
from numpy.linalg import norm


import geometry_msgs.msg
import hlpr_trac_ik.srv
import sensor_msgs.msg

import control_msgs.msg
import actionlib
import math
import trajectory_msgs.msg

def simplify_angle(angle):
    simplified_angle = []
    for i in range(0,len(angle)):
        previous_rev = math.floor(angle[i] / (2.0 * math.pi)) * 2.0 * math.pi
        next_rev = math.ceil(angle[i] / (2.0 * math.pi)) * 2.0 * math.pi
        # if math.fabs(angle[i] - previous_rev) < math.fabs(angle[i] - next_rev):
        #     simplified_angle.append(angle[i]-previous_rev)
        # else:
        #     simplified_angle.append(angle[i]-next_rev)
        if angle[i] <=0:
            simplified_angle.append(angle[i]-next_rev)
        else:
            simplified_angle.append(angle[i]-previous_rev)
    return simplified_angle


def normalize_angle(angle):
    normalized_angle = []
    for i in range(0,len(angle)):
        new_curr_angle = angle[i]
        while new_curr_angle<=-math.pi:
            new_curr_angle = new_curr_angle + 2*math.pi
        while new_curr_angle>math.pi:
            new_curr_angle = new_curr_angle - 2*math.pi
        normalized_angle.append(new_curr_angle)
    return normalized_angle

def find_nearby_angle(desired_angle, curr_angle):
    nearby_angle = curr_angle
    desired_angle = simplify_angle(desired_angle)
    for i in range(0, len(curr_angle)):
        previous_rev = math.floor(curr_angle[i] / (2.0 * math.pi)) * 2.0 * math.pi
        next_rev = math.ceil(curr_angle[i] / (2.0 * math.pi)) * 2.0 * math.pi
        if curr_angle[i] <=0:
            nearby_angle[i] = next_rev + desired_angle[i]
        else:
            nearby_angle[i] = previous_rev + desired_angle[i]

    return nearby_angle


class GCPathFollower:
    # User Inputs
    DATA_NAME = 'dataset4_3.txt'                                            # WARNING: Check the file is in data folder
    TRAC_IK_SERVICE_NAME = '/hlpr_trac_ik'
    JOINT_STATE_TOPIC = '/joint_states'
    JOINT_POS_ACTION = 'jaco_arm/arm_controller/trajectory'               # This Action Call produces jerky
    # trajectory
    JOINT_VEL_ACTION = 'jaco_arm/timed_arm_controller/trajectory'#'jaco_arm/joint_velocity_controller/trajectory'     # This Action Call smooths the trajectory
    PREFIX = 'jaco_'                                                        # Set Prefix to 'right_'/'jaco_'
    MAX_JOINT_VEL = 0.5                                                     # Dictates time between points in position control

    # Minimum threshold for a point to be included in trajectory. Identical adjacent points cause velocity control failure
    MIN_POS_DIFF = 0.01
    MIN_ANG_DIFF = 0.01
    POSE_TOLERANCE = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]              # Allowed tolerance in position and rotation

    r = rospkg.RosPack()
    PACKAGE_PATH = r.get_path('path_follower')

    def __init__(self):

        # Initialize Node
        rospy.init_node('follow_gc_path', anonymous=False)

        self.controlMode = rospy.get_param('~control_mode', 'velocity')
        self.dataName = rospy.get_param('~data_name', self.DATA_NAME)
        self.dataMode = rospy.get_param('~data_mode', 'cartesian')
        self.armPrefix = rospy.get_param('~arm_prefix', self.PREFIX)

        self.jointStateMsg = None
        self.jointStateArm = None
        self.ikSuccessFlag = False
        self.jointTrajectory = None
        self.goalList = []

        self.dataPath = self.PACKAGE_PATH + '/data_cartesian/' + self.dataName
        self.jointDataPath = self.PACKAGE_PATH + '/data_joints/' + self.dataName

        if self.controlMode == 'velocity':
            self.jointTrajAction = self.JOINT_VEL_ACTION
        elif self.controlMode == 'position':
            self.jointTrajAction = self.JOINT_POS_ACTION
        else:
            rospy.logerr('Invalid Control Mode - Valid Inputs: velocity/position')

        if self.dataMode == 'cartesian':
            self.loadCartesianPath()
            self.findJointTrajectory()
            self.executeTrajectory()
        elif self.dataMode == 'joints':
            self.loadJointTrajectory()
            self.executeTrajectory()
        else:
            rospy.logerr('Invalid Data Mode - Valid Inputs: cartesian/joints')

    def jointStateCallback(self, receivedJointStateMsg):
        self.jointStateMsg = receivedJointStateMsg

        # Populating joint names
        if self.jointStateMsg is not None:
            for numJoint in range(len(self.jointStateMsg.name)):
                if self.jointStateMsg.name[numJoint] == self.armPrefix + 'shoulder_pan_joint':
                    shoulderPanIndex = numJoint
                if self.jointStateMsg.name[numJoint] == self.armPrefix + 'shoulder_lift_joint':
                    shoulderLiftIndex = numJoint
                if self.jointStateMsg.name[numJoint] == self.armPrefix + 'elbow_joint':
                    elbowIndex = numJoint
                if self.jointStateMsg.name[numJoint] == self.armPrefix + 'wrist_1_joint':
                    wrist1Index = numJoint
                if self.jointStateMsg.name[numJoint] == self.armPrefix + 'wrist_2_joint':
                    wrist2Index = numJoint
                if self.jointStateMsg.name[numJoint] == self.armPrefix + 'wrist_3_joint':
                    wrist3Index = numJoint

            self.jointStateArm = [self.jointStateMsg.position[shoulderPanIndex],
                                  self.jointStateMsg.position[shoulderLiftIndex],
                                  self.jointStateMsg.position[elbowIndex],
                                  self.jointStateMsg.position[wrist1Index],
                                  self.jointStateMsg.position[wrist2Index],
                                  self.jointStateMsg.position[wrist3Index]]

    def loadCartesianPath(self):
        """
        Loads the Path from file specified earlier
        """
        rospy.loginfo('Loading Data ...')
        data = numpy.loadtxt(self.dataPath)

        posList = []
        quatList = []
        posList.append(data[0,0:3])
        quatList.append(data[0,3:7])

        # Check to send waypoint only if they are different (trajectory execution fails else)
        for i in range(1,len(data)):
            currPos = posList[-1]
            currQuat = quatList[-1]

            posDiff = norm(data[i,0:3]-currPos)
            orientDiff = 1 - (dot(data[i,3:7], currQuat))**2

            if posDiff >= self.MIN_POS_DIFF or orientDiff >= self.MIN_ANG_DIFF:
                posList.append(data[i,0:3])
                quatList.append(data[i,3:7])

        # Populating waypoint list
        for i in range(len(posList)):
            goalPosition = geometry_msgs.msg.Point(x=posList[i][0], y=posList[i][1], z=posList[i][2])
            goalOrientation = geometry_msgs.msg.Quaternion(x=quatList[i][0], y=quatList[i][1], z=quatList[i][2], w=quatList[i][3])
            goalPose = geometry_msgs.msg.Pose(position=goalPosition, orientation=goalOrientation)
            self.goalList.append(goalPose)

    def loadJointTrajectory(self):
        """
        Loads a pre-computed joint trajectory if required
        """
        rospy.loginfo('Loading Joint Data ...')
        data = numpy.loadtxt(self.jointDataPath)

        loadedTraj = []

        for i in range(len(data)):
            tempTrajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
            tempTrajPoint.positions = data[i]
            loadedTraj.append(tempTrajPoint)

        self.jointTrajectory = loadedTraj


    def findJointTrajectory(self):
        """
        Calls hlpr_trac_IK to find joint trajectory
        """

        jointStateSub = rospy.Subscriber(self.JOINT_STATE_TOPIC,
                                         sensor_msgs.msg.JointState,
                                         self.jointStateCallback)
        try:
            rospy.wait_for_message(self.JOINT_STATE_TOPIC,
                                   sensor_msgs.msg.JointState, timeout=2)
        except rospy.ROSException, e:
            rospy.logerr("Joint State Callback Time Out: %s", str(e))

        rospy.loginfo('Waiting for TRAC_IK Service')
        rospy.wait_for_service(self.TRAC_IK_SERVICE_NAME)
        tracIKService = rospy.ServiceProxy(self.TRAC_IK_SERVICE_NAME,
                                           hlpr_trac_ik.srv.IKHandler)
        rospy.loginfo('TRAC_IK Service is Ready')

        rospy.loginfo('Calling TRAC_IK...')
        tracIKReq = hlpr_trac_ik.srv.IKHandlerRequest(origin=self.jointStateArm, goals=self.goalList,
                                                      tolerance=self.POSE_TOLERANCE, verbose=True)
        #tracIKReq = hlpr_trac_ik.srv.IKHandlerRequest(origin=[-math.pi/2, math.pi, math.pi/2, 0, math.pi/2, math.pi/4], goals=goalList, tolerance=tolerance, verbose=True)
        tracIKRes = tracIKService(tracIKReq)
        self.ikSuccessFlag = tracIKRes.success
        self.jointTrajectory = tracIKRes.poses

        if self.ikSuccessFlag:
            rospy.loginfo('Found Joint Trajectory!')
        else:
            rospy.logerr('IK Failed to Find Trajectory - Try Increasing POSE_TOLERANCE')


    def executeTrajectory(self):
        """
        Passes the trajectory to controller
        """

        jointStateSub = rospy.Subscriber(self.JOINT_STATE_TOPIC,
                                         sensor_msgs.msg.JointState,
                                         self.jointStateCallback)
        try:
            rospy.wait_for_message(self.JOINT_STATE_TOPIC,
                                   sensor_msgs.msg.JointState, timeout=2)
        except rospy.ROSException, e:
            rospy.logerr("Joint State Callback Time Out: %s", str(e))

        trajActionClient = actionlib.SimpleActionClient(self.jointTrajAction,
                                                        control_msgs.msg.FollowJointTrajectoryAction)
        try:
            trajActionClient.wait_for_server()
        except rospy.ROSException, e:
            rospy.logerr("Timed Out waiting for Trajectory Follower Action Server: %s", str(e))


        trajActionGoal = control_msgs.msg.FollowJointTrajectoryGoal()

        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'shoulder_pan_joint')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'shoulder_lift_joint')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'elbow_joint')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'wrist_1_joint')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'wrist_2_joint')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'wrist_3_joint')

        trajActionGoal.trajectory.points = []
        timeFromStart = 1.0

        # fill in the joint positions (velocities of 0 mean that the arm
        # will try to stop briefly at each waypoint)
        for j in range(len(self.jointTrajectory)):
            seconds = 0.04                                      #set this to delta_t
            timeFromStart = timeFromStart + seconds

            tempTrajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
            tempTrajPoint.positions = self.jointTrajectory[j].positions
            tempTrajPoint.time_from_start = rospy.Duration(timeFromStart)
            trajActionGoal.trajectory.points.append(tempTrajPoint)
            print tempTrajPoint.positions

        trajActionGoal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        rospy.loginfo('Sending goal to joint_trajectory_action')
        trajActionClient.send_goal(trajActionGoal)

        trajActionClient.wait_for_result()

        try:
            rospy.wait_for_message(self.JOINT_STATE_TOPIC,
                                   sensor_msgs.msg.JointState, timeout=2)
        except rospy.ROSException, e:
            rospy.logerr("Joint State Callback Time Out: %s", str(e))

        rospy.loginfo('joint angles after trajectory: ' + str(self.jointStateArm))

        trajActionResult = trajActionClient.get_result()
        #print trajActionResult


if __name__ == '__main__':
    trajFollower = GCPathFollower()
