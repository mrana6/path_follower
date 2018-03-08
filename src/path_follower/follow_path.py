#!/usr/bin/env python

import rospy
import rospkg
import numpy
import operator

from numpy.linalg.linalg import dot
from numpy.linalg import norm


import geometry_msgs.msg
import hlpr_trac_ik.srv
import sensor_msgs.msg

import control_msgs.msg
import actionlib
import math
import trajectory_msgs.msg

import intera_interface


class PathFollower:
    # User Inputs
    DATA_NAME = 'test.txt'                                                   # WARNING: Check the file is in data folder
    TRAC_IK_SERVICE_NAME = '/hlpr_trac_ik'
    JOINT_STATE_TOPIC = '/robot/joint_states'
    JOINT_VEL_ACTION = '/robot/limb/right/follow_joint_trajectory'           # This Action Call smooths the trajectory
    PREFIX = 'right_'                                                        # Set Prefix to 'right_'/'jaco_'


    # Minimum threshold for a point to be included in trajectory. Identical adjacent points cause velocity control failure
    MIN_POS_DIFF = 0.0
    MIN_ANG_DIFF = 0.0
    MIN_JOINT_DIFF = 0.0
    POSE_TOLERANCE = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]              # Allowed tolerance in position and rotation

    r = rospkg.RosPack()
    PACKAGE_PATH = r.get_path('path_follower')

    def __init__(self):

        # Initialize Node
        rospy.init_node('path_follower', anonymous=False)

        self.dataName = rospy.get_param('~data_name', self.DATA_NAME)
        self.dataMode = rospy.get_param('~data_mode', 'cartesian')
        self.armPrefix = rospy.get_param('~arm_prefix', self.PREFIX)

        self.jointStateArm = None
        self.ikSuccessFlag = False

        self.jointTrajectory = []
        self.goalList = []
        self.timeList = []

        self.dataPath = self.PACKAGE_PATH + '/data_cartesian/' + self.dataName
        self.jointDataPath = self.PACKAGE_PATH + '/data_joints/' + self.dataName

        self.jointTrajAction = self.JOINT_VEL_ACTION

        self.trajActionClient = actionlib.SimpleActionClient(self.jointTrajAction,
                                                        control_msgs.msg.FollowJointTrajectoryAction)
        try:
            self.trajActionClient.wait_for_server()
        except rospy.ROSException, e:
            rospy.logerr("Timed Out waiting for Trajectory Follower Action Server: %s", str(e))


        jointStateSub = rospy.Subscriber(self.JOINT_STATE_TOPIC,
                                         sensor_msgs.msg.JointState,
                                         self.jointStateCallback)
        try:
            rospy.wait_for_message(self.JOINT_STATE_TOPIC,
                                   sensor_msgs.msg.JointState, timeout=2)
        except rospy.ROSException, e:
            rospy.logerr("Joint State Callback Time Out: %s", str(e))


        if self.dataMode == 'cartesian':
            self.loadCartesianPath()
            self.findJointTrajectory()
            self.gotoInit()
            self.executeTrajectory()
        elif self.dataMode == 'joints':
            self.loadJointTrajectory()
            self.gotoInit()
            self.executeTrajectory()
        else:
            rospy.logerr('Invalid Data Mode - Valid Inputs: cartesian/joints')



    def jointStateCallback(self, jointStateMsg):
        # Populating joint names
        if len(jointStateMsg.position)>1:
            self.jointStateArm = [jointStateMsg.position[1],
                                  jointStateMsg.position[2],
                                  jointStateMsg.position[3],
                                  jointStateMsg.position[4],
                                  jointStateMsg.position[5],
                                  jointStateMsg.position[6],
                                  jointStateMsg.position[7]]

    def loadCartesianPath(self):
        """
        Loads the Path from file specified earlier
        """
        rospy.loginfo('Loading Data ...')
        time = numpy.loadtxt(self.dataPath, delimiter=',', usecols=(0,))
        time = time - time[0]

        data = numpy.loadtxt(self.dataPath, delimiter=',', usecols=(1,2,3,4,5,6,7))

        posList = []
        quatList = []
        posList.append(data[0,0:3])
        quatList.append(data[0,3:7])

        self.timeList.append(time[0])

        # Check to send waypoint only if they are different (trajectory execution fails else)
        for i in range(1,len(data)):
            currPos = posList[-1]
            currQuat = quatList[-1]

            posDiff = norm(data[i,0:3]-currPos)
            orientDiff = 1 - (dot(data[i,3:7], currQuat))**2

            if posDiff >= self.MIN_POS_DIFF or orientDiff >= self.MIN_ANG_DIFF:
                posList.append(data[i,0:3])
                quatList.append(data[i,3:7])
                self.timeList.append(time[i])

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
        time = numpy.loadtxt(self.jointDataPath, delimiter=',', usecols=(0,))
        data = numpy.loadtxt(self.jointDataPath, delimiter=',', usecols=(1,2,3,4,5,6,7))

        jointList = []
        jointList.append(data[0,:])

        time = time - time[0]

        self.timeList.append(time[0])

        # Check to append joint values only when they are different
        for i in range(1, len(data)):
            currJoint = jointList[-1]

            jointDiff = norm(data[i]-currJoint)

            if jointDiff >= self.MIN_JOINT_DIFF:
                jointList.append(data[i])
                self.timeList.append(time[i])

        # Populating joint trajectory
        for i in range(len(jointList)):
            trajPoint = trajectory_msgs.msg.JointTrajectoryPoint(positions=jointList[i])
            self.jointTrajectory.append(trajPoint)
       

    def findJointTrajectory(self):
        """
        Calls hlpr_trac_IK to find joint trajectory
        """
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

    def gotoInit(self):
        limb = intera_interface.Limb('right')
        initPoint = self.jointTrajectory[0].positions
        
        init = {'right_j0': initPoint[0], 'right_j1': initPoint[1],'right_j2': initPoint[2], \
        'right_j3': initPoint[3], 'right_j4': initPoint[4], 'right_j5': initPoint[5], 'right_j6': initPoint[6]}

        limb.move_to_joint_positions(init)

    def preemptExecution(self):
        if (self.trajActionClient.gh is not None and
            self.trajActionClient.get_state() == actionlib.GoalStatus.ACTIVE):
            self.trajActionClient.cancel_goal()

        #delay to allow for terminating handshake
        rospy.sleep(0.1)

    def executeTrajectory(self):
        """
        Passes the trajectory to controller
        """

        trajActionGoal = control_msgs.msg.FollowJointTrajectoryGoal()

        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'j0')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'j1')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'j2')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'j3')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'j4')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'j5')
        trajActionGoal.trajectory.joint_names.append(self.armPrefix + 'j6')


        trajActionGoal.trajectory.points = []
        for j in range(len(self.jointTrajectory)):
            tempTrajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
            tempTrajPoint.positions = self.jointTrajectory[j].positions
            tempTrajPoint.time_from_start = rospy.Duration(self.timeList[j])
            trajActionGoal.trajectory.points.append(tempTrajPoint)

        trajActionGoal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        rospy.on_shutdown(self.preemptExecution)

        rospy.loginfo('Sending goal to joint_trajectory_action')
        self.trajActionClient.send_goal(trajActionGoal)

        self.trajActionClient.wait_for_result()

        try:
            rospy.wait_for_message(self.JOINT_STATE_TOPIC,
                                   sensor_msgs.msg.JointState, timeout=2)
        except rospy.ROSException, e:
            rospy.logerr("Joint State Callback Time Out: %s", str(e))


        finalError = map(operator.sub, self.jointStateArm, list(self.jointTrajectory[-1].positions))

        rospy.loginfo('Final Point Error: ' + ','.join([str(x) for x in finalError]))

        trajActionResult = (self.trajActionClient.get_result().error_code == 0)

        if trajActionResult:
            rospy.loginfo('Trajectory execution successful!')
            return True
        else:
            rospy.logerr('Trajectory execution failure!')
            return False

if __name__ == '__main__':
    trajFollower = PathFollower()
