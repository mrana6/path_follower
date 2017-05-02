#!/usr/bin/env python
import rospy
import rail_manipulation_msgs.srv
import geometry_msgs.msg
import numpy
from numpy.linalg.linalg import dot
from numpy.linalg import norm
import rospkg

# User Inputs
DATA_NAME = 'dataset1_1.txt'
CARTESIAN_SERVICE_NAME = '/hlpr_moveit_wrapper/cartesian_path'
BASE_FRAME = 'right_base_link'

r = rospkg.RosPack()
PACKAGE_PATH = r.get_path('rail_generalized_cylinder')
DATA_PATH = PACKAGE_PATH + '/data/' + DATA_NAME

if __name__ == '__main__':
    # NOTE: Increase max_curvature in moveit wrapper config file!

    rospy.init_node('execute_trajectory')
    print DATA_PATH
    rospy.loginfo('Waiting for Cartesian Path Service')
    rospy.wait_for_service(CARTESIAN_SERVICE_NAME)
    cartesianPathService = rospy.ServiceProxy(CARTESIAN_SERVICE_NAME,
                                              rail_manipulation_msgs.srv.CartesianPath)

    rospy.loginfo('Service Cartesian Path is ready')
    rospy.loginfo('Loading Data ...')

    data = numpy.loadtxt(DATA_PATH)

    wpList = []
    pos = []
    quat = []

    pos.append(data[0,0:3])
    quat.append(data[0,3:7])

    # Check to send waypoint only if they are different (trajectory execution fails else)
    for i in range(1,len(data)):
        currPos = pos[-1]
        currQuat = quat[-1]

        posDiff = norm(data[i,0:3]-currPos)
        orientDiff = 1 - (dot(data[i,3:7], currQuat))**2

        if posDiff >= 0.01 or orientDiff >= 0.01:
            pos.append(data[i,0:3])
            quat.append(data[i,3:7])
            print i

    # Populating waypoint list
    for i in range(len(pos)):
        wpPosition = geometry_msgs.msg.Point(x=pos[i][0], y=pos[i][1], z=pos[i][2])
        wpOrientation = geometry_msgs.msg.Quaternion(x=quat[i][0], y=quat[i][1], z=quat[i][2], w=quat[i][3])
        wpPose = geometry_msgs.msg.Pose(position=wpPosition, orientation=wpOrientation)
        wp = geometry_msgs.msg.PoseStamped(pose=wpPose)
        wp.header.frame_id = BASE_FRAME
        wpList.append(wp)
    #waypoint = geometry_msgs.msg.PointStamped()

    rospy.loginfo('Planning and Executing Trajectory ...')
    cartesianPathReq = rail_manipulation_msgs.srv.CartesianPathRequest(waypoints=wpList, avoidCollisions=False)
    cartesianPathRes = cartesianPathService(cartesianPathReq)

    if cartesianPathRes.success:
        rospy.loginfo('Path Planning Succeeded!')
    else:
        rospy.loginfo('Path Planning Failed!')

    rospy.loginfo('Percentage of Path Planned = ' + str(cartesianPathRes.completion))

