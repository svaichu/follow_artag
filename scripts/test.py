#!/usr/bin/env python3
import rospy
import actionlib
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_geometry_msgs  # Import necessary for transformations

from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathActionFeedback
from welding_robot_msgs.msg import WeldingPathActionGoal
from welding_robot_msgs.msg import WeldingPathActionResult
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult

import moveit_commander
import sys


current_pose = None
goal_poses = []

# mutex = threading.Lock()


# def getTF(data):
#     # global current_pose
#     # mutex.acquire(blocking=True)

#     # for marker in data.markers:
#     #     current_pose = marker.pose



#     # tf_buffer = args

#     # for marker in data.markers:
#     #     # rospy.loginfo("Marker ID: %s", marker.id)
#     #     # rospy.loginfo("Position: (%s, %s, %s)", marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z)
#     #     # rospy.loginfo("Orientation: (%s, %s, %s, %s)", marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)

#     try:
#         transform = tf_buffer.lookup_transform('base_link', "inverted_ar_frame", rospy.Time(0), rospy.Duration(1.0))
#         transformed_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(tf_buffer.inverted_ar_frame, transform)
#         # rospy.loginfo("Transformed Pose in Map Frame: %s", transformed_pose)
#         current_pose = transformed_pose.pose
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#         rospy.logerr("TF2 error: %s", e)
    
    # mutex.release()

# def keyboard_callback(timer):
    # mutex.acquire(blocking=True)
    # rospy.loginfo('ready for input')
    # print('dfdfsfdf')
    # user_input = input()
    # if user_input == 'y':
    #     goal_poses.append(current_pose)
    #     rospy.loginfo('added pose {}'.format(current_pose))
    # if user_input == 'q':
    #     move()
    # mutex.release()


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ar_track_alvar_subscriber', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    mv_grp = moveit_commander.MoveGroupCommander("manipulator")

    # rospy.Subscriber("ar_pose_marker", AlvarMarkers, pose_callback, (tf_buffer))
    # rospy.Timer(rospy.Duration(1), pose_callback)
    while True:
        rospy.loginfo('ready for input')
        user_input = input()
        try:
            if user_input == 'y':
                try:
                    transform = tf_buffer.lookup_transform('base_link', "inverted_ar_frame", rospy.Time(0), rospy.Duration(1.0))
                    transform_pose = PoseStamped()
                    transform_pose.header = transform.header
                    transform_pose.pose.position = transform.transform.translation
                    transform_pose.pose.orientation = transform.transform.rotation
                    transform_pose.pose.position.z = .13
                    transform_pose.pose.position.x += 0.028
                    transform_pose.pose.position.y += 0.01
                    transform_pose.pose.orientation = Quaternion(0,0.9636305,0,-0.2672384 )
                    # transformed_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(tf_buffer.inverted_ar_frame, transform))
                    # rospy.loginfo("Transformed Pose in Map Frame: {}".format(transform_pose))
                    current_pose = transform_pose
                    
                    print(current_pose)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logerr("TF2 error: %s", e)
                    continue
                goal_poses.append(current_pose)
                
                rospy.loginfo('added pose {}'.format(current_pose))
            if user_input == 'q':
                print("Processing ", len(goal_poses))
                
                for pose in goal_poses:
                    mv_grp.set_pose_target(pose)
                    mv_grp.go(wait=True)

                goal_poses = []
        except KeyboardInterrupt:
            break

    
    
