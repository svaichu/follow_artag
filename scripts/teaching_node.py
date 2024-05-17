#!/usr/bin/env python3
import rospy
import actionlib
import tf2_ros
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_geometry_msgs  # Import necessary for transformations

from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathActionFeedback
from welding_robot_msgs.msg import WeldingPathActionGoal
from welding_robot_msgs.msg import WeldingPathActionResult
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult


current_pose = None
goal_poses = []
rospy.init_node('ar_track_alvar_subscriber', anonymous=True)


def callback(data, args):
    global current_pose
    tf_buffer = args

    for marker in data.markers:
        # rospy.loginfo("Marker ID: %s", marker.id)
        # rospy.loginfo("Position: (%s, %s, %s)", marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z)
        # rospy.loginfo("Orientation: (%s, %s, %s, %s)", marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)

        try:
            transform = tf_buffer.lookup_transform('map', marker.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(header=marker.header, pose=marker.pose.pose), transform)
            # rospy.loginfo("Transformed Pose in Map Frame: %s", transformed_pose)
            current_pose = transformed_pose.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF2 error: %s", e)
            continue

def listener():
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
      # List to hold up to 10 goal poses
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback, (tf_buffer))



if __name__ == '__main__':
    listener()
    client = actionlib.SimpleActionClient("ur_mover", WeldingPathAction)

    print("=== waiting for server ...")
    client.wait_for_server()
    while True:
        if len(goal_poses) >= 10:
            rospy.loginfo("Maximum number of goal poses (10) already saved.")

            goal = WeldingPathGoal(poses=goal_poses, goal_description = "This is my funny goal Description")
            client.send_goal(goal)

            client.wait_for_result()

            result = client.get_result()

            if result == 1:
                print("=== EXECUTION WAS SUCCESSFUL")
            
            elif result == -1:
                print("=== ERROR! Execution failed")
        goal_poses = []
            
        if input("Save this pose as goal pose? (y/n): ").strip().lower() == 'y':
            if current_pose is not None :
                goal_poses.append(current_pose)
            rospy.loginfo("Goal pose for marker %s saved in the map frame.", current_pose)