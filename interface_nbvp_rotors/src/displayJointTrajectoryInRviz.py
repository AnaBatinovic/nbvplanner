#!/usr/bin/env python
import sys, os
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint, \
	JointTrajectory, JointTrajectoryPoint
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Empty
from pykdl_utils.kdl_kinematics import KDLKinematics

class displayMultiDOFTrajectoryInRviz():

	def __init__(self):
		# Subscribes to trajectory and stores it in message below
		self.trajectorySub = rospy.Subscriber(
			"/red/joint_trajectory", JointTrajectory, 
			self.trajectoryCallback)
		self.trajectory = JointTrajectory()

		# Clear path
		rospy.Subscriber("clearVisualizationPath", Empty, self.clearPathCallback)

		# Loop rate determines display rate of path in rviz
		self.loopRate = rospy.get_param("loop_rate", 30)
		self.appendFlag = rospy.get_param("append", False)
		

		# Path is sent to Rviz for display
		self.pathPub = rospy.Publisher("path", Path, queue_size=10)
		self.map_frame = "mavros/world"
		self.path = Path()
		for i in range(2):
			tempPoseStamped = PoseStamped()
			self.path.poses.append(tempPoseStamped)
		# In addition, current position of robot can be displayed, in that case
		# subscriber to position feedback will be needed
		#self.pointPub = rospy.Publisher("position", PointStamped, queue_size=1)


	def run(self):
		r = rospy.Rate(self.loopRate)
		while not rospy.is_shutdown():
			#pt = PointStamped()
			#pt.header.stamp = rospy.Time.now()
			#pt.header.frame_id = "world"
			#self.pointPub.publish(pt)

			self.path.header.stamp = rospy.Time.now()
			self.path.header.frame_id = "mavros/world"
			self.pathPub.publish(self.path)

			r.sleep()

	def trajectoryCallback(self, data):
		target_path_msg = Path()
		target_path_msg.header.stamp = rospy.Time.now()
		target_path_msg.header.frame_id = self.map_frame
		#For every points in trajectory message
		for point in data.points: 
			joint_angles = point.positions
			pose_stamped_msg = PoseStamped()
			transform_mat = self.kdl_kin.forward(joint_angles)
			#Get pose from transform matrix
			pose_stamped_msg.pose.position.x = transform_mat[0,3]
			pose_stamped_msg.pose.position.y = transform_mat[1,3]
			pose_stamped_msg.pose.position.z = transform_mat[2,3]
			target_path_msg.poses.append(pose_stamped_msg)
			rospy.logdebug('adding pose to path')

		#Publish path
		rospy.logdebug('publishing path')
		self.pathPub.publish(target_path_msg)
		# joinTrajecPoint = JointTrajectoryPoint ()
		# joinTrajecPoint = data.points
		# self.trajectory.points.append(joinTrajecPoint)
		# # print len(self.trajectory.points)

		# if self.appendFlag == False:
		# 	# print "Append Flag False"
		# 	self.path.poses = []

		# for i in range(len(self.trajectory.points)):
		# 	tempPoseStamped = PoseStamped()
		# 	tempPoseStamped.pose.position.x = self.trajectory.points[i].positions[i][0]
		# 	tempPoseStamped.pose.position.y = self.trajectory.points[i].positions[1]
		# 	tempPoseStamped.pose.position.z = self.trajectory.points[i].positions[2]
		# 	tempPoseStamped.pose.orientation.w = 1.0
		# 	tempPoseStamped.header.stamp = rospy.Time.now()
		# 	tempPoseStamped.header.frame_id = "mavros/world"
		# 	self.path.poses.append(tempPoseStamped)

	def clearPathCallback(self, msg):
		self.path.poses = []

if __name__ == "__main__":
	rospy.init_node('displayJointTrajectoryInRviz')
	a = displayMultiDOFTrajectoryInRviz()
	a.run()
