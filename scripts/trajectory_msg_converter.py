#!/usr/bin/env python

"""@trajectory_msg_converter.py
This node converts Fast-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by geometric_controller
Authors: Mohamed Abdelkader
"""

# Imports
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        fast_planner_traj_topic = rospy.get_param('~fast_planner_traj_topic', 'planning/pos_cmd')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', 'command/trajectory')
        
        setpoint_position_pub_topic = rospy.get_param('~setpoint_position_pub_topic', '/mavros/setpoint_position/local')
        setpoint_raw_pub_topic = rospy.get_param('~setpoint_raw_pub_topic', '/mavros/setpoint_raw/local')

        # Publisher for geometric_controller
        self.traj_pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=1)
        
        # Publisher for mavros
        self.setpoint_position_pub = rospy.Publisher(setpoint_position_pub_topic, PoseStamped, queue_size=1)
        self.setpoint_raw_pub = rospy.Publisher(setpoint_raw_pub_topic, PositionTarget, queue_size=1)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)

        rospy.spin()

    def fastPlannerTrajCallback(self, msg):
        # 1. geometric_controller
        # position and yaw
        pose = Transform()
        pose.translation.x = msg.position.x
        pose.translation.y = msg.position.y
        pose.translation.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        pose.rotation.x = q[0]
        pose.rotation.y = q[1]
        pose.rotation.z = q[2]
        pose.rotation.w = q[3]

        # velocity
        vel = Twist()
        vel.linear = msg.velocity
        # TODO: set vel.angular to msg.yaw_dot

        # acceleration
        acc = Twist()
        acc.linear = msg.acceleration

        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(pose)
        traj_point.velocities.append(vel)
        traj_point.accelerations.append(acc)

        traj_msg = MultiDOFJointTrajectory()

        traj_msg.header = msg.header
        traj_msg.points.append(traj_point)
        self.traj_pub.publish(traj_msg)
        
        # 2. setpoint_position_pub
        # setpoint_msg = PoseStamped()
        # setpoint_msg.header = msg.header
        # setpoint_msg.pose.position.x = msg.position.x
        # setpoint_msg.pose.position.y = msg.position.y
        # setpoint_msg.pose.position.z = msg.position.z
        # setpoint_msg.pose.orientation.w = q[3]
        # setpoint_msg.pose.orientation.x = q[0]
        # setpoint_msg.pose.orientation.y = q[1]
        # setpoint_msg.pose.orientation.z = q[2]
        # self.setpoint_position_pub.publish(setpoint_msg)
        
        # 3. setpoint_raw_pub
        target_raw_pose = PositionTarget()
        target_raw_pose.header = msg.header
        
        target_raw_pose.coordinate_frame = 1
        target_raw_pose.position = msg.position
        target_raw_pose.velocity = msg.velocity
        target_raw_pose.yaw = msg.yaw
        
        motion_type = 0
        if(motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE
        if(motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW
        if(motion_type == 2):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW
                            
        self.setpoint_raw_pub.publish(target_raw_pose)             

if __name__ == '__main__':
    obj = MessageConverter()