#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import rospy
from mavros_msgs.msg import PositionTarget, State, HomePosition
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math
 
 
msg = """
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%%%%%%%%%%%%%%%%%%%%%%%
offboard_cotrol
%%%%%%%%%%%%%%%%%%%%%%%
---------------------------
CTRL-C to quit
 
"""

class Offboard:
    
    def __init__(self):
        
        rospy.init_node('PX4_AuotFLy' ,anonymous=True)
        
        self.cur_Position_Target = PositionTarget()
        self.mavros_state = State()
        self.current_position = Point()
        self.current_yaw = 0
        self.done = 0
        self.reach = 0
        
        '''
        ros services
        '''
        self.armServer = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.setModeServer = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        ''' 
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        '''
        ros subscribers
        '''
        self.mavros_state_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback, queue_size=1)
            
        print("Initialized")
        
    def start(self):
        '''
        main ROS thread
        '''
        
        if self.armServer(True) :
            print("Vehicle arming succeed!")
        if self.setModeServer(custom_mode='OFFBOARD'):
            print("Vehicle offboard succeed!")
        else:
            print("Vehicle offboard failed!")
            
        self.cur_Position_Target = self.Intarget_local()
            
        while not rospy.is_shutdown():
                self.run_state_update()
                time.sleep(0.2)
                        
    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]
        return rotate_z_rad
        
    def local_pose_callback(self, msg):
        self.current_position = msg.pose.position
        self.current_yaw = self.q2yaw(msg.pose.orientation)

    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        if self.mavros_state.armed == 0 :
            print("disarm")
                       
    def Intarget_local(self):
        set_target_local = PositionTarget()
        set_target_local.type_mask = 0b100111111000  
        set_target_local.coordinate_frame = 1
        set_target_local.position.x = 0
        set_target_local.position.y = 0
        set_target_local.position.z = 2
        set_target_local.yaw = 0
        
        return set_target_local
    
    def run_state_update(self):
        if self.done == 0:
            
            if self.mavros_state.armed == 0 :
                if self.mavros_state.connected == True:
                    if self.armServer(True) :
                        print("Vehicle arming succeed!")
                        
            if self.mavros_state.mode != 'OFFBOARD':
                self.setModeServer(custom_mode='OFFBOARD')
                self.local_target_pub.publish(self.cur_Position_Target)
                print("wait offboard")
            else: 
                self.local_target_pub.publish(self.cur_Position_Target)
                # print("local_target_pub.publish")
            
            # check reach and set hover
            if((self.reach == 0) and abs(self.current_position.x - self.cur_Position_Target.position.x) < 0.1) \
                and (abs(self.current_position.y - self.cur_Position_Target.position.y) < 0.1)  \
                and (abs(self.current_position.z - self.cur_Position_Target.position.z) < 0.1):
                # self.setModeServer(custom_mode='HOVER')
                self.reach = 1
                print("reach!")
                
            # exit
            if((self.reach == 1) and ((abs(self.current_position.x - self.cur_Position_Target.position.x) > 0.25) \
                or (abs(self.current_position.y - self.cur_Position_Target.position.y) > 0.25) \
                or (abs(self.current_position.z - self.cur_Position_Target.position.z) > 0.25))):
                self.done = 1
                print("done!")
                
                       
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    print (msg)
    
    offboard = Offboard()
    offboard.start()