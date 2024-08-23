#!/usr/bin/env python2

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ikh_ros_msgs.srv import SetFloatList, SetFloatListResponse 
from mbf_msgs.srv import CheckPoint, CheckPointResponse
import math
from tf import transformations

TOLERANCE = 0.1
CMD_CUTOFF = 5.0

class Teleport():
    def __init__(self):
        self.s = rospy.Service('teleport', CheckPoint, self.p_control)
        self.response = CheckPointResponse()
        self.pub = rospy.Publisher('joint_cmd_vel', Twist, queue_size=20)

        rospy.loginfo("/aristos/mobile_robot_simulator/teleport service initialized")


    def p_control(self, req): 
        try: 
            cmd = Twist()
            # setX = req.point.point.x    
            # setY = req.point.point.y    
            goal = req.point.point
            cpose = rospy.wait_for_message('/aristos/ekf/global/pose_estimation', Odometry, timeout=2)
            dist = np.linalg.norm([goal.x-cpose.pose.pose.position.x, goal.y-cpose.pose.pose.position.y])

            while(dist>TOLERANCE):
                dist = np.linalg.norm([goal.x-cpose.pose.pose.position.x, goal.y-cpose.pose.pose.position.y])
                cpose = rospy.wait_for_message('/aristos/ekf/global/pose_estimation', Odometry, timeout=2)

                cmd_np = self.compute_cmd(goal, cpose)
                cmd.linear.x = max(min(cmd_np[0],CMD_CUTOFF),-CMD_CUTOFF)
                cmd.linear.y = max(min(cmd_np[1],CMD_CUTOFF),-CMD_CUTOFF)
                self.pub.publish(cmd)
                rospy.sleep(0.1)

            self.response.cost = dist
            self.response.state = 1
            return self.response
        except Exception as e:
            rospy.logerr("Exception: "+str(e))
            self.response.cost = 0
            self.response.state = 0      
            return self.response   

    def compute_cmd(self, goal, cpose):
        goal_ = self.point2np(goal)
        cpose_ = self.point2np(cpose.pose.pose.position)
        error = goal_ - cpose_
        R = transformations.quaternion_matrix(self.quat2np(cpose.pose.pose.orientation))
        return R[0:3,0:3].transpose().dot(error)

    def point2np(self, p):
        return np.array([p.x, p.y, p.z])
    
    def quat2np(self, q):
        return np.array([q.x, q.y, q.z, q.w])
    

if __name__=="__main__":
    try:
        rospy.init_node('teleport_mobile_simulator_srv', anonymous=False)
        ctrl = Teleport()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass