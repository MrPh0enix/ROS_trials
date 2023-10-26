#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String



class TurtleBotObstacle:

    def __init__(self):
        self.vel = Twist()
        self.stop = False
        self.curr_state = 'randm_mvr'


    def lidar_callback(self, data):

        flag = False
        
        if min(data.ranges[315:360]+data.ranges[0:45]) <= 0.3:
            flag = True
           
            
        if flag == True and self.curr_state == 'randm_mvr':
            self.curr_state = 'collision_stop'
        elif flag == False and self.curr_state == 'collision_stop':
            self.curr_state = 'randm_mvr'

            
        

    def obs_avoidance(self):
        sub_lidar = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        pub_msg = rospy.Publisher("state_msg", String, queue_size=10)
        rospy.init_node('obs_avoidance_node', anonymous=True)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.curr_state == 'collision_stop':
                
                pub_msg.publish('collision_stop')
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                pub_vel.publish(self.vel)

                self.vel.angular.z = 0.2
                
                while self.curr_state == 'collision_stop':
                    pub_vel.publish(self.vel)
                    rate.sleep()
            
                self.vel.angular.z = 0
                pub_vel.publish(self.vel)
                
                pub_msg.publish('randm_mvr')
                # self.stop = False
                rate.sleep()
        


if __name__ == '__main__':
    turt_lidar = TurtleBotObstacle()
    turt_lidar.obs_avoidance()