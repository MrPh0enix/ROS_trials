#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import random, math

class TurtleBotFollower:
    
    def __init__(self):
        self.vel = Twist()
        self.curr_state = None
        self.angle_to_turn = None


    def lidar_callback(self, data):

        min_i = min(data.ranges[270:315])
        min_idx = None
        if min_i <= 0.5 and self.curr_state != 'following_wall' and self.curr_state != 'wall_follow':
            for idx, val in enumerate(data.ranges[270:315]):
                if val == min_i:
                    min_idx = idx
            self.angle_to_turn = (360-min_idx)*(math.pi/180)
            print('angle to turn: ',self.angle_to_turn)
            self.curr_state = 'wall_follow'
            print('wall_detected')


    def wall_follow(self):
        sub_lidar = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        pub_msg = rospy.Publisher("state_msg", String, queue_size=10)
        rospy.init_node('wall_follower_node', anonymous=True)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.curr_state == 'wall_follow':

                pub_msg.publish('wall_follow')
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                pub_vel.publish(self.vel)
                
                ang_vel = 0.1
                self.vel.angular.z = ang_vel
                t_turn = self.angle_to_turn/ang_vel 
                t_init = rospy.Time.now().to_sec()

                print(t_turn)

                while rospy.Time.now().to_sec()-t_init <= t_turn:
                    pub_vel.publish(self.vel)
                    rate.sleep()

                # while self.angle_to_turn >= 0:
                #     pub_vel.publish(self.vel)
                #     rate.sleep()

                self.vel.angular.z = 0
                pub_vel.publish(self.vel)

                self.vel.linear.x = 0.2
                pub_vel.publish(self.vel)

                self.curr_state = 'following_wall'




if __name__ == '__main__':
    turt = TurtleBotFollower()
    turt.wall_follow()   