#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import random



class TurtleBotMover:

    def __init__(self):
        self.posX = None
        self.vel = Twist()
        self.collision_stop = False
        self.curr_state = 'randm_mvr'
    

    def state_callback(self, msg):
        # if msg.data == 'collision_stop':
        #     print('stopped')
        #     self.collision_stop = True 
        # elif msg.data == 'collision_stop_done':
        #     print('started')
        #     self.collision_stop = False  

        self.curr_state = msg.data 
            


    def random_mover(self):
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        sub_colission = rospy.Subscriber("state_msg", String, self.state_callback)
        
        rospy.init_node('random_mvr_node', anonymous=True)
        rate = rospy.Rate(10)

        linear_spd = 0.2
        ang_spd = 0.4
        dist_to_mv = 1


        while not rospy.is_shutdown():
            if self.curr_state == 'randm_mvr':
                
                rot_sec = random.randint(0,10)
                rot_dir = random.choice([-1,1])
                self.vel.angular.z = rot_dir*ang_spd
                t_init = rospy.Time.now().to_sec()
                
                while rospy.Time.now().to_sec()-t_init < rot_sec and self.curr_state == 'randm_mvr':
                    pub.publish(self.vel)
                    rate.sleep()
                
                self.vel.angular.z = 0
                self.vel.linear.x = linear_spd
                t_init = rospy.Time.now().to_sec()
                t_dur = dist_to_mv/linear_spd

                while rospy.Time.now().to_sec()-t_init <= t_dur and self.curr_state == 'randm_mvr':
                    pub.publish(self.vel)
                    rate.sleep()
                
                
                self.vel.linear.x = 0
                pub.publish(self.vel)




if __name__ == "__main__":

    turt = TurtleBotMover()
    turt.random_mover()
    
    
    