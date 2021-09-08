#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist

#mover el dron con ROS, lo manda hacia adelante y ya

def main():
    print('Starting move node...')
    print(' ')
    rospy.init_node('move', anonymous=True) #Inicio nodo
    pub = rospy.Publisher('/quad_cmd_twist', Twist, queue_size=10)
    rate = rospy.Rate(10) #10hz

    vel_robot = Twist()
    while not rospy.is_shutdown():
        print('entre')
        vel_robot.linear.x = 0
        vel_robot.linear.y = 0
        vel_robot.linear.z = 0
        vel_robot.angular.z = 0
        pub.publish(vel_robot)

        time.sleep(10)
        print('adelante')
        vel_robot.linear.x = 1
        vel_robot.linear.y = 0
        vel_robot.linear.z = 1
        vel_robot.angular.z = 0
        pub.publish(vel_robot)

        
        rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print('Nodo detenido')