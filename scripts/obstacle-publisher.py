#! /usr/bin/env python3

import rospy
from hackathon_trials.msg import List
from geometry_msgs.msg import Point
import time

if __name__ == '__main__':

    rospy.init_node('obstacle_publisher')
    obstaclePublisher = rospy.Publisher('obstacles', List, queue_size=1)

    obstacleList = List()
    obstacles = obstacleList.list
    # Populating obstacles
    for i in range(0, 12, 3):
        for j in range(0, 12, 3):
            if not (i == 0 and j == 0):
                obstacles.append(Point(float(i)/2, float(j)/2, 0.0))

    
    # while not rospy.is_shutdown():
    #     connections = obstaclePublisher.get_num_connections()
    #     if connections > 0:
    #         time.sleep(1.0)
    #         obstaclePublisher.publish(obstacleList)
    #         break
    print("Publishing obstacles in 3 seconds")
    time.sleep(3.0)
    obstaclePublisher.publish(obstacles)
    print("Published")
    time.sleep(1)
    print("Obstacles published. Now exiting...")

