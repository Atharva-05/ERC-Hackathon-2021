#! /usr/bin/env python3

# ROS Imports
import rospy
from geometry_msgs.msg import Point
# Custom message Import
from hackathon_trials.msg import List

# List.msg definition
# geometry_msg/Point[] list

# Other Import
import time

if __name__ == '__main__':

    # Initializing node
    rospy.init_node('obstacle_publisher')
    obstaclePublisher = rospy.Publisher('obstacles', List, queue_size=1)

    # Initializing empty list
    obstacleList = List()
    obstacles = obstacleList.list

    # Populating obstacles
    for i in range(0, 12, 3):
        for j in range(0, 12, 3):
            if not (i == 0 and j == 0):
                obstacles.append(Point(float(i)/2, float(j)/2, 0.0))
    
    print("OBSTACLE PUBLISHER: Publishing obstacles when subscriber connects")

    while not rospy.is_shutdown():
        # Obstacles need to be published only once since obstacles are static and known
        connections = obstaclePublisher.get_num_connections()
        # print("OBSTACLE PUBLISHER: Connections: %d"%connections)
        if connections > 0:
            # Waiting to make sure all nodes have started before publishing
            time.sleep(1.0)
            obstaclePublisher.publish(obstacleList)
            print("OBSTACLE PUBLISHER: Obstacles published")
            break

    time.sleep(1)
    print("OBSTACLE PUBLISHER: Terminating obstacle publisher node")