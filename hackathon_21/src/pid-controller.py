#! /usr/bin/env python3

# ROS Imports
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from hackathon_21.msg import List
# Other imports
import math
from matplotlib import pyplot as plt
import time

# To be debugged and converted to local variable
global previousVelocity
previousVelocity = 0.0

# Class definition to control the velocity of the robot
class PIDController():

    def __init__(self, startPoint, goalPoint, odometrySubscriber, velocityPublisher) -> None:
        # Initializing PID constants
        self.kp = 5.0
        self.ki = 0.001
        self.kd = 0.001
        # Initializing computational terms
        self.currentError = 0.0
        self.previousError = 0.0
        self.integral = 0.0
        self.currentVelocity = 0.0
        self.previousVelocity = 0.0

        # Setting start and goal points to function parameters
        self.startPoint = startPoint
        self.currentPoint = startPoint
        self.goalPoint = goalPoint

        # Initializing empty lists for plotting graphs after completion
        # Code for this is commented
        self.pos_x = []
        self.pos_y = []
        self.vel = []
        self.time_graph = []
        self.timestamp = 0

        # Initializing Twist which will be published and Publisher, Subscriber
        self.velToPublish = Twist()

        # Initializing publisher and subscriber as variables of the PIDController object
        self.odometrySubscriber = odometrySubscriber
        self.velocityPublisher = velocityPublisher

        # self.theta is the inclination of the line joining startPoint and goalPoint
        # When magnitude velocity of is generated by PID, it will be broken into x and y components of velocity
        self.theta = math.atan2(self.goalPoint.y - self.startPoint.y, self.goalPoint.x - self.startPoint.x)


    # Controls velocity gain by maintaining a constant linear acceleration
    def ramped_vel(self, v_prev, v_target, t_prev, t_now, ramp_rate=0.1):
        max_vel_step = ramp_rate 
        sign = 1.0 if (v_target > v_prev) else -1.0
        # returns min of: required final velocity AND final velocity after max acceleration for the given time interval 
        error = math.fabs(v_target - v_prev)
        if error < max_vel_step:
            return v_target
        else:
            return v_prev + sign * max_vel_step

    # Runs in a loop till goal point is reached
    def getPIDValue(self):
        
        global previousVelocity

        # Calculating error and PID terms for this iteration
        self.currentError = getDistance(self.goalPoint, currentPoint)
        p = self.kp * self.currentError
        i = self.ki * self.integral
        d = self.kd * (self.currentError - self.previousError)

        # Squishing and scaling of PID return value to match the requirements and velocity constraints
        self.currentVelocity = 25 * math.tanh((p + i + d)/100)

        # Exit condition for when goal is reached
        if self.currentError < 0.05:
            self.velToPublish.linear.x = 0.0
            self.velToPublish.linear.y = 0.0
            self.velocityPublisher.publish(self.velToPublish)
            return True
        
        
        # Velocity will be set as minimum of three terms
        # 1 Return value of ramped_vel which adds a constraint on maximum acceleration
        # 2 Squished and scaled PID value
        # 3 Max velocity (pre determined)
        # Logic explained in readme.md
        self.currentVelocity = min(self.ramped_vel(previousVelocity, self.currentVelocity, None, None, ramp_rate=0.075), self.currentVelocity, 0.9)
        self.vel.append(self.currentVelocity)

        # Exit condition for when velocity is too small
        # 0.03 is less than the linear acceleration (0.075) so that this does not trigger in the first iteration itself
        if self.currentVelocity < 0.03:
            self.velToPublish.linear.x = 0.0
            self.velToPublish.linear.y = 0.0
            self.velocityPublisher.publish(self.velToPublish)
            return True

        # Publishing the PID controlled velocity to /cmd_vel
        self.velToPublish.linear.x = (self.currentVelocity * math.cos(self.theta))
        self.velToPublish.linear.y = (self.currentVelocity * math.sin(self.theta))
        self.velocityPublisher.publish(self.velToPublish)

        # Updating positions for position vs time graph
        # Code for plotting commented 
        self.pos_x.append(currentPoint.x)
        self.pos_y.append(currentPoint.y)

        # Preparing for next iteration
        previousVelocity = self.currentVelocity
        self.previousError = self.currentError
        self.integral += self.currentError/100

        # Returns False since goal is not reached yet
        return False


### GLOBAL METHODS ###

# Callback function for odometry subscriber
def odometryCallback(Odometry):
    if Odometry is not None:
        # Sets current point(global variable) value to odometry readings
        currentPoint.x = Odometry.pose.pose.position.x
        currentPoint.y = Odometry.pose.pose.position.y


# Callback function for path planner subscriber
def pathPlannerCallback(pathList):
    # Good enough to subscribe to planned path only once since obstacles are static
    # Boolean hasPath to track whether the node has received the path from path planner node 
    global hasPath
    if not hasPath and pathList is not None:
        for point in pathList.list:
            pathArray.append(point)
        hasPath = True
        pathPlannerSubscriber.unregister()


def getVectorMagnitude(x, y):
    # Returns magnitude of a vector with components parameters x and y
    # Declared for convenience
    return math.sqrt((math.pow(x, 2) + math.pow(y, 2)))

def getDistance(pointA, pointB):
    # Returns Euclidian distance between parameters pointA and pointB
    return math.sqrt(math.pow(pointA.x - pointB.x, 2) + math.pow(pointA.y - pointB.y, 2))


if __name__ == '__main__':

    # Array initialized to store path received from path planner
    pathArray = []
    hasPath = False

    print("PID node starting...")
    time.sleep(3)
    print("PID Controller started. Point to point motion will be regulated by the controller")
    
    # Current point is now global
    currentPoint = Point(0, 0, 0)
    previousPoint = Point(0, 0, 0) 
    currentVel = 0.0

    # Initializing node and publisher, subscibers
    rospy.init_node('pid_controller')
    velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    velocityPublisher.get_num_connections()
    odometrySubscriber = rospy.Subscriber('odom', Odometry, odometryCallback)
    pathPlannerSubscriber = rospy.Subscriber('path', List, pathPlannerCallback)

    rate = rospy.Rate(10) # 10 Hz

    # Loop to wait for pathArray to be populated by subscriber callback
    while(len(pathArray) <= 0):
        continue

    for i in range(len(pathArray) - 1):
        # PID Controller used for every for point to point motion
        # pathArray elements are points to be traverse in order

        currentPoint = pathArray[i]
        goalPoint = pathArray[i+1]

        while not rospy.is_shutdown():

            controller = PIDController(currentPoint, goalPoint, odometrySubscriber=odometrySubscriber, velocityPublisher=velocityPublisher)
            velToPublish = Twist()
            isDone = controller.getPIDValue()
            if(isDone):
                print("Goal point (%f %f) reached successfully"%(controller.goalPoint.x, controller.goalPoint.y))
                # Waiting time between distinct paths
                time.sleep(1.0)
                break
            rate.sleep()

    print("Reached goal point: ")
    print(currentPoint)
    print("Terminating node")