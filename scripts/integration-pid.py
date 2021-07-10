#! /usr/bin/env python3

# Issues
# Fix: plateau level velocity as a function of distance


# ROS Imports
from os import path
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from hackathon_trials.msg import List
# Other imports
import math
from matplotlib import pyplot as plt
import time

class PIDController():

    def __init__(self, startPoint, goalPoint, odometrySubscriber, velocityPublisher) -> None:
        # Initializing PID constants
        self.kp = 7.5
        self.ki = 0.002
        self.kd = 0.002
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
        self.pos_x = []
        self.pos_y = []
        self.vel = []
        self.time_graph = []
        self.timestamp = 0

        # Initializing Twist which will be published and Publisher, Subscriber
        self.velToPublish = Twist()

        self.odometrySubscriber = odometrySubscriber
        self.velocityPublisher = velocityPublisher

        # self.theta is the inclination of the line joining startPoint and goalPoint
        self.theta = math.atan2(self.goalPoint.y - self.startPoint.y, self.goalPoint.x - self.startPoint.x)


    def ramped_vel(self, v_prev, v_target, t_prev, t_now, ramp_rate=0.05):
    # ramp_rate = magnitude of maximum acceleration (in this case just acceleration)
    # print("t prev:%f\t" % (ramp_rate))
        max_vel_step = ramp_rate 
        sign = 1.0 if (v_target > v_prev) else -1.0
        # return min of: required final velocity AND final velocity after max acceleration for the given time interval 
        error = math.fabs(v_target - v_prev)
        if error < max_vel_step:
            print("Ramped vel returned %f"%v_target)
            return v_target
        else:
            print("Ramped vel returned %f"%(v_prev + sign * max_vel_step))
            return v_prev + sign * max_vel_step

    def getPIDValue(self):
        
        self.time_graph.append(self.timestamp)
        self.timestamp += 1
        # Calculating error and P I D terms for this iteration

        self.currentError = getDistance(self.goalPoint, currentPoint)
        p = self.kp * self.currentError
        i = self.ki * self.integral
        d = self.kd * (self.currentError - self.previousError)

        # Set velocity to current PID return value
        self.currentVelocity = 500 * math.tanh((p + i + d)/1000)
        print("PID spits out %f"%self.currentVelocity)

        # Exit condition for when goal is reached
        if self.currentError < 0.05:
            self.velToPublish.linear.x = 0.0
            self.velToPublish.linear.y = 0.0
            self.velocityPublisher.publish(self.velToPublish)
            return True
        
        print("Self current velocity is now %f"%self.currentVelocity)
        # Scaling the velocity as per needs(under experimentation)
        self.currentVelocity = min(self.ramped_vel(self.previousVelocity, self.currentVelocity, None, None, 0.05), self.currentVelocity, 1.5)
        self.vel.append(self.currentVelocity)

        # Exit condition for when velocity is too small
        # 0.05 is less than the linear acceleration so that this does not trigger in the first iteration itself
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
        self.pos_x.append(currentPoint.x)
        self.pos_y.append(currentPoint.y)

        # Preparing for next iteration
        self.previousVelocity = self.currentVelocity
        self.previousError = self.currentError
        self.integral += self.currentError/1000

        return False

        # print("Current velocity: %f "%(self.currentVelocity))            
        # print("Current position: (%f, %f) "%(self.currentPoint.x, self.currentPoint.y))

def odometryCallback(Odometry):
    if Odometry is not None:
        print("Odometry data received: Coordinates (%f, %f)"%(Odometry.pose.pose.position.x, Odometry.pose.pose.position.y))
        currentPoint.x = Odometry.pose.pose.position.x
        currentPoint.y = Odometry.pose.pose.position.y
    # currentVel = math.sqrt(math.pow(Odometry.twist.twist.linear.x,2) + math.pow(Odometry.twist.twist.linear.y,2))
    # print("Current position received is (%f %f)"%(currentPoint.x, currentPoint.y))
    # Test passed: variable currentPoint is being updated

def pathPlannerCallback(pathList):
    global hasPath
    if not hasPath and pathList is not None:
        for point in pathList.list:
            pathArray.append(point)
        hasPath = True
        pathPlannerSubscriber.unregister()

def getVectorMagnitude(x, y):
    return math.sqrt((math.pow(x, 2) + math.pow(y, 2)))

def getDistance(pointA, pointB):
    # Returns Euclidian distance between parameters pointA and pointB
    return math.sqrt(math.pow(pointA.x - pointB.x, 2) + math.pow(pointA.y - pointB.y, 2))



if __name__ == '__main__':

    pathArray = []
    hasPath = False

    print("PID node starting...")
    time.sleep(5)

    # Start point input from user
    # x = float(input("Current x coordinate: "))
    # y = float(input("Current y coordinate: "))
    
    # # Current point is now global
    currentPoint = Point(0, 0, 0)

    # # Goal point input from user
    # x = float(input("Goal x coordinate: "))
    # y = float(input("Goal y coordinate: "))
    # goalPoint = Point(x, y, 0)
    
    currentVel = 0.0

    rospy.init_node('pid_controller')
    velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    velocityPublisher.get_num_connections()
    odometrySubscriber = rospy.Subscriber('odom', Odometry, odometryCallback)
    pathPlannerSubscriber = rospy.Subscriber('path', List, pathPlannerCallback)

    rate = rospy.Rate(10)

    previousPoint = Point(0, 0, 0)

    
    while(len(pathArray) <= 0):
        pass

    print("Printing objects received")
    print(pathArray)

    

    for i in range(len(pathArray) - 1):

        currentPoint = pathArray[i]
        goalPoint = pathArray[i+1]
        print("Current point is: (%f %f)"%(currentPoint.x, currentPoint.y))
        print("Current point is: (%f %f)"%(goalPoint.x, goalPoint.y))

        while not rospy.is_shutdown():
            print("Entered while loop")

            controller = PIDController(currentPoint, goalPoint, odometrySubscriber=odometrySubscriber, velocityPublisher=velocityPublisher)   
            velToPublish = Twist()
            # velToPublish.linear.x, velToPublish.linear.y = controller.getPIDValue()
            isDone = controller.getPIDValue()
            print("Controller returned some value")
            # print("Velocity values received %f %f "%(x, y))
            # velToPublish.linear.x = x
            # velToPublish.linear.y = y
            # # print("velToPublish is %f")%float((getVectorMagnitude(velToPublish.linear.x, velToPublish.linear.y)))
            # velocityPublisher.publish(velToPublish)
            if(isDone):
                print("Goal point (%f %f) reached successfully"%(controller.goalPoint.x, controller.goalPoint.y))
                break
            rate.sleep()

    # plt.title("PID Controller")
    # plt.axes().set_facecolor("#121212")
    # plt.plot(controller.time_graph, controller.vel, color='white')
    # plt.grid(True, color='k')
    # plt.show()
    # print(controller.vel)   
        

    print(currentPoint)