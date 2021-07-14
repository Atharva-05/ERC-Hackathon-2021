#! /usr/bin/env python3

# ROS Imports
import rospy
from geometry_msgs.msg import Point
from PIL import Image

# Custom message Import
# List.msg definition
# geometry_msg/Point[] lis
from hackathon_trials.msg import List

# Other Imports
import time
import math
import random
from matplotlib import pyplot as plt

# Class definition of a Node for implemnting path planning algorithms
class Node():

    def __init__(self, point, planner, isRoot=False):

        # the parameter planner is an instance of Planner() class and stores nodes
        
        self.point = point
        self.x = point.x
        self.y = point.y
        if isRoot:
            # Id of parent(root) node 0
            self.id = 0
        else:
            self.id = len(planner.nodes)
            # ID of every new node is the number of nodes already existing
            # ID of parent/root node is 0 

        self.parentNodeID = 0
        # Initialized to zero, will be set to nearest node's ID when it is connected

    # getter setter methods
    def getID(self):
        return self.id

    def getParentNodeID(self):
        return self.parentNodeID

    def getPoint(self):
        return self.point.x, self.point.y

    def setParentNodeID(self, parentNodeID):
        self.parentNodeID = parentNodeID

# Class definiton for planner which managees the path planning algorithm
class Planner():

    # Implementation of a crude variation of RRT algorithm

    def __init__(self, goalPoint, width=6, height=6, botRadius=0.30, obstacleRadius=0.25):
        
        # Initializing empty lists
        self.points = []
        self.obstacles = []
        self.path = []
        self.nodes = []

        # Initializing parameters 
        self.width = width      # Points lying between (0, width) and
        self.height = height    # (0, height) will be sampled
        self.obstacleRadius = obstacleRadius
        self.botRaduis = botRadius
        # clearanceRadius value is used to check whether a point lies within an obstacle or not
        # i.e. will it be in contact with an obstacle when it is at that point
        self.clearanceRadius = obstacleRadius + botRadius
        
        # Root node is starting point (0, 0) with index 0
        self.rootNode = Node(Point(0.0, 0.0, 0.0), self, True)
        self.nodes.append(self.rootNode)
        self.goalPoint = goalPoint
        self.isGoalReached = False
        self.targetNode = Node(self.goalPoint, self)
        print("Path Planner: Clearance radius set to: %f"%self.clearanceRadius)

    def getDistance(self, pointA, pointB):
        # Returns Euclidian distance between parameters pointA and pointB
        return(math.sqrt(math.pow(pointB.x - pointA.x, 2) + math.pow(pointB.y - pointA.y, 2)))

    def generateRandomPoint(self):
        # Generates points that are not within bonk distance of any obstacle
        pointGenerated = False
        while not pointGenerated:
            x = random.uniform(0.0, self.width)
            y = random.uniform(0.0, self.height)
            point = Point(x, y, 0.0)
            if self.isFree(point):
                return point
            else:
                continue
         
    def isFree(self, point):
        # Returns true if parameter point does not lie within bonk distance of any obstacle
        for obs in self.obstacles:
            if(self.getDistance(obs, point) < self.clearanceRadius):
                return False
        return True

    def generateObstacles(self, obstacles):
        # Populates obstacles array of planner object
        self.obstacles = obstacles
    
    def drawObstacles(self):
        # Plots obstacles using matplotlib
        x_pts = []
        y_pts = []
        for i in range(0, len(self.obstacles)):
            x_pts.append(self.obstacles[i].x)
            y_pts.append(self.obstacles[i].y)
            # tempCircle = plt.Circle((self.obstacles[i].x, self.obstacles[i].y), 0.25)
            plt.gca().add_patch(plt.Circle((self.obstacles[i].x, self.obstacles[i].y), 0.25))

    def drawLine(self, pointA, pointB, width=0.5, color='green'):
        # Plots a line between two points
        plt.plot([pointA.x, pointB.x], [pointA.y, pointB.y], linewidth=width, color=color)

    def drawPoint(self, point):
        # Plots the given point
        plt.plot(point.x, point.y, 'ro', marker='*')

    def drawAllPaths(self):
        # draws all possible paths that don't cross obstacles
        for node in self.nodes:
            for anotherNode in self.nodes:
                if self.isPathValid(node.point, anotherNode.point) and node.point != anotherNode.point:
                    self.drawLine(node.point, anotherNode.point)

    def isPathValid(self, pointA, pointB):
        # Checks if a path between two points crosses an obstacle

        for i in range(0, 101):
            u = i/100
            x = u * pointA.x + (1-u) * pointB.x
            y = u * pointA.y + (1-u) * pointB.y
            if not self.isFree(Point(x, y, 0.0)):
                return False
        return True
    
    def newNode(self, point, planner):
        # initializes new node and adds it to the list of nodes of planner object
        node = Node(point, planner)
        self.nodes.append(node)
        return node

    def findNearestNode(self, newNode):
        # Finds and returns nearest node's ID
        minDist = self.getDistance(newNode.point, Point())
        # Testing statement
        # print("Min distance I start with is: %f"%minDist)
        nearestNodeID = 0
        for node in self.nodes:
            if self.getDistance(node.point, newNode.point) < minDist and self.isPathValid(newNode.point, node.point) and node.id != newNode.id:
                nearestNodeID = node.id
                minDist = self.getDistance(node.point, newNode.point)
        newNode.setParentNodeID(nearestNodeID)
        return nearestNodeID

    def connectNodes(self, node, parentNode):
        # Connects node to parentNode i.e assigns node.parent = parentNode
        node.setParentNodeID(parentNode.id)

    def drawPath(self):
        # Currently draws all possible paths
        # Update: now draws final path
        for node in self.path:
            self.drawLine(node.point, self.nodes[node.parentNodeID])

    def setGoalPoint(self, goalPoint):
        self.goalPoint = goalPoint

    def nodeVicinity(self, newNode, dmin=0.20):
        # Returns true if there are no nodes within dmin distance of parameter newNode
        for node in self.nodes:
            dist = self.getDistance(newNode.point, node.point)
            if dist < dmin:
                return False
        return True

    def initializeNodes(self, limit=3000):
        counter = 0
        while counter < limit and not self.isGoalReached:
            counter += 1
            point = self.generateRandomPoint()
            newNode = Node(point, self, False)
            nearestID = self.findNearestNode(newNode)
            if self.isPathValid(point, self.nodes[nearestID].point) and self.nodeVicinity(newNode):
                newNode = self.newNode(point, self)
                self.connectNodes(newNode, self.nodes[nearestID])

                if self.isPathValid(point, self.goalPoint):
                    self.targetNode = self.newNode(self.goalPoint, self)
                    # goalNode = self.newNode(self.goalPoint, self)
                    self.connectNodes(self.targetNode, newNode)
                    self.isGoalReached = True
                    break
                    

                if self.getDistance(newNode, self.goalPoint) < 0.15:
                    print("Target node is (%f %f)"%(newNode.point.x, newNode.point.y))
                    self.targetNode = newNode
                    self.isGoalReached = True
                    break
            else:
                continue

    def plotPath(self):
        targetNode = self.targetNode
        # print("Final target node is (%f %f)"%(targetNode.point.x, targetNode.point.y))
        self.path.append(targetNode)
        parentIDList = []
        parentIDList.append(targetNode.getID())
        parentIDList.append(targetNode.getParentNodeID())

        while True:
            parentID =  self.nodes[parentIDList[-1]].getParentNodeID()
            parentIDList.append(parentID)
            if parentID == 0:
                break

        return parentIDList


# Callback function for subscriber to obstacle publisher
def obstacleSubscriberCallback(obstacleList):
    # Needs to execute only one call since obstacles are static and known
    global hasObstacles
    if not hasObstacles and obstacleList is not None:
        for obstacle in obstacleList.list:
            obstacles.append(obstacle)
        hasObstacles = True
        print("PATH PLANNER: Obstacles received")
        obstacleSubscriber.unregister()
        

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('path_planner')
    
    # Initializing empty list which will be populated by subscriber callback
    obstacles = []
    
    pathPlannerPublisher = rospy.Publisher('path', List, queue_size=1)
    obstacleSubscriber = rospy.Subscriber('obstacles', List, obstacleSubscriberCallback)

    hasObstacles = False
    # Loop to wait till subscriber receives obstacles. 
    # Boolean value flipped in callback after successful execution of callback
    while not hasObstacles:
        continue

    n = 40
    x_pts = []
    y_pts = []

    # Hardcoded parameters for sampling logic
    width = 5.5
    height = 5.5
    botRadius = 0.4
    obstacleRadius = 0.25 # Improvement: take as parameter from obstacle publisher
    
    # Default goal point
    goal_x = 5.0
    goal_y = 5.0
    
    # Instantiating object of Planner class
    planner = Planner(Point(goal_x, goal_y, 0.0),width, height, botRadius, obstacleRadius)
    # Sets and plots obstacles received from obstacle publisher
    planner.generateObstacles(obstacles)
    planner.drawObstacles()
    
    # Code to get a valid goal point input
    # Temporary sleep line put to avoid problems in launch file
    # When obstacle publisher node is killed, it prints to the terminal,
    # at that point the path planner node should not be taking input 
    time.sleep(3)
    validInput = False
    print("\nPATH PLANNER: RRT Path Planner Node")
    while not validInput:
        # print("Enter Goal x coordinate")
        goal_x = float(input("Enter Goal x coordinate: "))
        # print("\nEnter Goal y coordinate")
        goal_y = float(input("Enter Goal y coordinate: "))
        goalPoint = Point(goal_x, goal_y, 0.0)
        if(planner.isFree(goalPoint)):
            print("\nGoal point is valid")
            planner.goalPoint = goalPoint
            validInput = True
        else:
            print("\n *ALERT*: Goal point lies within bonk distance of an obstacle")
            print("Please enter another goal point")

    # Sets final goal point
    planner.goalPoint = Point(goal_x, goal_y, 0.0)

    print("\nPATH PLANNER: Computing path...\n")

    # Actual path planning starts which will be put in loop

    # Debugging: If the limit of number of iterations (3000 as of now) is crossed before
    # a point within

    # Initializes nodes and plans a path using a modified version of RRT algorithm
    planner.initializeNodes()

    # Setting plot parameters
    plt.xlim(-1, 6)
    plt.ylim(-1, 6)
    plt.axes().set_facecolor("#121212")
    plt.title("RRT Path Planner")

    # Testing
    print("PATH PLANNER: Length of planner.nodes: %d"%len(planner.nodes))

    # Plots all sampled points
    for node in planner.nodes:
        planner.drawPoint(node.point)
        
    # Connects all nodes with their parent
    # for node in planner.nodes:
    #     planner.drawLine(node.point, planner.nodes[node.parentNodeID].point, width=0.5, color='blue')
    
    # planner.plotPath() returns a list of IDs of parents of all nodes involved in the path
    parentIDList = planner.plotPath()

    # Message to publish
    msgList = List()
    pathList = msgList.list

    for id in parentIDList:
        planner.drawLine(planner.nodes[id].point, planner.nodes[planner.nodes[id].getParentNodeID()], width=2)
        # Appending list pathList to be published
        pathList.append(planner.nodes[id].point)

    # The list is populated in the reverse direction from the goal point and hence should be reversed
    pathList.reverse()
    
    print("PATH PLANNER: Goalpoint coordinates are: (%f, %f)"%(planner.targetNode.point.x, planner.targetNode.point.y))
    print("PATH PLANNER: Length of path is: %d"%len(pathList))
    print("PATH PLANNER: Number of nodes is: %d"%len(planner.nodes))

    print("Path Planner: Starting publishing when subscriber connects...")
    
    # Shows the planned path and saves to a .png file
    plt.savefig('path.png')
    plt.show()
    
    # Waiting to ensure controller node has started
    time.sleep(3)

    while not rospy.is_shutdown():
        # Publishes path only once when the controller node establishes a connection
        # by subscribing to the 'path' topic
        connections = pathPlannerPublisher.get_num_connections()
        if connections > 0 and hasObstacles:
            time.sleep(1)
            print("Publishing calculated path...")
            pathPlannerPublisher.publish(pathList)
            break
    # Shows the path planned
    
    print("Path published successfully. Terminating node")

        
