#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from hackathon_trials.msg import List
import time
import math
import random
from matplotlib import pyplot as plt

class Node():

    def __init__(self, point, planner, isRoot=False):
        self.point = point
        self.x = point.x
        self.y = point.y
        if isRoot:
        # Id of parent node 0, since planner.nodes is uninitialized yet
            self.id = 0
        else:
            self.id = len(planner.nodes)
        # ID set serially next number from no of nodes

        self.parentNodeID = 0
       # Initialized to zero, will be set to nearest node's ID when it is connected

    def getID(self):
        return self.id

    def setParentNodeID(self, parentNodeID):
        self.parentNodeID = parentNodeID

    def getParentNodeID(self):
        return self.parentNodeID

    def getPoint(self):
        return self.point.x, self.point.y

class Planner():

    def __init__(self, goalPoint, width=6, height=6, botRadius=0.30, obstacleRadius=0.25):
        self.points = []
        self.obstacles = []
        self.path = []
        self.width = width
        self.height = height
        self.obstacleRadius = obstacleRadius
        self.botRaduis = botRadius
        self.clearanceRadius = obstacleRadius + botRadius
        self.nodes = []
        # Root node is starting point (0, 0) with index 0
        self.rootNode = Node(Point(0.0, 0.0, 0.0), self, True)
        self.nodes.append(self.rootNode)
        self.goalPoint = goalPoint
        self.isGoalReached = False
        self.targetNode = Node(self.goalPoint, self)
        print("Clearance radius set to: %f"%self.clearanceRadius)

    def getDistance(self, pointA, pointB):
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

    def populate(self, n):
        # Used for testing
        flag = True
        counter = 0
        while flag:
            point = self.generateRandomPoint()
            if point is not None:
            # if(self.isFree(point)):
                self.points.append(point)
                counter += 1
            if(counter > n):
                flag = False

    def generateObstacles(self, obstacles):
        # Populates obstacles array of planner object
        self.obstacles = obstacles
    
    def drawObstacles(self):
        # Plots obstacles
        x_pts = []
        y_pts = []
        for i in range(0, len(self.obstacles)):
            x_pts.append(self.obstacles[i].x)
            y_pts.append(self.obstacles[i].y)
            # tempCircle = plt.Circle((self.obstacles[i].x, self.obstacles[i].y), 0.25)
            plt.gca().add_patch(plt.Circle((self.obstacles[i].x, self.obstacles[i].y), 0.25))

    def drawPoints(self):
        # Plots points
        x_pts = []
        y_pts = []
        for i in range(len(self.points) - 1):
            x_pts.append(self.points[i].x)
            y_pts.append(self.points[i].y)
        return x_pts, y_pts

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

        # r = self.getDistance(pointA, pointB)
        # theta = math.atan2(pointB.y - pointA.y, pointB.x - pointA.x)
        for i in range(0, 101):
            u = i/100
            x = u * pointA.x + (1-u) * pointB.x
            y = u * pointA.y + (1-u) * pointB.y
            if not self.isFree(Point(x, y, 0.0)):
                return False

        return True
    
    def newNode(self, point, planner):
        # initializes new node and adds it to the list of nodes of planner
        node = Node(point, planner)
        self.nodes.append(node)
        return node

    def samplePoint(self):
        # Generates and returns a random point
        flag = True
        while flag:
            point = self.generateRandomPoint()
            if point is not None:
            # if(self.isFree(point)):
                self.points.append(point)
                # flag = False
                return point

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
        node.setParentNodeID(parentNode.id)

    def drawPoints(self):
        x_pts = []
        y_pts = []
        for i in range(len(self.points) - 1):
            x_pts.append(self.nodes[i].point.x)
            y_pts.append(self.points[i].point.y)
        return x_pts, y_pts

    def drawPath(self):
        # Currently draws all possible paths
        # Update: now draws final path
        for node in self.path:
            self.drawLine(node.point, self.nodes[node.parentNodeID])

    def setGoalPoint(self, goalPoint):
        self.goalPoint = goalPoint


    def nodeVicinity(self, newNode, dmin=0.20):
        # Returns true if there are no nodes within dmin m of parameter node
        for node in self.nodes:
            dist = self.getDistance(newNode.point, node.point)
            if dist < dmin:
                return False
        return True

    def initializeNodes(self, limit=1000):
        counter = 0
        while counter < limit and not self.isGoalReached:
            counter += 1
            point = self.generateRandomPoint()
            newNode = Node(point, self, False)
            nearestID = self.findNearestNode(newNode)
            if self.isPathValid(point, self.nodes[nearestID].point) and self.nodeVicinity(newNode):
                newNode = self.newNode(point, self)
                self.connectNodes(newNode, self.nodes[nearestID])
                if self.getDistance(newNode, self.goalPoint) < 0.2:
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

### End of import

def obstacleSubscriberCallback(obstacleList):
    for obstacle in obstacleList.list:
        pass



if __name__ == '__main__':

    rospy.init_node('path_planner')

    n = 40
    x_pts = []
    y_pts = []
    width = 5.5
    height = 5.5
    botRadius = 0.4
    obstacleRadius = 0.25
    validInput = False
    goal_x = 5.0
    goal_y = 5.0
    
    planner = Planner(Point(goal_x, goal_y, 0.0),width, height, botRadius, obstacleRadius)
    plt.xlim(-1, 6)
    plt.ylim(-1, 6)

    obstacles = []
    # Populating obstacles
    for i in range(0, 12, 3):
        for j in range(0, 12, 3):
            if not (i == 0 and j == 0):
                obstacles.append(Point(float(i)/2, float(j)/2, 0.0))


    plt.axes().set_facecolor("#121212")
    planner.generateObstacles(obstacles)
    planner.drawObstacles()
    
    ##--- Code to get goal point input ---###
    print("\n\n\nRRT Path Planning")
    while not validInput:
        print("Enter Goal x coordinate")
        goal_x = float(input())
        print("\nEnter Goal y coordinate")
        goal_y = float(input())
        goalPoint = Point(goal_x, goal_y, 0.0)
        if(planner.isFree(goalPoint)):
            print("\nGoal point is valid")
            planner.goalPoint = goalPoint
            validInput = True
        else:
            print("\n *ALERT*: Goal point lies within bonk distance of an obstacle")
            print("Please enter another goal point")

    ##--- END ---###

    planner.goalPoint = Point(goal_x, goal_y, 0.0)
    print("Goalpoint taken as (%f, %f)\n"%(goal_x, goal_y))

    print("\nComputing path...\n")

    # Pseudo code

    planner.initializeNodes()

    plt.title("RRT Path Planner")
    for node in planner.nodes:
        planner.drawPoint(node.point)
        
    for node in planner.nodes:
        planner.drawLine(node.point, planner.nodes[node.parentNodeID].point, width=0.5, color='blue')
        # planner.drawLine(Point(0,0), Point(3,3))
    # plt.draw()

    # for node in (planner.nodes):
    #     print("Node (%f %f) and parent (%f %f)"%(node.x, node.y, 
    #     planner.nodes[node.parentNodeID].x, planner.nodes[node.parentNodeID].y ) )

    parentIDList = planner.plotPath()

    for id in parentIDList:
        planner.drawLine(planner.nodes[id].point, planner.nodes[planner.nodes[id].getParentNodeID()], width=2)
    
    # print("Length of path in nodes: %d"%len(planner.path))
    # Change this to include goal point in path plotting
    # planner.drawLine(planner.targetNode.point, planner.nodes[planner.targetNode.getParentNodeID()].point, width=2)
    # print("Target node is (%f %f)"%(planner.targetNode.point.x, planner.targetNode.point.y))
    
    plt.show()

    print("Goalpoint coordinates are: (%f, %f)"%(planner.targetNode.point.x, planner.targetNode.point.y))
    # for node in planner.nodes:
    #     print("Distance from goalpoint is: %f"%planner.getDistance(node.point, planner.targetNode.point))

    print("Length of path is: %d"%len(planner.path))
    print("Number of nodes is: %d"%len(planner.nodes))
    print('Main executed')


    msgList = List()
    pathList = msgList.list
    pathList.append(Point(0.0, 0.0, 0.0))
    pathList.append(Point(5.0, 5.0, 0.0))
    pathList.append(Point(-3.0, 8.0, 0.0))
    pathList.append(Point(0.0, 0.0, 0.0))

    pathPlannerPublisher = rospy.Publisher('path', List, queue_size=1)
    obstacleSubscriber = rospy.Subscriber('obstacles', List, obstacleSubscriberCallback)

    print("Starting publishing when subscriber connects...")
    time.sleep(3)

    
    while not rospy.is_shutdown():
        connections = pathPlannerPublisher.get_num_connections()
        if connections > 0:
            time.sleep(1)
            pathPlannerPublisher.publish(pathList)
            break
    
    print("Published successfully. Terminating node...")

        
