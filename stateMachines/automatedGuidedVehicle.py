'''
Created on Apr 25, 2020

@author: MrMcG
'''
import math
import threading
from enum import Enum
from ros_shim import rospy
from ros_shim import std_msgs
import logging

logging.basicConfig(level=logging.INFO)

class Command(Enum):
    START = "START"
    STOP = "STOP"

class State(Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STOP = "STOP"
    STRAIGHT = "STRAIGHT"

class AGV(object):
    '''
    classdocs
    '''
    t = 0
    command = Command.STOP
    state = Command.STOP
    currentX = 52.5
    currentY = 267.5
    currentA = 0
    homeBaseX = 52.5
    homeBaseY = 267.5
    destinationX = 0
    destinationY = 0
    startingAngle = math.pi/2
    currentPackage = {}
    displacement = 0
    angularVelocity = 0
    xVelocity = 0
    yVelocity= 0
    
    e1 = 7
    e2 = 10

    def __init__(self,inputQueue):
        '''
        Constructor
        '''
        self.inputQueue = inputQueue
        self.xVelocityPub = rospy.Publisher("/AGVxVelocity",std_msgs.Float64)
        self.yVelocityPub = rospy.Publisher("/AGVyVelocity",std_msgs.Float64)
        self.aVelocityPub = rospy.Publisher("/AGVangularVelocity",std_msgs.Float64)
        self.pickupPublisher = rospy.Publisher("/pickup",std_msgs.Dict)
        
        rospy.Subscriber("/AGVxPosition",std_msgs.Float64,self.updateXPosition)
        rospy.Subscriber("/AGVyPosition",std_msgs.Float64,self.updateYPosition)
        rospy.Subscriber("/AGVanglePosition",std_msgs.Float64,self.updateAPosition)
        rospy.Subscriber("/time", std_msgs.Float64, self.updateTime)
        rospy.Subscriber("/displacement", std_msgs.Float64, self.updateDisplacement)
        
        #create thread
        self.thread = threading.Thread(target=self.run, args=())
        self.thread.daemon = True
        #thread.start() #start the execution
            
    def getThread(self):
        return self.thread
    
    def updateXPosition(self,x):
        self.currentX = x
    
    def updateYPosition(self,y):
        self.currentY = y
    
    def updateAPosition(self,a):
        self.currentA = a
    
    def updateTime(self,time):
        self.t += 0.1
    
    def updateDisplacement(self,displacement):
        self.displacement = displacement
    
    def stop(self):
        info = self.inputQueue.get()
        self.pickupPublisher.publish(info['id'])
        self.destinationX = info['locationX']
        self.destinationY = info['locationY']
        if info:
            self.currentPackage = info
            self.command = Command.START
            
    def publishPosition(self,xVel=None,yVel=None,aVel=None):
        if xVel is not None:
            self.xVelocityPub.publish(xVel)
        if yVel is not None:
            self.yVelocityPub.publish(yVel)
        if aVel is not None:
            self.aVelocityPub.publish(aVel)
        
    def straightState(self):
        xVelocity = -10 * math.cos(self.currentA)
        yVelocity = -10 * math.sin(self.currentA) 
        self.publishPosition(xVelocity, yVelocity, None)
        
    def rightState(self):
        angularVelocity = -(math.pi/16)
        xVelocity =  10 / (1 + math.exp(-(self.t-5))) * math.cos(self.currentA) * self.t
        yVelocity =  10 / (1 + math.exp(-(self.t-5))) * math.sin(self.currentA) * self.t
        self.publishPosition(xVelocity,yVelocity,angularVelocity)
        
    def leftState(self):
        self.angularVelocity = math.pi / 16
        self.xVelocity = 30 / (1 + math.exp(-(self.t-5))) * math.cos(self.currentA) * self.t
        self.yVelocity = 30 / (1 + math.exp(-(self.t-5))) * math.sin(self.currentA) * self.t
        self.publishPosition(None,None,self.angularVelocity)
        
    def deliverPackage(self):
        self.publishPosition(0, 0, 0)
        logging.info("Delivering package")
        self.currentPackage = {}
        
    def updateState(self,state):
        self.state = state
    
    def start(self):
        print("AGV starting")
        while self.currentPackage:
            logging.info("dis: " + str(self.displacement))
            logging.info("curX: " + str(self.currentX))
            logging.info("curY: " + str(self.currentY))
            logging.info("curA: " + str(self.currentA))
            logging.info("==============")
            if self.state == State.STOP:
                self.publishPosition(0, 0, 0)
                
            elif self.state == State.STRAIGHT:
                self.straightState()
                if self.displacement > self.e2:
                    self.publishPosition(0, 0, 0)
                    self.updateState(State.RIGHT)
                elif self.displacement < -(self.e2):
                    self.publishPosition(0, 0, 0)
                    self.updateState(State.LEFT)
                    
            elif self.state == State.RIGHT:
                self.rightState()
                if self.displacement < self.e1:
                    self.publishPosition(0, 0, 0)
                    self.updateState(State.STRAIGHT)
                elif self.displacement < -(self.e2):
                    self.publishPosition(0, 0, 0)
                    self.updateState(State.LEFT)
                
            elif self.state == State.LEFT:
                self.leftState()
                if self.displacement < self.e1:
                    self.publishPosition(0, 0, 0)
                    self.updateState(State.STRAIGHT)
                elif self.displacement > self.e2:
                    self.publishPosition(0, 0, 0)
                    self.updateState(State.RIGHT)
            
            if (self.destinationX - 4 < self.currentX < self.destinationX + 4):
                self.deliverPackage()
            
            rospy.sleep(0.1)
        
    
    
    def run(self):
        print("Started AGV thread...")
        while True:
            if self.command == Command.STOP:
                self.stop()
        
            elif self.command == Command.START:
                self.state = State.STRAIGHT
                self.start()
                
                