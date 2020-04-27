'''
Created on Apr 25, 2020

@author: MrMcG
'''
import math
import threading
import time
import logging

logging.basicConfig(level=logging.INFO)
class AGV(object):
    '''
    classdocs
    '''
    t = 0
    command = "STOP"
    state = "STOP"
    xCoord = 52.5
    yCoord = 267.5
    homeBaseX = 52.5
    homeBaseY = 267.5
    anglePosition = math.pi/2
    inputQueue = None
    currentPackage = {}
    xPub = None
    yPub = None
    aPub = None
    displacement = 0
    angularVelocity = 0
    xVelocity = 0
    yVelocity= 0
    
    e1 = 7
    e2 = 10

    def __init__(self,inputQueue,xVelocityPub,yVelocityPubself,angularVelocityPub,displacement,t):
        '''
        Constructor
        '''
        self.t = t
        self.inputQueue = inputQueue
        self.xPub = xVelocityPub
        self.yPub = yVelocityPubself
        self.aPub = angularVelocityPub
        self.displacement = displacement
        
        #create thread
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start() #start the execution
            
    def getXPosition(self):
        return self.xCoord
    
    def getYPosition(self):
        return self.yCoord
    
    def getAnglePosition(self):
        return self.anglePosition
    
    def updateDisplacement(self,displacement):
        self.displacement = displacement
    
    def stop(self):
        info = self.inputQueue.get()
        if info:
            self.currentPackage = info
            self.command = "START"
    
    def start(self):
        print("AGV starting")
        while self.currentPackage:
            if self.state == "STOP":
                self.angularVelocity = 0
                self.xVelocity = 0
                self.yVelocity= 0
            elif self.state == "STRAIGHT":
                self.angularVelocity = 0
                self.xVelocity = 10 * math.cos(self.angularVelocity)
                self.yVelocity = 10 * math.sin(self.angularVelocity) 
                self.xPub.publish(self.xVelocity)
                self.yPub.publish(self.yVelocity)
            elif self.state == "RIGHT":
                self.angularVelocity = -(math.pi/16)
                self.xVelocity +=  10 / (1 + math.exp(-(self.t-5))) * math.cos(self.angularVelocity) * self.t
                self.yVelocity +=  10 / (1 + math.exp(-(self.t-5))) * math.sin(self.angularVelocity) * self.t
                logging.info(self.xVelocity)
                logging.info(self.yVelocity)
                self.aPub.publish(self.angularVelocity)
            elif self.state == "LEFT":
                self.angularVelocity = math.pi / 16
                self.xVelocity = 30 / (1 + math.exp(-(self.t-5))) * math.cos(self.angularVelocity) * self.t
                self.yVelocity = 30 / (1 + math.exp(-(self.t-5))) * math.sin(self.angularVelocity) * self.t
                self.aPub.publish(self.angularVelocity)
            
            
            #self.aPub.publish(self.angularVelocity)
            #print("dis: " + str(self.displacement))
            logging.info("dis: " + str(self.displacement))
            
            if ((self.e2 > self.displacement > self.e1) or self.displacement > self.e2):
                logging.info("Entering right state")
                self.xPub.publish(0)
                self.yPub.publish(0)
                self.t = 0
                self.state = "RIGHT"
                
            elif ((self.e1 > self.displacement) and self.state != "STRAIGHT"):
                logging.info("Entering straight state")
                self.state = "STRAIGHT"
            
            self.t += 0.1
        
    
    
    def run(self):
        print("Started AGV thread...")
        while True:
            if self.command == "STOP":
                self.stop()
        
            elif self.command == "START":
                self.state = "STRAIGHT"
                self.start()
                
                