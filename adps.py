'''
Created on Apr 25, 2020

@author: MrMcG
'''
import queue
import sys
from ros_shim import rospy
from ros_shim import std_msgs
from stateMachines.automatedGuidedVehicle import AGV
from stateMachines.schedulingService import SS
from stateMachines.deliveryService import DS

t = 0

class ADPS(object):
    '''
    classdocs
    '''
    algorithm = ""
    requestor = None
    requests = []
    displacement = 0
    t = 0
    displacementSubscriber = None
    timeSubscriber = None
    
    xVelocityPublisher = None
    yVelocityPublisher = None
    angularVelocityPublisher = None
    
    deliveryPublisher = None
   
    
    ssTodsQueue = queue.Queue()
    dsToagvQueue = queue.Queue()

    def __init__(self):
        '''
        Constructor
        '''
        print("ADPS initializing...")
        rospy.Subscriber("/displacement", std_msgs.Float64, self.getDisplacement)
        rospy.Subscriber("/request", std_msgs.List, self.getRequest)
        rospy.Subscriber("/time", std_msgs.Float64, self.getTime)
        rospy.Subscriber("/algorithm",std_msgs.String,self.getAlgorithm)
        
        
        self.AGVxPositionPub = rospy.Publisher("/AGVxPosition",std_msgs.Float64)
        self.AGVyPositionPub = rospy.Publisher("/AGVyPosition",std_msgs.Float64)
        self.AVGanglePositionPub = rospy.Publisher("/AVGanglePosition",std_msgs.Float64)
            
        self.xVelocityPub = rospy.Publisher("/AGVxVelocity",std_msgs.Float64)
        self.yVelocityPub = rospy.Publisher("/AGVyVelocity",std_msgs.Float64)
        self.angularVelocityPub = rospy.Publisher("/AGVangularVelocity",std_msgs.Float64)
        
        self.deliveryPub = rospy.Publisher("delivered",std_msgs.Dict)
        
        
        
    def getRequest(self, requests):
        self.requests = requests
        
    
    def getDisplacement(self, displacement):
        self.displacement = displacement
        #sys.stdout.flush()

    def getTime(self,x):
        global t
        t += 0.1
        
    def getAlgorithm(self, algorithm):
        self.algorithm = algorithm
        
    def run(self):
        global t
        print("ADPS started....")
        ss = SS(self.ssTodsQueue)
        ds = DS(self.ssTodsQueue,self.dsToagvQueue)
        agv = AGV(self.dsToagvQueue,
                  self.xVelocityPub,self.yVelocityPub,self.angularVelocityPub,
                  self.displacement,t)
        
        while True:
            ss.updateState(self.requests)
            agv.updateDisplacement(self.displacement)
            rospy.sleep(0.1)
            #self.xVelocityPub.publish(agv.getXPosition())
            #self.yVelocityPub.publish(agv.getYPosition())
            #self.angularVelocityPub.publish(agv.getAnglePosition())
            
        

