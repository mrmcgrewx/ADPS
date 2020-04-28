'''
Created on Apr 25, 2020

@author: MrMcG
'''
import queue
from ros_shim import rospy
from ros_shim import std_msgs
from stateMachines.automatedGuidedVehicle import AGV
from stateMachines.deliveryService import DS
from stateMachines.schedulingService import SS

class ADPS(object):
    '''
    classdocs
    '''
    ssTodsQueue = queue.Queue()
    dsToagvQueue = queue.Queue()

    def __init__(self):
        '''
        Constructor
        '''
        print("ADPS initializing...")
        #rospy.Subscriber("/time", std_msgs.Float64, self.getTime)       
        #self.deliveryPub = rospy.Publisher("delivered",std_msgs.Dict)
        
    def run(self):
        global t
        print("ADPS started....")
        ss = SS(self.ssTodsQueue)
        ds = DS(self.ssTodsQueue,self.dsToagvQueue)
        agv = AGV(self.dsToagvQueue)
        
        ssThread = ss.getThread()
        dsThread = ds.getThread()
        agvThread = agv.getThread()    
        
        ssThread.start()
        dsThread.start()
        agvThread.start()
        
        
        ssThread.join()
        dsThread.join()
        agvThread.join()

