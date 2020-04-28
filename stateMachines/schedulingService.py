'''
Created on Apr 25, 2020

@author: MrMcG
'''
import time
import threading
from ros_shim import rospy
from ros_shim import std_msgs

class SS(object):
    '''
    classdocs
    '''
    
    state = ""
    requests = []
    currentRequest = {}
    outputQueue = None


    def __init__(self, outputQueue):
        '''
        Constructor
        '''
        self.state = "IDLE"
        self.outputQueue = outputQueue
        rospy.Subscriber("/request", std_msgs.List, self.updateRequests)
        
        #create thread
        self.thread = threading.Thread(target=self.run, args=())
        self.thread.daemon = True
        #thread.start() #start the execution
            
    def getThread(self):
        return self.thread
            
    def updateRequests(self, requests):
        self.requests = requests
    
    def getRequest(self):
        if self.requests:
            self.currentRequest = self.requests[0]
            self.outputQueue.put(self.currentRequest)
            self.state = "SERVING"
    
    def idle(self):
        #print("SS State: IDLE")
        self.getRequest()
        
    
    def serving(self):
        pass
    
    def run(self):
        print("Started Scheduling Service thread...")
        while True:
            if self.state == "IDLE":
                self.idle()
                
            elif self.state == "SERVING":
                self.serving()
                
        time.sleep(5)