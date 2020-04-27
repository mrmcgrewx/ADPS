'''
Created on Apr 25, 2020

@author: MrMcG
'''
import queue 
import time
import threading

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
        
        #create thread
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start() #start the execution
        
            
    def updateState(self,requests):
        self.requests = requests
    
    def getRequest(self):
        if self.requests:
            self.currentRequest = self.requests[0]
            self.outputQueue.put(self.currentRequest)
            self.state = "SERVING"
    
    def idle(self):
        print("SS State: IDLE")
        self.getRequest()
        time.sleep(2)
    
    def serving(self):
        print("SS State: SERVING")
        time.sleep(4)
    
    def run(self):
        print("Started Scheduling Service thread...")
        while True:
            if self.state == "IDLE":
                self.idle()
                
            elif self.state == "SERVING":
                self.serving()
                
    