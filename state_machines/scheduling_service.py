'''
Created on Apr 25, 2020

@author: MrMcG
'''
import threading
from enum import Enum
from ros_shim import rospy, std_msgs

class State(Enum):
    IDLE = 'IDLE'
    SERVING = 'SERVING'

class SS:
    '''This class implements the Scheduling Service
    
    This class has two states 'IDLE' and 'SERVING'
    When idling this class will read from the input queue 
    '''
    
    state = ""
    requests = []
    current_request = {}

    def __init__(self, output_queue):
        self.state = State.IDLE
        self.output_queue = output_queue
        rospy.subscriber("/request", std_msgs.list, self.update_requests)
        
        self.thread = threading.Thread(target=self.run, args=())
        self.thread.daemon = True
            
    def get_thread(self):
        return self.thread
            
    def update_requests(self, requests):
        self.requests = requests
    
    def get_request(self):
        if self.requests:
            self.current_request = self.requests[0]
            self.output_queue.put(self.current_request)
            self.state = State.SERVING
    
    def idle(self):
        self.get_request()
        
    
    def serving(self):
        '''
        TODO Implement queue for receiving delivered confirmation
        '''
        pass
    
    def run(self):
        print("Started Scheduling Service thread...")
        while True:
            if self.state == State.IDLE:
                self.idle()
                
            elif self.state == State.SERVING:
                self.serving()