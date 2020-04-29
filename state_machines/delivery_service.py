'''
Created on Apr 25, 2020

@author: MrMcG
'''
import threading

class DS:
    '''This class implements the Delivery Service
    
    The delivery service manages incoming requests from the scheduling service and directs
    the AGV to deliver the assigned package
    '''
    
    def __init__(self, input_queue, output_queue):
        self.input_queue = input_queue
        self.output_queue = output_queue
        
        self.thread = threading.Thread(target=self.run, args=())
        self.thread.daemon = True
            
    def get_thread(self):
        return self.thread
        
    def run(self):
        print("Started Delivery Service thread...")
        while True:
            if not self.input_queue.empty():
                res = self.input_queue.get()
                print("DS got message from queue")
                print(res)
                self.output_queue.put(res)