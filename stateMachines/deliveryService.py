'''
Created on Apr 25, 2020

@author: MrMcG
'''
import threading

class DS(object):
    '''
    classdocs
    '''
    inputQueue = None
    outputQueue = None

    def __init__(self, inputQueue, outputQueue):
        '''
        Constructor
        '''
        self.inputQueue = inputQueue
        self.outputQueue = outputQueue
        
        #create thread
        self.thread = threading.Thread(target=self.run, args=())
        self.thread.daemon = True
        #thread.start() #start the execution
            
    def getThread(self):
        return self.thread
        
    def run(self):
        print("Started Delivery Service thread...")
        while True:
            if not self.inputQueue.empty():
                res = self.inputQueue.get()
                print("DS got message from queue")
                print(res)
                self.outputQueue.put(res)
        