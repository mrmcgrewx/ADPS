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
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start() #start the execution
        
    def run(self):
        print("Started Delivery Service thread...")
        while True:
            if not self.inputQueue.empty():
                res = self.inputQueue.get()
                print("DS got message from queue")
                print(res)
                self.outputQueue.put(res)
        