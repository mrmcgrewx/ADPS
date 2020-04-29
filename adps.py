'''
Created on Apr 25, 2020

@author: MrMcG
'''
import queue
from state_machines.automated_guided_vehicle import AGV
from state_machines.delivery_service import DS
from state_machines.scheduling_service import SS

class ADPS:
    '''This class implements the automated delivery package service
    
    The ADPS comprises of three state machines: Scheduling Service, Delivery Service,
    and Automated guided vehicle. The machines interact with each other through FIFO queues
    '''

    def __init__(self):
        self.ss_to_ds_queue = queue.Queue()
        self.ds_to_agv_queue = queue.Queue()
        
    def run(self):
        ss = SS(self.ss_to_ds_queue)
        ds = DS(self.ss_to_ds_queue,self.ds_to_agv_queue)
        agv = AGV(self.ds_to_agv_queue)
        
        ss_thread = ss.get_thread()
        ds_thread = ds.get_thread()
        agv_thread = agv.get_thread()    
        
        ss_thread.start()
        ds_thread.start()
        agv_thread.start()
        
        
        ss_thread.join()
        ds_thread.join()
        agv_thread.join()