'''
Created on Apr 25, 2020

@author: MrMcG
'''
import logging
from adps import ADPS
from ros_shim import rospy

logging.basicConfig(level=logging.INFO)
if __name__ == '__main__':
    # Initialize APDS
    nodeName = "Project 3 Node"
    rospy.init_node(nodeName)
    adps = ADPS()
    adps.run()