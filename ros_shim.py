########################################################################
# ROS system filler implementation for ECE813 
# To use, import rospy with: import rospy-shim as rospy
# 
# DO NOT MODIFY THE FILE, OR THE RESULTS FOR YOUR CODE WILL DIFFER WHEN
# SUBMITTED AND AUTO-GRADED BY THE SYSTEM
#######################################################################

import subprocess
import time
import sys
import requests
import json
import threading

# Wrapper for ROS primitive types
# mapping to their msg specification
class Std_msgs():
    def __init__(self):
        self.name = ""

    def String(self):
        return "String"
    
    def Bool(self):
        return "Bool"

    def Int64(self):
        return "Int64"
    
    def Float64(self):
        return "Float64"
    
    def List(self):
        return "List"
    
    def Dict(self):
        return "Dictionary"

std_msgs = Std_msgs()

# ROS publisher 
class PublisherWrap():
    def __init__(self, topic, msg_type,parentNode):
        self.topic = topic
        self.msg_type = msg_type()
        self.parent = parentNode

    #send the data over to the topic
    def publish(self, message):
        #send the message to the publishers for this topic
        r = requests.post(self.parent.hostname + self.topic,
            params={
                "{}".format(self.msg_type): message
            }
        )
        #print(r.content)
        return r.json()[self.msg_type]

# ROS subscriber to a topic
class SubscriberWrap():
    #initialization 
    #will start a thread for the callback fn
    def __init__(self, topic, msg_type, callback, parentNode):
        self.topic = topic
        self.rate = 0.5
        self.msg_type = msg_type()
        self.parent = parentNode
        self.callback = callback
        self.isRegistered = True

        #create thread
        thread = threading.Thread(target=self.perform, args=())
        thread.daemon = True
        thread.start() #start the execution

    #perform the action while registered
    def perform(self):
        while self.isRegistered:
           # print(self.topic)
            r = requests.get(self.parent.hostname + self.topic)
            self.callback(r.json()[self.msg_type])
            time.sleep(self.rate)

    #unregister the subscription
    def unregister(self):
        self.isRegistered = False
    
# ROSPY library wrapper
class Rospy():

    #initialize the ROS instance to connect to the simulator
    #requires that the simulation program is already running
    def __init__(self):
        self.subscriptions = {}
        self.topics = {}
        self.node_name = ""
        self.hostname = "http://localhost:3000"

        try:
            #get the topics from the other node
            r = requests.get(self.hostname)
            self.topics = r.json()
        except:
            print("No simulation program found running!")
            sys.exit(1)

    # create a new publisher object and register with the node
    # @param topic
    # @param msgType
    def Publisher(self, topic, msgType):
        if topic in self.topics:
            print("WARNING: Topic %s is already defined"%topic)

        pub = PublisherWrap(topic, msgType, self)
        self.topics[topic] = msgType()
        return pub
        
    #create a new subscription object and register with the node
    # @param topic
    # @param msgType
    # @param callback
    def Subscriber(self, topic, msgType, callback):
        self.subscriptions[topic] = SubscriberWrap(topic, msgType, callback, self)
        return self.subscriptions[topic]

    # ROS sleep function for provided duration
    # @param time Integer seconds to sleep
    def sleep(self, duration):
        time.sleep(duration)

    # ROS function to view an array of 
    # arrays for all topics and msg types
    # @return [[str, str]]
    def get_published_topics(self):
        res = []
        for tp, ob in self.topics.items():
            res.append([tp, ob])
        return res
        
    # ROS function for initialization of the node
    def init_node(self, node_name):
        print("Node Started for %s"% node_name)
        self.node_name = node_name


rospy = Rospy()