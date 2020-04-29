'''
Created on Apr 25, 2020

@author: MrMcG
'''
import logging
import math
import threading
from enum import Enum
from ros_shim import rospy
from ros_shim import std_msgs

logging.basicConfig(level=logging.INFO)

class Orientation(Enum):
    CLOCKWISE = "CLOCKWISE"
    COUNTERCLOCKWISE = "COUNTERCLOCKWISE"

class Command(Enum):
    START = "START"
    STOP = "STOP"

class State(Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STOP = "STOP"
    STRAIGHT = "STRAIGHT"

class AGV:
    '''This class implements the Automated Guided Vehicle
    
    The AGV state machine implements four states to drive around
    the track and deliver packages from the pickup station to the
    destination house
    '''
    
    t = 0
    command = Command.STOP
    state = Command.STOP
    orientation = Orientation.COUNTERCLOCKWISE
    current_x = 52.5
    current_y = 267.5
    current_a = 0
    home_base_x = 52.5
    home_base_y = 267.5
    destination_x = 0
    destination_y = 0
    starting_angle = math.pi/2
    current_package = {}
    displacement = 0
    angular_velocity = 0
    x_velocity = 0
    y_velocity= 0
    
    e1 = 7
    e2 = 10

    def __init__(self,input_queue):
        self.input_queue = input_queue
        self.x_velocity_pub = rospy.publisher("/AGVxVelocity",std_msgs.float_64)
        self.y_velocity_pub = rospy.publisher("/AGVyVelocity",std_msgs.float_64)
        self.a_velocity_pub = rospy.publisher("/AGVangularVelocity",std_msgs.float_64)
        self.pickup_publisher = rospy.publisher("/pickup",std_msgs.dict)
        
        rospy.subscriber("/AGVxPosition",std_msgs.float_64,self.update_xposition)
        rospy.subscriber("/AGVyPosition",std_msgs.float_64,self.update_yposition)
        rospy.subscriber("/AGVanglePosition",std_msgs.float_64,self.update_aposition)
        rospy.subscriber("/time", std_msgs.float_64, self.update_time)
        rospy.subscriber("/displacement", std_msgs.float_64, self.update_displacement)
        
        #create thread
        self.thread = threading.Thread(target=self.run, args=())
        self.thread.daemon = True
            
    def get_thread(self):
        return self.thread
    
    def update_xposition(self,x):
        self.current_x = x
    
    def update_yposition(self,y):
        self.current_y = y
    
    def update_aposition(self,a):
        if self.orientation == Orientation.CLOCKWISE:
            self.current_a = -(a)
        else:
            self.current_a = a
    
    def update_time(self,time):
        self.t += 0.1
    
    def update_displacement(self,displacement):
        self.displacement = displacement
    
    def determine_orientation(self,info):
        y_des = info['locationY']
        if y_des < self.current_y:
            self.orientation = Orientation.CLOCKWISE
    
    def stop(self):
        info = self.input_queue.get()
        self.determine_orientation(info)
        self.pickup_publisher.publish(info['id'])
        self.pickup_publisher.publish(info['id'])
        self.destination_x = info['locationX']
        self.destination_y = info['locationY']
        if info:
            self.current_package = info
            self.command = Command.START
            
    def publish_position(self,x_vel=None,y_vel=None,a_vel=None):
        if x_vel is not None:
            self.x_velocity_pub.publish(x_vel)
        if y_vel is not None:
            self.y_velocity_pub.publish(y_vel)
        if a_vel is not None:
            self.a_velocity_pub.publish(a_vel)
        
    def straight_state(self):
        x_velocity = 10 * math.cos(self.current_a)
        y_velocity = 10 * math.sin(self.current_a) 
        self.publish_position(x_velocity, y_velocity, None)
        
    def right_state(self):
        angular_velocity = -(math.pi/16)
        a = (30 / (1 + math.exp(-(self.t-5))))
        x_velocity =  a * math.cos(angular_velocity) * self.t
        y_velocity =  a * math.sin(angular_velocity) * self.t
        logging.info("x_vel: " + str(x_velocity))
        logging.info("y_vel: " + str(y_velocity))
        self.publish_position(x_velocity,y_velocity,angular_velocity)
        
    def left_state(self):
        angular_velocity = math.pi / 16
        a = 30 / (1 + math.exp(-(self.t-5)))
        x_velocity = a * math.cos(angular_velocity) * self.t
        y_velocity = a * math.sin(angular_velocity) * self.t
        logging.info("x_vel: " + str(x_velocity))
        logging.info("y_vel: " + str(y_velocity))
        self.publish_position(x_velocity,y_velocity,angular_velocity)
        
    def deliver_package(self):
        self.publish_position(0, 0, 0)
        logging.info("Delivering package")
        self.command = Command.STOP
        self.current_package = {}
        
    def update_state(self,state):
        self.t = 0
        self.publish_position(0, 0, 0)
        self.state = state
    
    def start(self):
        print("AGV starting")
        while self.current_package:
            logging.info("dis: " + str(self.displacement))
            if self.state == State.STOP:
                self.publish_position(0, 0, 0)
                
            elif self.state == State.STRAIGHT:
                self.straight_state()
                if self.displacement > self.e2:
                    self.update_state(State.RIGHT if self.orientation == Orientation.CLOCKWISE else State.LEFT)
                    logging.info("====================== RIGHT STATE =========================")
                elif self.displacement < -(self.e2):
                    self.update_state(State.LEFT if self.orientation == Orientation.CLOCKWISE else State.RIGHT)
                    logging.info("====================== LEFT STATE =========================")
                    
            elif self.state == State.RIGHT:
                self.right_state()
                if self.displacement < self.e1:
                    self.update_state(State.STRAIGHT if self.orientation == Orientation.CLOCKWISE else State.LEFT)
                    logging.info("====================== STRAIGHT STATE =========================")
                elif self.displacement < -(self.e2):
                    self.update_state(State.LEFT if self.orientation == Orientation.CLOCKWISE else State.STRAIGHT)
                    logging.info("====================== LEFT STATE =========================")
                
            elif self.state == State.LEFT:
                self.left_state()
                if self.displacement < self.e1:
                    self.update_state(State.STRAIGHT if self.orientation == Orientation.CLOCKWISE else State.RIGHT)
                    logging.info("====================== STRAIGHT STATE =========================")
                elif self.displacement > self.e2:
                    self.update_state(State.RIGHT if self.orientation == Orientation.CLOCKWISE else State.STRAIGHT)
                    logging.info("====================== RIGHT STATE =========================")
            
            if (self.destination_x - 4 < self.current_x < self.destination_x + 4):
                self.deliver_package()
            
    def run(self):
        print("Started AGV thread...")
        while True:
            if self.command == Command.STOP:
                self.stop()
        
            elif self.command == Command.START:
                self.state = State.STRAIGHT
                self.start()