#!/usr/bin/env python3

# Copyright 2017 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Modified & extended for K8 robot support by Carl Pickin 28 Aug 2018

# K8 Arduino Cerebral Cortex Commands
# ===================================
#
#   See enmu below for full list

import PyCmdMessenger
import time
import serial
import logging
import platform
import subprocess
import sys
import aiy.assistant.auth_helpers
from aiy.assistant.library import Assistant
import aiy.audio
import aiy.voicehat
from google.assistant.library.event import EventType
import threading

# Initialize an ArduinoBoard instance.  This is where you specify baud rate 
#arduino = PyCmdMessenger.ArduinoBoard("/dev/ttyACM0",baud_rate=9600,timeout=300.0)
arduino = PyCmdMessenger.ArduinoBoard("/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_557363233393519142B0-if00",baud_rate=9600,timeout=300.0)

# List of K8 Arduino command names (and formats for their associated arguments). These must be in the same order as in the sketch.
commands = [["spin",""],
            ["anticlockwise","i"],
            ["anticlockwise_until",""],            
            ["clockwise","i"],
            ["clockwise_until",""],
            ["backup","i"],            
            ["forward","i"],
            ["forward_until",""],
            ["ping_all",""],
            ["pan_tilt_test",""],
            ["lidar_read",""],
            ["follow_right_wall","d"],
            ["follow_left_wall","d"],            
            ["pings","iii"],
            ["lidar_val","ii"],            
            ["error","s"]]

c = PyCmdMessenger.CmdMessenger(arduino,commands)

usb_power_on = True
usb_used = True
location = "Kitchen"

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s:%(name)s:%(message)s"
)


def power_off_pi():
    aiy.audio.say('Good bye!')
    subprocess.call('sudo shutdown now', shell=True)
    

def reboot_pi():
    aiy.audio.say('See you in a bit!')
    subprocess.call('sudo reboot', shell=True)


def say_ip():
    ip_address = subprocess.check_output("hostname -I | cut -d' ' -f1", shell=True)
    aiy.audio.say('My IP address is %s' % ip_address.decode('utf-8'))


def go_walkabout():
    for i in range(0, 4):
        check_usb_power ()
        c.send("forward_until")
        aiy.audio.say('Zoom zoom')
        msg = c.receive()[1]
        leftDist = msg[0]
        centerDist = msg[1]
        rightDist = msg[2]
        print (leftDist," ",centerDist," ", rightDist )    
        
        while (leftDist<40 or rightDist<40 or centerDist<40):
            if (centerDist <40):                            
                check_usb_power ()                        
                c.send("anticlockwise_until")
                msg = c.receive()[1]
                leftDist = msg[0]
                centerDist = msg[1]
                rightDist = msg[2]
                print (leftDist," ",centerDist," ", rightDist )  
                
            elif (leftDist < 40):
                check_usb_power ()
                c.send("clockwise_until")
                msg = c.receive()[1]
                leftDist = msg[0]
                centerDist = msg[1]
                rightDist = msg[2]
                print (leftDist," ",centerDist," ", rightDist )    
                            
            elif (rightDist < 40):
                check_usb_power ()
                c.send("anticlockwise_until")
                msg = c.receive()[1]
                leftDist = msg[0]
                centerDist = msg[1]
                rightDist = msg[2]
                print (leftDist," ",centerDist," ", rightDist )    
                
    aiy.audio.say('I enjoyed that')
    spin()
    aiy.audio.say('Lets do it again')


def forward(distance):
# Send forward commamd through PyCmdMessenger
    check_usb_power ()
    c.send("forward",distance)
    msg = c.receive()[1]
    leftDist = msg[0]
    centerDist = msg[1]
    rightDist = msg[2]
    print (leftDist," ",centerDist," ", rightDist ) 
 
def backward(distance):
# Send backup commamd through PyCmdMessenger
    check_usb_power ()
    c.send("backup",distance)
    msg = c.receive()[1]
    leftDist = msg[0]
    centerDist = msg[1]
    rightDist = msg[2]
    print (leftDist," ",centerDist," ", rightDist ) 


def clockwise(turn_angle):
# Send clockwise commamd through PyCmdMessenger
    check_usb_power ()
    c.send("clockwise",turn_angle)
    msg = c.receive()[1]
    leftDist = msg[0]
    centerDist = msg[1]
    rightDist = msg[2]
    print (leftDist," ",centerDist," ", rightDist )       


def anticlockwise(turn_angle):
# Send anticlockwise commamd through PyCmdMessenger
    check_usb_power ()
    c.send("anticlockwise",turn_angle)
    msg = c.receive()[1]
    leftDist = msg[0]
    centerDist = msg[1]
    rightDist = msg[2]
    print (leftDist," ",centerDist," ", rightDist )    


def clockwise_until():
# Send clockwise_until commamd through PyCmdMessenger
    check_usb_power ()
    c.send("clockwise_until")
    msg = c.receive()[1]
    leftDist = msg[0]
    centerDist = msg[1]
    rightDist = msg[2]
    print (leftDist," ",centerDist," ", rightDist )

def anticlockwise_until():
# Send anticlockwise_until commamd through PyCmdMessenger
    check_usb_power ()
    c.send("anticlockwise_until")
    msg = c.receive()[1]
    leftDist = msg[0]
    centerDist = msg[1]
    rightDist = msg[2]
    print (leftDist," ",centerDist," ", rightDist )


def forward_until():
# Send forward_until commamd through PyCmdMessenger
    check_usb_power ()
    c.send("forward_until")
    msg = c.receive()[1]
    leftDist = msg[0]
    centerDist = msg[1]
    rightDist = msg[2]
    print (leftDist," ",centerDist," ", rightDist )    
    

def follow_right_wall(distance):
    c.send("follow_right_wall",distance)
    msg = c.receive()[1]


def follow_left_wall(distance):
    c.send("follow_left_wall",distance)
    msg = c.receive()[1]
    
    
def spin():
# Send spin commamd through PyCmdMessenger
    check_usb_power ()
    c.send("spin")
    msg = c.receive()[1]
    leftDist = msg[0]
    centerDist = msg[1]
    rightDist = msg[2]
    print (leftDist," ",centerDist," ", rightDist )    
    
    
def pan_tilt_test():
# Send pan_tilt_test commamd through PyCmdMessenger
    check_usb_power ()
    c.send("pan_tilt_test")
    print ("Pan Tilt Test Issued")   
     
     
def get_lidar_range():
# Send lidar_read commamd through PyCmdMessenger
    check_usb_power ()
    c.send("lidar_read")
    msg = c.receive()[1]
    range = msg[0]
    strength = msg[1]
    print ("Lidar range = ", range, "   Lidar strength = ", strength )    
    
    
def go_to_charger():
    global location 
    check_usb_power ()
    if location == "Kitchen":
        c.send("follow_right_wall",1190)
        print ("follow_right_wall Issued")
        msg = c.receive()[1]
        anticlockwise (130)
        backward (30)
    elif location == "Fire":
        c.send("follow_left_wall",1130)
        print ("follow_left_wall Issued")
        msg = c.receive()[1]
        clockwise (80)
        backward (30)
    location = "Charger"    
    aiy.audio.say('I am by the charger', lang='en-GB')


def go_to_kitchen():
    global location 
    check_usb_power ()
    if location == "Charger":
        c.send("follow_left_wall",1145)
        print ("follow_left_wall Issued")
        msg = c.receive()[1]
        clockwise (90)
        backward (30)
    elif location == "Fire":
        c.send("follow_left_wall",2250)
        print ("follow_left_wall Issued")
        msg = c.receive()[1]
        clockwise (90)
        backward (30)
    location = "Kitchen" 
    aiy.audio.say('I am in the kitchen', lang='en-GB')
    

def go_to_fire():
    global location 
    check_usb_power ()
    if location == "Charger":
        c.send("follow_right_wall",1135)
        print ("follow_right_wall Issued")
        msg = c.receive()[1]
        clockwise (180)
    elif location == "Kitchen":
        c.send("follow_right_wall",2285)
        print ("follow_right_wall Issued")
        msg = c.receive()[1]
        clockwise (180)
    location = "Fire"
    aiy.audio.say('I am by the fire', lang='en-GB')
   

def do_a_square():
    forward (50)
    anticlockwise(90)
    forward (50)
    anticlockwise(90)
    forward (50)
    anticlockwise(90)
    forward (50)
    anticlockwise(90)


def process_event(assistant, event):
    global usb_used
    global location
    print(event)
    status_ui = aiy.voicehat.get_status_ui()
    if event.type == EventType.ON_START_FINISHED:
        status_ui.status('ready')
        if sys.stdout.isatty():
            aiy.audio.say('Hello, I am, a robot. My name is K 8, but you can call me google', lang='en-GB')
            print('Say "OK, Google" then speak, or press Ctrl+C to quit...')

    elif event.type == EventType.ON_CONVERSATION_TURN_STARTED:
        status_ui.status('listening')

    elif event.type == EventType.ON_RECOGNIZING_SPEECH_FINISHED and event.args:
        print('You said:', event.args['text'])
        text = event.args['text'].lower()
        if text in ("power off","shutdown","shut down"):
            assistant.stop_conversation()
            power_off_pi()
        elif text == 'reboot':
            assistant.stop_conversation()
            reboot_pi()
        elif text == 'ip address':
            assistant.stop_conversation()
            say_ip()
            power_off_usb ()
        elif text in ("introduction","tell me about yourself","what is your name","introduce yourself"):
            assistant.stop_conversation()
            aiy.audio.say('I am, a, robot. My name is K 8, but you can call me google. I am powered by Anne arduino, and a rasberry pi. I also have an L 2 9 3, H bridge chip, a, t f mini lidar sensor, an MPU6o5o gyroscope, 2 DC motors with encoders and 3 ultrasonic range finders. My code is written in C++ and Python', lang='en-GB')
        elif text in ("go walkabout","talk about", "go and play", "going to play","don't play"):
            assistant.stop_conversation()
            go_walkabout()
        elif text in ("follow right 200","follow right 300","follow right 400","follow right 500","follow right 600","follow right 700", "follow right 800"):
            assistant.stop_conversation()
            word_list = text.split()  
            distance = int(word_list [-1])
            follow_right_wall (distance)
        elif text in ("follow left 200","follow left 300","follow left 400","follow left 500","follow left 600","follow left 700", "follow left 800"):
            assistant.stop_conversation()
            word_list = text.split()  
            distance = int(word_list [-1])
            follow_left_wall (distance)           
        elif text in ("forward 10", "forward 25", "forward 50", "forward 75", "forward 100", "forward 200", "forward 300", "forward 400"):
            assistant.stop_conversation()
            forward(int(text.partition(' ')[-1]))
        elif text in ("forward", "forwards"):
            assistant.stop_conversation()
            forward(50)
        elif text in ("reverse 10","reverse 25", "reverse 50", "reverse 75", "reverse 100", "reverse 200", "reverse 300", "reverse 400"):
            assistant.stop_conversation()
            backward(int(text.partition(' ')[-1]))
        elif text in ("backward", "backwards", "reverse"):
            assistant.stop_conversation()
            backward(50)            
        elif text in ("right 10", "right 45", "right 90", "right 180", "right 270", "right 360", "clockwise 10", "clockwise 45", "clockwise 90", "clockwise 180", "clockwise 270", "clockwise 360"):
            assistant.stop_conversation()
            clockwise (int(text.partition(' ')[-1]))
        elif text in ("right", "clockwise"):
            assistant.stop_conversation()
            clockwise(90)          
        elif text in ("left 10", "left 45", "left 90", "left 180", "left 270", "left 360", "anticlockwise 10", "anticlockwise 45", "anticlockwise 90", "anticlockwise 180", "anticlockwise 270", "anticlockwise 360"):
            assistant.stop_conversation()
            anticlockwise (int(text.partition(' ')[-1]))
        elif text in ("left", "anticlockwise"):
            assistant.stop_conversation()
            anticlockwise(90)   
        elif text in ("go forward", "go forwards"):
            assistant.stop_conversation()
            forward_until()
        elif text in ("go right","golite", "go blind", "get right", "get it right", "go runt", "dendrite","go light","goflight","gogoinflight","go to right"):
            assistant.stop_conversation()
            clockwise_until()
        elif text in ("go left", "go to left"):
            assistant.stop_conversation()
            anticlockwise_until()
        elif text in ("spin", "spain"):
            assistant.stop_conversation()
            spin()
        elif text in ("do a square", "do a squat"):
            assistant.stop_conversation()
            do_a_square()
        elif text in ("pan tilt test","servey test", "servo test", "soto test", "7 test", "soho test", "southern test","set a test", "tilt test", "time till test", "Suffolk test", "till test"):
            assistant.stop_conversation()
            pan_tilt_test()
        elif text in ("laser test", "blazer test", "laser range", "lidar range", "lydall range", "lidar test", "light off test"):
            assistant.stop_conversation()
            get_lidar_range()
        elif text in ("go to the lounge", "go to the charger", "get to the charger", "go to your charger"):
            assistant.stop_conversation()
            go_to_charger()            
        elif text in ("go to the kitchen", "cancel the kitchen"):
            assistant.stop_conversation()
            go_to_kitchen()
        elif text in ("go to the fire"):
            assistant.stop_conversation()
            go_to_fire()  
        elif text in ("save power"):
            assistant.stop_conversation()
            usb_used = False
            power_down_usb_now()  
        elif text in ("full power"):
            assistant.stop_conversation()
            check_usb_power()
        elif text in ("you are in the kitchen"):
            assistant.stop_conversation()
            location = "Kitchen"
            aiy.audio.say('I am in the kitchen', lang='en-GB')
        elif text in ("you are by the charger"):
            assistant.stop_conversation()
            location = "Charger"
            aiy.audio.say('I am by the charger', lang='en-GB')
        elif text in ("you are by the fire"):
            assistant.stop_conversation()
            location = "Fire"
            aiy.audio.say('I am by the fire', lang='en-GB')

    elif event.type == EventType.ON_END_OF_UTTERANCE:
        status_ui.status('thinking')

    elif (event.type == EventType.ON_CONVERSATION_TURN_FINISHED
          or event.type == EventType.ON_CONVERSATION_TURN_TIMEOUT
          or event.type == EventType.ON_NO_RESPONSE):
        status_ui.status('ready')

    elif event.type == EventType.ON_ASSISTANT_ERROR and event.args and event.args['is_fatal']:
        sys.exit(1)
        
               
        
def main():

    if platform.machine() == 'armv6l':
        print('Cannot run hotword demo on Pi Zero!')
        exit(-1)
        
    credentials = aiy.assistant.auth_helpers.get_assistant_credentials()
    with Assistant(credentials) as assistant:
        for event in assistant.start():
            process_event(assistant, event)
            print ("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")



def power_down_usb():  # timer based power down
    global usb_power_on
    global usb_used
    if usb_power_on == True and usb_used == False: #power is on, and not been used since last check
        p = subprocess.Popen(["./usb_power_off.sh"], stdout=subprocess.PIPE)
        print ("USB powered off")
        usb_power_on = False
    threading.Timer(420, power_down_usb).start() #check again in 7 min
    usb_used = False


def power_down_usb_now():  # voice based power down - does not start another timer
    global usb_power_on
    global usb_used
    if usb_power_on == True and usb_used == False: #power is on, and not been used since last check
        p = subprocess.Popen(["./usb_power_off.sh"], stdout=subprocess.PIPE)
        print ("USB & HDMI powered off")
        usb_power_on = False
    usb_used = False


def check_usb_power ():
    global usb_power_on
    global usb_used
    global arduino
    global c
    if usb_power_on == False:
        p = subprocess.Popen(["./usb_power_on.sh"], stdout=subprocess.PIPE)
        aiy.audio.say('Please wait', lang='en-GB')
        print("Powering USB & HDMI on")
        usb_power_on = True
        time.sleep(1)
        print ("USB Arduino and HDMI initialized")        
        arduino = PyCmdMessenger.ArduinoBoard("/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_557363233393519142B0-if00",baud_rate=9600,timeout=300.0)
        c = PyCmdMessenger.CmdMessenger(arduino,commands)
    usb_used = True



#Actual Execution Code Starts Here

if __name__ == '__main__':
    power_down_usb()
    main()
    
