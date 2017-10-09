#!/usr/bin/env python
# coding=utf-8

import maestro
import sys

channel = int(sys.argv[1])
target = int(sys.argv[2])

servo = maestro.Controller()
servo.setAccel(channel,4)      #set servo 0 acceleration to 4
servo.setTarget(channel,target)  
servo.close

print "===== CHANNEL :",channel,"___ PWM :",target,"====="



