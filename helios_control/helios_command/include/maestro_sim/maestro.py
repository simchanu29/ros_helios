#! /usr/bin/env python
# coding=utf-8
import serial

"""
Driver custom by Simon CHANU
Nécessite d'être en mode dual port avec la Maestro

Normalement on en a pas besoin mais si pour une raison inconnue le driver habituel ne fonctionne pas, celui-ci pourrait fonctionner
"""

class Controller:
    """
    Pour que ça fonctionne il faut placer la carte en mode USB dual port
    """
    def __init__(self, ttyStr="/dev/ttyACM0"):
        self.conn = serial.Serial(ttyStr)

    def setTarget(self, channel, target):
        command = [0x84, channel, target & 0x7F, target >> 7 & 0x7F]
        self.conn.write(command)

    def setSpeed(self, channel, target):
        command = [0x84, channel, target & 0x7F, target >> 7 & 0x7F]
        self.conn.write(command)

    def setAccel(self, channel, target):
        command = [0x84, channel, target & 0x7F, target >> 7 & 0x7F]
        self.conn.write(command)
