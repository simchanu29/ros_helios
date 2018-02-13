#!/usr/bin/env python
# coding=utf-8

# Author : Simon
# Notes  : Utilise un message specifique
# Input  : Reçois les commandes des autres noeuds pour les moteurs sous la forme d'une pin et d'une commande en pwm
import rospy
from std_msgs.msg import Float32
from maestro.maestro import Controller
# from maestro_sim.maestro import Controller # En cas de problème de driver, celui-ci est plus compréhensible et se debug bien
from helios_command.msg import PwmCmd


# 1000 : marche avant
# 1500 : statique
# 2000 : marche arrière
# Channel : 0:L 1:R

# def callbackT1(msg):
#
#     thrust1 = msg.data # Left -1;1
#     rospy.loginfo("Suscriber T1 received : %d", int(thrust1*700+6000))
#     maestro.setTarget(0,int(-thrust1*2000+6000))
#
# def callbackT2(msg):
#
#     thrust2 = msg.data # Right -1;1
#     rospy.loginfo("Suscriber T2 received : %d", int(thrust2*700+6000))
#     maestro.setTarget(1,int(-thrust2*2000+6000))

class PWMBoard(Controller):
    def __init__(self, port, pwm_devices, command_types):
        Controller.__init__(self, ttyStr=port)

        # Formattage des données (quelle pin de la carte associée à quoi)
        self.devices_by_pins = self.gen_dic_by_pin_keys(pwm_devices)
        self.devices_by_name = pwm_devices
        self.types = command_types
        print 'devices_by_pins : ', self.devices_by_pins

        for device in self.devices_by_name:
            pin = self.devices_by_name[device]['pin']
            command_type = self.devices_by_name[device]['command_type']
            self.setAccel(pin, self.types[command_type]['accel'])

    def gen_dic_by_pin_keys(self, pwm_devices):
        """
        Transforme la table de hachage où on accède aux numéros des pins par le nom de l'appareil en une table de
        hachage où on accède au nom de l'appareil par son numéro de pin associé
        :param pwm_devices:
        :return pin_dic:
        """
        pin_dic = dict()
        for device in pwm_devices:
            print 'device :', device
            pin = int(pwm_devices[device]['pin'])
            pin_dic[pin] = device
        return pin_dic

    def cb_pwm(self, msg):

        print 'pin :', msg.pin  # pin en int
        print 'cmd :', msg.command  # commande en float

        # Gestion du type de commande
        device_name = self.devices_by_pins[msg.pin]
        print 'device_name', device_name
        type = self.devices_by_name[device_name]['command_type']
        print 'type', type
        range = self.types[type]['range']
        range_min = range[0]
        range_max = range[1]
        range_tot = range_max - range_min
        range_zero = range_min + range_tot / 2.0
        print 'range', range

        # Calcul de la commande en pwm
        cmd = (msg.command - range_zero) * 1000 / range_tot + 1500
        print 'pwm sent to board :', int(cmd)

        # Envoi de la commande (traduction en polulu 0-2000 = 0-8192)
        cmd = int(cmd*4.096)
        print 'cmd sent to board :', int(cmd)
        self.setTarget(int(msg.pin), int(cmd))

    def hotfix_publisher(self, curr1, curr2, volt1, volt2):
#        rospy.loginfo("getting positions")
        volt1_val = self.getPosition(8)
        curr1_val = self.getPosition(9)
        volt2_val = self.getPosition(10)
        curr2_val = self.getPosition(11)

        volt1_val = volt1_val/255.75*5.0*15.7*0.91
        volt2_val = volt2_val/255.75*5.0*15.7*0.91

#        rospy.loginfo("Sensors values")
        volt1.publish(volt1_val)
#        rospy.loginfo(volt1_val)
        volt2.publish(volt2_val)
#        rospy.loginfo(volt2_val)
        curr1.publish(curr1_val)
#        rospy.loginfo(curr1_val)
        curr2.publish(curr2_val)
#        rospy.loginfo(curr2_val)

if __name__ == '__main__':

    rospy.init_node('driver_maestro')

    rospy.loginfo("driver_maestro Node Initialised")
    port = rospy.get_param('~port', "/dev/ttyACM0")
    pwm_devices = rospy.get_param('~pwm_device')
    command_types = rospy.get_param('~command_type')

    maestro = PWMBoard(port, pwm_devices, command_types)
    rospy.Subscriber('pwm_cmd', PwmCmd, maestro.cb_pwm)

    # Hotfix TODO faire un truc plus propre
    curr1 = rospy.Publisher('curr_1', Float32, queue_size=1)
    volt1 = rospy.Publisher('volt_1', Float32, queue_size=1)
    curr2 = rospy.Publisher('curr_2', Float32, queue_size=1)
    volt2 = rospy.Publisher('volt_2', Float32, queue_size=1)

    while not rospy.is_shutdown():
        try:
            rospy.rostime.wallsleep(0.5)
            maestro.hotfix_publisher(curr1, curr2, volt1, volt2)
        except rospy.ROSInterruptException:
            maestro.close()
