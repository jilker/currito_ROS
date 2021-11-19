#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import subprocess, shlex, psutil
import rosbag


class CurritoController():
    def __init__(self):
        self.ceja_izq_consigna = 0
        self.ceja_der_consigna = 0
        self.cresta_consigna = 0
        self.cuello_consigna = 0
        self.cuerpo_consigna = 0

        self.msg_mando = Joy()
        self.msg_arduino = Joy()

        self.recording = False

        self.msg_mando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.msg_mando.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.pressed = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.mode = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        rospy.init_node('procesa_mando')
        self.pub = rospy.Publisher("/joy2", Joy, queue_size=10)
        # rospy.Timer(rospy.Duration(Ts), publica)

        rospy.Subscriber("/joy", Joy, self.callback)

    def control_mando(self):

        self.ceja_izq_consigna = (self.msg_mando.axes[0] + 1)/2 *180
        self.ceja_der_consigna = (self.msg_mando.axes[1] + 1)/2 *180
        self.cresta_consigna = (self.msg_mando.axes[2] + 1)/2 *180
        self.cuello_consigna = (self.msg_mando.axes[3] + 1)/2 *180
        self.cuerpo_consigna = (self.msg_mando.axes[4] + 1)/2 *180


    def control_automatico(self):
        # TODO: Movimiento automatico
        # self.ceja_izq_consigna = 0
        # self.ceja_der_consigna = 0
        # self.cresta_consigna = 0
        # self.cuello_consigna = 0
        # self.cuerpo_consigna = 0
        pass

    def start_record(self):

        self.recording = True
        print("Trying to start record")
        self.command = "rosbag record -O subset /joy2"
        self.command = shlex.split(self.command)
        self.rosbag_proc = subprocess.Popen(self.command)
        print("RECORD RUNNING")

    def play_bag(self):

        self.bag = rosbag.Bag('subset.bag')
        for topic, msg, t in self.bag.read_messages(topics=['/joy2']):
            self.pub.publish(msg)
        self.bag.close()

    def end_record(self):
        print("Trying to close the record")
        self.recording = False
        for proc in psutil.process_iter():
            if "record" in proc.name() and set(self.command[2:]).issubset(proc.cmdline()):
                proc.send_signal(subprocess.signal.SIGINT)

        self.rosbag_proc.send_signal(subprocess.signal.SIGINT)


    def publica(self):

        # Mientras se controla con el mando:
        if self.mode[0] == 0:
            self.control_mando()
        elif self.mode[0] == 1:
            self.control_automatico()
        elif self.mode[2] == 1:
            self.play_bag()



        # self.msg_arduino.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.msg_arduino.axes = [self.ceja_izq_consigna, self.ceja_der_consigna, self.cresta_consigna, self.cuello_consigna, self.cuerpo_consigna, 0]

        self.pub.publish(self.msg_arduino)

    def callback(self, msg):


        for i in range(len(msg.buttons)):
            if(msg.buttons[i] == 0):
                self.pressed[i] = 0

            elif(msg.buttons[i] == 1 and self.pressed[i] == 0):
                self.pressed[i] = 1
                
                if self.mode[i] == 1:
                    self.mode[i] = 0
                else:
                    self.mode[i] = 1

        if self.mode[1] == 1 and self.recording == False:
            self.start_record()

        if self.recording and self.mode[1] == 0:
            self.end_record()

        self.msg_mando = msg

        self.publica()



if __name__ == "__main__":


    currito = CurritoController()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    
