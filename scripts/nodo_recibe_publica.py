#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import subprocess, shlex, psutil
import rosbag

import time

import os   # para contar el numero de archivos en ./rosbags


class CurritoController():
    def __init__(self):
        self.ceja_izq_consigna = 0
        self.ceja_der_consigna = 0
        self.cresta_consigna = 0
        self.cuello_consigna = 0
        self.cuerpo_consigna = 0
        self.boca_consigna = 0
        self.rueda_izq_consigna = 0
        self.rueda_der_consigna = 0

        self.msg_mando = Joy()
        self.msg_arduino = Joy()

        self.recording = False

        self.msg_mando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.msg_mando.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.pressed = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.mode = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.bag_dir = "rosbags/"
        self.bag_cont = len(os.listdir(self.bag_dir))
        print(str(self.bag_cont))

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
        self.boca_consigna = (self.msg_mando.axes[5] + 1)/2 *180

        # self.rueda_izq_consigna = self.msg_mando.axes[6]*0                      # Por definir
        # self.rueda_der_consigna = self.msg_mando.axes[7]*0                      # Por definir

        # Posible calculo:
        #   self.error_izq = self.msg_mando.axes[6]
        #   self.control_ruedas()   # Controlador proporcional que calcula el valor de self.rueda_izq_consigna a partir de self.error_izq



    def control_automatico(self):
        # TODO: Movimiento automatico
        # self.ceja_izq_consigna = 0
        # self.ceja_der_consigna = 0
        # self.cresta_consigna = 0
        # self.cuello_consigna = 0
        # self.cuerpo_consigna = 0
        # self.boca_consigna = 0

        # self.error_izq = 0
        # self.error_der = 0
        # self.control_ruedas()
        # self.rueda_izq_consigna = 0
        # self.rueda_der_consigna = 0
        pass

    def start_record(self):

        self.recording = True
        print("Trying to start record")
        self.bag_cont = self.bag_cont + 1
        self.command = "rosbag record -O " + self.bag_dir + "subset" + str(self.bag_cont) + " /joy2"
        self.command = shlex.split(self.command)
        self.rosbag_proc = subprocess.Popen(self.command)
        print("RECORD RUNNING")

    def end_record(self):
        print("Trying to close the record")
        self.recording = False
        for proc in psutil.process_iter():
            if "record" in proc.name() and set(self.command[2:]).issubset(proc.cmdline()):
                proc.send_signal(subprocess.signal.SIGINT)

        self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
        print("RECORD CLOSED")


# Marcelo: Esta funcion hay que revisarla. No estoy seguro de si entendí bien como funcionaba, pero en caso de que si, parece que escribia todos los mensajes
# de golpe sin dejar pausa entre ellos
    def play_bag(self):
        print("PLAYING BAG")
        self.bag = rosbag.Bag(self.bag_dir + "subset" + str(self.bag_cont) + ".bag")
        for topic, msg, t in self.bag.read_messages(topics=['/joy2']):
            self.pub.publish(msg)
            time.sleep(1/10) # pusa para que los mensajes se envien a 10 Hz
        self.bag.close()
        # Desactivamos mode[2] para no reproducir la bag en bucle
        self.mode[2] = 0
        print("FIN PLAYING BAG")


    def publica(self):

        # En cualquiera de los modos, el boton 2 permite reproducir la grabacion
        if self.mode[2] == 1:
            self.play_bag()
        # Mientras se controla con el mando:
        elif self.mode[0] == 0:   # El boton 0 es el que decide el cambio entre modo Manodo y Automático
            self.control_mando()

        elif self.mode[0] == 1:
            self.control_automatico()

        # self.msg_arduino.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.msg_arduino.axes = [self.ceja_izq_consigna, self.ceja_der_consigna, self.cresta_consigna, self.cuello_consigna, self.cuerpo_consigna, self.boca_consigna, self.rueda_izq_consigna, self.rueda_der_consigna]
        self.pub.publish(self.msg_arduino)


    def callback(self, msg):


        for i in range(len(msg.buttons)):
            if(msg.buttons[i] == 0):
                self.pressed[i] = 0

            elif(msg.buttons[i] == 1 and self.pressed[i] == 0):
                self.pressed[i] = 1

                if self.mode[i] == 1:
                    self.mode[i] = 0
                    if i == 0:
                        print("Modo MANDO")
                else:
                    self.mode[i] = 1
                    if i == 0:
                        print("Modo AUTOMATICO")

        if self.mode[1] == 1 and self.recording == False:
            self.start_record()

        if self.recording and self.mode[1] == 0:
            self.end_record()

        self.msg_mando = msg

        self.publica()



if __name__ == "__main__":


    currito = CurritoController()

    print("Modo MANDO (modo de inicio del sistema)")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
