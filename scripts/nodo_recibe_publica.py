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

        # Conteo de los Rosbags existentes
        self.bag_dir = "rosbags/"
        self.bag_cont = len(os.listdir(self.bag_dir))
        print("Número de rosbags existentes: " + str(self.bag_cont))

        # Control automatico de las ruedas:
        self.error_horizontal = 0
        self.error_vertical = 0
        self.area_pelota = 0

        rospy.init_node('procesa_mando')
        self.pub = rospy.Publisher("/joy2", Joy, queue_size=10)
        # rospy.Timer(rospy.Duration(Ts), publica)

        rospy.Subscriber("/joy", Joy, self.callback)

        rospy.Subscriber("/datos_pelota", Joy, self.callback_pelota)




    def control_mando(self):
        # Reasignados los ejes que se atribuyen a cada motor para que el control sea intuitivo
        # Es mejor hacer la reasignación siempre desde aqui para no torcar más lo de arduino
        # Los ejes empiezan en el 0

        # Cejas: mismo eje: el 5 (R2)
        self.ceja_izq_consigna = (self.msg_mando.axes[5] + 1)/2 *180
        self.ceja_der_consigna = (self.msg_mando.axes[5] + 1)/2 *180

        # Cresta: eje 2 (L2)
        self.cresta_consigna = (self.msg_mando.axes[2] + 1)/2 *180

        # Cuello: eje 4 (R arriba abajo)
        self.cuello_consigna = (self.msg_mando.axes[4] + 1)/2 *180
        # Rotacion cuerpo: eje 3 (R derecha izqueirda)
        self.cuerpo_consigna = (self.msg_mando.axes[4] + 1)/2 *180

        # Boca: mismo eje que las cejas, por ponerle alguno
        self.boca_consigna = (self.msg_mando.axes[5] + 1)/2 *180


        # Ruedas: en el modo manual responden de forma proporcional a la puslación de los joysticks:
        # El error puede ir entre [-1, 1], y las ruedas pueden girar a velocidad [-255, 255]
        # De los 255 valores, se reparten 128 para Avance y 127 para rotación
        # NOTA: hay que pensar algo para el umbral inferior: las ruedas no se mueven para actuaciones menores a 60

        # Avance: eje 1 (L arruva abajo)
        avance = round(128 * self.msg_mando.axes[1])

        # Rotacion: eje 0 (L derecha izquierda)
        rotacion = round(127 * self.msg_mando.axes[0])

        # Las ruedas tienen mismo Avance, opuesta Rotación. Y además sus motores están puestos cada uno en un sentido
        self.rueda_izq_consigna =   avance + rotacion
        self.rueda_der_consigna = -(avance - rotacion)



    def control_automatico(self):
        # En el control automático de seguimiento de la pelota, se controlan Cuello, Cuerpo y Ruedas
        self.ceja_izq_consigna = 90
        self.ceja_der_consigna = 90
        self.cresta_consigna = 90
        self.boca_consigna = 90

        # Cuello y cuerpo: seguimiento pelota: movimiento a velocidad constante controlada hasta quedar mirándola
        if(self.error_vertical>0 and self.cuello_consigna<180):
            self.cuello_consigna = self.cuello_consigna + 1
        elif (self.error_vertical<0 and self.cuello_consigna>0):
            self.cuello_consigna = self.cuello_consigna - 1


        if(self.error_horizontal>0 and self.cuerpo_consigna<180):
            self.cuerpo_consigna = self.cuerpo_consigna + 1
        elif (self.error_horizontal<0 and self.cuerpo_consigna>0):
            self.cuerpo_consigna = self.cuerpo_consigna - 1

        # Ruedas: hasta que veamos como se comporta el currito, esta puesto para que los movimientos sean a velocidad constante y baja
        # Rotación: usando la posición del cuello: si las ruedas intentan girar hasta que el cuello esté recto, se logra el seguimiento de la pelota
        rotacion =  self.cuerpo_consigna/180*2 - 1  # conversión de [0, 180] a [-1,1]
        rotacion = round(127 * rotacion)    # conversión de [-1,1] a [-127, 127]

        # Avance: si la pelota está lejos, el currito avanza a velocidad constante pero baja
        avance = 0
        area_deseada = 50   # El area suele variar entre 10 y 100 (con pantalla de 720 de alto)
        if(self.area_pelota<area_deseada):
            avance = 80 # el umbral inferior es 60
        elif(self.area_pelota>area_deseada):
            avance = -80

        # Las ruedas tienen mismo Avance, opuesta Rotación. Y además sus motores están puestos cada uno en un sentido
        self.rueda_izq_consigna =   avance + rotacion
        self.rueda_der_consigna = -(avance - rotacion)





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



    def callback_pelota(self, msg):
        self.error_horizontal = msg.axes[0]
        self.error_vertical = msg.axes[1]
        self.area_pelota = msg.axes[2]






if __name__ == "__main__":


    currito = CurritoController()

    print("Modo MANDO (modo de inicio del sistema)")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
