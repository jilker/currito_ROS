#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy

ceja_izq_consigna = 0
ceja_der_consigna = 0
cresta_consigna = 0
cuello_consigna = 0
cuerpo_consigna = 0

msg_mando = Joy()
msg_mando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
msg_mando.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
Ts = 1/10

modo = 0

msg_arduino = Joy()

def control_mando():
    global ceja_izq_consigna, ceja_der_consigna, cresta_consigna, cuello_consigna, cuerpo_consigna

    ceja_izq_consigna = (msg_mando.axes[0] + 1)/2 *180
    ceja_der_consigna = (msg_mando.axes[1] + 1)/2 *180
    cresta_consigna = (msg_mando.axes[2] + 1)/2 *180
    cuello_consigna = (msg_mando.axes[3] + 1)/2 *180
    cuerpo_consigna = (msg_mando.axes[4] + 1)/2 *180


def control_automatico():
    global ceja_izq_consigna, ceja_der_consigna, cresta_consigna, cuello_consigna, cuerpo_consigna

    ceja_izq_consigna = 0
    ceja_der_consigna = 0
    cresta_consigna = 0
    cuello_consigna = 0
    cuerpo_consigna = 0


def publica(event):
    global modo

    # Mientras se controla con el mando:
    if modo==0:
        control_mando()
    elif modo==1:
        control_automatico()



    msg_arduino.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    msg_arduino.axes = [ceja_izq_consigna, ceja_der_consigna, cresta_consigna, cuello_consigna, cuerpo_consigna, 0]

    pub.publish(msg_arduino)

def callback(recibido):
    global msg_mando
    global modo

    if(recibido.buttons[0] == 1):
        modo = 0
    elif(recibido.buttons[1] == 1):
        modo = 1

    msg_mando = recibido

if __name__ == "__main__":
    nh = rospy.init_node('procesa_mando')
    pub = rospy.Publisher("/joy2", Joy, queue_size=10)
    rospy.Timer(rospy.Duration(Ts), publica)

    rospy.Subscriber("/joy", Joy, callback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    
