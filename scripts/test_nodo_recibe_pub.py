#!/usr/bin/env python3
import rospy
from nodo_recibe_publica import CurritoController

if __name__ == "__main__":
    currito = CurritoController()
    currito.mode[0] = 1
    rate = rospy.Rate(0.5)
    currito.ceja_izq_consigna = 90  # Pedir menor mvto a la ceja izq para evitar fallo
    currito.ceja_der_consigna = 90
    currito.cresta_consigna = 90
    currito.cuello_consigna = 90
    currito.cuerpo_consigna = 90
    currito.boca_consigna = 90

    currito.rueda_izq_consigna = 0
    currito.rueda_der_consigna = 0

    while not rospy.is_shutdown():
        currito.publica()
        if currito.ceja_izq_consigna == 135:
            currito.ceja_izq_consigna = 90
            currito.ceja_der_consigna = 90
            currito.cresta_consigna = 90
            currito.cuello_consigna = 90
            currito.cuerpo_consigna = 90
            currito.boca_consigna = 90
        elif currito.ceja_izq_consigna == 90:
            currito.ceja_izq_consigna = 45
            currito.ceja_der_consigna = 180
            currito.cresta_consigna = 180
            currito.cuello_consigna = 180
            currito.cuerpo_consigna = 90 # 180-25
            currito.boca_consigna = 180
        else:
            currito.ceja_izq_consigna = 135
            currito.ceja_der_consigna = 0
            currito.cresta_consigna = 0
            currito.cuello_consigna = 0
            currito.cuerpo_consigna = 90 #25
            currito.boca_consigna = 0
        rate.sleep()
