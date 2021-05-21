#!/usr/bin/env python
# encoding: utf-8
import sys 
import rospy
import numpy as np
import os
# Importar mensajes de los distintos paquetes para guardar variables en ellos.
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose
from std_msgs.msg import Float64,Int32,Bool, String
from esquema_lider_seguidor.msg import Histograma
from esquema_lider_seguidor.msg import OrdenFormacion

from tf.transformations import euler_from_quaternion, quaternion_from_euler


class FormacionSeguidores():
    #Inicializan de la clase
    def __init__(self):
        self.seguidor_mas_lejano = 0
        self.Posicion_seg_1 = Pose()
        self.Posicion_seg_2 = Pose()
        self.Posicion_seg_3 = Pose()
        self.Vel_seg_1 = Twist()
        self.Vel_seg_2 = Twist()
        self.Vel_seg_3 = Twist()
        self.Velocidad = Twist()
        self.FS = rospy.Subscriber("/RealizarFormacion", Bool,self.FSCallback,queue_size=1)
        self.ps1 = rospy.Subscriber("/tb3_1/amcl_pose", PoseWithCovarianceStamped,self.poseCallback1,queue_size=1)
        self.ps2 = rospy.Subscriber("/tb3_2/amcl_pose", PoseWithCovarianceStamped,self.poseCallback2,queue_size=1)
        self.ps3 = rospy.Subscriber("/tb3_3/amcl_pose", PoseWithCovarianceStamped,self.poseCallback3,queue_size=1)
        self.S2AP = rospy.Subscriber('/tb3_1/AlcanzoPos', Bool,self.S1APCallback,queue_size=1)
        self.S2AP = rospy.Subscriber('/tb3_2/AlcanzoPos', Bool,self.S2APCallback,queue_size=1)
        self.S2AP = rospy.Subscriber('/tb3_3/AlcanzoPos', Bool,self.S3APCallback,queue_size=1)
        self.S1VP = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
        self.S2VP = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=1)
        self.S3VP = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=1)
        self.B1P = rospy.Publisher('/tb3_1/Bandera', Bool, queue_size=1)
        self.C1P = rospy.Publisher('/tb3_1/Coordenada', PoseWithCovarianceStamped, queue_size=1)
        self.B2P = rospy.Publisher('/tb3_2/Bandera', Bool, queue_size=1)
        self.C2P = rospy.Publisher('/tb3_2/Coordenada', PoseWithCovarianceStamped, queue_size=1)
        self.B3P = rospy.Publisher('/tb3_3/Bandera', Bool, queue_size=1)
        self.C3P = rospy.Publisher('/tb3_3/Coordenada', PoseWithCovarianceStamped, queue_size=1)
        self.S1P2 = rospy.Publisher('/tb3_1/S1P2', PoseWithCovarianceStamped, queue_size=1)
        self.S1G = rospy.Publisher('/GrafSeg1', Bool, queue_size=1)
        self.OP = rospy.Publisher("/tb3/Orden", OrdenFormacion, queue_size=1)
        self.EstadoPublisher=rospy.Publisher('Accion_del_sistema',String,queue_size=10)
        self.DetErrorPublisher=rospy.Publisher('/DetRegErr',Bool,queue_size=10)
        self.frecuencia = rospy.Rate(100)
        self.separacion_formacion = 0.26 # Distancia en metros que los robots guardan entre sí para la formación
        self.rapidez_angular = 0.5
        self.rapidez_lineal = 0.1
        self.S1AP_Ban = False
        self.S2AP_Ban = False
        self.S3AP_Ban = False
        self.RF_Bandera = False
    #Funciones callback que actualizan las posiciones y orientaciones de los
    #seguidores a partir de la información recibida de su respectivo AMCL.
    def FSCallback(self, dato):
        self.RF_Bandera = dato.data
    
    def S1APCallback(self, dato):
        self.S1AP_Ban = dato.data
    
    def S2APCallback(self, dato):
        self.S2AP_Ban = dato.data

    def S3APCallback(self, dato):
        self.S3AP_Ban = dato.data
    
    def poseCallback1(self, dato):
        self.Posicion_seg_1.x = dato.pose.pose.position.x 
        self.Posicion_seg_1.y = dato.pose.pose.position.y
        quat_amcl = dato.pose.pose.orientation
        lista_quat = [quat_amcl.x, quat_amcl.y, quat_amcl.z, quat_amcl.w]
        (a_x, a_y, a_z) = euler_from_quaternion(lista_quat)
        self.Posicion_seg_1.theta = self.angulo_positivo(a_z)
    def poseCallback2(self, dato):
        self.Posicion_seg_2.x = dato.pose.pose.position.x 
        self.Posicion_seg_2.y = dato.pose.pose.position.y
        quat_amcl = dato.pose.pose.orientation
        lista_quat = [quat_amcl.x, quat_amcl.y, quat_amcl.z, quat_amcl.w]
        (a_x, a_y, a_z) = euler_from_quaternion(lista_quat)
        self.Posicion_seg_2.theta = self.angulo_positivo(a_z)
    def poseCallback3(self, dato):
        self.Posicion_seg_3.x = dato.pose.pose.position.x 
        self.Posicion_seg_3.y = dato.pose.pose.position.y
        quat_amcl = dato.pose.pose.orientation
        lista_quat = [quat_amcl.x, quat_amcl.y, quat_amcl.z, quat_amcl.w]
        (a_x, a_y, a_z) = euler_from_quaternion(lista_quat)
        self.Posicion_seg_3.theta = self.angulo_positivo(a_z)
    
    def angulo_positivo(self, an):
        if an < 0:
            an = 2*np.pi + an
        return an
    #Solo convierte el ángulo en positivo
    
    def distancia(self,x1,y1,x2,y2):
        return ((x1-x2)**2 + (y1-y2)**2)**0.5
    #Calcula la distancia entre dos puntos

    def RealizarFormacion(self):
        
        Posicion_lider = rospy.wait_for_message("/tb3_0/amcl_pose", PoseWithCovarianceStamped)
        lider_x = Posicion_lider.pose.pose.position.x
        lider_y = Posicion_lider.pose.pose.position.y
        distancia1 = self.distancia(lider_x,lider_y,self.Posicion_seg_1.x,self.Posicion_seg_1.y)
        distancia2 = self.distancia(lider_x,lider_y,self.Posicion_seg_2.x,self.Posicion_seg_2.y)
        distancia3 = self.distancia(lider_x,lider_y,self.Posicion_seg_3.x,self.Posicion_seg_3.y)
        distancias = [distancia1, distancia2, distancia3]
        distancia_max = 0
        tercer_robot = 0
        distancia_med = 0
        segundo_robot = 0
        distancia_min = 0
        primer_robot = 0
        self.EstadoPublisher.publish("Calculando orden de formacion")
        for i in range(3):
            if distancias[i] > distancia_max:
                distancia_max = distancias[i]
                tercer_robot = i + 1
        pos = [self.Posicion_seg_1, self.Posicion_seg_2, self.Posicion_seg_3]
        pos_tercer = pos[tercer_robot - 1]
        if tercer_robot == 1:
            dis2 = self.distancia(pos_tercer.x,pos_tercer.y,self.Posicion_seg_2.x,self.Posicion_seg_2.y)
            dis3 = self.distancia(pos_tercer.x,pos_tercer.y,self.Posicion_seg_3.x,self.Posicion_seg_3.y)
            if dis2 > dis3:
                segundo_robot = 3
                primer_robot = 2
            else:
                segundo_robot = 2
                primer_robot = 3
        elif tercer_robot == 2:
            dis2 = self.distancia(pos_tercer.x,pos_tercer.y,self.Posicion_seg_1.x,self.Posicion_seg_1.y)
            dis3 = self.distancia(pos_tercer.x,pos_tercer.y,self.Posicion_seg_3.x,self.Posicion_seg_3.y)
            if dis2 > dis3:
                segundo_robot = 3
                primer_robot = 1
            else:
                segundo_robot = 1
                primer_robot = 3
        elif tercer_robot == 3:
            dis2 = self.distancia(pos_tercer.x,pos_tercer.y,self.Posicion_seg_1.x,self.Posicion_seg_1.y)
            dis3 = self.distancia(pos_tercer.x,pos_tercer.y,self.Posicion_seg_2.x,self.Posicion_seg_2.y)
            if dis2 > dis3:
                segundo_robot = 2
                primer_robot = 1
            else:
                segundo_robot = 1
                primer_robot = 2
        else:
            while True:
                print "tercer_robot fuera de rango: %f"%tercer_robot
        self.EstadoPublisher.publish("Rotando seguidor trasero")
        self.Rotar(lider_x, lider_y, tercer_robot)
        Posiciones = [self.Posicion_seg_1, self.Posicion_seg_2, self.Posicion_seg_3]
        Pos_seg_enm = Posiciones[segundo_robot - 1]
        Pos_seg_del = Posiciones[primer_robot - 1]
        Pos_seg_tras = Posiciones[tercer_robot - 1]
        dis_seg2 = self.distancia(Pos_seg_enm.x, Pos_seg_enm.y, Pos_seg_tras.x, Pos_seg_tras.y)
        dis_seg1 = self.distancia(Pos_seg_del.x, Pos_seg_del.y, Pos_seg_tras.x, Pos_seg_tras.y)
        if dis_seg2 > 0.4:
            alfa = np.arctan2((Pos_seg_enm.y-Pos_seg_tras.y),(Pos_seg_enm.x-Pos_seg_tras.x))
            x_offset1 = abs(np.cos(Pos_seg_tras.theta - alfa)*dis_seg2)
        else:
            x_offset1 = self.separacion_formacion
        if dis_seg1 > 0.8:
            alfa = np.arctan2((Pos_seg_del.y-Pos_seg_tras.y),(Pos_seg_del.x-Pos_seg_tras.x))
            x_offset2 = abs(np.cos(Pos_seg_tras.theta - alfa)*dis_seg1)

        else:
            x_offset2 = 2*self.separacion_formacion
        if abs(x_offset1 - x_offset2) < 0.38:
            x_offset2 += 0.38 - abs(x_offset1 - x_offset2)
        angulos = [self.Posicion_seg_1.theta, self.Posicion_seg_2.theta, self.Posicion_seg_3.theta]
        angulo_orientacion = angulos[tercer_robot - 1]
        coordenadas_x = [self.Posicion_seg_1.x, self.Posicion_seg_2.x, self.Posicion_seg_3.x]
        x_ref = coordenadas_x[tercer_robot - 1]
        x1 = x_ref + x_offset1*np.cos(angulo_orientacion)
        coordenadas_y = [self.Posicion_seg_1.y, self.Posicion_seg_2.y, self.Posicion_seg_3.y]
        y_ref = coordenadas_y[tercer_robot - 1]
        y1 = y_ref + x_offset1*np.sin(angulo_orientacion)
        Objetivo = PoseWithCovarianceStamped()
        Objetivo.header.stamp = rospy.Time.now()
        Objetivo.pose.pose.position.x = x1
        Objetivo.pose.pose.position.y = y1
        Banderas = [self.B1P, self.B2P, self.B3P]
        Pub_coordenadas = [self.C1P, self.C2P, self.C3P]
        bandera = Banderas[segundo_robot - 1]
        Pub = Pub_coordenadas[segundo_robot - 1]
        Posiciones = [self.Posicion_seg_1, self.Posicion_seg_2, self.Posicion_seg_3]
        Posicion = Posiciones[segundo_robot - 1]
        distancia = self.distancia(x1,y1,Posicion.x,Posicion.y)
        self.EstadoPublisher.publish("Rotando seguidor de en medio")
        self.Rotar(x1, y1, segundo_robot)
        for i in range(20):
            bandera.publish(True)
            self.frecuencia.sleep()
        banderas_pos = [self.S1AP_Ban, self.S2AP_Ban, self.S3AP_Ban]
        ban_pos = banderas_pos[segundo_robot - 1]
        self.EstadoPublisher.publish("Yendo hacia la formacion")
        while not ban_pos:
            Pub.publish(Objetivo)
            self.frecuencia.sleep()
            banderas_pos = [self.S1AP_Ban, self.S2AP_Ban, self.S3AP_Ban]
            ban_pos = banderas_pos[segundo_robot - 1]
        for i in range(20):
            bandera.publish(False)
            self.frecuencia.sleep()
        self.EstadoPublisher.publish("Lugar en la formacion alcanzado")
        self.S1AP_Ban = False
        self.S2AP_Ban = False
        self.S3AP_Ban = False
        self.Rotar(lider_x, lider_y, segundo_robot)
        if dis_seg2 <= 0.4:
            self.ComprobarDistancia(tercer_robot, segundo_robot, self.separacion_formacion*0.95)
        x1 = x_ref + x_offset2*np.cos(angulo_orientacion%(2*np.pi))
        y1 = y_ref + x_offset2*np.sin(angulo_orientacion%(2*np.pi))
        Objetivo.header.stamp = rospy.Time.now()
        Objetivo.pose.pose.position.x = x1
        Objetivo.pose.pose.position.y = y1
        bandera = Banderas[primer_robot - 1]
        Pub = Pub_coordenadas[primer_robot - 1]
        Posiciones = [self.Posicion_seg_1, self.Posicion_seg_2, self.Posicion_seg_3]
        Posicion = Posiciones[primer_robot - 1]
        distancia = self.distancia(x1,y1,Posicion.x,Posicion.y)
        self.EstadoPublisher.publish("Rotando seguidor delantero")
        self.Rotar(x1, y1, primer_robot)
        for i in range(20):
            bandera.publish(True)
            self.frecuencia.sleep()
        banderas_pos = [self.S1AP_Ban, self.S2AP_Ban, self.S3AP_Ban]
        ban_pos = banderas_pos[primer_robot - 1]
        self.EstadoPublisher.publish("Yendo hacia la formacion")
        while not ban_pos:
            self.S1P2.publish(Objetivo)
            self.S1G.publish(True)
            Pub.publish(Objetivo)
            self.frecuencia.sleep()
            banderas_pos = [self.S1AP_Ban, self.S2AP_Ban, self.S3AP_Ban]
            ban_pos = banderas_pos[primer_robot - 1]
        for i in range(20):
            bandera.publish(False)
            self.frecuencia.sleep()
        self.EstadoPublisher.publish("Lugar en la formacion alcanzado")
        self.S1AP_Ban = False
        self.S2AP_Ban = False
        self.S3AP_Ban = False
        self.Rotar(lider_x, lider_y, primer_robot) 
        if dis_seg1 <= 1:
            self.ComprobarDistancia(segundo_robot, primer_robot, self.separacion_formacion)
        self.EstadoPublisher.publish("Se ha conformado la formacion")
        self.EncontrarLider(primer_robot, segundo_robot, tercer_robot)
        self.EstadoPublisher.publish("Se ha alcanzado al lider")
        self.RF_Bandera = False


    def EncontrarLider(self,primer, segundo, tercer):
        self.EstadoPublisher.publish("Comenzando movimiento en formacion")
        OrdenFormacion = [primer, segundo, tercer]
        for i in range(40):
            self.OP.publish(OrdenFormacion)
            self.frecuencia.sleep()
        Posicion_lider = rospy.wait_for_message("/tb3_0/amcl_pose", PoseWithCovarianceStamped)
        Posiciones =[self.Posicion_seg_1,self.Posicion_seg_2,self.Posicion_seg_3]
        Pos1 = Posiciones[primer-1]
        lider_x = Posicion_lider.pose.pose.position.x
        lider_y = Posicion_lider.pose.pose.position.y
        l_x = lider_x
        l_y = lider_y
        lider_x = lider_x - 0.7*self.separacion_formacion*np.cos(np.arctan2((l_y-Pos1.y),(l_x - Pos1.x)))
        lider_y = lider_y - 0.7*self.separacion_formacion*np.sin(np.arctan2((l_y-Pos1.y),(l_x - Pos1.x)))    
        Objetivo = PoseWithCovarianceStamped()
        Objetivo.header.stamp = rospy.Time.now()
        Objetivo.pose.pose.position.x = lider_x
        Objetivo.pose.pose.position.y = lider_y  
        banderas = [self.B1P, self.B2P, self.B3P]
        b1 = banderas[primer - 1]
        b2 = banderas[segundo - 1]
        b3 = banderas[tercer - 1]
        coordenadas = [self.C1P, self.C2P, self.C3P]
        c1 = coordenadas[primer - 1]
        c2 = coordenadas[segundo - 1]
        c3 = coordenadas[tercer - 1]
        Velocidad_pubs = [self.S1VP, self.S2VP, self.S3VP]
        V1 = Velocidad_pubs[primer - 1]
        V2 = Velocidad_pubs[segundo - 1]
        V3 = Velocidad_pubs[tercer - 1]


        for i in range(20):
            b1.publish(True)
            self.frecuencia.sleep()
        for i in range(20):
            b2.publish(True)
            self.frecuencia.sleep()
        for i in range(20):
            b3.publish(True)
            self.frecuencia.sleep()
        distancia2=0.0
        distancia3 = 0.0
        for i in range(20):
            c1.publish(Objetivo)
            self.frecuencia.sleep()
        while not self.S1AP_Ban or not self.S2AP_Ban or not self.S3AP_Ban:
            Posiciones =[self.Posicion_seg_1,self.Posicion_seg_2,self.Posicion_seg_3]
            Pos1 = Posiciones[primer-1]
            angdir = np.arctan2((l_y-Pos1.y),(l_x - Pos1.x))
            angdir = angdir%(2*np.pi)
            x2 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir)
            y2 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir)


            x2_1 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir-1*np.pi/72)
            y2_1 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir-1*np.pi/72)
            x2_2 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir-2*np.pi/72)
            y2_2 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir-2*np.pi/72)
            x2_3 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir-3*np.pi/72)
            y2_3 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir-3*np.pi/72)
            x2_4 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir-4*np.pi/72)
            y2_4 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir-4*np.pi/72)
            x2_5 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir-5*np.pi/72)
            y2_5 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir-5*np.pi/72)
            x2_6 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir-6*np.pi/72)
            y2_6 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir-6*np.pi/72)
            x2_7 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir+1*np.pi/72)
            y2_7 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir+1*np.pi/72)
            x2_8 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir+2*np.pi/72)
            y2_8 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir+2*np.pi/72)
            x2_9 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir+3*np.pi/72)
            y2_9 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir+3*np.pi/72)
            x2_10 = Pos1.x -0.45*self.separacion_formacion*np.cos(angdir+4*np.pi/72)
            y2_10 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir+4*np.pi/72)
            x2_11 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir+5*np.pi/72)
            y2_11 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir+5*np.pi/72)
            x2_12 = Pos1.x - 0.45*self.separacion_formacion*np.cos(angdir+6*np.pi/72)
            y2_12 = Pos1.y - 0.45*self.separacion_formacion*np.sin(angdir+6*np.pi/72)

            posics = [self.Posicion_seg_1, self.Posicion_seg_2, self.Posicion_seg_3]
            pos_2 = posics[segundo - 1]
            

            lista_x = [x2, x2_1, x2_2, x2_3, x2_4, x2_5, x2_6, x2_7, x2_8, x2_9, x2_10, x2_11, x2_12]
            lista_y = [y2, y2_1, y2_2, y2_3, y2_4, y2_5, y2_6, y2_7, y2_8, y2_9, y2_10, y2_11, y2_12]
            an_rel_min = 2*np.pi
            x_c = 0
            y_c = 0
            ind = 0
            for i in range(13):
                an_des = np.arctan2(lista_y[i]-pos_2.y,lista_x[i]-pos_2.x)
                an_des = an_des%(2*np.pi)
                an_rel = abs(an_des - pos_2.theta)
                if an_rel < an_rel_min:
                    an_rel_min = an_rel
                    ind = i
            # x_c = lista_x[ind]
            # y_c = lista_y[ind]

            x_c = x2
            y_c = y2


            Objetivo = PoseWithCovarianceStamped()
            Objetivo.header.stamp = rospy.Time.now()
            Objetivo.pose.pose.position.x = x_c
            Objetivo.pose.pose.position.y = y_c

            for i in range(10):
                c2.publish(Objetivo)
                self.frecuencia.sleep()
            
            
            Posiciones =[self.Posicion_seg_1,self.Posicion_seg_2,self.Posicion_seg_3]
            Pos2 = Posiciones[segundo-1]
            angdir = np.arctan2((l_y-Pos2.y),(l_x - Pos2.x))
            angdir = angdir%(2*np.pi)
            x2 = Pos2.x - 0.45*self.separacion_formacion*np.cos(angdir)
            y2 = Pos2.y - 0.45*self.separacion_formacion*np.sin(angdir)
            
            
            x2_1 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir-1*np.pi/72)
            y2_1 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir-1*np.pi/72)
            x2_2 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir-2*np.pi/72)
            y2_2 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir-2*np.pi/72)
            x2_3 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir-3*np.pi/72)
            y2_3 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir-3*np.pi/72)
            x2_4 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir-4*np.pi/72)
            y2_4 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir-4*np.pi/72)
            x2_5 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir-5*np.pi/72)
            y2_5 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir-5*np.pi/72)
            x2_6 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir-6*np.pi/72)
            y2_6 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir-6*np.pi/72)
            x2_7 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir+1*np.pi/72)
            y2_7 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir+1*np.pi/72)
            x2_8 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir+2*np.pi/72)
            y2_8 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir+2*np.pi/72)
            x2_9 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir+3*np.pi/72)
            y2_9 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir+3*np.pi/72)
            x2_10 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir+4*np.pi/72)
            y2_10 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir+4*np.pi/72)
            x2_11 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir+5*np.pi/72)
            y2_11 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir+5*np.pi/72)
            x2_12 = Pos2.x -  0.45*self.separacion_formacion*np.cos(angdir+6*np.pi/72)
            y2_12 = Pos2.y -  0.45*self.separacion_formacion*np.sin(angdir+6*np.pi/72)

            posics = [self.Posicion_seg_1, self.Posicion_seg_2, self.Posicion_seg_3]
            pos_3 = posics[tercer - 1]
            

            
            lista_x = [x2, x2_1, x2_2, x2_3, x2_4, x2_7, x2_8, x2_9, x2_10]
            lista_y = [y2, y2_1, y2_2, y2_3, y2_4, y2_7, y2_8, y2_9, y2_10]
            an_rel_min = 2*np.pi
            x_c = 0
            y_c = 0
            ind = 0
            for i in range(9):
                an_des = np.arctan2(lista_y[i]-pos_3.y,lista_x[i]-pos_3.x)
                an_des = an_des%(2*np.pi)
                an_rel = abs(an_des - pos_3.theta)
                if an_rel < an_rel_min:
                    an_rel_min = an_rel
                    ind = i
            x_c = lista_x[ind]
            y_c = lista_y[ind]

            
                


            
            Objetivo = PoseWithCovarianceStamped()
            Objetivo.header.stamp = rospy.Time.now()
            Objetivo.pose.pose.position.x = x_c
            Objetivo.pose.pose.position.y = y_c
            for i in range(10):
                c3.publish(Objetivo)
                self.frecuencia.sleep()
           
            
            
    
            



            

           


            
        
        for i in range(20):
            self.DetErrorPublisher.publish(True)
            self.frecuencia.sleep()
        
        
        for i in range(20):
            b1.publish(False)
            self.frecuencia.sleep()
        for i in range(20):
            b2.publish(False)
            self.frecuencia.sleep()
        for i in range(20):
            b3.publish(False)
            self.frecuencia.sleep()

        self.Velocidad.linear.x = 0
        self.Velocidad.angular.z = 0
        for i in range(20):
            V1.publish(self.Velocidad)
            self.frecuencia.sleep()
        for i in range(20):
            V2.publish(self.Velocidad)
            self.frecuencia.sleep()
        for i in range(20):
            V3.publish(self.Velocidad)
            self.frecuencia.sleep()
        
        
        

        

        

    
    
    def ComprobarDistancia(self, robot_ref, robot_del, separacion):
        Posiciones = [self.Posicion_seg_1, self.Posicion_seg_2, self.Posicion_seg_3]
        pos_ref = Posiciones[robot_ref - 1]
        pos_del = Posiciones[robot_del - 1]
        distancia = self.distancia(pos_ref.x, pos_ref.y, pos_del.x, pos_del.y)
        publicadores = [self.S1VP, self.S2VP, self.S3VP]
        pub = publicadores[robot_del - 1]
        if distancia > separacion:
            self.Velocidad.linear.x = - self.rapidez_lineal*0.2
            while distancia > separacion:
                pub.publish(self.Velocidad)
                self.frecuencia.sleep()
                Posiciones = [self.Posicion_seg_1, self.Posicion_seg_2, self.Posicion_seg_3]
                pos_del = Posiciones[robot_del - 1]
                pos_ref = Posiciones[robot_ref - 1]
                distancia = self.distancia(pos_ref.x, pos_ref.y, pos_del.x, pos_del.y)
                print "Moviéndose hacia atrás"
        else:
            self.Velocidad.linear.x = self.rapidez_lineal*0.2
            while distancia < separacion:
                pub.publish(self.Velocidad)
                self.frecuencia.sleep()
                Posiciones = [self.Posicion_seg_1, self.Posicion_seg_2, self.Posicion_seg_3]
                pos_del = Posiciones[robot_del - 1]
                pos_ref = Posiciones[robot_ref - 1]
                distancia = self.distancia(pos_ref.x, pos_ref.y, pos_del.x, pos_del.y)
                print "Moviéndose hacia adelante"
        self.Velocidad.linear.x = 0
        for i in range(30):
            pub.publish(self.Velocidad)
            self.frecuencia.sleep()




    def Rotar(self, x_des, y_des, robot):
        angulo_robot = 0
        if robot == 1:
            angulo_robot = self.Posicion_seg_1.theta
            x = self.Posicion_seg_1.x
            y = self.Posicion_seg_1.y
        elif robot == 2:
            angulo_robot = self.Posicion_seg_2.theta
            x = self.Posicion_seg_2.x
            y = self.Posicion_seg_2.y
        else:
            angulo_robot = self.Posicion_seg_3.theta
            x = self.Posicion_seg_3.x
            y = self.Posicion_seg_3.y
        an_des = np.arctan2(y_des-y,x_des-x)
        an_des = an_des%(2*np.pi)
        angulo_relativo = an_des - angulo_robot
        angulo_relativo_pos = self.angulo_positivo(angulo_relativo)
        while abs(angulo_relativo) > 0.08:
            if (angulo_relativo_pos < np.pi):
                if angulo_relativo_pos < 0.31:
                    k_an = 3.8*(angulo_relativo_pos/np.pi)
                else:
                    k_an = 1
            else:
                if (angulo_relativo_pos > 5.96):
                    k_an = -3.8* (1 - ((angulo_relativo_pos - np.pi)/np.pi))
                else:
                    k_an = -1
            self.Velocidad.angular.z = k_an*self.rapidez_angular
            if robot == 1:
                self.S1VP.publish(self.Velocidad)
            elif robot == 2:
                self.S2VP.publish(self.Velocidad)
            else:
                self.S3VP.publish(self.Velocidad)
            self.frecuencia.sleep()
            angulo_robot = 0
            if robot == 1:
                angulo_robot = self.Posicion_seg_1.theta
                x = self.Posicion_seg_1.x
                y = self.Posicion_seg_1.y
            elif robot == 2:
                angulo_robot = self.Posicion_seg_2.theta
                x = self.Posicion_seg_2.x
                y = self.Posicion_seg_2.y
            else:
                angulo_robot = self.Posicion_seg_3.theta
                x = self.Posicion_seg_3.x
                y = self.Posicion_seg_3.y
            an_des = np.arctan2(y_des-y,x_des-x)
            an_des = an_des%(2*np.pi)
            angulo_relativo = an_des - angulo_robot
            angulo_relativo_pos = self.angulo_positivo(angulo_relativo)
        self.Velocidad.angular.z = 0
        publicadores = [self.S1VP, self.S2VP, self.S3VP]
        publicador = publicadores[robot - 1]
        for i in range(100):
            publicador.publish(self.Velocidad)
            self.frecuencia.sleep()

        
        
        
    

def Rutina():
    rospy.init_node('CoordinacionSistema', anonymous = True)
    Sistema = FormacionSeguidores()
    while True:
        if Sistema.RF_Bandera:
            Sistema.RealizarFormacion()



if __name__ == '__main__':
    try:
        Rutina()
    except rospy.ROSInterruptException:
        pass
