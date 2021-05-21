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
from std_msgs.msg import Float64,Int32,Bool
from esquema_lider_seguidor.msg import Histograma
from esquema_lider_seguidor.msg import OrdenFormacion


from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MovimientoSeguidor:
    def __init__(self):
        self.Posicion = Pose()
        self.DelPos = Pose()
        self.MedPos = Pose()
        self.TrasPos = Pose()
        self.Velocidad = Twist()
        self.ps = rospy.Subscriber("/tb3_3/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)
        self.coordenada = rospy.Subscriber("/tb3_3/Coordenada", PoseWithCovarianceStamped, self.coordenadaCallback,queue_size=1)
        self.bandera = rospy.Subscriber("/tb3_3/Bandera", Bool, self.banderaCallback,queue_size=1)
        self.bandera = rospy.Subscriber("/tb3/Orden", OrdenFormacion, self.OrdenCallback,queue_size=1)
        self.moverse = False
        self.PosFormacion = 0
        self.OrdenFormacion = []
        self.SelectorRutina = 1
        self.Velocidad_publisher = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)
        self.Objetivo_publisher = rospy.Publisher('/tb3_3/AlcanzoPos', Bool, queue_size=1)
        self.Error_publisher = rospy.Publisher('/tb3_3/Error', Float64, queue_size=1)
        self.frecuencia = rospy.Rate(160)
        self.rapidez_lineal = 0.1 # 10 cm/ seg
        self.rapidez_angular = 0.4 # 23°/seg
        self.coordenada = Pose()
    def OrdenCallback(self, dato):
        self.OrdenFormacion = list(dato.Orden)
        for i in range(3):
            if self.OrdenFormacion[i] == 3:
                self.PosFormacion =  i + 1
        for i in range(100):
            print "Mi posicion es %i"%self.PosFormacion
        self.SelectorRutina = 2
        if self.PosFormacion == 1 or self.PosFormacion == 3:
            P1 = "/tb3_"
            P2 = str(self.OrdenFormacion[1])
            P3 = "/amcl_pose"
            Nombre = P1+P2+P3
            self.MSP = rospy.Subscriber(Nombre, PoseWithCovarianceStamped,self.medioCallback,queue_size=1)
        else:
            P1 = "/tb3_"
            P2 = str(self.OrdenFormacion[0])
            P3 = str(self.OrdenFormacion[2])
            P4 = "/amcl_pose"
            Nombre1 = P1+P2+P4
            Nombre2 = P1+P3+P4
            self.DSP = rospy.Subscriber(Nombre1, PoseWithCovarianceStamped,self.delanteCallback,queue_size=1)
            self.TSP = rospy.Subscriber(Nombre2, PoseWithCovarianceStamped,self.traseroCallback,queue_size=1)
    


    def medioCallback(self, dato):
        self.MedPos.x=dato.pose.pose.position.x 
        self.MedPos.y=dato.pose.pose.position.y

    def delanteCallback(self, dato):
        self.DelPos.x=dato.pose.pose.position.x 
        self.DelPos.y=dato.pose.pose.position.y
    
    def traseroCallback(self, dato):
        self.TrasPos.x=dato.pose.pose.position.x 
        self.TrasPos.y=dato.pose.pose.position.y
    
    def banderaCallback(self, dato):
        self.moverse = dato.data
    def coordenadaCallback(self, dato):
        self.coordenada.x = dato.pose.pose.position.x
        self.coordenada.y = dato.pose.pose.position.y
    def poseCallback(self,pose_amcl):
        print "Recibí mi ubicación"
        self.Posicion.x=pose_amcl.pose.pose.position.x 
        self.Posicion.y=pose_amcl.pose.pose.position.y
        # Actualizar las coordenadas (x,y) del seguidor 1 a partir de la información recibida del nodo AMCL.
        quat_amcl = pose_amcl.pose.pose.orientation
        lista_quat = [quat_amcl.x, quat_amcl.y, quat_amcl.z, quat_amcl.w]
        (a_x, a_y, a_z) = euler_from_quaternion(lista_quat)
        self.Posicion.theta = self.angulo_positivo(a_z)
        # Actualiza el valor del ángulo de yaw (eje z) del robot a partir de la información recibida del nodo AMCL.
    def angulo_positivo(self, an):
        if an < 0:
            an = 2*np.pi + an
        return an
        #Solo convierte el ángulo en positivo

    def distancia(self,x1,y1,x2,y2):
        return ((x1-x2)**2 + (y1-y2)**2)**0.5
        #Calcula la distancia entre dos puntos

    def Orientar(self, x, y):
        an_des = np.arctan2(y-self.Posicion.y,x-self.Posicion.x)
        an_des = an_des%(2*np.pi)
        angulo_relativo = an_des - self.Posicion.theta
        angulo_relativo_pos = self.angulo_positivo(angulo_relativo)
        abs_realtivo = abs(angulo_relativo)
        if (angulo_relativo_pos < np.pi):
            if (abs_realtivo > 1.047):
                k_an = 1
            elif (abs_realtivo < 1.047 and abs_realtivo > 0.523):
                k_an = 0.8
            elif(abs_realtivo < 0.523 and abs_realtivo > 0.04):
                k_an =0.5*(abs_realtivo/0.523)
            else:
                k_an = 0
        else:
            if (abs_realtivo > 1.047):
                k_an = -1
            elif (abs_realtivo < 1.047 and abs_realtivo > 0.523):
                k_an = -0.8
            elif(abs_realtivo < 0.523 and abs_realtivo > 0.04):
                k_an = -0.5*(abs_realtivo/0.523)
            else:
                k_an = 0
        print "Vel_an: %f"%self.Velocidad.angular.z
        print " abs angulo relativo: %f"%abs(angulo_relativo)
        print "k_an: %f"%k_an
        self.Velocidad.angular.z = k_an*self.rapidez_angular
    def Mover(self,x, y):
        distancia = self.distancia(self.Posicion.x, self.Posicion.y, self.coordenada.x, self.coordenada.y)
        an_des = np.arctan2(y-self.Posicion.y,x-self.Posicion.x)
        an_des = an_des%(2*np.pi)
        angulo_relativo = an_des - self.Posicion.theta
        angulo_relativo_pos = self.angulo_positivo(angulo_relativo)
        abs_realtivo = abs(angulo_relativo)
        if (abs_realtivo > 1.57):
            k_lin = 0
        elif (abs_realtivo < 1.57 and abs_realtivo > 0.7):
            k_lin = 2*0.225*(np.pi - abs_realtivo)/np.pi
        elif (abs_realtivo < 0.7 and abs_realtivo > 0.09):
            k_lin = 2*0.3*(2.0944 - abs_realtivo)/2.0944
        else:
            k_lin = 1
        if distancia < 0.1:
            k_lin = 0.18
        elif distancia < 0.02:
            k_lin = 0
            k_an = 0
        self.Velocidad.linear.x = k_lin*self.rapidez_lineal
        print "Distancia: %f"%distancia
        print "Velocidad lineal: %f"%self.Velocidad.linear.x
        print "k_lin: %f"%k_lin







    def EncontrarPunto(self):
            print "La bandera moverse es"
            print self.moverse
            if self.moverse:
                while self.Posicion.x == 0 and self.Posicion.y == 0:
                    print "Estoy esperando mi posición"
                distancia = self.distancia(self.Posicion.x, self.Posicion.y, self.coordenada.x, self.coordenada.y)
                an_des = np.arctan2(self.coordenada.y-self.Posicion.y,self.coordenada.x-self.Posicion.x)
                an_des = an_des%(2*np.pi)
                angulo_relativo = an_des - self.Posicion.theta
                orientarse = abs(angulo_relativo) > 0.04
                moverse = distancia > 0.02
                print "Distancia es: %f"%distancia
                print "x_robot = %f"%self.Posicion.x
                print "y_robot = %f"%self.Posicion.y
                while moverse and self.moverse:
                    self.Orientar(self.coordenada.x, self.coordenada.y)
                    self.Mover(self.coordenada.x, self.coordenada.y)
                    self.Velocidad_publisher.publish(self.Velocidad)
                    self.frecuencia.sleep()
                    print "Estoy imprimiendo velocidad"
                    distancia = self.distancia(self.Posicion.x, self.Posicion.y, self.coordenada.x, self.coordenada.y)
                    an_des = np.arctan2(self.coordenada.y-self.Posicion.y,self.coordenada.x-self.Posicion.x)
                    an_des = an_des%(2*np.pi)
                    angulo_relativo = an_des - self.Posicion.theta
                    orientarse = abs(angulo_relativo) > 0.04
                    moverse = distancia > 0.02
                    if not moverse:
                        for i in range(20):
                            self.Objetivo_publisher.publish(True)
                            self.frecuencia.sleep()
                self.Velocidad.linear.x = 0
                self.Velocidad.angular.z = 0
                for i in range(40):
                    self.Velocidad_publisher.publish(self.Velocidad)
                    self.frecuencia.sleep()
                print "Terminado"
            else:
                self.Velocidad.linear.x = 0
                self.Velocidad.angular.z = 0
                self.Velocidad_publisher.publish(self.Velocidad)
                self.frecuencia.sleep()

    def EncontrarPuntoEnFormacion(self):
            print "La bandera moverse es"
            print self.moverse
            if self.moverse:
                while self.Posicion.x == 0 and self.Posicion.y == 0:
                    print "Estoy esperando mi posición"
                distancia = self.distancia(self.Posicion.x, self.Posicion.y, self.coordenada.x, self.coordenada.y)
                an_des = np.arctan2(self.coordenada.y-self.Posicion.y,self.coordenada.x-self.Posicion.x)
                an_des = an_des%(2*np.pi)
                angulo_relativo = an_des - self.Posicion.theta
                orientarse = abs(angulo_relativo) > 0.04
                moverse = distancia > 0.02
                print "Distancia es: %f"%distancia
                print "x_robot = %f"%self.Posicion.x
                print "y_robot = %f"%self.Posicion.y
                while moverse and self.moverse:
                    if self.PosFormacion == 1:
                        d1 = self.distancia(self.Posicion.x, self.Posicion.y, self.MedPos.x, self.MedPos.y)
                        if d1 > 0.35:
                            self.Velocidad.linear.x = 0
                            self.Velocidad.angular.z = 0
                            self.Velocidad_publisher.publish(self.Velocidad)
                            self.frecuencia.sleep()
                            print "Estoy esperando al de atrás"
                        else:
                            self.Orientar(self.coordenada.x, self.coordenada.y)
                            self.Mover(self.coordenada.x, self.coordenada.y)
                            self.Velocidad_publisher.publish(self.Velocidad)
                            self.frecuencia.sleep()
                            print "Estoy imprimiendo velocidad"
                            distancia = self.distancia(self.Posicion.x, self.Posicion.y, self.coordenada.x, self.coordenada.y)
                            an_des = np.arctan2(self.coordenada.y-self.Posicion.y,self.coordenada.x-self.Posicion.x)
                            an_des = an_des%(2*np.pi)
                            angulo_relativo = an_des - self.Posicion.theta
                            orientarse = abs(angulo_relativo) > 0.04
                            moverse = distancia > 0.02
                            if not moverse:
                                for i in range(20):
                                    self.Objetivo_publisher.publish(True)
                                    self.frecuencia.sleep()
                    elif self.PosFormacion == 2:
                        d1 = self.distancia(self.Posicion.x, self.Posicion.y, self.DelPos.x, self.DelPos.y)
                        d2 = self.distancia(self.Posicion.x, self.Posicion.y, self.TrasPos.x, self.TrasPos.y)
                        if d1 < 0.25 or d2 > 0.35:
                            self.Velocidad.linear.x = 0
                            self.Velocidad.angular.z = 0
                            self.Velocidad_publisher.publish(self.Velocidad)
                            if d1 < 0.25:
                                for i in range(20):
                                    self.Objetivo_publisher.publish(True)
                                    self.frecuencia.sleep()
                                
                        else:
                            self.Orientar(self.coordenada.x, self.coordenada.y)
                            self.Mover(self.coordenada.x, self.coordenada.y)
                            self.Velocidad_publisher.publish(self.Velocidad)
                            self.frecuencia.sleep()
                            print "Estoy imprimiendo velocidad"
                            distancia = self.distancia(self.Posicion.x, self.Posicion.y, self.coordenada.x, self.coordenada.y)
                            self.Error_publisher.publish(distancia)
                            an_des = np.arctan2(self.coordenada.y-self.Posicion.y,self.coordenada.x-self.Posicion.x)
                            an_des = an_des%(2*np.pi)
                            angulo_relativo = an_des - self.Posicion.theta
                            orientarse = abs(angulo_relativo) > 0.04
                            moverse = distancia > 0.02
                            for i in range(20):
                                    self.Objetivo_publisher.publish(False)
                                    self.frecuencia.sleep()

                    else:
                        d1 = self.distancia(self.Posicion.x, self.Posicion.y, self.MedPos.x, self.MedPos.y)
                        if d1 < 0.25:
                            self.Velocidad.linear.x = 0
                            self.Velocidad.angular.z = 0
                            self.Velocidad_publisher.publish(self.Velocidad)
                            for i in range(20):
                                    self.Objetivo_publisher.publish(True)
                                    self.frecuencia.sleep()
                        else:
                            self.Orientar(self.coordenada.x, self.coordenada.y)
                            self.Mover(self.coordenada.x, self.coordenada.y)
                            self.Velocidad_publisher.publish(self.Velocidad)
                            self.frecuencia.sleep()
                            print "Estoy imprimiendo velocidad"
                            distancia = self.distancia(self.Posicion.x, self.Posicion.y, self.coordenada.x, self.coordenada.y)
                            an_des = np.arctan2(self.coordenada.y-self.Posicion.y,self.coordenada.x-self.Posicion.x)
                            an_des = an_des%(2*np.pi)
                            angulo_relativo = an_des - self.Posicion.theta
                            orientarse = abs(angulo_relativo) > 0.04
                            moverse = distancia > 0.02
                            for i in range(20):
                                    self.Objetivo_publisher.publish(False)
                                    self.frecuencia.sleep()
                                    


                self.Velocidad.linear.x = 0
                self.Velocidad.angular.z = 0
                for i in range(40):
                    self.Velocidad_publisher.publish(self.Velocidad)
                    self.frecuencia.sleep()
                print "Terminado"
            else:
                self.Velocidad.linear.x = 0
                self.Velocidad.angular.z = 0
                self.Velocidad_publisher.publish(self.Velocidad)
                self.frecuencia.sleep()

def rutina_seguidor():
    rospy.init_node('MovimientoSeguidor3', anonymous = True)
    Seguidor1 = MovimientoSeguidor()
    while True:
        if Seguidor1.SelectorRutina == 1:
            Seguidor1.EncontrarPunto()
        elif Seguidor1.SelectorRutina == 2:
            Seguidor1.EncontrarPuntoEnFormacion()


    


if __name__ == '__main__':
    try:
        rutina_seguidor()
    except rospy.ROSInterruptException:
        pass