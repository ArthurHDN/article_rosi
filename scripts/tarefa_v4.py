#!/usr/bin/env python
#################################
# CODIGO DE CONTROLE PRINCIAPAL #
#################################
# Este no calcula os sinais de velocidade para o robo

from __future__ import print_function # apenas para imprimir os erros, caso existam e debuga-los
# Pacotes
import rospy
import roslib
import sys
import cv2
import numpy as np
# Tipos de mensagens utilizadas
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from sensor_msgs.msg import Image
# Ferramentas para o processamento dos dados
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, sqrt, atan, pi

# Classe que contem os metodos necessarios para o programa
class RosiCmdVelClass():

	# Constantes de controle
	Kp = 0.5 # Ganho proporcional
	d = 0.2 # Distancia entre o centro de massa e o ponto a ser controlado por Feedback Linearization
	Err = 0.5 # Erro admitido de distancia ao ponto

	curve_type = 2

	# Construtor
	def __init__(self):
		# Posicao (x, y, theta)
		self.pos_x = 0.1
		self.pos_y = 0.1
		self.angle = 0.1
		# Topicos que subscreve e publica
		self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)
		self.pub_cmd_vel = rospy.Publisher('/aai_rosi_cmd_vel', Twist, queue_size=1)
		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(10)

		# Mensagem de inicializacao
		rospy.loginfo('Controle de alto nivel iniciado: Campos Vetorias Artificias')

		vel_msg = Twist()

		# Loop principal que manda as velocidades para o robo ate que ele chegue nas proximidades do ponto
		while not rospy.is_shutdown():

			[V, W] = self.calc_vel()
			vel_msg.linear.x = V
			vel_msg.angular.z = W

			self.pub_cmd_vel.publish(vel_msg)
			node_sleep_rate.sleep()

	def calc_vel(self):
		if self.curve_type == 1:
			# Smooth Square
			fi = self.pos_x**4 + self.pos_y**4 - 1
			grad_fi = [4*(self.pos_x)**3 , 4*(self.pos_y)**3]
			Beta_fi = [-4*(self.pos_y)**3 , 4*(self.pos_x)**3]
		elif self.curve_type == 2:
			# Race Track
			if self.pos_x <= 1 and self.pos_x >= -1 and self.pos_y >= 0:
				fi = self.pos_y-1
				grad_fi = [ 0 , 1]
				Beta_fi = [-1 , 0]
			elif self.pos_x <= 1 and self.pos_x >= -1 and self.pos_y < 0:
				fi = -self.pos_y-1
				grad_fi = [0 ,-1]
				Beta_fi = [1 , 0]
			elif self.pos_x < -1:
				fi = (self.pos_x+1)**2 + self.pos_y**2 -1
				grad_fi = [ 2*(self.pos_x+1) , 2*self.pos_y]
				Beta_fi = [-2*self.pos_y , 2*(self.pos_x+1)]
			elif self.pos_x > 1:
				fi = (self.pos_x-1)**2 + self.pos_y**2 -1
				grad_fi = [ 2*(self.pos_x-1) , 2*self.pos_y]
				Beta_fi = [-2*self.pos_y , 2*(self.pos_x-1)]
			else:
				print('x,y:' + ' ' + str(self.pos_x) + ' ' + str(self.pos_y))
				print('Nao calculou')
				return (0,0)
		G = -2/pi * atan(fi)
		H = sqrt(1 - G**2)
		
		u = G*grad_fi[0] + H*Beta_fi[0]
		v = G*grad_fi[1] + H*Beta_fi[1]

		norm_grad = sqrt(grad_fi[0]**2 + grad_fi[1]**2)
		try: 
			vel_x = u/(2*norm_grad)
			vel_y = v/(2*norm_grad)
		except ZeroDivisionError:
			print('norma grad = 0')
			vel_x = 0
			vel_y = 0
		# print('---------------------------------------')
		# print('x,y:' + ' ' + str(self.pos_x) + ' ' + str(self.pos_y))
		# print('G e H:' + ' ' + str(G) + ' ' + str(H))
		# print('Convergir:' + ' ' + str(grad_fi[0]) + ' ' + str(grad_fi[1]))
		# print('Circular:' + ' ' + str(Beta_fi[0]) + ' ' + str(Beta_fi[1]))
		# print('Velocidade:' + ' ' + str(vel_x) + ' ' + str(vel_y))

		# Feedback Linearization
		V_forward = cos(self.angle) * vel_x + sin(self.angle) * vel_y
		W_angular = (-sin(self.angle) / self.d) * vel_x + (cos(self.angle) / self.d) * vel_y

		return (V_forward, W_angular)

	# Callback da posicao
	def callback_pose(self, data):
		if self.curve_type == 1:
			c_x = -27
			c_y = -0.5
			r_x = 34
			r_y = 4
		elif self.curve_type == 2:
			c_x = -29.5 
			c_y = -0.1
			r_x = 36
			r_y = 3.5

			r_x = r_x/2

		q_x = data.orientation.x
		q_y = data.orientation.y
		q_z = data.orientation.z
		q_w = data.orientation.w
		# Orientacao de quaternios para angulos de Euler
		euler_angles = euler_from_quaternion([q_x, q_y, q_z, q_w])

		self.pos_x  = (data.position.x - c_x)/r_x
		self.pos_y = (data.position.y - c_y)/r_y
		self.angle = euler_angles[2] # Apenas o angulo de Euler no eixo z nos interessa

# Funcao main
if __name__ == '__main__':

    rospy.init_node('calculo_vel', anonymous=True)

    try:
        node_obj = RosiCmdVelClass()
    except rospy.ROSInterruptException:
        pass
