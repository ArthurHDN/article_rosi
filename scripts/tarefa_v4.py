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
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
# Ferramentas para o processamento dos dados
#from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, sqrt, atan, pi
# path para escrever arquivos txt
import os 

# Classe que contem os metodos necessarios para o programa
class RosiCmdVelClass():

	# Constantes de controle
	Kp = 0.5 # Ganho proporcional
	d = 0.1 # Distancia entre o centro de massa e o ponto a ser controlado por Feedback Linearization
	Err = 0.5 # Erro admitido de distancia ao ponto

	curve_type = 2

	# Construtor
	def __init__(self):
		# Posicao (x, y, theta)
		self.pos_x = 0.1
		self.pos_y = 0.1
		self.angle = 0.1

		# Parametros de variacao no tempo
		self.time = 0
		self.t0 = 0
		self.a = 0
		self.b = 3.5
		self.a_ant = self.a

		# Parametros curva
		self.c_x = -25 
		self.c_y = -0.1
		self.r_x = 34
		self.r_y = 3.5

		freq = 10.0

		# Topicos que subscreve e publica
		self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)
		self.sub_time = rospy.Subscriber('/simulation/time', Float32, self.callback_time)
		self.pub_cmd_vel = rospy.Publisher('/aai_rosi_cmd_vel', Twist, queue_size=1)
		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(freq)

		# Mensagem de inicializacao
		rospy.loginfo('Controle de alto nivel iniciado: Campos Vetorias Artificias')

		vel_msg = Twist()

		THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
		my_file_path = os.path.join(THIS_FOLDER, '../text/myfile.txt')
		write_handle = open(my_file_path, 'w')
		write_handle.write('')
		write_handle.close()

		# Loop principal que manda as velocidades para o robo ate que ele chegue nas proximidades do ponto
		while not rospy.is_shutdown():

			self.verifica_a()

			[V, W, vx, vy] = self.calc_vel()
			vel_msg.linear.x = V
			vel_msg.angular.z = W

			self.pub_cmd_vel.publish(vel_msg)
			node_sleep_rate.sleep()
			with open(my_file_path, 'a') as write_handle:
			  	msg = str(self.time) + '\t' + str(self.pos_x) + '\t' + str(self.pos_y) + '\t' + str(self.angle) + '\t' + str(vx) + '\t' + str(vy) + '\t' + str(V) + '\t' + str(W) + '\t' + str(self.c_x) + '\t' + str(self.c_y) + '\t' + str(self.r_x) + '\t' + str(self.r_y) + '\t' + str(self.a) + '\n'
			  	write_handle.write(msg)

	def verifica_a(self):
		try:
			self.a = rospy.get_param('a')
		except KeyError:
			return
		if self.a != self.a_ant:
			self.t0 = self.time
			self.b = self.r_y
			self.a_ant = self.a
			print('Valor de a atualizado')


	def calc_vel(self):

		self.r_y = self.a*(self.time - self.t0) + self.b

		c_x = self.c_x
		c_y = self.c_y
		r_x = self.r_x / 2
		r_y = self.r_y

		x = (self.pos_x - c_x)/r_x
		y = (self.pos_y - c_y)/r_y

		print('t =', self.time)
		print('t0 =', self.t0)
		print('a =', self.a)
		print('r =', self.r_y)

		if self.curve_type == 1:
			# Smooth Square
			fi = x**4 + y**4 - 1
			grad_fi = [4*(x)**3 , 4*(y)**3]
			Beta_fi = [-4*(y)**3 , 4*(x)**3]
		elif self.curve_type == 2:
			# Race Track
			if x <= 1 and x >= -1 and y >= 0:
				fi = y-1
				grad_fi = [ 0 , 1]
				Beta_fi = [-1 , 0]
				dFdy = 1
			elif x <= 1 and x >= -1 and y < 0:
				fi = -y-1
				grad_fi = [0 ,-1]
				Beta_fi = [1 , 0]
				dFdy = -1
			elif x < -1:
				fi = (x+1)**2 + y**2 -1
				grad_fi = [ 2*(x+1) , 2*y]
				Beta_fi = [-2*y , 2*(x+1)]
				dFdy = 2*y
			elif x > 1:
				fi = (x-1)**2 + y**2 -1
				grad_fi = [ 2*(x-1) , 2*y]
				Beta_fi = [-2*y , 2*(x-1)]
				dFdy = 2*y

		G = -2/pi * atan(8*fi)
		H = sqrt(1 - G**2)

		norm_grad = 2*sqrt(grad_fi[0]**2 + grad_fi[1]**2)

		dt = self.a * (y/r_y) * dFdy 
		
		P = [-(dt*grad_fi[0])/norm_grad , (dt*grad_fi[1])/norm_grad ]

		u = G*grad_fi[0] + H*Beta_fi[0] + P[0]
		v = G*grad_fi[1] + H*Beta_fi[1] + P[1]
		
		try: 
			vel_x = u/norm_grad
			vel_y = v/norm_grad
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

		return (V_forward, W_angular, vel_x, vel_y)

	def callback_time(self, data):
		self.time = data.data

	# Callback da posicao
	def callback_pose(self, data):

		q_x = data.orientation.x
		q_y = data.orientation.y
		q_z = data.orientation.z
		q_w = data.orientation.w
		# Orientacao de quaternios para angulos de Euler
		euler_angles = euler_from_quaternion([q_x, q_y, q_z, q_w])

		self.pos_x  = data.position.x
		self.pos_y = data.position.y
		self.angle = euler_angles[2] # Apenas o angulo de Euler no eixo z nos interessa

# Funcao main
if __name__ == '__main__':

    rospy.init_node('calculo_vel', anonymous=True)

    try:
        node_obj = RosiCmdVelClass()
    except rospy.ROSInterruptException:
        pass
