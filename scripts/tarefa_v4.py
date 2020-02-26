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
import sensor_msgs.point_cloud2 as pc2
# Tipos de mensagens utilizadas
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
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

	curve_type = 2 # Tipo da curva

	r_min = 2.6 # Parametros da curva
	r_max = 8

	disc_pontos = list() # Pontos de descontinuidade
	rp_disc_pontos = list()

	# Construtor
	def __init__(self):
		# Posicao (x, y, theta)
		self.pos_x = 0.1
		self.pos_y = 0.1
		self.angle = 0.1

		# Parametros de variacao no tempo
		self.time = 0
		self.t0 = 0
		self.a = 0; rospy.set_param('a', 0)
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
		self.sub_disc = rospy.Subscriber('/aai_disc_cloud', PointCloud2, self.callback_disc)
		self.pub_cmd_vel = rospy.Publisher('/aai_rosi_cmd_vel', Twist, queue_size=1)
		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(freq)

		# Mensagem de inicializacao
		rospy.loginfo('Controle de alto nivel iniciado: Campos Vetorias Artificias')

		vel_msg = Twist()

		# Escrita do txt com os estados do robo e da curva
		THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
		my_file_path = os.path.join(THIS_FOLDER, '../text/myfile.txt')
		write_handle = open(my_file_path, 'w')
		write_handle.write('')
		write_handle.close()

		# Loop principal que manda as velocidades para o robo ate que ele chegue nas proximidades do ponto
		while not rospy.is_shutdown():

			self.calcula_a() # Calcula a variacao do tempo

			self.verifica_a() # Verifica

			[V, W, vx, vy] = self.calc_vel() # Calcula a velocidade
			vel_msg.linear.x = V
			vel_msg.angular.z = W

			# Publicacao
			self.pub_cmd_vel.publish(vel_msg)
			node_sleep_rate.sleep()

			# Escrita do arquivo
			with open(my_file_path, 'a') as write_handle:
			  	msg = str(self.time) + '\t' + str(self.pos_x) + '\t' + str(self.pos_y) + '\t' + str(self.angle) + '\t' + str(vx) + '\t' + str(vy) + '\t' + str(V) + '\t' + str(W) + '\t' + str(self.c_x) + '\t' + str(self.c_y) + '\t' + str(self.r_x) + '\t' + str(self.r_y) + '\t' + str(self.a) + '\n'
			  	write_handle.write(msg)

	# Verifica variacao de a e limita os raios
	def verifica_a(self):
		try:
			self.a = rospy.get_param('a')
		except KeyError:
			return
		if self.a != self.a_ant:
			self.t0 = self.time
			self.b = self.r_y
			self.a_ant = self.a
			#print('Valor de a atualizado')
		if self.r_y <= self.r_min:
			#print('Valor de ry limitado inferiormente')
			rospy.set_param('a', 0)
			self.r_y = self.r_min + 0.1
			self.b = self.r_y
		if self.r_y >= self.r_max:
			#print('Valor de ry limitado superiormente')
			rospy.set_param('a', 0)
			self.r_y = self.r_max - 0.1
			self.b = self.r_y

	# Calcula a variacao no tempo -- MELHORAR
	def calcula_a(self):
		L = 0.8
		Ka = 2
		if len(self.disc_pontos) == 0:
			if (self.r_min - self.r_y)**2 > 0.01:
				if self.r_min < self.r_y:
					print('Caso 0.1.1')
					a = -0.3
				else:
					print('Caso 0.1.2')
					a = 0.3
			else:
				print('Caso 0.2')
				a = 0.0
			rospy.set_param('a', a)
			return

		else:
			p_dir = self.disc_pontos[0]
			p_esq = p_dir
			for p in self.disc_pontos:
				if (p[1] - self.c_y)**2 < (p_esq[1] - self.c_y)**2:
					p_esq = p
				if (p[1] - self.c_y)**2 > (p_dir[1]- self.c_y)**2:
					p_dir = p

			print(p_esq)
			print(p_dir)

			if abs(p_esq[1] - self.c_y) > abs(self.r_min  - self.c_y) + L/2:
				if (self.r_min - self.r_y)**2 > 0.1**2:
					if self.r_min < self.r_y:
						t_estrela = abs(p_esq[0] - self.pos_x)/0.5
						print('Caso 1.1.1')
						a = -Ka*float( ( sqrt( (p_esq[1] - self.pos_y)**2 ) + L/2)/t_estrela)
						if a < -0.5:
							a = -0.5
					else:
						print('Caso 1.1.2')
						a = 0.3
				else:
					print('Caso 1.1.3')
					a = 0.0
				rospy.set_param('a', a)
				return
			elif abs(p_dir[1] - self.c_y) < abs(self.pos_y - self.c_y) - L:
				print('Caso 2')
				a = 0.0
				rospy.set_param('a', a)
				return
			else:
				print('Caso 3') 
				if p_esq[0] < p_dir[0]:
					x = p_esq[0]
				else:
					x = p_dir[0]
				t_estrela = abs(x - self.pos_x)/0.5
				a = Ka*float( ( sqrt( (p_dir[1] - self.pos_y)**2 ) + L/2)/t_estrela)
				if a > 0.5:
					a = 0.5
				rospy.set_param('a', a)
				return

	# Calcula a velocidade por Campos Vetoriais Artificiais
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
		print('np =', len(self.disc_pontos))

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

	# Robot to World frame points
	def world_frame(self, rp):
		rf = np.array([ [rp[0]], [rp[1]], [1] ])
		H = np.array([ [cos(self.angle), -sin(self.angle), self.pos_x], [sin(self.angle), cos(self.angle), self.pos_y], [0,0,1] ])
		wf = H.dot(rf)
		wp = (wf[0], wf[1])
		return wp

	def inTC(self, p):
		x = p[0]
		y = p[1]
		if self.pos_y >= 0:
			if y <= 2: # and y >= -2:# and x <= -1.7 and x >= -50 :
				return True
			else:
				return False
		elif self.pos_y < 0:
			if y >= -2: # and y >= -2:# and x <= -1.7 and x >= -50 :
				return True
			else:
				return False

	# Callback pontos
	def callback_disc(self, data):
		pontos = pc2.read_points_list(data, field_names = ("x", "y", "z"), skip_nans=True)
		self.disc_pontos = list()
		self.rp_disc_pontos = list()
		for p in pontos:
			rp = (p[0], p[1])
			wp = self.world_frame(rp) 
			if True: #not self.inTC(wp):
				if self.pos_y >= 0:
					if self.pos_x > wp[0] - 0.1:
						self.disc_pontos.append(wp)
						self.rp_disc_pontos.append(rp)
				else:
					if self.pos_x < wp[0] + 0.1:
						self.disc_pontos.append(wp)
						self.rp_disc_pontos.append(rp)


	# Callback do tempo
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
		if self.pos_x < -42:
			self.r_min = 3.5
		else:
			self.r_min = 2.6 

# Funcao main
if __name__ == '__main__':

    rospy.init_node('calculo_vel', anonymous=True)

    try:
        node_obj = RosiCmdVelClass()
    except rospy.ROSInterruptException:
        pass
