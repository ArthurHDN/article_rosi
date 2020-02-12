#!/usr/bin/env python
#################################
# CODIGO DE CONTROLE PRINCIAPAL #
#################################
# Este no calcula os sinais de velocidade para o robo e tambem
# indica quando devem ser feitas as tarefas de toque e de escada

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

	# Construtor
	def __init__(self):
		# Posicao (x, y, theta)
		self.pos_x = 0.1
		self.pos_y = 0.1
		self.angle = 0.1
		# Posicao relativa do ponto a ser desviado (obstaculo)
		self.xd = 10
		self.yd = 10
		# Variavel de controle
		self.state = 0
		# Tratamento e conversao de imagem
		self.bridge = CvBridge()

		# Topicos que subscreve e publica
		self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)
		self.pub_cmd_vel = rospy.Publisher('/aai_rosi_cmd_vel', Twist, queue_size=1)
		#self.image_sub = rospy.Subscriber('/aai_depth_show',Image,self.callback_image)
		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(10)

		# Mensagem de inicializacao
		rospy.loginfo('Controle de alto nivel iniciado: Campos Vetorias Artificias')

		vel_msg = Twist()

		# Loop principal que manda as velocidades para o robo ate que ele chegue nas proximidades do ponto
		while not rospy.is_shutdown():
			
				
			#try:
			[V, W] = self.calc_vel(self.pos_x, self.pos_y, self.angle)

			vel_msg.linear.x = V
			vel_msg.angular.z = W
			#except:
				#vel_msg.linear.x = 0
				#vel_msg.angular.z = 0
				#print('velocidade zerada')

			self.pub_cmd_vel.publish(vel_msg)
			node_sleep_rate.sleep()

			
			# Este programa terminou sua rotina
			#vel_msg.linear.x = 0
			#vel_msg.angular.z = 0
			#self.pub_cmd_vel.publish(vel_msg)
			#foo() # :)
			#break

	def calc_vel(self, current_x, current_y, current_theta):
		#Curva 1 smooth square  param
		# c_x = -27
		# c_y = -0.35
		# r_x = 33 
		# r_y = 3.55
		# fi = ((self.pos_x - c_x)/r_x)**4 + ((self.pos_y - c_y)/r_y)**4 - 1
		# grad_fi = [(4/r_x)*(((self.pos_x - c_x)/r_x)**3) , (4/r_y)*(((self.pos_y - c_y)/r_y)**3)]
		# Beta_fi = [ -(4/r_y)*((self.pos_y - c_y)/r_y)**3 , (4/r_x)*((self.pos_x - c_x)/r_x)**3]

		fi = self.pos_x**4 + self.pos_y**4 - 1
		grad_fi = [4*(self.pos_x)**3 , 4*(self.pos_y)**3]
		Beta_fi = [-4*(self.pos_y)**3 , 4*(self.pos_x)**3]

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
		print('---------------------------------------')
		print('x,y:' + ' ' + str(self.pos_x) + ' ' + str(self.pos_y))
		print('G e H:' + ' ' + str(G) + ' ' + str(H))
		print('Convergir:' + ' ' + str(grad_fi[0]) + ' ' + str(grad_fi[1]))
		print('Circular:' + ' ' + str(Beta_fi[0]) + ' ' + str(Beta_fi[1]))
		print('Velocidade:' + ' ' + str(vel_x) + ' ' + str(vel_y))

		# Feedback Linearization
		V_forward = cos(current_theta) * vel_x + sin(current_theta) * vel_y
		W_angular = (-sin(current_theta) / self.d) * vel_x + (cos(current_theta) / self.d) * vel_y

		return (V_forward, W_angular)


	# Callback da posicao
	def callback_pose(self, data):

		c_x = -33
		c_y = -0.5
		r_x = 31
		r_y = 4

		q_x = data.orientation.x
		q_y = data.orientation.y
		q_z = data.orientation.z
		q_w = data.orientation.w
		# Orientacao de quaternios para angulos de Euler
		euler_angles = euler_from_quaternion([q_x, q_y, q_z, q_w])

		self.pos_x  = (data.position.x - c_x)/r_x
		self.pos_y = (data.position.y - c_y)/r_y
		self.angle = euler_angles[2] # Apenas o angulo de Euler no eixo z nos interessa


	# def callback_image(self,data):
	# 	# try except caso hajam erros
	# 	try:
	# 		cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
	# 		#cv_image = cv2.flip(cv_image, 1)
	# 	except CvBridgeError as e:
	# 		print(e)
	# 	depth_array = np.array(cv_image, dtype=np.float32)
	# 	cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
	# 	# #Para poder ver a imagem depth do kinect
	# 	#cv2.imshow("Kinect Depth", depth_array)
	# 	#cv2.waitKey(3)

	# 	prox = depth_array[0][0]
	# 	linha = 0
	# 	coluna = 640
	# 	#colunas = list()
	# 	for i in range(int(0.7*data.height)):
	# 		if min(depth_array[i]) < prox:
	# 			prox = min(depth_array[i])
	# 			linha = i

	# 	for i in range(data.width):
	# 		if abs(depth_array[linha][i] - prox) < 0.001:
	# 			#colunas.append(i)
	# 			if abs(310 - i) < abs(310 - coluna):
	# 				coluna = i

	# 	# Transformacao Homogenea para converter a posicao relativa em posicao absoluta
	# 	Kpx = 0.005
	# 	Kvalor = 5
	# 	xd = cos(self.angle)*(310-coluna)*Kpx - sin(self.angle)*prox*Kvalor
	# 	yd = sin(self.angle)*(310-coluna)*Kpx + cos(self.angle)*prox*Kvalor

	# 	self.xd = xd
	# 	self.yd = yd

# Mensagem de despedida e finalizacao
def foo():
	print('\n'*5)
	print('A execucao chegou ao fim.\n')
	print('A equipe AAI Robotics agradece pela oportunidade!')
	print('Foi desafiador para todos os tres membros do grupo.')
	print('Aprendemos muito com o desafio e estamos orgulhosos de termos chegado ate aqui.\n')
	print('Foi um prazer!\n')
	print('Atenciosamente,\n')
	print('Alvaro Rodrigues Araujo')
	print('Arthur Henrique Dias Nunes')
	print('Israel Filipe Silva Amaral')
	print('\nAAI Robotics - Universidade Federal de Minas Gerais')

# Funcao main
if __name__ == '__main__':

    rospy.init_node('calculo_vel', anonymous=True)

    try:
        node_obj = RosiCmdVelClass()
    except rospy.ROSInterruptException:
        pass
