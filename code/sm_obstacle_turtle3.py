#!/usr/bin/env python
""" ROS python pogramming with finite state machines to describe a robot's behaviors
    Vincent Hugel

    Seatech/SYSMER 2A Course
    free to use so long as the author and other contributers are credited.

	modifie par

	Albane Ordrenneau
	Adrien Mojika


"""
#colours = {
#	"default"    :    "\033[0m",
#	# style
#	"bold"       :    "\033[1m",
#	"underline"  :    "\033[4m",
#	"blink"      :    "\033[5m",
#	"reverse"    :    "\033[7m",
#	"concealed"  :    "\033[8m",
#	#couleur texte
#	"black"      :    "\033[30m",
#	"red"        :    "\033[31m",
#	"green"      :    "\033[32m",
#	"yellow"     :    "\033[33m",
#	"blue"       :    "\033[34m",
#	"magenta"    :    "\033[0m",
#	"cyan"       :    "\033[36m",
#	"white"      :    "\033[37m",
#	#couleur fond
#	"on_black"   :    "\033[40m",
#	"on_red"     :    "\033[41m",
#	"on_green"   :    "\033[42m",
#	"on_yellow"  :    "\033[43m",
#	"on_blue"    :    "\033[44m",
#	"on_magenta" :    "\033[45m",
#	"on_cyan"    :    "\033[46m",
#	"on_white"   :    "\033[47m" }

#############################################################################
# imports
#############################################################################
import rospy, time, sys, os, math
import numpy as np
from sensor_msgs.msg import Joy, LaserScan,Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import String, Bool, Float32MultiArray

from fsm import fsm
#!/usr/bin/env python
# -*- coding: utf-8 -*-

#############################################################################
# class RobotBehavior
#############################################################################
class RobotBehavior(object):
	#############################################################################
	# constructor, called at creation of instance
	#############################################################################
	def __init__(self, handle_pub, T):
	
		self.vector_odom = rospy.Publisher('/vector_odom', Float32MultiArray, queue_size = 1)
		self.vector_visp = rospy.Publisher('/vector_visp', Float32MultiArray, queue_size = 1)
		self.calib= rospy.Publisher('/calibration', Bool, queue_size = 1)
		self.pub1= rospy.Publisher('/pos_odom_init', Float32MultiArray, queue_size = 1)
		self.pub2= rospy.Publisher('/pos_odom_final', Float32MultiArray, queue_size = 1)
		self.pub3= rospy.Publisher('/pos_visp_init', Float32MultiArray, queue_size = 1)
		self.pub4= rospy.Publisher('/pos_visp_final', Float32MultiArray, queue_size = 1)
			
			
		self.bin_calibration_visp=False
		self.bin_calibration_odom=False
		self.valeur_odom_prise=True
		self.valeur_visp_prise=True
		
		self.twist = Twist()
		self.twist_real = Twist()
		self.vreal = 0.0 # longitudial velocity
		self.wreal = 0.0 # angular velocity
		#self.vmax = 0.188
		self.vmax = 0.17
		#self.wmax = 1.297
		self.wmax = self.vmax/0.135
		self.previous_signal = 0
		self.button_pressed = False
		self.joy_activated = False

		self.pub = handle_pub
		self.T = T

		self.PvL = 0.7
		self.entraxe=0.27
		self.dist_detection = 0.5
		self.r_rotation = 1

		self.ModeCarre=False
		self.ModeRond=False
		self.ModeTriangle=False
		self.ModeCroix=False
		
		self.obstacle_detecte=False

		self.message = True
		self.message_info=False
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Rond=False
		self.message_Triangle=False
		self.message_Croix=False
		self.message_ChangementDirection=False
		
		self.init_LidarCallback=True
		
		self.init_odom=True
		self.init_visp=True
		self.Mr0= np.zeros(6)
		self.rkMr0=np.zeros(6)
		
		self.cMm0=np.zeros((4,4))
		self.cMr0=np.zeros((4,4))
		self.rkMr0v=np.zeros(6)	

		#matrice changement de repere entre le QRcode et le centre de rotation du robot
		self.m0Mr0=np.zeros((4,4))
		self.m0Mr0[0,0]=1
		self.m0Mr0[1,1]=1
		self.m0Mr0[2,2]=1
		self.m0Mr0[0,3]=0.064
		self.m0Mr0[1,3]=0
		self.m0Mr0[2,3]=-(0.4335-0.025)
		#self.m0Mr0[2,3]=-0.12
		self.m0Mr0[3,3]=1
		#self.m0Mr0=np.eye(4,3)
		#print "m0Mr0"
		#print self.m0Mr0
		

		self.present_state="Start"
		self.next_state=[]

		# instance of fsm with source state, destination state, condition (transition), callback (defaut: None)
		self.fs = fsm([ ("Start","JoyControl", True),
			#transitions de base
			("JoyControl","AutonomousMode", self.check_JoyControl_To_AutonomousMode, self.DoAutonomousMode),
			("AutonomousMode","JoyControl", self.check_To_JoyControl, self.DoJoyControl),

			#transitions trajectoire predefinies
			("JoyControl","Carre", self.check_JoyControl_To_Carre, self.DoCarre),
			("JoyControl","Rond", self.check_JoyControl_To_Rond, self.DoRond),
			("JoyControl","Triangle", self.check_JoyControl_To_Triangle, self.DoTriangle),
			("JoyControl","Croix", self.check_JoyControl_To_Croix, self.DoCroix),
			("Carre","JoyControl", self.check_To_JoyControl, self.DoJoyControl),
			("Rond","JoyControl", self.check_To_JoyControl, self.DoJoyControl),
			("Triangle","JoyControl", self.check_To_JoyControl, self.DoJoyControl),
			("Croix","JoyControl", self.check_To_JoyControl, self.DoJoyControl),
			
			#transitions obstacle
			("AutonomousMode","ChangementDirection", self.check_AutonomousMode_To_ChangementDirection, self.DoChangementDirection),
			("ChangementDirection","AutonomousMode", self.check_ChangementDirection_To_AutonomousMode, self.DoAutonomousMode),	
			("ChangementDirection","JoyControl", self.check_To_JoyControl, self.DoJoyControl),		

			#transitions pour rester dans le meme etat
			("JoyControl","JoyControl", self.KeepJoyControl, self.DoJoyControl),
			("AutonomousMode","AutonomousMode", self.KeepAutonomousMode, self.DoAutonomousMode),
			("ChangementDirection","ChangementDirection", self.KeepChangementDirection, self.DoChangementDirection),
			("Carre","Carre", self.KeepCarre, self.DoCarre),
			("Rond","Rond", self.KeepRond, self.DoRond),
			("Triangle","Triangle", self.KeepTriangle, self.DoTriangle),
			("Croix","Croix", self.KeepCroix, self.DoCroix) ])


	#############################################################################
	# callback for Joystick feedback
	#############################################################################
	def Joystickcallback(self,data):
	    	#rospy.loginfo(rospy.get_name() + ": j'ai recu %f,%f", data.axes[1],data.axes[2])


		#########################################################################
		# Calcul for twist data
		#########################################################################

		#Jl joystick gauche : commande lineaire
		#Ja joystick droit  : commande angulaire
		Jl = data.axes[1]
		Ja = data.axes[3]

		sgnJl = np.sign(Jl)
		sgnJa = np.sign(Ja)

		Jl = abs(Jl)
		Ja = abs(Ja)

		#reglage pourcentage vitesse lineaire / angulaire
		#si PvL = 0.75 => 75% vitesse lineaire et 25% vitesse angulaire
		IncPvL=0.1
		if (data.buttons[1]) & (self.PvL <= (1-IncPvL) ):
			self.PvL+=IncPvL
		if (data.buttons[2]) & (self.PvL >=IncPvL):
			self.PvL-=IncPvL

		Pw = 1.0 - self.PvL

		delta_L =  1.0 - self.PvL
		delta_W = self.PvL

		#calcul de la commande lineraire et angulaire
		Cl = ( (1.0 -Ja) * delta_L + self.PvL) * Jl * self.vmax
		Ca = ( (1.0 -Jl) * delta_W + Pw) * Ja * self.vmax

		if self.message_info:
			print "Jl",Jl," \t Ja",Ja," \t signJl",sgnJl," \t signJa",sgnJa
			print "Cl",Cl," \t Ca",Ca
			

		#conversion en vitesse angulaire
		Ca = Ca/(self.entraxe/2.0)
		if self.message_info:
			print "Ca rad",Ca," \t self.PvL",self.PvL, "\n"


		#########################################################################

		self.twist.linear.x = Cl * sgnJl
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = Ca * sgnJa

		# for transition conditions of fsm
		if (not self.button_pressed):
			self.button_pressed = (self.previous_signal==0 and data.buttons[0]==1)
		self.previous_signal = data.buttons[0];

		self.joy_activated = (abs(data.axes[0])>0.001 or abs(data.axes[1])>0.001 or abs(data.axes[3])>0.001 or abs(data.axes[4])>0.001)

		#Transitions trajectoires predefinies
		if data.axes[6]>0.5:
			self.ModeCarre=True
		else:
			self.ModeCarre=False
		if data.axes[6]<-0.5:
			self.ModeRond=True
		else:
			self.ModeRond=False
		if data.axes[7]>0.5:
			self.ModeTriangle=True
		else:
			self.ModeTriangle=False
		if data.axes[7]<-0.5:
			self.ModeCroix=True
		else:
			self.ModeCroix=False



	#############################################################################
	# callback for IMU feedback
	#############################################################################
	def IMUcallback(self,data):
		x=data.orientation.x

	#############################################################################
	# callback for Lidar feedback
	#############################################################################
	def Lidarcallback(self,data):
		global fwd_angle_max,fwd_angle_min,fwd_dist_max,fwd_dist_min
		global fwd_max_debut,fwd_max_fin,fwd_min_debut,fwd_min_fin
		
		global degre_fin_espace_libre, degre_debut_espace_libre
		
		bin_esp_lib=True
		angle_min=data.angle_min
		angle_max=data.angle_max
		angle_inc=data.angle_increment
		time_inc=data.time_increment
		time=data.scan_time
		range_min=data.range_min
		range_max=data.range_max
		nb_obs=0
		EL=[]
		lenEL=[]
		distEL=[]
		
		if self.init_LidarCallback:	
			#print "camera ok"
			self.init_CameraCallback=False
			fwd_angle_max=math.atan((self.entraxe/2)/self.dist_detection)
			fwd_angle_min=math.atan((self.entraxe/2)/data.range_min)
			fwd_dist_max=math.sqrt(((self.entraxe/2)**2)+(self.dist_detection**2))
			fwd_dist_min=math.sqrt(((self.entraxe/2)**2)+(data.range_min**2))
			fwd_max_debut=int((len(data.ranges)/2)-(fwd_angle_max/data.angle_increment))
			fwd_max_fin=int((len(data.ranges)/2)+(fwd_angle_max/data.angle_increment))
			fwd_min_debut=int((len(data.ranges)/2)-(fwd_angle_min/data.angle_increment))
			fwd_min_fin=int((len(data.ranges)/2)+(fwd_angle_min/data.angle_increment))
			
		
		#print angle_min,angle_max,angle_inc
		#print time_inc,time,range_min,range_max
		for i in range(0,len(data.ranges)-1):
			if data.ranges[i] < self.dist_detection and data.ranges[i] > range_min and bin_esp_lib:
				bin_esp_lib=False
				degre_fin_espace_libre=i
				EL.append(i)
				nb_obs+=1
				#print "fin espace libre",degre_fin_espace_libre,data.ranges[i]
			if not(bin_esp_lib) and (data.ranges[i] > self.dist_detection or data.ranges[i] < range_min):
				bin_esp_lib=True
				degre_debut_espace_libre=i
				EL.append(i)
				#print "debut espace libre",degre_debut_espace_libre,data.ranges[i-1]
		#print len(EL), EL,nb_obs
				
		for i in range(1,len(EL),2):
			#print i
			if i==len(EL)-1 and len(EL)%2==0: #si le nombre de point de debut/fin obstacle est paire  et que je suis a la derniere valeur du tableau (il n'y a pas d'obstacle en face)
				#l'espace libre de fin se termine avec la 1ere valeur du tableau
				#print i, EL[i], EL[0]
				lenEL.append(360-EL[i]+EL[0])
				#print(lenEL[len(lenEL)-1]),data.ranges[EL[i]-1],data.ranges[EL[0]]
				distEL.append(math.sqrt(math.pow(data.ranges[EL[i]-1],2)+math.pow(data.ranges[EL[0]],2)-2*data.ranges[EL[i]-1]*data.ranges[EL[0]]*math.cos(lenEL[len(lenEL)-1])))
				if distEL[len(distEL)-1] > self.entraxe and EL[i] >= fwd_min_debut and EL[i] <= fwd_max_debut:
					print "pas d'obstacle en face"
					self.obstacle_detecte=False
				else:
					self.obstacle_detecte=True
			else:
				self.obstacle_detecte=True
				#print i , EL[i], EL[i+1]
				lenEL.append(EL[i+1]-EL[i])
				#print(lenEL[len(lenEL)-1]),data.ranges[EL[i]-1],data.ranges[EL[i+1]]
				distEL.append(math.sqrt(math.pow(data.ranges[EL[i]-1],2)+math.pow(data.ranges[EL[i+1]],2)-2*data.ranges[EL[i]-1]*data.ranges[EL[i+1]]*math.cos(lenEL[len(lenEL)-1])))
		
			
		#print lenEL,distEL
		
		#print "\n"
		
	#############################################################################
	# callback for Odom feedback
	#############################################################################
	def Odomcallback(self,data):
		global i
		rkMr0=Float32MultiArray
		tx=data.pose.pose.position.x
		ty=data.pose.pose.position.y
		tz=data.pose.pose.position.z

		rx=data.pose.pose.orientation.x
		ry=data.pose.pose.orientation.y
		rz=data.pose.pose.orientation.z

		#print "tx",tx,"\t ty",ty,"\t rz",rz


		#########################################################################
		# Matrices d'odometrie
		#########################################################################

		if self.init_odom == True:
			
			self.Mr0= np.array([tx,ty,tz,rx,ry,rz])

			self.init_odom = False

		else:

			self.rkMr0= Float32MultiArray(data=(np.array([tx,ty,tz,rx,ry,rz]) - self.Mr0))

			self.vector_odom.publish(self.rkMr0)
		
		#print "bin_calibration",self.bin_calibration_odom,"self.valeur_odom_prise",self.valeur_odom_prise
		if self.bin_calibration_odom and self.valeur_odom_prise:
			print "odom1"
			self.pub1.publish(self.rkMr0)
			self.valeur_odom_prise=False
			i=1	
		elif not(self.bin_calibration_odom) and not(self.valeur_odom_prise)and i==1:
			print "odom2"
			self.pub2.publish(self.rkMr0)
			i=0
			self.valeur_odom_prise=True

			
			"""
			print " ##########################################"
			print " Matrice origine \n", self.Mr0
			print "\n Matrice odom \n", np.array([tx,ty,tz,rx,ry,rz])
			print "\n Matrice finale \n", rkMr0
			"""


	#############################################################################
	# callback for Vispposition feedback
	#############################################################################
	def Visppositioncallback(self,data):
		global j
		tx=data.pose.position.x
		ty=data.pose.position.y
		tz=data.pose.position.z

		rx=data.pose.orientation.x
		ry=data.pose.orientation.y
		rz=data.pose.orientation.z
		w=data.pose.orientation.w
	
		#print "tx",tx,"\t ty",ty,"\t tz",tz,"\t rx",rx,"\t ry",ry,"\t rz",rz,"\t w",w


		#########################################################################
		# Matrices de verite terrain
		#########################################################################
		
		if self.init_visp == True:
			
			#fonction qui retourne la matrice cMm0
			self.cMm0=self.quat2matrix(tx,ty,tz,w,rx,ry,rz)

			self.cMr0=self.cMm0.dot(self.m0Mr0)

			self.init_visp = False
			
			print "1",tx,ty,tz,w,rx,ry,rz
			print "cMm0\n",self.cMm0 ,"\ncMr0\n", self.cMr0,"\nm0Mr0\n", self.m0Mr0

		else:
			
			#fonction qui retourne la matrice cMmk
			cMmk=self.quat2matrix(tx,ty,tz,w,rx,ry,rz)
		
			r0Mrkv=np.linalg.inv(self.cMr0).dot(cMmk.dot(self.m0Mr0))
			
			#fonction qui retourne translation et rotation en euler
			self.rkMr0v= Float32MultiArray(data=(self.matrix_to_euler(r0Mrkv)))


			#calcul de erreur entre self.rkMr0 et self.rkMr0v
			#print "\nMatrix odom \n",self.rkMr0
			#print "\nMatrix visp\n",self.rkMr0v
			self.rkMr0v.layout.data_offset=data.header.seq

			self.vector_visp.publish(self.rkMr0v)
			
		#print "bin_calibration",self.bin_calibration_visp,"self.valeur_visp_prise",self.valeur_visp_prise
		if self.bin_calibration_visp and self.valeur_visp_prise:
			print "visp1"
			self.pub3.publish(self.rkMr0v)
			self.valeur_visp_prise=False
			j=1	
		elif not(self.bin_calibration_visp) and not(self.valeur_visp_prise)and j==1:
			print "visp2"
			self.pub4.publish(self.rkMr0v)
			j=0
			self.valeur_visp_prise=True



	#############################################################################
	# smoothing velocity function to avoid brutal change of velocity
	#############################################################################
	def smooth_velocity(self):
		accmax = 0.01;
		accwmax = 0.05;
		vjoy = 0.0
		wjoy = 0.0
		vold = 0.0
		wold = 0.0

		#filter twist
		vjoy = self.twist.linear.x
		vold = self.vreal
		deltav_max = accmax / self.T

		#vreal
		if abs(vjoy - self.vreal) < deltav_max:
			self.vreal = vjoy
		else:
			sign_ = 1.0
			if (vjoy < self.vreal):
				sign_ = -1.0
			else:
				sign_ = 1.0
			self.vreal = vold + sign_ * deltav_max

		#saturation
		if (self.vreal > self.vmax):
			self.vreal = self.vmax
		elif (self.vreal < -self.vmax):
			self.vreal = -self.vmax

		#filter twist
		wjoy = self.twist.angular.z
		wold = self.wreal
		deltaw_max = accwmax / self.T

		#wreal
		if abs(wjoy - self.wreal) < deltaw_max:
			self.wreal = wjoy
		else:
			sign_ = 1.0
			if (wjoy < self.wreal):
				sign_ = -1.0
			else:
				sign_ = 1.0
			self.wreal = wold + sign_ * deltaw_max
		#saturation
		if (self.wreal > self.wmax):
			self.wreal = self.wmax
		elif (self.wreal < -self.wmax):
			self.wreal = -self.wmax

		self.twist_real.linear.x = self.vreal
		self.twist_real.angular.z = self.wreal
		
	

	#############################################################################
	# functions for fsm transitions
	#############################################################################
	#transitions de base
	def check_JoyControl_To_AutonomousMode(self,fss):
		return self.button_pressed

	#transitions securite retour mode manuel
	def check_To_JoyControl(self,fss):
		return self.joy_activated

	#transition vers les trajectoires predefinies
	def check_JoyControl_To_Carre(self,fss):
		return (self.ModeCarre)
	def check_JoyControl_To_Rond(self,fss):
		return (self.ModeRond)
	def check_JoyControl_To_Triangle(self,fss):
		return (self.ModeTriangle)
	def check_JoyControl_To_Croix(self,fss):
		return (self.ModeCroix)

	#transitions obstacle
	def check_AutonomousMode_To_ChangementDirection(self,fss):
		#return(self.obstacle_detecte)
		return False
	def check_ChangementDirection_To_AutonomousMode(self,fss):
		return(not(self.obstacle_detectee))
	
	#transition pour rester dans le meme etat
	def KeepJoyControl(self,fss):
		self.next_state=[]
		self.present_state="JoyControl"
		self.next_state.append(self.check_JoyControl_To_AutonomousMode(fss))
		self.next_state.append(self.check_JoyControl_To_Carre(fss))
		self.next_state.append(self.check_JoyControl_To_Rond(fss))
		self.next_state.append(self.check_JoyControl_To_Triangle(fss))
		self.next_state.append(self.check_JoyControl_To_Croix(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepJoyControl",self.check_JoyControl_To_AutonomousMode(fss)
			sys.stdout.write("\033[37m")
		if self.check_JoyControl_To_AutonomousMode(fss):
			return False
		elif self.check_JoyControl_To_Carre(fss):
			return False
		elif self.check_JoyControl_To_Rond(fss):
			return False
		elif self.check_JoyControl_To_Triangle(fss):
			return False
		else:
			return not(self.check_JoyControl_To_Croix(fss))

	def KeepAutonomousMode(self,fss):
		self.next_state=[]
		self.present_state="AutonomousMode"
		self.next_state.append(self.check_To_JoyControl(fss))
		self.next_state.append(self.check_AutonomousMode_To_ChangementDirection(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepAutonomousMode",self.check_To_JoyControl(fss),self.check_AutonomousMode_To_ChangementDirection(fss)
			sys.stdout.write("\033[37m")
		if self.check_To_JoyControl(fss):
			return False
		else:
			return (not(self.check_AutonomousMode_To_ChangementDirection(fss)))

	def KeepCarre(self,fss):
		self.next_state=[]
		self.present_state="Carre"
		self.next_state.append(self.check_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepCarre",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		if self.ModeCarre:
			self.message_Carre=False
		return (not(self.check_To_JoyControl(fss)))

	def KeepRond(self,fss):
		self.next_state=[]
		self.present_state="Rond"
		self.next_state.append(self.check_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepRond",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		if self.ModeRond:
			self.message_Rond=False
		return (not(self.check_To_JoyControl(fss)))

	def KeepTriangle(self,fss):
		self.next_state=[]
		self.present_state="Triangle"
		self.next_state.append(self.check_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepTriangle",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		if self.ModeTriangle:
			self.message_Triangle=False
		return (not(self.check_To_JoyControl(fss)))

	def KeepCroix(self,fss):
		self.next_state=[]
		self.present_state="Croix"
		self.next_state.append(self.check_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepCroix",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		if self.ModeCroix:
			self.message_Croix=False
		return (not(self.check_To_JoyControl(fss)))
		
	def KeepChangementDirection(self,fss):
		self.next_state=[]
		self.present_state="ChangementDirection"
		self.next_state.append(self.check_ChangementDirection_To_AutonomousMode(fss))
		self.next_state.append(self.check_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepChangementDirection",self.check_To_JoyControl(fss),self.check_ChangementDirection_To_AutonomousMode(fss)
			sys.stdout.write("\033[37m")
		if (self.check_To_JoyControl(fss)) :
			return False
		else:
			return not(self.check_ChangementDirection_To_AutonomousMode(fss))

	#############################################################################
	# functions for instructions inside states of fsm
	#############################################################################
	#fonctions de base
	def DoJoyControl(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Rond=False
		self.message_Triangle=False
		self.message_Croix=False
		self.message_ChangementDirection=False
		if (not(self.message_JoyControl) and self.message):
			print 'Mode Manuel'
			print '+--------------------------------------------------------------------------------+'
			print '| Joystick gauche 	> avant = avance                                         |'
			print '| 			> arriere = recule                                       |'
			print '| Joystick droit	> droit = sens horaire			 		 |'
			print '|		 	> gauche = sens anti horaire                             |'
			print '| Bouton A		> Mode Automatique                                       |'
			print '| Bouton B		> Augmente le ratio vitesse lineaire/angulaire           |'
			print '| Bouton X		> Diminue le ratio vitesse lineaire/angulaire            |'
			print '| Bouton Haut		> Mode Triangle						 |'
			print '| Bouton Bas		> Mode Croix						 |'
			print '| Bouton Gauche		> Mode Carre						 |'
			print '| Bouton Droit		> Mode Rond						 |'
			print '| Si autre Mode, les 4 manipulations des joysticks ramene au Mode Manuel  	 |'
			print '+--------------------------------------------------------------------------------+'
			
			print '\n\n\n'
			self.message_JoyControl=True
		self.button_pressed =  False;
		self.smooth_velocity()
  		self.pub.publish(self.twist_real)
  		self.calib.publish(False)
		sys.stdout.write("\033[37m")
		pass

	def DoAutonomousMode(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_Carre=False
		self.message_Rond=False
		self.message_Triangle=False
		self.message_Croix=False
		self.message_ChangementDirection=False
		if (not(self.message_AutonomousMode) and self.message):
			print 'Mode Automatique'
			self.message_AutonomousMode=True
		self.button_pressed =  False;
		go_fwd = Twist()
		go_fwd.linear.x = self.vmax/2.0;
		go_fwd.angular.z= 0;
		self.pub.publish(go_fwd)
		sys.stdout.write("\033[37m")
		pass

	def DoCarre(self,fss,value):
		global bin_avance, bin_tourne, etape

		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Rond=False
		self.message_Triangle=False
		self.message_Croix=False
		self.message_ChangementDirection=False
		if (not(self.message_Carre) and self.message):
			print 'Mode Carre'
			self.message_Carre=True

			bin_avance=False
			bin_tourne=False
			etape=0


		if (etape%2 == 0) and ( etape < 8 ):
			etape , bin_avance, bin_tourne = self.avance(1.0, bin_avance, bin_tourne)
			#print "avance"

		elif (etape%2 == 1) and (etape < 8):
			etape , bin_avance, bin_tourne = self.rotation(90, bin_avance, bin_tourne)
			#print "rotation"

		else:
			self.robot_stop()
			#print "stop"

		sys.stdout.write("\033[37m")
		pass

	def DoRond(self,fss,value):
		global g,d

		#Calcul commande trajectoire
		perimetre_cercle = self.r_rotation * 2.0 * math.pi
		vL = self.vmax
		deltaV = (vL*(self.r_rotation + (self.entraxe/2.0) ) / self.r_rotation) - vL

		k = self.vmax/(deltaV + vL)

		vL = k*vL
		wA = k*deltaV/(self.entraxe/2.0)

		temps_th = perimetre_cercle / vL

		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Triangle=False
		self.message_Croix=False
		self.message_ChangementDirection=False

		d=rospy.get_time()
		if (not(self.message_Rond) and self.message):
			print 'Mode Rond'
			print "avance"
			self.twist.linear.x = vL
			self.twist.linear.y = 0
			self.twist.linear.z = 0
			self.twist.angular.x = 0
			self.twist.angular.y = 0
			self.twist.angular.z = wA
			self.pub.publish(self.twist)

			self.message_Rond=True
			print "temps:",temps_th, "\t \t vL:",vL,"\t \t wA:",wA
			d=rospy.get_time()
			g=d+temps_th
		sys.stdout.write("\033[37m")

		if d>=g:
			self.twist.linear.x = 0
			self.twist.angular.z = 0

			self.pub.publish(self.twist)
			self.robot_stop()

			#print d,g, g-d

		pass

	def DoTriangle(self,fss,value):
		global bin_avance, bin_tourne, etape, angle1, angle2, angle3, distance1, distance2, distance3

		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Rond=False
		self.message_Croix=False
		self.message_ChangementDirection=False
		if (not(self.message_Triangle) and self.message):
			print 'Mode Triangle'
			self.message_Triangle=True

			#calcul distance et angle pour revenir sur ligne depart

			angle1 = 145-90
			angle2 = -100

			distance1 = 1.5

			angle3 = 180 - angle1 - angle2

			distance2 = abs( distance1 * ( math.cos(angle2) - (math.cos(angle1)*math.cos(angle3)) ) / (1-math.pow( math.cos(angle3) ,2)) )

			bin_avance=False
			bin_tourne=False
			etape=0
		#print "triangle", etape

		if (etape == 0):
			etape, bin_avance, bin_tourne = self.rotation(angle1+90, bin_avance, bin_tourne)
			#print "rotation"
			#print "etape", etape

		elif (etape == 1):
			etape, bin_avance, bin_tourne = self.avance(distance1, bin_avance, bin_tourne)
			#print "avance"

		elif(etape == 2):
			etape, bin_avance, bin_tourne = self.rotation(angle2, bin_avance, bin_tourne)
			#print "rotation"

		elif(etape == 3):
			etape, bin_avance, bin_tourne = self.avance(distance2, bin_avance, bin_tourne)
			#print "avance"

		elif(etape == 4):
			etape, bin_avance, bin_tourne = self.rotation(-(angle1+angle2)-90, bin_avance, bin_tourne)
			#print "rotation"

		else:
			self.robot_stop()
			#print "etape", etape
			#print " distance 2", distance2
			#print "stop"

		sys.stdout.write("\033[37m")
		pass

	def DoCroix(self,fss,value):
		global bin_avance, bin_tourne, etape
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Rond=False
		self.message_Triangle=False		
		self.message_ChangementDirection=False
		if (not(self.message_Croix) and self.message):
			print 'Mode Croix'
			self.message_Croix=True
			bin_avance=False
			bin_tourne=False
			etape=0
		sys.stdout.write("\033[37m")
		if etape == 0 :
			#print "avance"
			self.bin_calibration_visp=(True)
			self.bin_calibration_odom=(True)
			etape, bin_avance, bin_tourne = self.avance(1, bin_avance, bin_tourne)
		else:
			#print "stop"
			self.bin_calibration_visp=(False)
			self.bin_calibration_odom=(False)
			self.robot_stop()
		self.calib.publish(self.bin_calibration_visp)
		pass
	
	def DoChangementDirection(self,fss,value):
	 	sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Rond=False
		self.message_Triangle=False
		self.message_Croix=False
		if (not(self.message_ChangementDirection) and self.message):
			print 'Mode ChangementDirection'
			self.message_ChangementDirection=True
		sys.stdout.write("\033[37m")
		pass

	#############################################################################
	# move function
	#############################################################################

	def avance(self,distance,bin_avance,bin_tourne):

		global g
		#print "bin_avance: ", bin_avance, "bin_tourne: ", bin_tourne
		if (not(bin_avance)):
			sgn_dist = np.sign(distance)
			temps_th = abs(distance) / self.vmax

			self.twist.linear.x = self.vmax * sgn_dist
			self.twist.linear.y = 0
			self.twist.linear.z = 0
			self.twist.angular.x = 0
			self.twist.angular.y = 0
			self.twist.angular.z = 0
			self.pub.publish(self.twist)

			d=rospy.get_time()
			g=d+temps_th
			#print "avance 1"
			print "d: ",d,"\t g: ",g, "\t delta: ", g-d
			#print "vitesse: ", self.vmax * sgn_dist, "\n"

			bin_avance = True
			bin_tourne = False

		d=rospy.get_time()
		if d>=g:
			self.twist.linear.x = 0
			self.pub.publish(self.twist)
			#print "avance termine"
			#print "d: ",d,"\t g: ",g, "\t delta: ", g-d, "\n"
			rospy.sleep(1)
			return etape+1 , bin_avance, bin_tourne

		else:
			#print "avance encore"
			#print "d: ",d,"\t g: ",g, "\t delta: ", g-d, "\n"
			#print "delta: ", g-d
			return etape , bin_avance, bin_tourne

		pass



	def rotation(self,angle_d,bin_avance,bin_tourne):

		global g
		if (not(bin_tourne)):
			sgn_angle = np.sign(angle_d)
			angle_r = (abs(angle_d) * math.pi)/180.0
			temps_th = angle_r / self.wmax

			self.twist.linear.x = 0
			self.twist.linear.y = 0
			self.twist.linear.z = 0
			self.twist.angular.x = 0
			self.twist.angular.y = 0
			self.twist.angular.z = self.wmax * sgn_angle
			self.pub.publish(self.twist)

			d=rospy.get_time()
			g=d+temps_th

			bin_avance = False
			bin_tourne = True

		d=rospy.get_time()
		if d>=g:
			self.twist.angular.z = 0
			self.pub.publish(self.twist)
			rospy.sleep(2)
			return etape+1 , bin_avance, bin_tourne

		else:
			return etape , bin_avance, bin_tourne

		pass


	def robot_stop(self):

		self.twist.linear.x = 0
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = 0
		self.pub.publish(self.twist)
		pass
		
	###############################################################
	###############################################################
	
	
	def quat2matrix(self,tx,ty,tz,w,rx,ry,rz):
		r11=1.0-(2.0*ry*ry)-(2.0*rz*rz)	
		r12=(2.0*rx*ry)-(2.0*w*rz)
		r13=(2.0*rx*rz)+(2.0*w*ry)
		r21=(2.0*rx*ry)+(2.0*w*rz)
		r22=1.0-(2.0*rx*rx)-(2.0*rz*rz)
		r23=(2.0*ry*rz)-(2.0*w*rx)
		r31=(2.0*rx*rz)-(2.0*w*ry)
		r32=(2.0*ry*rz)+(2.0*w*rx)
		r33=1.0-(2.0*rx*rx)-(2.0*ry*ry)
		return(np.array([[r11,r12,r13,tx],[r21,r22,r23,ty],[r31,r32,r33,tz],[0.0,0.0,0.0,1.0]]))
		
	def matrix2odom(self,M):
		tx=M[0,3]
		ty=M[1,3]
		tz=M[2,3]
		rx=math.atan2(M[1,2],M[2,2])
		ry=math.atan2(-M[0,2],math.sqrt(M[0,0]*M[0,0]+M[0,1]*M[0,1]))
		rz=math.atan2(math.sin(rx)*M[2,0]-math.cos(rx)*M[1,0],math.cos(rx)*M[1,1]-math.sin(rx)*M[2,1])
		return np.array([tx,ty,tz,rx,ry,rz])
		
	def matrix_to_euler(self,R):
		sy=math.sqrt(R[0,0]*R[0,0]+R[1,0]*R[1,0])
		singular=sy<1e-6
		if not singular:
			rx=math.atan2(R[2,1],R[2,2])
			ry=math.atan2(-R[2,0],sy)
			rz=math.atan2(R[1,0],R[0,0])
		else:
			rx=math.atan2(-R[1,2],R[1,1])
			ry=math.atan2(-R[2,0],sy)
			rz=0
		tx=R[0,3]
		ty=R[1,3]
		tz=R[2,3]
		return np.array([tx,ty,tz,rx,ry,rz])

	"""def quaternion_to_euler(self,w, x, y, z):
		ysqr=y*y
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + ysqr)
		X = math.degrees(math.atan2(t0, t1))
	
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		Y = math.degrees(math.asin(t2))
	
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (ysqr + z * z)
		Z = math.degrees(math.atan2(t3, t4))
		
		return X, Y, Z
		
	def euler_to_quaternion(self,pitch,roll,yaw):
		cy=math.cos(yaw*0.5)
		sy=math.sin(yaw*0.5)
		cr=math.cos(roll*0.5)
		sr=math.sin(roll*0.5)
		cp=math.cos(pitch*0.5)
		sp=math.sin(pitch*0.5)
		
		W=cy*cr*cp+sy*sr*sp
		X=cy*sr*cp-sy*cr*sp
		Y=cy*cr*sp+sy*sr*cp
		Z=sy*cr*cp-cy*sr*sp
		
		return W, X, Y, Z
		
	def euler_to_matrix(self,tx,ty,tz,rx,ry,rz):
		R00=(math.cos(rz)*math.cos(rx))-(math.sin(rz)*math.cos(ry)*math.sin(rx))
		R01=(-math.cos(rz)*math.sin(rx))-(math.sin(rz)*math.cos(ry)*math.cos(rx))
		R02=(math.sin(rz)*math.cos(ry))
		R03=tx
		R10=(math.sin(rz)*math.cos(rx))+(math.cos(rz)*math.cos(ry)*math.sin(rx))
		R11=(-math.sin(rz)*math.sin(rx))+(math.cos(rz)*math.cos(ry)*math.sin(rx))
		R12=(-math.cos(rz)*math.sin(ry))
		R13=ty
		R20=(math.sin(ry)*math.sin(rx))
		
		R21=(math.sin(ry)*math.cos(rx))
		R22=(math.cos(ry))
		R23=tz
		return(np.array([[R00,R01,R02,R03],[R10,R11,R12,R13],[R20,R21,R22,R23],[0.0,0.0,0.0,1.0]]))
	
	"""
		

#############################################################################
# main function
#############################################################################
if __name__ == '__main__':
	try:
		time.sleep(3)
		sys.stdout.write("\033[34m")
		print '\n\n\n\n---------------Bienvenue sur la commande d un TurtleBot 3---------------'
		rospy.init_node('joy4ctrl')
		print 'Vous travaillez sous Gazebo ou sur le robot reel'
		pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		#pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
		# turtlesim
		sys.stdout.write("\033[37m")
		Hz = 10
    		rate = rospy.Rate(Hz) # 10hz
		T = 1.0/Hz
		MyRobot = RobotBehavior(pub,T)
        	rospy.Subscriber("joy", Joy, MyRobot.Joystickcallback)
		rospy.Subscriber("imu",Imu, MyRobot.IMUcallback)
		rospy.Subscriber("scan",LaserScan, MyRobot.Lidarcallback)
		rospy.Subscriber("/odom",Odometry, MyRobot.Odomcallback)
		rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped, MyRobot.Visppositioncallback)
		MyRobot.fs.start("Start")
		# loop at rate Hz
    		while not rospy.is_shutdown():
    			sys.stdout.write("\033[0m")
    			ret = MyRobot.fs.event(MyRobot.next_state)
	        	rate.sleep()

	except rospy.ROSInterruptException:
		print '-------------------------- Programme terminee --------------------------'
		print '------------------------------ A  Bientot ------------------------------'

        	pass


