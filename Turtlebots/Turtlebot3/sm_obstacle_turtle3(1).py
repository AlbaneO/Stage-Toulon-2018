#!/usr/bin/env python
""" ROS python pogramming with finite state machines to describe a robot's behaviors
    Vincent Hugel

    Seatech/SYSMER 2A Course
    free to use so long as the author and other contributers are credited.

	modifie par

	Albane Ordrenneau
	

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
from std_msgs.msg import String

from geometry_msgs.msg import Twist
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
		self.twist = Twist()
		self.twist_real = Twist()
		self.vreal = 0.0 # longitudial velocity
		self.wreal = 0.0 # angular velocity
		self.vmax = 0.188 
		self.wmax = 1.297 
		self.previous_signal = 0
		self.button_pressed = False
		self.joy_activated = False

		self.pub = handle_pub
		self.T = T
		
		self.PvL=0.7
		self.entraxe=0.29
		
		self.ModeCarre=False
		self.ModeRond=False
		self.ModeTriangle=False
		self.ModeCroix=False
					
		self.message = True
		self.message_info=False	
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Rond=False
		self.message_Triangle=False		
		self.message_Croix=False
		
				
		self.present_state="Start"
		self.next_state=[]
		
		# instance of fsm with source state, destination state, condition (transition), callback (defaut: None)
		self.fs = fsm([ ("Start","JoyControl", True ),
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

			#transitions pour rester dans le meme etat
			("JoyControl","JoyControl", self.KeepJoyControl, self.DoJoyControl),
			("AutonomousMode","AutonomousMode", self.KeepAutonomousMode, self.DoAutonomousMode),
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
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepAutonomousMode",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		return (not(self.check_To_JoyControl(fss)))
		
	def KeepCarre(self,fss):
		self.next_state=[]
		self.present_state="Carre"
		self.next_state.append(self.check_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepCarre",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		return (not(self.check_To_JoyControl(fss)))
		
	def KeepRond(self,fss):
		self.next_state=[]
		self.present_state="Rond"
		self.next_state.append(self.check_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepRond",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		return (not(self.check_To_JoyControl(fss)))
		
	def KeepTriangle(self,fss):
		self.next_state=[]
		self.present_state="Triangle"
		self.next_state.append(self.check_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepTriangle",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		return (not(self.check_To_JoyControl(fss)))
		
	def KeepCroix(self,fss):
		self.next_state=[]
		self.present_state="Croix"
		self.next_state.append(self.check_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepCroix",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		return (not(self.check_To_JoyControl(fss)))
		
	#############################################################################
	# functions for instructions inside states of fsm
	#############################################################################
	#fonctions de base	
	def DoJoyControl(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_AutonomousMode=False
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
			self.message_JoyControl=True
		self.button_pressed =  False;
		self.smooth_velocity()
  		self.pub.publish(self.twist_real)
		sys.stdout.write("\033[37m")
		pass

	def DoAutonomousMode(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		if (not(self.message_AutonomousMode) and self.message):
			print 'Mode Automatique'
			self.message_AutonomousMode=True
		self.button_pressed =  False;
		go_fwd = Twist()
		go_fwd.angular.z= 0;
		go_fwd.linear.x = self.vmax/2.0;
	  	self.pub.publish(go_fwd)
		sys.stdout.write("\033[37m")
		pass

	def DoCarre(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False	
		self.message_Rond=False
		self.message_Triangle=False
		self.message_Croix=False
		if (not(self.message_Carre) and self.message):
			print 'Mode Carre'
			self.message_Carre=True
		sys.stdout.write("\033[37m")
		pass
		
	def DoRond(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Triangle=False
		self.message_Croix=False
		if (not(self.message_Rond) and self.message):
			print 'Mode Rond'
			self.message_Rond=True
		sys.stdout.write("\033[37m")
		pass
		
	def DoTriangle(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Rond=False
		self.message_Croix=False
		if (not(self.message_Triangle) and self.message):
			print 'Mode Triangle'
			self.message_Triangle=True
		sys.stdout.write("\033[37m")
		pass
		
	def DoCroix(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Carre=False
		self.message_Rond=False
		self.message_Triangle=False
		if (not(self.message_Croix) and self.message):
			print 'Mode Croix'
			self.message_Croix=True
		sys.stdout.write("\033[37m")
		pass


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
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)	
		# turtlesim
		sys.stdout.write("\033[37m")
		Hz = 10
    		rate = rospy.Rate(Hz) # 10hz
		T = 1.0/Hz
		MyRobot = RobotBehavior(pub,T);
        	rospy.Subscriber("joy", Joy, MyRobot.Joystickcallback)
		rospy.Subscriber("/imu",Imu, MyRobot.IMUcallback)	
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
        	
        	
