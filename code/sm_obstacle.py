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
from sensor_msgs.msg import Joy, LaserScan,Imu
from kobuki_msgs.msg import BumperEvent, WheelDropEvent
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
		self.diameter= 0.4#0.3515 #metres 
		self.twist = Twist()
		self.twist_real = Twist()
		self.vreal = 0.0 # longitudial velocity
		self.wreal = 0.0 # angular velocity
		self.vmax = 0.3 #1.5
		self.wmax = 2.0 #4.0
		self.previous_signal = 0
		self.button_pressed = False
		self.joy_activated = False

		self.bumper_activated = False #un bumper est actionner
		self.bumper_center = False #quel bumper est activer
		self.bumper_right = False
		self.bumper_left = False
		
		self.orientation = False
		self.rotate_over=False
		self.rotate_is_stopped= False
		self.compteur = 0
		
		self.wheel_on_ground= True #roue sur le sol

		self.pub = handle_pub
		self.T = T
		
		self.init_CameraCallback=True
		self.init_IMUCallback=True
		self.dist_detection=1.0 #distance a partir de laquel on dit qu un obstacle est trop pres
		self.obstacle= 0
		self.lambda_rotate=0.0
			
		self.message = True
		self.message_info=False	
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Retreat=False
		self.message_Rotate=False
		self.message_Stop=False
		self.message_ChangeDirection=False
		self.message_OnGround=False
		
		self.present_state="Start"
		self.next_state=[]
		
		# instance of fsm with source state, destination state, condition (transition), callback (defaut: None)
		self.fs = fsm([ ("Start","JoyControl", True ),
			#transitions de base
			("JoyControl","AutonomousMode", self.check_JoyControl_To_AutonomousMode, self.DoAutonomousMode),
			("AutonomousMode","JoyControl", self.check_To_JoyControl, self.DoJoyControl),

			#transitions pour l arret bumper
			("AutonomousMode","Retreat",self.check_AutonomousMode_To_Retreat,self.DoRetreat),
			("Retreat","Rotate",self.check_Retreat_To_Rotate,self.DoRotate),
			("Rotate","Stop",self.check_Rotate_To_Stop,self.DoStop),
			("Stop","AutonomousMode",self.check_Stop_To_AutonomousMode,self.DoAutonomousMode),
			
			#transitions securite retour mode manuel
			("Retreat","JoyControl",self.check_To_JoyControl,self.DoJoyControl),
			("Rotate","JoyControl",self.check_To_JoyControl,self.DoJoyControl),
			("Stop","JoyControl",self.check_To_JoyControl,self.DoJoyControl),
			
			#transitions securite si absence contact roue sol
			("JoyControl","OnGround",self.check_To_OnGround,self.DoOnGround),
			("AutonomousMode","OnGround",self.check_To_OnGround,self.DoOnGround),
			("Retreat","OnGround",self.check_To_OnGround,self.DoOnGround),
			("Rotate","OnGround",self.check_To_OnGround,self.DoOnGround),
			("Stop","OnGround",self.check_To_OnGround,self.DoOnGround),
			("OnGround","JoyControl",self.check_OnGround_To_JoyControl,self.DoJoyControl),

			#transitions kinect
			("AutonomousMode","ChangeDirection",self.check_AutonomousMode_To_ChangeDirection,self.DoChangeDirection),
			("ChangeDirection","AutonomousMode",self.check_ChangeDirection_To_AutonomousMode,self.DoAutonomousMode),
			("ChangeDirection","Retreat",self.check_ChangeDirection_To_Retreat,self.DoRetreat),
			("ChangeDirection","JoyControl",self.check_To_JoyControl,self.DoJoyControl),
			("ChangeDirection","OnGround",self.check_To_OnGround,self.DoOnGround),
			

			#transitions pour rester dans le meme etat
			("JoyControl","JoyControl", self.KeepJoyControl, self.DoJoyControl),
			("AutonomousMode","AutonomousMode", self.KeepAutonomousMode, self.DoAutonomousMode),
			("Retreat","Retreat",self.KeepRetreat,self.DoRetreat),			
			("Rotate","Rotate",self.KeepRotate,self.DoRotate),
			("Stop","Stop",self.KeepStop,self.DoStop),
			("ChangeDirection","ChangeDirection",self.KeepChangeDirection,self.DoChangeDirection),
			("OnGround","OnGround",self.KeepOnGround,self.DoOnGround) ])


	#############################################################################
	# callback for Joystick feedback
	#############################################################################
	def Joystickcallback(self,data):
	    	#rospy.loginfo(rospy.get_name() + ": j'ai recu %f,%f", data.axes[1],data.axes[2])
		self.twist.linear.x = self.vmax * data.axes[1]
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = self.wmax*data.axes[2]
		#  rospy.loginfo(rospy.get_name() + ": I publish linear=(%f,%f,%f), angular=(%f,%f,%f)",twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z)

		# for transition conditions of fsm
		if (not self.button_pressed):
			self.button_pressed = (self.previous_signal==0 and data.buttons[0]==1)
		self.previous_signal = data.buttons[0];

		self.joy_activated = (abs(data.axes[0])>0.001 or abs(data.axes[1])>0.001 or abs(data.axes[2])>0.001 or abs(data.axes[3])>0.001 or abs(data.axes[4])>0.001 or abs(data.axes[5])>0.001)
		"""if(data.buttons[4] and data.buttons[5] and data.buttons[6] and data.buttons[7]):
			print 'shut up!'
		"""

	#############################################################################
	# callback for Bumper feedback
	#############################################################################
	def Bumpercallback(self,data):
		if (data.state == BumperEvent.RELEASED) :
			self.bumper_activated = False
		elif (data.state == BumperEvent.PRESSED):
			self.bumper_activated = True
			if (data.bumper == BumperEvent.LEFT) :
				#print 'Bumper Gauche'
				self.bumper_center = False
				self.bumper_right = False
				self.bumper_left = True
			elif (data.bumper == BumperEvent.CENTER) :
				#print 'Bumper Milieu'
				self.bumper_center = True
				self.bumper_right = False
				self.bumper_left = False
			elif (data.bumper == BumperEvent.RIGHT) :
				#print 'Bumper Droit'
				self.bumper_center = False
				self.bumper_right = True
				self.bumper_left = False


	#############################################################################
	# callback for Camera feedback
	#############################################################################
	def Cameracallback(self,data):
		global fwd_angle_max,fwd_angle_min,fwd_dist_max,fwd_dist_min
		global fwd_max_debut,fwd_max_fin,fwd_min_debut,fwd_min_fin
		
		debut_obstacle=-10
		debut_distance_obstacle=-10
		fin_obstacle=-10
		fin_distance_obstacle=-10
		obstacle=False
		debut_angle_obstacle=-10
		debut_laisse_passe=-10
		angle_obstacle_fin=-10
		fin_angle_obstacle=-10
		fin_laisse_passe=-10

		self.espace_libre_debut=-10
		self.espace_libre_fin=-10
		self.espace_libre_debut_dist=-10.0
		self.espace_libre_fin_dist=-10.0
		self.espace_pour_passer=-10
		
		
		if self.init_CameraCallback:	
			self.init_CameraCallback=False
			fwd_angle_max=math.atan((self.diameter/2)/self.dist_detection)
			fwd_angle_min=math.atan((self.diameter/2)/data.range_min)
			fwd_dist_max=math.sqrt(((self.diameter/2)**2)+(self.dist_detection**2))
			fwd_dist_min=math.sqrt(((self.diameter/2)**2)+(data.range_min**2))
			fwd_max_debut=int((len(data.ranges)/2)-(fwd_angle_max/data.angle_increment))
			fwd_max_fin=int((len(data.ranges)/2)+(fwd_angle_max/data.angle_increment))
			fwd_min_debut=int((len(data.ranges)/2)-(fwd_angle_min/data.angle_increment))
			fwd_min_fin=int((len(data.ranges)/2)+(fwd_angle_min/data.angle_increment))
		
		nb_obs=0
		#print '\n\nnew cam'
		#recherche des espaces libres et des obstacles
		for i in range(0,len(data.ranges)):#de la droite vers la gauche du turlte
			#definir le debut d un obstacle
			if data.ranges[i]>data.range_min and data.ranges[i]<self.dist_detection and not(obstacle) :	
				nb_obs+=1
				obstacle=True				
				debut_obstacle=i
				debut_distance_obstacle=data.ranges[i]
				#si il y a plusieurs obstacles
				#mise a jour de la fin de l obstacle precedent
				if nb_obs >1:
					debut_angle_obstacle=i*data.angle_increment-angle_obstacle_fin
					fin_angle_obstacle=debut_angle_obstacle
					#on calcule la place pour que le robot passe en fonction de l obstacle le plus proche
					if debut_distance_obstacle > fin_distance_obstacle:
						#si le debut de l obstacle suivant est plus proche que la fin de l obstacle precedent
						debut_laisse_passe=math.tan(debut_angle_obstacle)*debut_distance_obstacle
						fin_laisse_passe=debut_laisse_passe
					else:
						#si la fin de l obstacle precedent est plus proche que le debut de l obstacle suivant
						fin_laisse_passe=math.tan(fin_angle_obstacle)*fin_distance_obstacle
						debut_laisse_passe=fin_laisse_passe
				#	print "MAJ, fin obstacle",debut_angle_obstacle,fin_angle_obstacle,debut_laisse_passe,fin_laisse_passe
				#pour le 1er obstacle
				else:
					debut_angle_obstacle=i*data.angle_increment 
					debut_laisse_passe=math.tan(debut_angle_obstacle)*debut_distance_obstacle 
				
				#print "debut obstacle",nb_obs,debut_obstacle,debut_distance_obstacle,debut_angle_obstacle,debut_laisse_passe
			
			#definir la fin d un obstacle		
				#print "a", debut_obstacle,debut_distance_obstacle, debut_angle_obstacle,debut_laisse_passe,fin_angle_obstacle,fin_laisse_passe	
			elif obstacle and math.isnan(data.ranges[i]) and not(math.isnan(data.ranges[i-1])) and math.isnan(data.ranges[i+1]):
				obstacle=False
				fin_obstacle=i
				fin_distance_obstacle=data.ranges[i-1]
				angle_obstacle_fin=i*data.angle_increment
				fin_angle_obstacle=(abs(data.angle_min)+abs(data.angle_max))-(i*data.angle_increment)
				fin_laisse_passe=math.tan(fin_angle_obstacle)*fin_distance_obstacle
				#print "b", fin_obstacle, fin_distance_obstacle,fin_angle_obstacle,fin_laisse_passe
			
				#print "fin obstacle",nb_obs,fin_obstacle,fin_distance_obstacle,fin_angle_obstacle,fin_laisse_passe 
			
			

			#definir le plus grand espace libre
			if debut_laisse_passe > self.espace_pour_passer or fin_laisse_passe>self.espace_pour_passer:
				if fin_obstacle <0:
					self.espace_libre_debut=0
					self.espace_libre_debut_dist=self.dist_detection
				else:
					self.espace_libre_debut=fin_obstacle
					self.espace_libre_debut_dist=data.ranges[fin_obstacle-1]
				if debut_obstacle>fin_obstacle:
					self.espace_libre_fin=debut_obstacle
					self.espace_libre_fin_dist=data.ranges[debut_obstacle] 
					self.espace_pour_passer=debut_laisse_passe 				
				#print "c", self.espace_libre_debut, self.espace_libre_fin,self.espace_pour_passer, self.espace_libre_debut_dist,self.espace_libre_fin_dist
		
		
		#print "fin",nb_obs,self.espace_libre_debut, self.espace_libre_fin,self.espace_pour_passer
		if self.espace_libre_debut==0 and math.isnan(data.ranges[self.espace_libre_debut]):
			self.espace_libre_debut_dist=self.dist_detection
		if self.espace_libre_fin==len(data.ranges) and math.isnan(data.ranges[self.espace_libre_fin-1]):
			self.espace_libre_fin_dist=self.dist_detection
			
		#print "c2",obstacle,self.espace_libre_debut, self.espace_libre_fin,self.espace_pour_passer,self.espace_libre_debut_dist,self.espace_libre_fin_dist
			
		if self.espace_libre_fin < self.espace_libre_debut and not(obstacle):
			self.espace_libre_fin=len(data.ranges)
			self.espace_libre_fin_dist=data.ranges[len(data.ranges)-1]
			if self.espace_libre_fin==len(data.ranges) and math.isnan(data.ranges[self.espace_libre_fin-1]):
				self.espace_libre_fin_dist=self.dist_detection
				
			
			angle=(self.espace_libre_fin-self.espace_libre_debut)*data.angle_increment
			
			if self.espace_libre_debut_dist < self.espace_libre_fin_dist:
				self.espace_pour_passer=math.tan(angle)*self.espace_libre_debut_dist
			else:
				self.espace_pour_passer=math.tan(angle)*self.espace_libre_fin_dist
			
			
		#print "d", self.espace_libre_debut, self.espace_libre_fin,self.espace_pour_passer,self.espace_libre_debut_dist,self.espace_libre_fin_dist
		
		
		#choix de l'attitude a adopter
		self.lambda_rotate=((self.espace_libre_fin-self.espace_libre_debut)*data.angle_increment)*self.espace_pour_passer
		if self.lambda_rotate>1.0:
			self.lambda_rotate=1.0
			
		if nb_obs ==0:
			#print'pas d obstacle on continue'
			self.obstacle=0
		elif self.espace_pour_passer < self.diameter:
			#print'peut pas passer, demi-tour',self.espace_pour_passer
			self.obstacle=3
			self.lambda_rotate=data.ranges[len(data.ranges)/2]/self.dist_detection #distance en face
		else:
			#print len(data.ranges)/2,self.espace_libre_debut,self.espace_libre_fin,fwd_min_debut,fwd_min_fin,fwd_max_debut,fwd_max_fin
			if (self.espace_libre_debut >= fwd_min_debut and self.espace_libre_debut <= fwd_max_debut):
				#print'si pas d obstacle en face, et place pour passer, tout droit'
				self.obstacle=0
			if (self.espace_libre_debut < fwd_min_debut and self.espace_libre_fin > fwd_min_fin):
				self.obstacle=0
			elif (self.espace_libre_debut >= fwd_min_debut) :
				#print'on tourne a gauche'#,len(data.ranges)/2,self.espace_libre_fin, data.angle_increment,self.espace_libre_fin_dist,self.dist_detection
				self.obstacle=1
				self.lambda_rotate=(((self.espace_libre_fin-len(data.ranges)/2)*data.angle_increment)*self.espace_libre_fin_dist)/self.dist_detection
			elif (self.espace_libre_fin <= fwd_min_fin):
				#print'on tourne a droite' #,self.espace_libre_debut,len(data.ranges)/2,data.angle_increment,self.espace_libre_debut_dist, self.dist_detection
				self.obstacle=2
				self.lambda_rotate=(((len(data.ranges)/2-self.espace_libre_debut)*data.angle_increment)*self.espace_libre_debut_dist)/self.dist_detection
			else:
				print 'else...',self.espace_pour_passer,self.diameter,nb_obs
				print len(data.ranges)/2,self.espace_libre_debut,self.espace_libre_fin,fwd_min_debut,fwd_min_fin,fwd_max_debut,fwd_max_fin
			
				
					
	
	#############################################################################
	# callback for IMU feedback
	#############################################################################
	def IMUcallback(self,data):
		global Zorientation,Zplusplus,Zplusmoins,Zmoinsplus,Zmoinsmoins
		if self.init_IMUCallback:
			self.init_IMUCallback=False
			Zplusplus=-10.0
			Zplusmoins=-10.0
			Zmoinsplus=-10.0
			Zmoinsmoins=-10.0
			Zorientation=-10.0
		if self.orientation:
			Zorientation=abs(data.orientation.z)
			self.orientation=False
			if Zorientation <0.1 and Zorientation >-0.1:
				Zorientation=0.0
				Zplusplus=0.65
				Zplusmoins=0.55
				Zmoinsplus=-0.65
				Zmoinsmoins=-0.55
			elif Zorientation > 0.9 or Zorientation < -0.99:
				Zorientation=1.0
				Zplusplus=0.65
				Zplusmoins=0.55
				Zmoinsplus=-0.65
				Zmoinsmoins=-0.55
			elif Zorientation+0.65>0.999:
				Zplusplus=1.34-Zorientation
				Zplusmoins=1.44-Zorientation
				Zmoinsplus=Zorientation-0.65
				Zmoinsmoins=Zorientation-0.55
			elif -(Zorientation+0.65)>-0.999:
				Zplusplus=Zorientation+0.65
				Zplusmoins=Zorientation+0.55
				Zmoinsplus=-(1.34-Zorientation)
				Zmoinsmoins=-(1.34-Zorientation)
			else:
				Zplusplus=Zorientation+0.65
				Zplusmoins=Zorientation+0.55
				Zmoinsplus=Zorientation-0.65
				Zmoinsmoins=Zorientation-0.55
		
		#print abs(data.orientation.z),abs(Zorientation),abs(Zplus)-0.05,abs(Zplus)+0.05, abs(Zmoins)-0.05, abs(Zmoins)+0.05,abs(data.orientation.z) > abs(Zplus)-0.05 ,abs(data.orientation.z) < abs(Zplus)+0.05,abs(data.orientation.z) < abs(Zmoins)-0.05,abs(data.orientation.z) > abs(Zmoins)+0.05
		
		if (abs(data.orientation.z) > Zplusmoins and abs(data.orientation.z) < Zplusplus):
			#print "plus"
			self.rotate_over=True
		elif (abs(data.orientation.z) < Zmoinsmoins and abs(data.orientation.z) > Zmoinsplus):
			#print "moins"
			self.rotate_over=True
		else:
			self.rotate_over=False
				
		#print Zorientation, data.orientation.z,self.rotate_over
		if abs(data.angular_velocity.z) < 0.1:
			self.rotate_is_stopped=True
		else:
			self.rotate_is_stopped=False
	#############################################################################
	# callback for ROUE SOL feedback
	#############################################################################
	def Wheelcallback(self,data):
		if (data.state == WheelDropEvent.RAISED):
			self.wheel_on_ground=True
		elif (data.state == WheelDropEvent.DROPPED):
			self.wheel_on_ground=False
		
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
	
	#transitions pour l arret avec les bumpers
	def check_AutonomousMode_To_Retreat(self,fss):
		return (self.bumper_activated)

	def check_Retreat_To_Rotate(self,fss):
		return (not(self.bumper_activated) and self.compteur>=5)

	def check_Rotate_To_Stop(self,fss):
		return (self.rotate_over)

	def check_Stop_To_AutonomousMode(self,fss):
		return (self.rotate_is_stopped and self.compteur>=5)	

	#transitions securite retour mode manuel
	def check_To_JoyControl(self,fss):
		return self.joy_activated

	#transitions securite si absence contact roue sol
	def check_To_OnGround(self,fss):	#on s arret il y a plus de contact
		return not(self.wheel_on_ground)

	def check_OnGround_To_JoyControl(self,fss):	#on retourne en mode manu
		return (self.wheel_on_ground and self.compteur>=5)


	#transitions kinect
	def check_AutonomousMode_To_ChangeDirection(self,fss):
		return (self.obstacle>0)

	def check_ChangeDirection_To_AutonomousMode(self,fss):
		return (self.obstacle==0)

	def check_ChangeDirection_To_Retreat(self,fss):
		return (self.bumper_activated)

	#transition pour rester dans le meme etat
	def KeepJoyControl(self,fss):
		self.next_state=[]
		self.present_state="JoyControl"
		self.next_state.append(self.check_To_OnGround(fss))
		self.next_state.append(self.check_JoyControl_To_AutonomousMode(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepJoyControl",self.check_To_OnGround(fss),self.check_JoyControl_To_AutonomousMode(fss)
			sys.stdout.write("\033[37m")
		if self.check_To_OnGround(fss):
			return False
		else:
			return (not(self.check_JoyControl_To_AutonomousMode(fss)))

	def KeepAutonomousMode(self,fss):
		self.next_state=[]
		self.present_state="AutonomousMode"
		self.next_state.append(self.check_To_JoyControl(fss))
		self.next_state.append(self.check_To_OnGround(fss))
		self.next_state.append(self.check_AutonomousMode_To_Retreat(fss))
		self.next_state.append(self.check_AutonomousMode_To_ChangeDirection(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepAutonomousMode",self.check_To_JoyControl(fss),self.check_To_OnGround(fss),self.check_AutonomousMode_To_Retreat(fss),self.check_AutonomousMode_To_ChangeDirection(fss)
			sys.stdout.write("\033[37m")
		if self.check_To_JoyControl(fss):
			return False
		else:
			if self.check_To_OnGround(fss):
				return False
			else:
				if self.check_AutonomousMode_To_Retreat(fss):
					return False
				else:
					return (not(self.check_AutonomousMode_To_ChangeDirection(fss)))

	def KeepRetreat(self,fss):
		self.next_state=[]
		self.present_state="Retreat"
		self.next_state.append(self.check_To_JoyControl(fss))
		self.next_state.append(self.check_To_OnGround(fss))
		self.next_state.append(self.check_Retreat_To_Rotate(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepRetreat",self.check_To_JoyControl(fss),self.check_To_OnGround(fss),self.check_Retreat_To_Rotate(fss)
			sys.stdout.write("\033[37m")
		if self.check_To_JoyControl(fss):
			return False
		else:
			if self.check_To_OnGround(fss):
				return False
			else:
				return (not (self.check_Retreat_To_Rotate(fss)))

	def KeepRotate(self,fss):
		self.next_state=[]
		self.present_state="Rotate"
		self.next_state.append(self.check_To_JoyControl(fss))
		self.next_state.append(self.check_To_OnGround(fss))
		self.next_state.append(self.check_Rotate_To_Stop(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepRotate",self.check_To_JoyControl(fss),self.check_To_OnGround(fss),self.check_Rotate_To_Stop(fss)
			sys.stdout.write("\033[37m")
		if self.check_To_JoyControl(fss):
			return False
		else:
			if self.check_To_OnGround(fss):
				return False
			else:
				return (not self.check_Rotate_To_Stop(fss))

	def KeepStop(self,fss):
		self.next_state=[]
		self.present_state="Stop"
		self.next_state.append(self.check_To_JoyControl(fss))
		self.next_state.append(self.check_To_OnGround(fss))
		self.next_state.append(self.check_Stop_To_AutonomousMode(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepStop",self.check_To_JoyControl(fss),self.check_To_OnGround(fss),self.check_Stop_To_AutonomousMode(fss)
			sys.stdout.write("\033[37m")
		if self.check_To_JoyControl(fss):
			return False
		else:
			if self.check_To_OnGround(fss):
				return False
			else:
				return (not(self.check_Stop_To_AutonomousMode(fss)))

	def KeepChangeDirection(self,fss):
		self.next_state=[]
		self.present_state="ChangeDirection"
		self.next_state.append(self.check_To_JoyControl(fss))
		self.next_state.append(self.check_To_OnGround(fss))
		self.next_state.append(self.check_ChangeDirection_To_AutonomousMode(fss))
		self.next_state.append(self.check_ChangeDirection_To_Retreat(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepChangeDirection",self.check_To_JoyControl(fss),self.check_To_OnGround(fss),self.check_ChangeDirection_To_AutonomousMode(fss),self.check_ChangeDirection_To_Retreat(fss)
			sys.stdout.write("\033[37m")
		if self.check_To_JoyControl(fss):
			return False
		else:
			if self.check_To_OnGround(fss):
				return False
			else:
				if self.check_ChangeDirection_To_AutonomousMode(fss):
					return False
				else:
					return (not (self.check_ChangeDirection_To_Retreat(fss)))
	
	def KeepOnGround(self,fss):
		self.next_state=[]
		self.present_state="KeepOnGround"
		self.next_state.append(self.check_To_JoyControl(fss))
		self.next_state.append(self.check_OnGround_To_JoyControl(fss))
		if (self.message and self.message_info):
			sys.stdout.write("\033[36m")
			print "KeepOnGround",self.check_To_JoyControl(fss)
			sys.stdout.write("\033[37m")
		return (not(self.check_OnGround_To_JoyControl(fss)))
	

	#############################################################################
	# functions for instructions inside states of fsm
	#############################################################################
	#fonctions de base	
	def DoJoyControl(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_AutonomousMode=False
		self.message_Stop=False
		self.message_Retreat=False
		self.message_Rotate=False
		self.message_ChangeDirection=False
		self.message_OnGround=False
		if (not(self.message_JoyControl) and self.message):
			print 'Mode Manuel'
			print '+--------------------------------------------------------------------------------+'
			print '| Joystick gauche 	> avant = avance                                         |'
			print '| 			> arriere = recule                                       |'
			print '| Joystick droit	> droit = sens horaire                                	 |'
			print '|		 	> gauche = sens anti horaire                             |'
			print '| Triangle (1)		> Mode Automatique                                       |'
			print '| Si Mode Automatique, ces 4 manipulations des joysticks ramene au Mode Manuel   |'
			print '+--------------------------------------------------------------------------------+'
			self.message_JoyControl=True
			self.compteur=0
		self.button_pressed =  False;
		self.smooth_velocity()
  		self.pub.publish(self.twist_real)
		sys.stdout.write("\033[37m")
		pass

	def DoAutonomousMode(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_Stop=False
		self.message_Retreat=False
		self.message_Rotate=False
		self.message_ChangeDirection=False
		self.message_OnGround=False
		if (not(self.message_AutonomousMode) and self.message):
			print 'Mode Automatique'
			self.message_AutonomousMode=True
			self.compteur=0
		self.button_pressed =  False;
		go_fwd = Twist()
		go_fwd.angular.z= 0;
		go_fwd.linear.x = self.vmax/2.0;
	  	self.pub.publish(go_fwd)
		sys.stdout.write("\033[37m")
		pass

	#fonction pour l arret avec les bumpers
	def DoRetreat(self,fss,value):
		sys.stdout.write("\033[0m") #met le texte en couleur
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Stop=False
		self.message_Rotate=False
		self.message_ChangeDirection=False
		self.message_OnGround=False
		if (not(self.message_Retreat) and self.message):
			self.message_Retreat=True
			self.compteur=0
			print 'Mode Retreat Bumper'
			sys.stdout.write("\033[35m")
			if (self.bumper_left):
				print 'bumper gauche tourne horaire'
			elif (self.bumper_center):
				print 'bumper milieu tourne anti horaire'
			elif (self.bumper_right):
				print 'bumper droit tourne anti horaire'
			sys.stdout.write("\033[0m")	
		go_fwd=Twist()
		go_fwd.linear.x=-self.vmax/4
		go_fwd.linear.y=0.0
		go_fwd.linear.z=0.0
		go_fwd.angular.x=0.0
		go_fwd.angular.y=0.0
		go_fwd.angular.z=0.0
		self.pub.publish(go_fwd)
		self.compteur += 1
		sys.stdout.write("\033[37m")
		pass

	def DoRotate(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Stop=False
		self.message_Retreat=False
		self.message_ChangeDirection=False
		self.message_OnGround=False
		if (not(self.message_Rotate) and self.message):
			print 'Mode Rotation'
			self.message_Rotate=True	
			self.orientation=True		
		go_fwd=Twist()
		go_fwd.linear.x=0.0
		go_fwd.linear.y=0.0
		go_fwd.linear.z=0.0
		go_fwd.angular.x=0.0
		go_fwd.angular.y=0.0
		if (self.bumper_left) :	#si bumper de gauche
			go_fwd.angular.z = -self.wmax/2.0;
		elif (self.bumper_center): 	#si bumper du milieu
			go_fwd.angular.z = self.wmax/2.0;	
		elif (self.bumper_right):	#si bumper de droite
			go_fwd.angular.z = self.wmax/2.0;
		self.pub.publish(go_fwd)
		sys.stdout.write("\033[37m")
		pass

	def DoStop(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Retreat=False
		self.message_Rotate=False
		self.message_ChangeDirection=False
		self.message_OnGround=False
		if (not(self.message_Stop) and self.message):
			print 'Mode Stop'
			self.message_Stop=True
			self.compteur=0
			self.orientation=False
		go_fwd=Twist()
		go_fwd.linear.x=0.0
		go_fwd.linear.y=0.0
		go_fwd.linear.z=0.0
		go_fwd.angular.x=0.0
		go_fwd.angular.y=0.0
		go_fwd.angular.z=0.0
		self.pub.publish(go_fwd)
		self.compteur += 1
		sys.stdout.write("\033[37m")
		pass


	def DoChangeDirection(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Stop=False
		self.message_Retreat=False
		self.message_Rotate=False
		self.message_OnGround=False
		if (not(self.message_ChangeDirection) and self.message):
			print 'Mode Changement Direction'
			self.message_ChangeDirection=True
			self.compteur=0
		turn=Twist();
		turn.linear.y=0.0
		turn.linear.z=0.0
		turn.angular.x=0.0
		turn.angular.y=0.0
		sys.stdout.write("\033[35m")
		if self.obstacle==1:
			print "passage a gauche"
			turn.linear.x=(self.vmax/2)*self.lambda_rotate
			turn.angular.z=(self.wmax/2)*self.lambda_rotate
			#print self.espace_libre_debut,self.espace_libre_fin,self.espace_pour_passer
		elif self.obstacle==2:
			print "passage a droite"
			turn.linear.x=(self.vmax/2)*self.lambda_rotate
			turn.angular.z=-(self.wmax/2)*self.lambda_rotate
			#print self.espace_libre_debut,self.espace_libre_fin,self.espace_pour_passer
		elif self.obstacle==3:
			print "demi tour!!!"
			turn.linear.x=0
			turn.angular.z=(self.wmax/4)
			#print self.espace_libre_debut,self.espace_libre_fin,self.espace_pour_passer
		sys.stdout.write("\033[0m")
		self.pub.publish(turn)
		self.compteur+=1
		sys.stdout.write("\033[37m")
		pass

	#fonction securite si abscence contact roue sol
	def DoOnGround(self,fss,value):
		sys.stdout.write("\033[0m")
		self.message_JoyControl=False
		self.message_AutonomousMode=False
		self.message_Stop=False
		self.message_Retreat=False
		self.message_Rotate=False
		self.message_ChangeDirection=False
		if (not(self.message_OnGround) and self.message):
			print 'Put Robot OnGround'
			self.message_OnGround=True
			self.compteur=0
		go_fwd=Twist()
		go_fwd.linear.x=0.0
		go_fwd.linear.y=0.0
		go_fwd.linear.z=0.0
		go_fwd.angular.x=0.0
		go_fwd.angular.y=0.0
		go_fwd.angular.z=0.0
		self.pub.publish(go_fwd)
		self.compteur += 1
		sys.stdout.write("\033[37m")
		pass
	

#############################################################################
# main function
#############################################################################
if __name__ == '__main__':
	try:
		time.sleep(3)
		sys.stdout.write("\033[34m")
		print '\n\n\n\n---------------Bienvenue sur la commande d un TurtleBot---------------'
		print 'Qu elle est votre plateforme?'
		print 'TurtleBot Simulation = 0 sinon Simulation Gazebo ou Robot Reel'
				
		#choix=int(raw_input("votre choix?"))
		choix=1
		rospy.init_node('joy4ctrl')
		if (choix==0):
			print 'Vous travaillez sur la simulation'
			pub = rospy.Publisher('turtle1/cmd_vel', Twist)
		# real turtle
		else :
			print 'Vous travaillez sous Gazebo ou sur le robot reel'
			pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 1)	
		# turtlesim
		sys.stdout.write("\033[37m")
		Hz = 10
    		rate = rospy.Rate(Hz) # 10hz
		T = 1.0/Hz
		MyRobot = RobotBehavior(pub,T);
        	rospy.Subscriber("joy", Joy, MyRobot.Joystickcallback)
		rospy.Subscriber("mobile_base/events/bumper",BumperEvent, MyRobot.Bumpercallback)
		rospy.Subscriber("scan", LaserScan, MyRobot.Cameracallback)
		rospy.Subscriber("mobile_base/events/wheel_drop",WheelDropEvent, MyRobot.Wheelcallback)	
		rospy.Subscriber("mobile_base/sensors/imu_data",Imu, MyRobot.IMUcallback)	
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
        	
        	
