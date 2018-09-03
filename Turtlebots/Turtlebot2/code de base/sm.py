#!/usr/bin/env python
""" ROS python pogramming with finite state machines to describe a robot's behaviors
    Vincent Hugel

    Seatech/SYSMER 2A Course
    free to use so long as the author and other contributers are credited.

modifié par

	Albane Ordrenneau

"""
#############################################################################
# imports
#############################################################################
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from fsm import fsm

from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import Imu

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
		self.vmax = 1.5
		self.wmax = 4.0
		self.previous_signal = 0
		self.button_pressed = False
		self.joy_activated = False

		self.bumper_activated = False
		self.bumper_left=False
		self.bumper_center=False
		self.bumper_right=False
		self.rob_stopped = False
		self.rotate_over = False

		self.pub = handle_pub
		self.T = T
		# instance of fsm with source state, destination state, condition (transition), callback (defaut: None)
		self.fs = fsm([ ("Start","JoyControl", True ),
			("JoyControl","AutonomousMode1", self.check_JoyControl_To_AutonomousMode1, self.DoAutonomousMode1),
			("JoyControl","JoyControl", self.KeepJoyControl, self.DoJoyControl),
			("AutonomousMode1","JoyControl", self.check_AutonomousMode1_To_JoyControl, self.DoJoyControl),
			("AutonomousMode1","AutonomousMode1", self.KeepAutonomousMode1, self.DoAutonomousMode1),


			("AutonomousMode1","StopBumper",self.check_AutonomousMode1_To_StopBumper,self.DoStopBumper),
			("StopBumper","Rotate",self.check_StopBumper_To_Rotate,self.DoRotate),
			("Rotate","AutonomousMode1",self.check_Rotate_To_AutonomousMode1,self.DoAutonomousMode1),
			("StopBumper","StopBumper",self.KeepStopBumper,self.DoStopBumper),
			("Rotate","Rotate",self.KeepRotate,self.DoRotate)])


	#############################################################################
	# callback for joystick feedback
	#############################################################################
	def callback(self,data):
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

		self.joy_activated = (abs(data.axes[1])>0.001 or abs(data.axes[2])>0.001)
        self.bumper_activated = BumperEvent.PRESSED
        self.bumper_left = BumperEvent.LEFT
        self.bumper_center = BumperEvent.CENTER
        self.bumper_right = BumperEvent.RIGHT
        self.rob_stopped = orientation.velocity.covariance[1]

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
	def check_JoyControl_To_AutonomousMode1(self,fss):
		return self.button_pressed

	def check_AutonomousMode1_To_JoyControl(self,fss):
		#return self.button_pressed
		return self.joy_activated

	def KeepJoyControl(self,fss):
		return (not self.check_JoyControl_To_AutonomousMode1(fss))

	def KeepAutonomousMode1(self,fss):
		return (not (self.check_AutonomousMode1_To_JoyControl(fss) or self.check_AutonomousMode1_To_StopBumper(fss)))


	def check_AutonomousMode1_To_StopBumper(self,fss):
		return self.bumper_activated

	def check_StopBumper_To_Rotate(self,fss):
		return self.rob_stopped

	def check_Rotate_To_AutonomousMode1(self,fss):
		return self.rotate_over

	def KeepStopBumper(self,fss):
		return (not self.check_StopBumper_To_Rotate(fss))

	def KeepRotate(self,fss):
		return (not self.check_Rotate_To_AutonomousMode1(fss))



	#############################################################################
	# functions for instructions inside states of fsm
	#############################################################################
	def DoJoyControl(self,fss,value):
		self.button_pressed =  False;
		self.smooth_velocity()
  		self.pub.publish(self.twist_real)
		print 'joy control : ',self.twist_real
		pass

	def DoAutonomousMode1(self,fss,value):
		self.button_pressed =  False;
		# go forward
		go_fwd = Twist()
		go_fwd.linear.x = self.vmax/2.0;
	  	self.pub.publish(go_fwd)
		print 'autonomous : ',go_fwd
		pass

	def DoStopBumper(self,fss,value):

		pass

	def DoRotate(self,fss,value):

		pass


#############################################################################
# main function
#############################################################################
if __name__ == '__main__':
	try:

		rospy.init_node('joy4ctrl')
		# real turtle
		pub = rospy.Publisher('mobile_base/commands/velocity', Twist) #pub = rospy.Publisher('mobile_base/commands/velocity', Twist,queue_size = 1)
		# turtlesim
		#pub = rospy.Publisher('turtle1/cmd_vel', Twist)
		Hz = 10
    		rate = rospy.Rate(Hz) # 10hz
		T = 1.0/Hz

		MyRobot = RobotBehavior(pub,T);
        rospy.Subscriber("joy", Joy, MyRobot.callback)
		rospy.Subscriber('mobile_base/events/bumper', BumperEvent, MyRobot.callback)
		rospy.Subscriber("imu",Imu,MyRobot.callback)
		MyRobot.fs.start("Start")

		# loop at rate Hz
    		while not rospy.is_shutdown():
	   		ret = MyRobot.fs.event("")
	        	rate.sleep()

	except rospy.ROSInterruptException:
        	pass
