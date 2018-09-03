#!/usr/bin/env python
""" 
	BLUE ROV - Acquisition de la profondeur
	Albane Ordrenneau
	15/05/2018

"""
#############################################################################
# imports
#############################################################################
import rospy
import time

from sensor_msgs.msg import FluidPressure
from std_msgs.msg import String

water_depth=None

def WaterPressurecallback(data):
	global water_depth
	pressure=data.fluid_pressure
	atmospheric=101300 #pression atmopspherique
	gravity=9.80665
	rho=997 #masse volumique eau (kg/m³)
	water_depth=(pressure-atmospheric)/(rho*gravity)
	wather_depth=str(water_depth)
	
	
#############################################################################
# main function
#############################################################################
if __name__ == '__main__':
	try:
		rospy.init_node('waterpressure')
		rospy.Subscriber('/mavros/imu/water_pressure',FluidPressure,WaterPressurecallback)
		pub=rospy.Publisher('BR/water_depth',String,queue_size=1)
		rate=rospy.Rate(10)
		while water_depth is None and not rospy.is_shutdown():
			rospy.sleep(.1)
		while not rospy.is_shutdown():
			pub.publish(water_depth)
			rate.sleep()
	
	except rospy.ROSInterruptException:
		pass

