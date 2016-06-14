#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
import time
import datetime
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *

from bender_macros.speech import Talk

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #


def Tell_Name():
	rospy.loginfo('Bender: TellName')

	Talk.getInstance('My name is Bender',3)

def Tell_TeamName():
	rospy.loginfo('Bender: TellTeamName')

	Talk.getInstance('The name of my team is Homebreakers',5)


def TellTime_Now():
	rospy.loginfo('Bender: TellTimeNow')

	x = datetime.datetime.now()
	tim = 'The time is '+str(x.hour)+' with '+str(x.minute)+' minutes'

	Talk.getInstance(tim, 4)


def TellDate_Now():
	rospy.loginfo('Bender: TellDateNow')

	today = datetime.date.today()
	f_date = today.strftime('Today is %A %d  of %B %Y')

	Talk.getInstance(f_date,len(f_date)/8)


def TellDay_Today():
	rospy.loginfo('Bender: TellDayToday')

	today = datetime.date.today()
	day = today.strftime('Today is %A %d')

	Talk.getInstance(day,len(day/8))


def TellDay_Tomorrow():
	rospy.loginfo('Bender: TellDayTomorrow')

	tomorrow = datetime.date.today() + datetime.timedelta(days=1)
	daytomorrow = tomorrow.strftime('Tomorrow is %A %d')

	Talk.getInstance(day,len(daytomorrow/8))
	

def TellDay_Month():
	rospy.loginfo('Bender: TellDayMonth')

	today = datetime.date.today()
	month = today.strftime('Today is %d of %B')

	Talk.getInstance(month,len(month)/8)


def TellDay_Week():
	rospy.loginfo('Bender: TellDayWeek')

	today = datetime.date.today()
	month = today.strftime('Today is %A')

	Talk.getInstance(month,len(month)/8)


def Tell_Introduce():

	rospy.loginfo('Bender: Introduce')

	Talk.getInstance('Hi, my name is bender', 3)
	Talk.getInstance('and i represent the homebreakers team from the University of Chile', 5)