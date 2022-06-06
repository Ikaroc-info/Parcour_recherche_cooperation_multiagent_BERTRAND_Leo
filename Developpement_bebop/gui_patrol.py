#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
Software License Agreement (BSD)

file      gui_patrol.py
authors   leo bertrand <leobertand641@gmail.com>
copyright Copyright (c) 2019, LORIA (UMR 7503 CNRS, Universite de Lorraine, INRIA) , All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of LORIA nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

''' This code is a modification of the gui code from adrien guenard. It is able to give different commands to the bebop as takeoff
land, set target position and start and stop the control of the bebop. The command set target position will enable the
drone launch the patrol. The command start control will enable the drone to move. The command stop control will disable'''

import rospy
from std_msgs.msg import Empty
from bebop_control.srv import SetTargetPos
from Tkinter import *
from std_srvs.srv import Trigger


#const variables: 
BEBOP_TAKEOFF_TOPIC="/bebop/takeoff"
BEBOP_LAND_TOPIC="/bebop/land"
BEBOP_SETTARGET_SERVICE="/set_target_position" #ADD NAMESPACE
BEBOP_STARTCONTROL_SERVICE="/start_control"
BEBOP_STOPCONTROL_SERVICE="/stop_control"

#Min and max target values
XCMAX=2
XCMIN=-2
YCMAX=2.5
YCMIN=-2.5
ZCMAX=3
ZCMIN=0
YAWCMAX=180
YAWCMIN=-180

class controler_gui:

	def __init__(self):
		rospy.init_node('bebopcontroler_gui', anonymous=True) # Init ROS node
		rate=rospy.Rate(50) # Set rate 50hz
		rospy.loginfo('Bebop controler GUI started')

		#Wait for set target service from controler
		rospy.loginfo("Wait for bebop controler");
		#rospy.wait_for_service(BEBOP_SETTARGET_SERVICE)
		rospy.loginfo("Bebop controler ok");



		#Publishers
		#Outputs: 
		# Takeoff command: std_msgs/Empty
		# Land command: std_msgs/Empty
		self.takeoff_pub=rospy.Publisher(BEBOP_TAKEOFF_TOPIC,Empty,queue_size=1)
		self.land_pub=rospy.Publisher(BEBOP_LAND_TOPIC,Empty,queue_size=10)


		window=Tk() #create Tkinter window
		window.title("Bebop control")
		#window.geometry('350x200')
		window.configure(background='blue')

		frame=Frame(window,width=150,height=150,borderwidth=4) #create frame and place title in frame	
		frame.pack()
		#frame.pack(fill=BOTH)	

		#Create buttons
		takeoff_button=Button(frame, text="Takeoff", command=self.send_takeoff,height=2,width=20)
		takeoff_button.grid(row=0,column=0,columnspan=2)
		land_button=Button(frame, text="Land", command=self.send_land,height=2,width=20)
		land_button.grid(row=1,column=0,columnspan=2)
		start_control_button=Button(frame, text="Start ctrl", command=self.send_start_control,height=2)
		start_control_button.grid(row=2,column=0)
		stop_control_button=Button(frame, text="Stop ctrl", command=self.send_stop_control,height=2)
		stop_control_button.grid(row=2,column=1)


		#Form
		xc_label=Label(frame,text="Xc [-2m 2m]",fg="black",bg="white")
		self.xc_label_text_var=DoubleVar()
		xc_entry=Entry(frame,textvariable=self.xc_label_text_var)
		xc_entry.insert(END,'0')
		xc_label.grid(row=3,column=0)
		xc_entry.grid(row=3,column=1)

		yc_label=Label(frame,text="Yc [-2.5m 2.5m]",fg="black",bg="white")
		self.yc_label_text_var=DoubleVar()
		yc_entry=Entry(frame,textvariable=self.yc_label_text_var)
		yc_entry.insert(END,'0')
		yc_label.grid(row=4,column=0)
		yc_entry.grid(row=4,column=1)

		zc_label=Label(frame,text="Zc [0.5m 3m]",fg="black",bg="white")
		self.zc_label_text_var=DoubleVar(value='1.0')
		zc_entry=Entry(frame,textvariable=self.zc_label_text_var)

		zc_label.grid(row=5,column=0)
		zc_entry.grid(row=5,column=1)

		yaw_label=Label(frame,text="Yaw [-180° 180°]",fg="black",bg="white")
		self.yaw_label_text_var=DoubleVar()
		yaw_entry=Entry(frame,textvariable=self.yaw_label_text_var)
		yaw_entry.insert(END,'0')
		yaw_label.grid(row=6,column=0)
		yaw_entry.grid(row=6,column=1)

		settarget_button=Button(frame, text="Set target", command=lambda:self.call_target_service(),height=2,width=20)
		settarget_button.grid(row=7,column=0,columnspan=2)
		

		while not rospy.is_shutdown():
			#Do nothing
			window.mainloop()			
			#rate.sleep()

	def check_string_to_float(self,s):
		try:
			float(s)
			return True
		except:
			return False

	def send_land(self):
		self.send_stop_control()
		land_msg=Empty()
		self.land_pub.publish(land_msg)
		print "Land command sent"

	def send_takeoff(self):
		takeoff_msg=Empty()
		self.takeoff_pub.publish(takeoff_msg)
		print "Takeoff command sent"

	def saturate(self,value,max_value,min_value):
		if value>=max_value:
			value=max_value
		if value<=min_value:
			value=min_value
		return value

	def send_start_control(self):
		try:
			startcontrol=rospy.ServiceProxy(BEBOP_STARTCONTROL_SERVICE,Trigger)
			res=startcontrol()
			return res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def send_stop_control(self):
		try:
			stopcontrol=rospy.ServiceProxy(BEBOP_STOPCONTROL_SERVICE,Trigger)
			res=stopcontrol()
			return res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		

	def call_target_service(self):
		Xc=float(self.xc_label_text_var.get())
		Xc=self.saturate(Xc,XCMAX,XCMIN)
		Yc=float(self.yc_label_text_var.get())
		Yc=self.saturate(Yc,YCMAX,YCMIN)
		Zc=float(self.zc_label_text_var.get())
		Zc=self.saturate(Zc,ZCMAX,ZCMIN)
		Yawc=float(self.yaw_label_text_var.get())
		Yawc=self.saturate(Yawc,YAWCMAX,YAWCMIN)
		print 'Call target service'
		try:

#					if check_string_to_float(Xc):
			settarget=rospy.ServiceProxy(BEBOP_SETTARGET_SERVICE,SetTargetPos)

			res=settarget(float(Xc),float(Yc),float(Zc),float(Yawc)) #Add YAW
			return res
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

			
	
if __name__ == '__main__':
	try:	
		controler_gui()
	except rospy.ROSInterruptException:
		pass
