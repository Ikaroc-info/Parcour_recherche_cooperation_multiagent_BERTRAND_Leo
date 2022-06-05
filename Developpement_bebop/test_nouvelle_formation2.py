#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
Software License Agreement (BSD)

file      controlerXYZyaw.py
authors   adrien guenard <adrien.guenard@loria.fr>
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

from numpy.core.numeric import normalize_axis_tuple
import rospy
import numpy as np
from math import *
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from bebop_control.srv import *
from std_srvs.srv import Trigger


#const variables (do not modify): 
BEBOP_COMMANDS_TOPIC="/bebop/cmd_vel"

def addition_terme_poid(l1,l2):
		total=[]
		for i in range(len(l1)-1):
				total+=[l1[i]+l2[i]*l2[len(l2)-1]]
		total+=[l1[len(l1)-1]+l2[len(l2)-1]]
		return total

class Sec_drone:
	def __init__(self):
    	#Agent Optitrack position
		self.cam_X=0
		self.cam_Y=0
		self.cam_Z=0
		#flags
		self.cam_position_received=0
		#Previous position (to compute velocity)
		self.previous_X=0
		self.previous_Y=0
		self.previous_Z=0
		#Voisin proche
		self.voisin=[]

	def set_cam_coord(self,X,Y,Z):
		self.cam_X=X
		self.cam_Y=Y
		self.cam_Z=Z
		self.cam_yaw=0

	def set_previous_coord(self,X,Y,Z):
		self.previous_X=X
		self.previous_Y=Y
		self.previous_Z=Z
		self.leader=0 #### initier avec un topic ROS, un par formation
	
	def add_voisin(self,liste_drone):
    		self.voisin+=[liste_drone]
	
	def get_vitesse_X_Y_Z(self):
		return[(self.cam_X-self.previous_X)/0.02,(self.cam_Y-self.previous_Y)/0.02,(self.cam_Z-self.previous_Z)/0.02]
	
	def get_Cam_X_Y_Z(self):
    		return [self.cam_X,self.cam_Y,self.cam_Z]
	
	def calcul_distance(self,x,y):
    		return [(self.cam_X-x),(self.cam_Y-y)]

	def calcul_position_ideale(self,Drone,distance):
		norme=((self.cam_X-Drone.cam_X)**2+(self.cam_Y-Drone.cam_Y)**2)**(1/2)
		u=[(self.cam_X-Drone.cam_X)/norme, (self.cam_Y-Drone.cam_Y)/norme]
		return [u[0]*distance,u[1]*distance]
    		
	
	def intersection_2_cercle(self,xA,yA,xB,yB,d1,d2):
		solution1=[xA,yA] # nécessité traitement
		solution2=[xB,yB]
		if self.calcul_distance(solution1[0],solution1[1])<self.calcul_distance(solution2[0],solution2[1]):
    			return solution1
		else:
    			return solution2
	
	def traitement(self,position_desire): ### réutilisation du programme controle XYZ pour cette partie
		K1=0.6
		K2=0.4
		K3=1
		Kpyaw=-0.01
		self.Yawc=0
		VxC=K1*(position_desire[0]-self.cam_X)
		VyC=K1*(position_desire[1]-self.cam_Y)

		#Compute Vx, Vy from position derivatives
		Vx=(self.cam_X-self.previous_X)/0.02
		Vy=(self.cam_Y-self.previous_Y)/0.02
		Vyaw=(self.cam_yaw*180/3.1416-self.previous_camYaw_deg)/0.02
		
		self.previous_Y=self.cam_Y
		self.previous_X=self.cam_X
		self.previous_camYaw_deg=self.cam_yaw*180/3.1416

		#Compute speeds in UAV frame
		VxCdrone=cos(self.cam_yaw)*VxC+sin(self.cam_yaw)*VyC
		VyCdrone=-sin(self.cam_yaw)*VxC+cos(self.cam_yaw)*VyC
		Vxdrone=cos(self.cam_yaw)*Vx+sin(self.cam_yaw)*Vy
		Vydrone=-sin(self.cam_yaw)*Vx+cos(self.cam_yaw)*Vy


		#pitch, roll from VxC, VyC
		pitch_command=K2*(VxCdrone-Vxdrone)
		roll_command=-K2*(VyCdrone-Vydrone)
		
		#Altitude
		#vertical_velocity_command=K3*(self.Zc-self.cam_Z) arbitraire
		vertical_velocity_command=K3*(1-self.cam_Z)

		#Yaw
		#if((self.Yawc-(self.cam_yaw*180/3.1416))>180):
		#	yaw_velocity_command=Kpyaw*(self.Yawc-(self.cam_yaw*180/3.1416)-360)-abs(Kvyaw*(Vyaw))
		#elif((self.Yawc-(self.cam_yaw*180/3.1416))<-180):
		#	yaw_velocity_command=Kpyaw*(self.Yawc-(self.cam_yaw*180/3.1416)+360)-abs(Kvyaw*(Vyaw))
		#else:
		#	yaw_velocity_command=Kpyaw*(self.Yawc-(self.cam_yaw*180/3.1416))-abs(Kvyaw*(Vyaw))
		
		yaw_velocity_command=Kpyaw*(self.Yawc-(self.cam_yaw*180/3.1416))
		return [pitch_command,roll_command,vertical_velocity_command,yaw_velocity_command]

class Main_drone(Sec_drone):
    #def __init__(self,cam_X,cam_Y,cam_Z,cam_pitch,cam_roll,cam_yaw,bebop_roll,bebop_pitch):
	def __init__(self):
		super().__init__()
		#Bebop Optitrack orientation (in bebop ref frame)
		self.cam_pitch=0
		self.cam_roll=0
		self.cam_yaw=0
		#Bebop IMU pose
		self.bebop_roll=0
		self.bebop_pitch=0
		#Previous position (to compute velocity)
		self.previous_camYaw_deg=0
		#Start / stop control variable
		self.control_enable=0

	def set_cam_pitch_roll_yaw(self,pitch,roll,yaw):
		self.cam_pitch=pitch
		self.cam_roll=roll
		self.cam_yaw=yaw
	
	def calcul_distance(self,Drone):
		return [(self.cam_X-Drone.cam_X),(self.cam_Y-Drone.cam_Y),(self.cam_Z-Drone.cam_Z)]

class controler:

	#bebop reference frame angle orientation
	#Pitch: + when bebop forward, - when backward
	#roll: + when bebop on the left, - on the right
	#yaw: - when rotate clockwise


	#variables init

	#Connexion matrix
	A = np.array([[0, 1,1], [1, 0,1],[1,1,0]]) 
	dflock=1.5 #distance between agents in flock	
	
	agent_id=rospy.get_param('robot_id') 
	g=9.8
	
	Drone1=Main_drone()
	Drone2=Sec_drone()
	Drone3=Sec_drone()


	def __init__(self):
		rospy.init_node('agentflockingcontroler'+rospy.get_param('robot_name')) # Init ROS node
		rate=rospy.Rate(50) # Set rate 50hz
		rospy.loginfo(str(self.agent_id)+'Agent flocking controler started, wait for camera position')

		#Subscribers: 
		#Inputs: 
		# bebop position from camera: geometry_msgs/PoseStamped
		# bebop internal IMU orientation: nav_msgs/Odometry
		
		#get camera topic with tag name
		camera_position_topic1="/vrpn_client_node/"+rospy.get_param('tag_name_robot1')+"/pose"
		camera_position_topic2="/vrpn_client_node/"+rospy.get_param('tag_name_robot2')+"/pose"
		camera_position_topic3="/vrpn_client_node/"+rospy.get_param('tag_name_robot3')+"/pose"

		self.cam_pos_sub1=rospy.Subscriber(camera_position_topic1,PoseStamped,self.cam_pos_received1)
		self.cam_pos_sub2=rospy.Subscriber(camera_position_topic2,PoseStamped,self.cam_pos_received2)
		self.cam_pos_sub3=rospy.Subscriber(camera_position_topic3,PoseStamped,self.cam_pos_received3)

		#Publishers
		#Outputs: 
		# Roll, pitch, yaw: geometry_msgs/Twist
		self.commands_pub=rospy.Publisher(BEBOP_COMMANDS_TOPIC,Twist,queue_size=1)

		#services
		#Provide service to change target position, start and stop control
		s=rospy.Service('start_control',Trigger,self.handle_start_control);
		s=rospy.Service('stop_control',Trigger,self.handle_stop_control);
	
	
		while not rospy.is_shutdown():

			#Compute and publish control: pitch, roll, yaw			
			self.control()
			rate.sleep()

	
	def control(self):
		#Compute control

		#Init commadns (send zero if control disabled)
		yaw_velocity_command=0	#maintain yaw if 0
		vertical_velocity_command=0 #maintain altitude if 0
		pitch_command=0
		roll_command=0

		#A AJOUTER: recupération des positions de tous les drones (second temps car pr l'instant stockes en dur)
		#Nommer les topics avec les namespace?


		if(self.Drone1.control_enable==1):


			[x1,y1,z1]=self.Drone1.get_Cam_X_Y_Z()
			[vx1,vy1,vz1]=self.Drone1.get_vitesse_X_Y_Z()


			[x2,y2,z2]=self.Drone2.get_Cam_X_Y_Z()
			[vx2,vy2,vz2]=self.Drone2.get_vitesse_X_Y_Z()


			[x3,y3,z3]=self.Drone3.get_Cam_X_Y_Z()
			[vx3,vy3,vz3]=self.Drone3.get_vitesse_X_Y_Z()

			
			self.Drone1.set_previous_coord(x1,y1,z1)
			self.Drone2.set_previous_coord(x2,y2,z2)
			self.Drone3.set_previous_coord(x3,y3,z3)

			#récupération leader
        if self.Drone1.leader==1:
        # remain STable
            
        else:
            ### objectif
            objectif=[0,0,0] #x,y,poid total
            ### définition de l'écart, ici 1m
            ### définition de l'insertitude, ici 10 cm
            delta=10
            ##########################################################
            ################## implémentation urgence  ###############
            #########################################################
            urgence = 70
            ### définition de l'importancen de s'écarter contre se rapprocher
            K_eloignement=0.5
            K_urgence=15
            ### décompte voisin trop proche, trop loin
            voisin_trop_proche=[]
            voisin_trop_loin=[]
            voisin_urgence=[]
            for voisin in self.voisin:
                d=self.calcul_distance(voisin[0].cam_X,voisin[0].cam_Y)
                if d<voisin[1]*0.9:
                    if d<urgence:
                        voisin_urgence+=[[voisin,d]]
                    else:
                        voisin_trop_proche+=[voisin]
                elif d>voisin[1]*1.1:
                        voisin_trop_loin+=[voisin]

            for drone in self.liste_drone:
                if drone not in self.voisin:
                    d=self.calcul_distance(drone.cam_X,drone.cam_Y)
                    if d<urgence and d!=0:
                        voisin_urgence+=[[[drone,urgence],d]] ##################BUGGGGGGG,d

            
            for voisin in voisin_trop_proche:
                new_position=self.calcul_position_ideale(voisin[0],voisin[1])
                objectif=addition_terme_poid(objectif,new_position+[1])  				
              				
            for voisin in voisin_urgence:
                new_position=self.calcul_position_ideale(voisin[0][0],voisin[1])
                objectif=addition_terme_poid(objectif,new_position+[K_urgence*(urgence/d)**2])  

            for voisin in voisin_trop_loin:
                new_position=self.calcul_position_ideale(voisin[0],voisin[1])
                objectif=addition_terme_poid(objectif,new_position+[K_eloignement]) 
            if objectif[2]==0:        
                self.fen.after(20,self.calcul_position)
            else :			
                vrai_objectif=[objectif[0]/objectif[2],objectif[1]/objectif[2]] ### division par le poids
                [pitch_command,roll_command,vertical_velocity_command,yaw_velocity_command]=self.Drone1.traitement(vrai_objectif)





			
			#for file
			#rospy.loginfo(str(self.agent_id)+"X1 %s Y1 %s Z1 %s X2 %s Y2 %s Z2 %s HX1 %s HY1 %s HZ1 %s HX2 %s HY2 %s HZ2 %s VX1 %s VY1 %s VZ1 %s VX2 %s VY2 %s VZ2 %s U1 %s U2 %s U3 %s T %s Phid %s Thetad %s", str(x1),str(y1),str(z1), str(x2),str(y2),str(z2),str(H1[0]),str(H1[1]),str(H1[2]), str(H2[0]),str(H2[1]),str(H2[2]),str(vx1),str(vy1),str(vz1), str(vx2),str(vy2),str(vz2), str(U[0]),str(U[1]),str(U[2]), str(T),str(Phid),str(Thetad))

			#TODO: tune regarding max angle? Degree or radian?

		#send commands
		command_msg=Twist()
		command_msg.linear.x=pitch_command #Pitch
		command_msg.linear.y=roll_command #Roll
		command_msg.linear.z=vertical_velocity_command #Vertical velocity
		command_msg.angular.z=yaw_velocity_command #Yaw
		
		#rospy.loginfo("roll %s pitch %s yaw %s", self.cam_roll,self.cam_pitch,self.cam_yaw)

		self.commands_pub.publish(command_msg)

		#rospy.loginfo(str(self.agent_id)+"Commands Pitch %s Roll %s DZ %s Dyaw %s",str(command_msg.linear.x),str(command_msg.linear.y),str(command_msg.linear.z),str(yaw_velocity_command));
		#rospy.loginfo("camyaw %s yawc %s com_yawp %s",str(self.cam_yaw*180/3.1416),str(self.Yawc),str(yaw_velocity_command))
		#rospy.loginfo("X %s Y %s Z %s Xc %s Yc %s Zc %s",str(self.cam_X),str(self.cam_Y),str(self.cam_Z),str(self.Xc),str(self.Yc), str(self.Zc));
		#rospy.loginfo("VxC %s VyC %s Pitch %s Roll %s vertical_volocity %s",str(VxC),str(VyC),str(pitch_command),str(roll_command), str(vertical_velocity_command));

	def cam_pos_received1(self,data):
			self.cam_pos_received(self.Drone1,data)

	def cam_pos_received2(self,data):
    		self.cam_pos_received(self.Drone2,data)

	def cam_pos_received3(self,data):
		self.cam_pos_received(self.Drone3,data)

	def cam_pos_received(self,Drone,data):
		#Position received from cameras

		Drone.set_cam_coord(data.pose.position.x,-data.pose.position.y,data.pose.position.z)

		#Orientation received by cameras (quaternion)
		cam_orientation_x=data.pose.orientation.x
		cam_orientation_y=data.pose.orientation.y
		cam_orientation_z=data.pose.orientation.z
		cam_orientation_w=data.pose.orientation.w

		#get euler angles from quaternions
		angles=euler_from_quaternion([cam_orientation_w,cam_orientation_x,cam_orientation_y,cam_orientation_z]);

		#Transpose camera angles to get roll, pitch, yaw
		Drone.set_cam_pitch_roll_yaw(-angles[1],-angles[2],angles[0]+3.1416 )

		if (Drone.cam_yaw>3.1416):
			Drone.cam_yaw=Drone.cam_yaw-2*3.1416

		
		if (Drone.cam_position_received==0):
			rospy.loginfo(str(self.agent_id)+"Camera position 1 received")
			#Set received flag to 1
			Drone.cam_position_received=1

	def handle_start_control(self,req):
		#Start control service
		if ((self.Drone1.cam_position_received==1)&(self.Drone2.cam_position_received==1)&(self.Drone3.cam_position_received==1)):
			self.control_enable=1
			rospy.loginfo(str(self.agent_id)+"Start control")
		else:
			rospy.loginfo(str(self.agent_id)+"Impossible to start control: no camera position received")
		return [1,""]


	def handle_stop_control(self,req):
		#Stop control service
		self.Drone1.control_enable=0
		rospy.loginfo(str(self.agent_id)+"Stop control");
		return [1,""]


if __name__ == '__main__':
	try:	
		controler()
	except rospy.ROSInterruptException:
		pass