#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
Software License Agreement (BSD)

file      patrol_obstacle.py
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

''' '''

from math import cos, sin
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist , PoseStamped
from random import randint, random
from bebop_control.srv import *
from std_srvs.srv import Trigger

from tf.transformationpatrol_obstacle.pys import euler_from_quaternion


BEBOP_COMMANDS_TOPIC="/bebop/cmd_vel"
class Sec_drone:
    def __init__(self,liste_drone,leader):
        self.liste_drone=liste_drone
        #Agent Optitrack position and orientation
        self.cam_X=0
        self.cam_Y=0
        self.cam_Z=0
        self.cam_yaw=0
        #flags
        self.Yawc=0
        self.pitch_command=0
        self.roll_command=0
        self.flag=False
        self.vertical_velocity_command=0
        self.yaw_velocity_command=0
        self.cam_position_received=0
        self.leader=leader
        self.control_enable=0 
        self.patrol_st=0
        self.i=0
        #Previous position (to compute velocity)
        self.previous_X=0
        self.previous_Y=0
        self.previous_Z=0
        #fly modifications
        self.curent_obj=[1,-1] # first point to reach before lauching the patrol
        self.obstacles=[[0.8,0.4]] #list of different obstacles positions
        self.K_objective=1 #strength of the objective atraction

        #ROS initialisation (services and topics)
        rospy.init_node('bebopcontroler', anonymous=True)
        rospy.loginfo('Bebop controler started, wait for camera position')
        self.pub=rospy.Publisher(BEBOP_COMMANDS_TOPIC,Twist,queue_size=1)

        camera_position_topic="/vrpn_client_node/"+rospy.get_param('tag_name')+"/pose"
        self.cam_pos_sub=rospy.Subscriber(camera_position_topic,PoseStamped,self.cam_pos_received)
        self.rate = rospy.Rate(10) # 10hz
        s=rospy.Service('set_target_position',SetTargetPos,self.handle_set_target_position)
        s=rospy.Service('start_control',Trigger,self.handle_start_control)
        s=rospy.Service('stop_control',Trigger,self.handle_stop_control)
        self.publish()

    def publish(self):
        ''' publish the different commands to the bebop'''
        while not rospy.is_shutdown():
            if self.flag==True:
                self.objectif()
                command_msg=Twist()
                command_msg.linear.x=self.pitch_command#Pitch
                command_msg.linear.y=self.roll_command #Roll
                command_msg.linear.z=self.vertical_velocity_command
                command_msg.angular.z=self.yaw_velocity_command
                self.pub.publish(command_msg)
                self.rate.sleep()
        rospy.spin()

    def cam_pos_received(self,data):
        '''Position received from cameras'''
        self.set_previous_coord(self.cam_X,self.cam_Y,self.cam_Z)
        self.set_cam_coord(data.pose.position.x,-data.pose.position.y,data.pose.position.z)

        #Orientation received by cameras (quaternion)
        cam_orientation_x=data.pose.orientation.x
        cam_orientation_y=data.pose.orientation.y
        cam_orientation_z=data.pose.orientation.z
        cam_orientation_w=data.pose.orientation.w

        #get euler angles from quaternions
        angles=euler_from_quaternion([cam_orientation_w,cam_orientation_x,cam_orientation_y,cam_orientation_z]);

        #Transpose camera angles to get roll, pitch, yaw
        self.cam_pitch=-angles[1]
        self.cam_roll=-angles[2]
        self.cam_yaw=angles[0]+3.1416 #To force yaw zero forward

        if (self.cam_yaw>3.1416):
            self.cam_yaw=self.cam_yaw-2*3.1416

        if (self.cam_position_received==0):
            rospy.loginfo("Camera position received")

            #Set received flag to 1
            self.cam_position_received=1
            
            

    def set_cam_coord(self,X,Y,Z):
        self.cam_X=X
        self.cam_Y=Y
        self.cam_Z=Z

    def set_previous_coord(self,X,Y,Z):
        self.previous_X=X  
        self.previous_Y=Y
        self.previous_Z=Z
    

    
    
#### initier avec un topic ROS, un par formation

    def get_vitesse_X_Y_Z(self):
        return[(self.cam_X-self.previous_X)/0.02,(self.cam_Y-self.previous_Y)/0.02,(self.cam_Z-self.previous_Z)/0.02]

    def get_Cam_X_Y_Z(self):
        return [self.cam_X,self.cam_Y,self.cam_Z]
    
    def calcul_distance(self,x,y):
        return abs(self.cam_X-x)+abs(self.cam_Y-y)

            
    def objectif(self):
        '''function that give the objective to reach for the bebop, modifiing the curent_obj atribute and
        launch the calcul of the movement of the drone'''
        if abs(self.cam_X-self.curent_obj[0])+abs(self.cam_Y-self.curent_obj[1])<0.15:
            if self.i==0:
                self.curent_obj=[1,-1]
                self.i=1
            else:
                self.curent_obj=[1,1.8]
                self.i=0
        else:
            self.new_calcul_obstacle([self.curent_obj[0],self.curent_obj[1],1])
                

    def force_objectif(self,position_obj):
        K=self.K_objective
        norme=((self.cam_X-position_obj[0])**2+(self.cam_Y-position_obj[1])**2)**(1/2)
        u=[(self.cam_X-position_obj[0])/norme, (self.cam_Y-position_obj[1])/norme]
        return [-u[0]*K,-u[1]*K]


    def calcul_force_repulsion(self,position_ob,urgence,distance,K):
        norme=((self.cam_X-position_ob[0])**2+(self.cam_Y-position_ob[1])**2)**(1/2)
        u=[(self.cam_X-position_ob[0])/norme, (self.cam_Y-position_ob[1])/norme]
        return [u[0]*K*(urgence/distance)**2,u[1]*K*(urgence/distance)**2]


    def new_calcul_obstacle(self,objectif):
        if self.patrol_st==1 and self.control_enable==1:
            force=self.force_objectif([objectif[0],objectif[1]])#
            ################## implémentation urgence  ##############

            # objectif=[0,0,0] #x,y,poid total
            ### définition de l'écart, ici 1m
            ### définition de l'insertitude, ici 10 cm
            ##########################################################
            ################## implémentation urgence  ###############
            #########################################################
            distance_unit=0.70
            urgence = 1.2
            ### définition de l'importancen de s'écarter contre se rapprocher
            K_urgence=0.7
            ### décompte voisin trop proche, trop loin
            voisin_urgence=[]
            for voisin in self.obstacles:
                d=self.calcul_distance(voisin[0],voisin[1])
                rospy.loginfo(d)
                if d<urgence:
                    distance_unit=0.30
                    voisin_urgence+=[[voisin,d]]
            for voisin in voisin_urgence:
                new_force=self.calcul_force_repulsion(voisin[0],urgence,voisin[1],K_urgence)
                force=[force[0]+new_force[0],force[1]+new_force[1]]
            norme=abs(force[0])+abs(force[1])
            force=[force[0]/norme,force[1]/norme]
            vrai_objectif=[self.cam_X+distance_unit*force[0],self.cam_Y+distance_unit*force[1]] ### division par le poids
            self.traduction_angle(vrai_objectif[0],vrai_objectif[1],1)
        else:
            self.traduction_angle(self.cam_X,self.cam_Y,1)


    def traduction_angle(self,x,y,z):
        K1=0.6
        K2=0.4
        K3=1
        Kpyaw=-0.01

        VxC=K1*(x-self.cam_X)
        VyC=K1*(y-self.cam_Y)

        #Compute Vx, Vy from position derivatives
        Vx=(self.cam_X-self.previous_X)/0.02
        Vy=(self.cam_Y-self.previous_Y)/0.02
        
        self.previous_Y=self.cam_Y
        self.previous_X=self.cam_X
        self.previous_camYaw_deg=self.cam_yaw*180/3.1416

        #Compute speeds in UAV frame
        VxCdrone=cos(self.cam_yaw)*VxC+sin(self.cam_yaw)*VyC
        VyCdrone=-sin(self.cam_yaw)*VxC+cos(self.cam_yaw)*VyC
        Vxdrone=cos(self.cam_yaw)*Vx+sin(self.cam_yaw)*Vy
        Vydrone=-sin(self.cam_yaw)*Vx+cos(self.cam_yaw)*Vy


        #pitch, roll from VxC, VyC
        self.pitch_command=K2*(VxCdrone-Vxdrone)
    
        self.roll_command=-K2*(VyCdrone-Vydrone)
        
        #Altitude
        self.vertical_velocity_command=K3*(z-self.cam_Z)
        self.yaw_velocity_command=Kpyaw*(self.Yawc-(self.cam_yaw*180/3.1416))
        

    def handle_set_target_position(self,req):
        #Change target position 
        self.patrol_st=1
        self.flag=True
        return [1]

    def handle_start_control(self,req):
            #Start control service
        if (self.cam_position_received==1):
            self.control_enable=1
            rospy.loginfo("Start control")
        else:
            rospy.loginfo("Impossible to start control: no camera position received")
        return [1,""]


    def handle_stop_control(self,req):
    #Stop control service
        self.control_enable=0
        self.flag=False
        rospy.loginfo("Stop control");
        return [1,""]
    
def addition_terme_poid(l1,l2):
    total=[]
    for i in range(len(l1)-1):
        total+=[l1[i]+l2[i]*l2[len(l2)-1]]
    total+=[l1[len(l1)-1]+l2[len(l2)-1]]
    return total




def talker():

    Drone=Sec_drone([],1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass