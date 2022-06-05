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

import rospy
from math import *
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from bebop_control.srv import *
from std_srvs.srv import Trigger


#const variables (do not modify): 
BEBOP_COMMANDS_TOPIC="/bebop/cmd_vel"
K1=0.6
K2=0.4
K3=1
Kpyaw=-0.01 #Negative to fit with cbebop command orientation (positive yawp command-> rotation counter clockwise)


class controler:

	#bebop reference frame angle orientation
	#Pitch: + when bebop forward, - when backward
	#roll: + when bebop on the left, - on the right
	#yaw: - when rotate clockwise


    #target position
    Xc=0
    Yc=0
    Zc=0

    #traget yaw
    Yawc=0

    #Bebop Optitrack position
    cam_X=0
    cam_Y=0
    cam_Z=0

    #Bebop Optitrack orientation (in bebop ref frame)
    cam_pitch=0
    cam_roll=0
    cam_yaw=0

    #Bebop IMU pose
    bebop_roll=0
    bebop_pitch=0

    #flags
    cam_position_received=0
    imu_pose_received=0
    wanted_position_recorded=0

    #Previous position (to compute velocity)
    previous_X=0
    previous_Y=0
    previous_Z=0
    previous_camYaw_deg=0

    #Start / stop control variable
    control_enable=0

    #weight
    mass=0.5
    g=9.8
    T_0=mass*g

    patrol_rot=[[1,0,0],[-1,0,0]]
    id_curent_rot=0
    patrol_st=False

	

    def __init__(self):
        rospy.init_node('bebopcontroler', anonymous=True) # Init ROS node
        rate=rospy.Rate(50) # Set rate 50hz
        rospy.loginfo('Bebop controler started, wait for camera position')

        #Subscribers: 
        #Inputs: 
        # bebop position from camera: geometry_msgs/PoseStamped
        # bebop internal IMU orientation: nav_msgs/Odometry
        
        #get camera topic with tag name
        camera_position_topic="/vrpn_client_node/"+rospy.get_param('tag_name')+"/pose"
        self.cam_pos_sub=rospy.Subscriber(camera_position_topic,PoseStamped,self.cam_pos_received)


        #Publishers
        #Outputs: 
        # Roll, pitch, yaw: geometry_msgs/Twist
        self.commands_pub=rospy.Publisher(BEBOP_COMMANDS_TOPIC,Twist,queue_size=1)

        #services        self.commands_pub=rospy.Publisher(BEBOP_COMMANDS_TOPIC,Twist,queue_size=1)
        #Provide service to change target position, start and stop control
        #TODO: Namespace
        s=rospy.Service('set_target_position',SetTargetPos,self.handle_set_target_position);
        s=rospy.Service('start_control',Trigger,self.handle_start_control);
        s=rospy.Service('stop_control',Trigger,self.handle_stop_control);
	
	
        while not rospy.is_shutdown():

            #Compute and publish control: pitch, roll, yaw			
            self.control()
            rate.sleep()

	
    def control(self):
		#Compute control
        yaw_velocity_command=0	#maintain yaw if 0
        vertical_velocity_command=0 #maintain altitude if 0

        if(self.control_enable==1):
            self.launch_patrol()

            if(self.wanted_position_recorded==0):
                self.Xc=self.cam_X
                self.Yc=self.cam_Y
                self.Zc=self.cam_Z
                self.Yawc=self.cam_yaw*180/3.1416
                self.wanted_position_recorded=1


            #Compute commands
            #Vx, Vy, VxC, VyC are in camera frame
            #Vxdrone, Vydrone, VxCdrone, VyCdrone are in UAV frame (X front, Y on the right)
            #Vxdrone=cos(yaw)*Vxcam+sin(yaw)*Vycam
            #Vydrone=-sin(yaw)*Vxcam+cos(yaw)*Vycam
            # with yaw = angle between Xcam and Xuav

			#VxC,VyC from position error and target position
            VxC=K1*(self.Xc-self.cam_X)
            VyC=K1*(self.Yc-self.cam_Y)

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
            vertical_velocity_command=K3*(self.Zc-self.cam_Z)

            #Yaw
            #if((self.Yawc-(self.cam_yaw*180/3.1416))>180):
            #	yaw_velocity_command=Kpyaw*(self.Yawc-(self.cam_yaw*180/3.1416)-360)-abs(Kvyaw*(Vyaw))
            #elif((self.Yawc-(self.cam_yaw*180/3.1416))<-180):
            #	yaw_velocity_command=Kpyaw*(self.Yawc-(self.cam_yaw*180/3.1416)+360)-abs(Kvyaw*(Vyaw))
            #else:
            #	yaw_velocity_command=Kpyaw*(self.Yawc-(self.cam_yaw*180/3.1416))-abs(Kvyaw*(Vyaw))
            
            yaw_velocity_command=Kpyaw*(self.Yawc-(self.cam_yaw*180/3.1416))

            #send commands
            command_msg=Twist()
            command_msg.linear.x=pitch_command #Pitch
            command_msg.linear.y=roll_command #Roll
            command_msg.linear.z=vertical_velocity_command #Vertical velocity
            
            #command_msg.angular.z=0
            command_msg.angular.z=yaw_velocity_command #Yaw



            #rospy.loginfo("X %s Y %s Z %s", self.cam_X,self.cam_Y,self.cam_Z)
            #rospy.loginfo("roll %s pitch %s yaw %s", self.cam_roll,self.cam_pitch,self.cam_yaw)

            self.commands_pub.publish(command_msg)
            #rospy.loginfo("Send commands Pitch %s Roll %s DZ %s Dyaw %s",str(command_msg.linear.x),str(command_msg.linear.y),str(command_msg.linear.z),str(yaw_velocity_command));
            #rospy.loginfo("camyaw %s yawc %s com_yawp %s",str(self.cam_yaw*180/3.1416),str(self.Yawc),str(yaw_velocity_command))
            #rospy.loginfo("X %s Y %s Z %s Xc %s Yc %s Zc %s",str(self.cam_X),str(self.cam_Y),str(self.cam_Z),str(self.Xc),str(self.Yc), str(self.Zc));
            #rospy.loginfo("VxC %s VyC %s Pitch %s Roll %s vertical_volocity %s",str(VxC),str(VyC),str(pitch_command),str(roll_command), str(vertical_velocity_command));


    def cam_pos_received(self,data):
        #Position received from cameras

        self.cam_X=data.pose.position.x
        self.cam_Y=-data.pose.position.y
        self.cam_Z=data.pose.position.z
        #rospy.loginfo("Position received from cameras X %s Y %s Z %s",str(self.cam_X),str(self.cam_Y),str(self.cam_Z));

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

        #Angles in degrees
        pitch_deg=-angles[1]*180/3.1416
        roll_deg=-angles[2]*180/3.1416
        yaw_deg=angles[0]*180/3.1416+180
        
        #rospy.loginfo("from cam: roll %s pitch %s yaw %s", roll_deg,pitch_deg,yaw_deg)

        if (self.cam_position_received==0):
            rospy.loginfo("Camera position received")

        #Set received flag to 1
        self.cam_position_received=1

    #def imu_pose_received(self,data):
        #Pose received from Bebop IMU
        #rospy.loginfo("Pose received from IMU X %s Y %s Z %s",str(data.twist.twist.angular.x),str(data.twist.twist.angular.y),str(data.twist.twist.angular.z));

    def distance(self,coord1,coord2):
        return ((coord1[0]-coord2[0])**2+(coord1[1]-coord2[1])**2+(coord1[2]-coord2[2])**2)**(1/2)

    def handle_set_target_position(self,req):
        #Change target position 
        self.patrol_st=1

    #rospy.loginfo("Target position changed Xc %s Yc %s Zc %s Yawc %s",str(self.Xc),str(self.Yc), str(self.Zc), str(req.yawc));
        return 1

    def launch_patrol(self):
        if self.patrol_st==1:
            self.wanted_position_recorded=1
            if self.distance(self.patrol_rot[self.id_curent_rot],[self.cam_X,self.cam_Y,self.cam_Z])<0.2: #unité? 20 cm voulu ici
                if len(self.patrol_rot)-1==self.id_curent_rot:
                    self.id_curent_rot=0
                else :
                    self.id_curent_rot+=1
                    [self.Xc,self.Yc,self.Zc]=self.patrol_rot[self.id_curent_rot]
                    #self.Yawc=0		
                    self.Yawc=0


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
        rospy.loginfo("Stop control");
        return [1,""]


if __name__ == '__main__':
    try:	
       controler()
    except rospy.ROSInterruptException:
      pass
