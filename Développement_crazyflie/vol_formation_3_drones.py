# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017-2018 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Version of the AutonomousSequence.py example connecting to 10 Crazyflies.
The Crazyflies go straight up, hover a while and land but the code is fairly
generic and each Crazyflie has its own sequence of setpoints that it files
to.
The layout of the positions:
    x2      x1      x0
y3  10              4
            ^ Y
            |
y2  9       6       3
            |
            +------> X
y1  8       5       2
y0  7               1
"""
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from math import cos, sin
from random import randint, random

global Dict_link_Sec_drone
Dict_link_Sec_drone={}

# Change uris and sequences according to your setup
URI1 = 'radio://0/80/2M/E7E7E7E700'
URI2 = 'radio://0/80/2M/E7E7E7E701'
URI3 = 'radio://0/80/2M/E7E7E7E703'


# List of URIs, comment the one you do not want to fly
uris = [
    URI1,
    URI2,
    URI3,
]


def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)


class Sec_drone:
    def __init__(self,leader):
        #Agent Optitrack position
        self.cam_X=0
        self.cam_Y=0
        self.cam_Z=0
        #flags
        self.curent_obj=[0,0]
        self.Yawc=0
        self.pitch_command=0
        self.roll_command=0
        self.flag=False
        self.vertical_velocity_command=0
        self.yaw_velocity_command=0
        self.voisin=[]
        self.cam_position_received=0
        #Previous position (to compute velocity)
        self.previous_X=0
        self.previous_Y=0
        self.previous_Z=0
        #self.set_cam_coord(x,y,0)
        self.cam_yaw=0
        #Voisin proch
        self.leader=leader
        self.control_enable=0 
        self.patrol_st=0
        self.i=0
        self.tmp_point=[0,0,0.2]
        self.origin=[0,0,0]


    def set_cam_coord(self,X,Y,Z):
        if self.origin==[0,0,0]:
            self.origin=[X,Y,Z]
        self.cam_X=X
        self.cam_Y=Y
        self.cam_Z=Z

    def set_previous_coord(self,X,Y,Z):
        self.previous_X=X  
        self.previous_Y=Y
        self.previous_Z=Z
    
    def add_voisin(self,voisin):
        self.voisin.append([voisin,0.7])
#### initier avec un topic ROS, un par formation

    def get_vitesse_X_Y_Z(self):
        return[(self.cam_X-self.previous_X)/0.02,(self.cam_Y-self.previous_Y)/0.02,(self.cam_Z-self.previous_Z)/0.02]

    def get_Cam_X_Y_Z(self):
            return [self.cam_X,self.cam_Y,self.cam_Z]
    
    def calcul_distance(self,x,y):
            return ((self.cam_X-x)**2+(self.cam_Y-y)**2)**(1/2)
      
                
    def force_objectif(self,position_obj):
        K=1
        norme=((self.cam_X-position_obj[0])**2+(self.cam_Y-position_obj[1])**2)**(1/2)
        u=[(self.cam_X-position_obj[0])/norme, (self.cam_Y-position_obj[1])/norme]
        return [-u[0]*K,-u[1]*K]


    def calcul_force_repulsion_urgence(self,position_ob,urgence,distance,K):
        norme=((self.cam_X-position_ob[0])**2+(self.cam_Y-position_ob[1])**2)**(1/2)
        u=[(self.cam_X-position_ob[0])/norme, (self.cam_Y-position_ob[1])/norme]
        return [u[0]*K*(urgence/distance)**2,u[1]*K*(urgence/distance)**2]

    def calcul_force_ideale(self,Drone,distance):
        norme=((self.cam_X-Drone.cam_X)**2+(self.cam_Y-Drone.cam_Y)**2)**(1/2)
        u=[(self.cam_X-Drone.cam_X)/norme, (self.cam_Y-Drone.cam_Y)/norme]
        positon_ideal=[Drone.cam_X+u[0]*distance,Drone.cam_Y+u[1]*distance]
        return [positon_ideal[0]-self.cam_X,positon_ideal[1]-self.cam_Y]

    def new_calcul(self,c_time):
            if self.leader==1:
                if c_time//10==0:
                    self.tmp_point=[self.origin[0],self.origin[1],0.2]
                else :
                    self.tmp_point=[self.origin[0]+0.3,self.origin[1]+0.3,0.2]
            else :
                force=[0,0]
                distance_unit=0.7
                urgence = 0.1
                delta=0.1
                ### définition de l'importance de s'écarter contre se rapprocher
                K_eloignement=0.5
                K_urgence=0.7

                ### décompte voisin trop proche, trop loin
                voisin_trop_proche=[]
                voisin_trop_loin=[]
                voisin_urgence=[]
                for voisin in self.voisin:
                    d=self.calcul_distance(voisin[0].cam_X,voisin[0].cam_Y)
                    print(d)
                    if d<voisin[1]*(1-delta):
                        if d<urgence:
                            voisin_urgence+=[[voisin,d]]
                        else:
                            voisin_trop_proche+=[voisin]
                    elif d>voisin[1]*(1+delta):
                        voisin_trop_loin+=[voisin]

                for voisin in voisin_urgence:
                    print("trop proche")
                    new_force=self.calcul_force_repulsion_urgence(voisin[0],urgence,voisin[1],K_urgence)
                    force=addition_terme_poid(force,new_force+[1]) 
                    
                for voisin in voisin_trop_proche:
                    print("trop proche")
                    new_position=self.calcul_force_ideale(voisin[0],voisin[1])
                    force=addition_terme_poid(force,new_position+[K_eloignement]) 				
              				
                for voisin in voisin_trop_loin:
                    print("trop loin")
                    new_position=self.calcul_force_ideale(voisin[0],voisin[1])
                    force=addition_terme_poid(force,new_position+[K_eloignement]) 
                norme=(force[0]**2+(force[1])**2)**(1/2)
                if norme!=0:
                    vrai_objectif=[self.cam_X+distance_unit*force[0],self.cam_Y+distance_unit*force[1]]
                    self.tmp_point=[vrai_objectif[0],vrai_objectif[1],0.2]
                else :
                    if self.calcul_distance(self.origin[0],self.origin[1])>0.1:
                        self.origin=[self.cam_X,self.cam_Y,0.2]
                    self.tmp_point=[self.origin[0],self.origin[1],0.2]


def addition_terme_poid(l1,l2):
    total=[]
    for i in range(len(l2)-1):
        total+=[l1[i]+l2[i]*l2[len(l2)-1]]
    return total

def seq_unaire(Drone,cf,c_time):

    coord=Drone.get_Cam_X_Y_Z()
    Drone.set_previous_coord(coord[0],coord[1],coord[2])
    position=swarm.get_estimated_positions()[cf.link_uri]
    Drone.set_cam_coord(position[0],position[1],position[2])
    Drone.new_calcul(c_time)
    objectif_tmp=Drone.tmp_point
    cf.commander.send_position_setpoint(objectif_tmp[0],objectif_tmp[1],objectif_tmp[2], 0)



def run_sequence(scf):
    global Dict_link_Sec_drone
    cf = scf.cf
    drone=Dict_link_Sec_drone[cf.link_uri]
    end_time = time.time() + 30
    while time.time() < end_time:
        print(time.time() - end_time)
        c_time=time.time()-end_time+30
        seq_unaire(drone,cf,c_time)
        time.sleep(0.2)

    

    
    #sequence à effectuer



if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        # If the copters are started in their correct positions this is
        # probably not needed. The Kalman filter will have time to converge
        # any way since it takes a while to start them all up and connect. We
        # keep the code here to illustrate how to do it.
        # swarm.reset_estimators()

        # The current values of all parameters are downloaded as a part of the
        # connections sequence. Since we have 10 copters this is clogging up
        # communication and we have to wait for it to finish before we start
        # flying.
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(wait_for_param_download)
        ## attribution des drones à une url
        for i in range(len(uris)):
            if i==1:
                Dict_link_Sec_drone[uris[i]]=Sec_drone(1)
            else:
                Dict_link_Sec_drone[uris[i]]=Sec_drone(0)
        ## attribution des voisins
        for uri1 in uris:
            for uri2 in uris:
                if uri1!=uri2:
                    Dict_link_Sec_drone[uri1].add_voisin(Dict_link_Sec_drone[uri2])
            
        swarm.parallel(run_sequence)

    





BEBOP_COMMANDS_TOPIC="/bebop/cmd_vel"
