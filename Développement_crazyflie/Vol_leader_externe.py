'''This code will launch a group of drones in a formation. The leader will not move, but the followers will
the position of the leader (if it moves with an exterior way)'''
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from math import cos, sin
from random import randint, random

global Dict_link_Sec_drone
Dict_link_Sec_drone={}

# Change uris and sequences according to your setup
URI1 = 'radio://0/80/2M/E7E7E7E7E7'
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
        #fly modifications
        self.distance_between_drone=0.7 #distance between drones in the formation
        self.K_objectif=1 #strength of the force of the objectif
        self.K_urgence=0.7 #strength of the force of the repulsion in emergency situation
        self.K_eloignement=0.5#strength of the force of the repulsion and attraction of the drones
        self.distance_unit=0.9 #strength of the movements
        self.urgence = 0.1 #distance of the emergency situation
        self.incertitude=0.1 #% of incertitude on the poisition tolerated



    def set_cam_coord(self,X,Y,Z):
        ''' function which set the position atributes of the drone'''
        if self.origin==[0,0,0]:
            self.origin=[X,Y,Z]
        self.cam_X=X
        self.cam_Y=Y
        self.cam_Z=Z

    def set_previous_coord(self,X,Y,Z):
        ''' function which set the position previous atributes of the drone'''
        self.previous_X=X  
        self.previous_Y=Y
        self.previous_Z=Z
    
    def add_voisin(self,voisin):
        '''fuctions which add a voisin to the list of voisin o the drone'''
        self.voisin.append([voisin,self.distance_between_drone])

    def get_vitesse_X_Y_Z(self):
        ''' function which return the velocity of the drone'''
        return[(self.cam_X-self.previous_X)/0.02,(self.cam_Y-self.previous_Y)/0.02,(self.cam_Z-self.previous_Z)/0.02]

    def get_Cam_X_Y_Z(self):
        ''' function which return the position of the drone'''
        return [self.cam_X,self.cam_Y,self.cam_Z]
    
    def calcul_distance(self,x,y):
        ''' function which return the distance between the drone and the object'''
        return abs(self.cam_X-x)+abs(self.cam_Y-y)
                
    def force_objectif(self,position_obj):
        '''calcul the force of the objectif'''
        K=self.K_objectif
        norme=((self.cam_X-position_obj[0])**2+(self.cam_Y-position_obj[1])**2)**(1/2)
        u=[(self.cam_X-position_obj[0])/norme, (self.cam_Y-position_obj[1])/norme]
        return [-u[0]*K,-u[1]*K]

    def calcul_force_repulsion(self,position_ob,urgence,distance,K):
        '''calcul the force of the repulsion of obstacles'''
        norme=((self.cam_X-position_ob[0])**2+(self.cam_Y-position_ob[1])**2)**(1/2)
        u=[(self.cam_X-position_ob[0])/norme, (self.cam_Y-position_ob[1])/norme]
        return [u[0]*K*(urgence/distance)**2,u[1]*K*(urgence/distance)**2]

    def calcul_force_ideale(self,Drone,distance):
        '''calcul the force of the ideal position between two drones'''
        norme=((self.cam_X-Drone.cam_X)**2+(self.cam_Y-Drone.cam_Y)**2)**(1/2)
        u=[(self.cam_X-Drone.cam_X)/norme, (self.cam_Y-Drone.cam_Y)/norme]
        positon_ideal=[Drone.cam_X+u[0]*distance,Drone.cam_Y+u[1]*distance]
        return [positon_ideal[0]-self.cam_X,positon_ideal[1]-self.cam_Y]

    def new_calcul(self,c_time):
            ''' function which calculate the new movement of the drone'''
            if self.leader==1:
                if c_time//10==0:
                    self.tmp_point=[self.origin[0],self.origin[1],0.3]
                else :
                    self.tmp_point=[self.origin[0]+0.3,self.origin[1]+0.3,0.3]
            else :
                force=[0,0]

                delta=self.incertitude
                K_eloignement=self.K_eloignement
                distance_unit=self.distance_unit
                urgence =self.urgence
                K_urgence=self.K_urgence
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
                    new_force=self.calcul_force_repulsion_urgence(voisin[0],urgence,voisin[1],K_urgence)
                    force=addition_terme_poid(force,new_force+[1]) 
                for voisin in voisin_trop_proche:
                    new_position=self.calcul_force_ideale(voisin[0],voisin[1])
                    force=addition_terme_poid(force,new_position+[K_eloignement]) 					
                for voisin in voisin_trop_loin:
                    new_position=self.calcul_force_ideale(voisin[0],voisin[1])
                    force=addition_terme_poid(force,new_position+[K_eloignement]) 
                norme=(force[0]**2+(force[1])**2)**(1/2)
                if norme!=0:
                    vrai_objectif=[self.cam_X+distance_unit*force[0],self.cam_Y+distance_unit*force[1]]
                    self.tmp_point=[vrai_objectif[0],vrai_objectif[1],0.3]
                else :
                    if self.calcul_distance(self.origin[0],self.origin[1])>0.1:
                        self.origin=[self.cam_X,self.cam_Y,0.2]
                    self.tmp_point=[self.origin[0],self.origin[1],0.3]


def addition_terme_poid(l1,l2):
    ''' function which return the addition of two list with a multiplication of the poid'''
    total=[]
    for i in range(len(l2)-1):
        total+=[l1[i]+l2[i]*l2[len(l2)-1]]
    return total

def seq_unaire(Drone,cf,c_time):
    '''sequence that will be executed by the drone'''
    coord=Drone.get_Cam_X_Y_Z()
    Drone.set_previous_coord(coord[0],coord[1],coord[2])
    position=swarm.get_estimated_positions()[cf.link_uri]
    Drone.set_cam_coord(position[0],position[1],position[2])
    Drone.new_calcul(c_time)
    objectif_tmp=Drone.tmp_point
    if Drone.leader!=1:
        cf.commander.send_position_setpoint(objectif_tmp[0],objectif_tmp[1],objectif_tmp[2], 0)



def run_sequence(scf):
    ''' function which execute the sequence of the drone'''
    global Dict_link_Sec_drone
    cf = scf.cf
    drone=Dict_link_Sec_drone[cf.link_uri]
    end_time = time.time() + 30
    while time.time() < end_time:
        print(time.time() - end_time)
        c_time=time.time()-end_time+30
        seq_unaire(drone,cf,c_time)
        time.sleep(0.2)



if __name__ == '__main__':
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
            if i==0:
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
