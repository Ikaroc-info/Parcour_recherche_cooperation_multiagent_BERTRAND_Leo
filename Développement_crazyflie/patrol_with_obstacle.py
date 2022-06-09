''' This program will launch a patrol between differents positions with virtuals obstacles'''
import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from math import cos, sin
from random import randint, random

global Dict_link_Sec_drone
Dict_link_Sec_drone={}
flying_time=30
# Change uris and sequences according to your setup
URI1 = 'radio://0/80/2M/E7E7E7E703'


# List of URIs, comment the one you do not want to fly
uris = {
    URI1,
}


def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)


class Sec_drone:
    def __init__(self,liste_drone,leader):
        self.liste_drone=liste_drone
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
        self.tmp_point=[0,0,0.4]
        #fly modification
        self.voisin=[[0.2,0.2]] #list of obstacles
        self.K_objectif=1 #strength of the objectif
        self.disance_unit=0.20 #distance between the drone and the temporary objectif
        self.urgence= 0.4 #distance of emergency
        self.K_urgence=0.7 #strength of the emergency
        self.hauteur=0.3 #height of the drone

    def set_cam_coord(self,X,Y,Z):
        ''' function which set the position atributes of the drone'''
        self.cam_X=X
        self.cam_Y=Y
        self.cam_Z=Z

    def set_previous_coord(self,X,Y,Z):
        ''' function which set the position previous atributes of the drone'''
        self.previous_X=X  
        self.previous_Y=Y
        self.previous_Z=Z

    def get_vitesse_X_Y_Z(self):
        ''' function which return the velocity of the drone'''
        return[(self.cam_X-self.previous_X)/0.02,(self.cam_Y-self.previous_Y)/0.02,(self.cam_Z-self.previous_Z)/0.02]

    def get_Cam_X_Y_Z(self):
        ''' function which return the position of the drone'''
        return [self.cam_X,self.cam_Y,self.cam_Z]
    
    def calcul_distance(self,x,y):
        ''' function which return the distance between the drone and the object'''
        return abs(self.cam_X-x)+abs(self.cam_Y-y)
      
    def objectif(self):
        ''' function which return the objectif of the drone depending of the value of i and if too far, launch the calcul
        of the new_temporary objectif'''
        if abs(self.cam_X-self.curent_obj[0])+abs(self.cam_Y-self.curent_obj[1])<0.15:
            if self.i==0:
                self.curent_obj=[0,1]
                self.i=1
            else:
                self.curent_obj=[0,-1]
                self.i=0
        else:
            self.new_calcul_obstacle([self.curent_obj[0],self.curent_obj[1],1])
                
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


    def new_calcul_obstacle(self,objectif):
            ''' function which calculate the new movement of the drone'''
            force=self.force_objectif([objectif[0],objectif[1]])
            distance_unit=self.disance_unit
            urgence =self.urgence
            K_urgence=self.K_urgence
            voisin_urgence=[]
            for voisin in self.voisin:
                d=self.calcul_distance(voisin[0],voisin[1])
                if d<urgence:
                    distance_unit=self.disance_unit/2
                    voisin_urgence+=[[voisin,d]]
            for voisin in voisin_urgence:
                new_force=self.calcul_force_repulsion(voisin[0],urgence,voisin[1],K_urgence)
                force=[force[0]+new_force[0],force[1]+new_force[1]]
            norme=abs(force[0])+abs(force[1])
            force=[force[0]/norme,force[1]/norme]
            vrai_objectif=[self.cam_X+distance_unit*force[0],self.cam_Y+distance_unit*force[1]] 
            self.tmp_point=[vrai_objectif[0],vrai_objectif[1],self.hauteur]
        
    
def addition_terme_poid(l1,l2):
    ''' function which return the addition of two list with a multiplication of the poid'''
    total=[]
    for i in range(len(l1)-1):
        total+=[l1[i]+l2[i]*l2[len(l2)-1]]
    total+=[l1[len(l1)-1]+l2[len(l2)-1]]
    return total

def seq_unaire(Drone,cf):
    '''sequence that will be executed by the drone'''
    coord=Drone.get_Cam_X_Y_Z()
    Drone.set_previous_coord(coord[0],coord[1],coord[2])
    position=swarm.get_estimated_positions()[cf.link_uri]
    Drone.set_cam_coord(position[0],position[1],position[2])
    Drone.objectif()
    objectif_tmp=Drone.tmp_point
    cf.commander.send_position_setpoint(objectif_tmp[0],objectif_tmp[1],objectif_tmp[2], 0)


def run_sequence(scf):
    ''' function which execute the sequence of the drone'''
    global Dict_link_Sec_drone
    cf = scf.cf
    drone=Dict_link_Sec_drone[cf.link_uri]
    end_time = time.time() + flying_time
    while time.time() < end_time:
        seq_unaire(drone,cf)
        time.sleep(0.1)

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
        for uri in uris:
            Dict_link_Sec_drone[uri]=Sec_drone([],1)
        swarm.parallel(run_sequence)

    





BEBOP_COMMANDS_TOPIC="/bebop/cmd_vel"
