from tkinter import *
from random import *
class gui:
    def __init__(self):
        self.fen=Tk()
        self.can=Canvas(self.fen,width=1000,height=1000)
        self.can.pack()
        self.liste_drone=[]
        self.Drone1=Sec_drone(self.fen,self.can,500,500,1)
        self.crea_forma(20)
        self.Drone1.calcul_position()
        self.liste_drone+=[]
        self.fen.mainloop()

    def crea_forma(self,nmb_Drone_voulu):
        liste=[]
        for i in range(nmb_Drone_voulu):
            liste+=[Sec_drone(self.fen,self.can,randint(0,1000),randint(0,1000),0)]
        for drone in liste:
            self.Drone1.add_voisin(drone)
            drone.add_voisin(self.Drone1)
            for drone2 in liste:
                if drone!=drone2:
                    drone.add_voisin(drone2)
        for drone in liste:
            drone.calcul_position()


def addition_terme_poid(l1,l2):
    total=[]
    for i in range(len(l1)-1):
        total+=[l1[i]+l2[i]*l2[len(l2)-1]]
    total+=[l1[len(l1)-1]+l2[len(l2)-1]]
    return total

class Sec_drone:
    def __init__(self,fen,can,x,y,leader):
        self.fen=fen
        self.shape=can.create_oval(x-10,y-10,x+10,y+10,fill='black')
        self.can=can
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
        self.set_cam_coord(x,y,0)
        #Voisin proche
        self.leader=leader 
        self.voisin=[]

    def set_cam_coord(self,X,Y,Z):
        self.cam_X=X
        self.cam_Y=Y
        self.cam_Z=Z

    def set_previous_coord(self,X,Y,Z):
        self.previous_X=X
        self.previous_Y=Y
        self.previous_Z=Z
#### initier avec un topic ROS, un par formation

    def add_voisin(self,liste_drone):
            self.voisin+=[liste_drone]

    def get_vitesse_X_Y_Z(self):
        return[(self.cam_X-self.previous_X)/0.02,(self.cam_Y-self.previous_Y)/0.02,(self.cam_Z-self.previous_Z)/0.02]

    def get_Cam_X_Y_Z(self):
            return [self.cam_X,self.cam_Y,self.cam_Z]
	
    def calcul_distance(self,x,y):
            return ((self.cam_X-x)**2+(self.cam_Y-y)**2)/(1/2)

    def calcul_position_ideale(self,Drone,distance):
        norme=((self.cam_X-Drone.cam_X)**2+(self.cam_Y-Drone.cam_Y)**2)**(1/2)
        u=[(self.cam_X-Drone.cam_X)/norme, (self.cam_Y-Drone.cam_Y)/norme]
        return [Drone.cam_X+u[0]*distance,Drone.cam_Y+u[1]*distance]
            
    def calcul_position(self):
        if self.leader==1:
                    # remain STable
            return [self.cam_X,self.cam_Y]
        else:
            ### objectif
            objectif=[0,0,0] #x,y,poid total
            ### définition de l'écart, ici 1m
            distance=100
            ### définition de l'insertitude, ici 10 cm
            delta=5
            ### définition de l'importancen de s'écarter contre se rapprocher
            K1=0.3
            ### décompte voisin trop proche, trop loin
            voisin_trop_proche=[]
            voisin_trop_loin=[]
            for voisin in self.voisin:
                d=self.calcul_distance(voisin.cam_X,voisin.cam_Y)
                if d<distance-delta:
                        voisin_trop_proche+=[voisin]
                elif d>distance+delta:
                        voisin_trop_loin+=[voisin]
            
            for voisin in voisin_trop_proche:
                new_position=self.calcul_position_ideale(voisin,distance)
                objectif=addition_terme_poid(objectif,new_position+[1])  				

            ### pour plus, pourquoi ne pas refaire le premier cas avec les poids d'urgence?

            for voisin in voisin_trop_loin:
                new_position=self.calcul_position_ideale(voisin,distance)
                objectif=addition_terme_poid(objectif,new_position+[K1])  
            if objectif[2]==0:
                self.deplacement(self.cam_X,self.cam_Y)	
            else :			
                vrai_objectif=[objectif[0]/objectif[2],objectif[1]/objectif[2]] ### division par le poids
                self.deplacement(vrai_objectif[0],vrai_objectif[1])

    def deplacement(self,x,y):
        norme=((self.cam_X-x)**2+(self.cam_Y-y)**2)**(1/2) ### /5 for speed
        self.can.move(self.shape,-(self.cam_X-x)/norme,-(self.cam_Y-y)/norme)
        self.set_cam_coord(self.cam_X-(self.cam_X-x)/norme,self.cam_Y-(self.cam_Y-y)/norme,0)
        self.fen.after(10,self.calcul_position)
gui()