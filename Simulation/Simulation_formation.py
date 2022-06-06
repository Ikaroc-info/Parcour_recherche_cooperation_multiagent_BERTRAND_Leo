''' This is the lastest version of simulation, it is a bit more complexe than the previous one with the addition
of matrix formation, the target of each drone and the possibility of control the position (zqsd) of the drone leader '''
from tkinter import *
from random import *
class gui:
    def __init__(self):
        self.fen=Tk()
        self.can=Canvas(self.fen,width=1000,height=1000)
        self.can.pack()
        self.liste_drone=[]
        self.distance_voisin=100
        ################ mat is the matrix of the formation ###############
        #mat=[["Master",None],[None,None]] #test évitement si pas pris en compte
        #mat=[["Master",200,200],[200,None,100],[200,100,None]] #triangle isocelle
        #mat=[["Master",200,100,None],[200,None,None,100],[100,None,None,200],[None,100,200,None]] #trapèze
        #mat=[['Master', 227, 236, 341, 183], [227, None, 295, 235, 146], [236, 295, None, 203, 149], [341, 235, 203, None, 158], [183, 146, 149, 158, None]]
        mat=[["Master",200,100,223],[200,None,223,100],[100,223,None,200],[223,100,200,None]] #rectangle
        #####################################################################
        self.crea_formation_matrice(mat)
        self.fen.bind('<z>',self.dep_vert)
        self.fen.bind('<d>',self.dep_hor)
        self.fen.bind('<s>',self.dep_vert_inv)
        self.fen.bind('<q>',self.dep_hort_inv)
        self.fen.mainloop()

    def crea_formation_matrice(self,mat):
        '''creation of the formation with the matrix mat, adding neighbours for each drone'''
        self.liste_drone=[]
        for i in range(len(mat)):
            if mat[i][i]=='Master':
                self.Drone1=Sec_drone(self.fen,self.can,self.liste_drone,500,500,1)
                self.liste_drone+=[self.Drone1]
            else:
                self.liste_drone+=[Sec_drone(self.fen,self.can,self.liste_drone,randint(0,1000),randint(0,1000),0)]
        for i in range(len(self.liste_drone)):
            for j in range(len(self.liste_drone)):
                if mat[i][j]!='Master' and mat[i][j]!=None:
                    self.liste_drone[i].add_voisin(self.liste_drone[j],mat[i][j])
        for drone in self.liste_drone:
            drone.calcul_position()

    def dep_vert(self,event):
        '''move vertically the drone'''
        self.dep_man(0,-2)
    def dep_hor(self,event):
        '''move horizontally the drone'''
        self.dep_man(2,0)
    def dep_vert_inv(self,event):
        '''move vertically the drone'''
        self.dep_man(0,2)
    def dep_hort_inv(self,event):
        '''move horizontally the drone'''
        self.dep_man(-2,0)
    
    def dep_man(self,delta_x,delta_y):
        '''move the leader'''
        self.can.move(self.Drone1.shape,delta_x,delta_y)
        self.Drone1.set_cam_coord(self.Drone1.cam_X+delta_x,self.Drone1.cam_Y+delta_y,0)


def addition_terme_poid(l1,l2):
    ''' function which add two list term by term with a poise multiplicator'''
    total=[]
    for i in range(len(l1)-1):
        total+=[l1[i]+l2[i]*l2[len(l2)-1]]
    total+=[l1[len(l1)-1]+l2[len(l2)-1]]
    return total

class Sec_drone:
    def __init__(self,fen,can,liste_drone,x,y,leader):
        self.liste_drone=liste_drone
        self.fen=fen
        self.col=["#"+''.join([choice('0123456789ABCDEF') for j in range(6)])]
        self.target=can.create_rectangle(x-5,y-5,x+5,y+5,fill=self.col)
        self.shape=can.create_oval(x-10,y-10,x+10,y+10,fill=self.col)
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
        #fly modification
        self.K_eloignement=0.5 #strength of the repulsion and attraction between drone
        self.K_urgence=15 # strength of emergency repulsion
        self.urgence= 70 #distance of emergency
        self.delta=0.1 # % of tolerance for the position of the drone

    def set_cam_coord(self,X,Y,Z):
        '''Set the camera receved coordinates'''
        self.cam_X=X
        self.cam_Y=Y
        self.cam_Z=Z

    def set_previous_coord(self,X,Y,Z):
        '''Set the previous camera coordinates'''
        self.previous_X=X  
        self.previous_Y=Y
        self.previous_Z=Z

    def add_voisin(self,liste_drone,distance_voisin):
        '''Add a drone to the list of voisin'''
        self.voisin+=[[liste_drone,distance_voisin]]

    def get_vitesse_X_Y_Z(self):
        '''Return the velocity of the drone'''
        return[(self.cam_X-self.previous_X)/0.02,(self.cam_Y-self.previous_Y)/0.02,(self.cam_Z-self.previous_Z)/0.02]

    def get_Cam_X_Y_Z(self):
        '''Return the camera coordinates'''
        return [self.cam_X,self.cam_Y,self.cam_Z]
	
    def calcul_distance(self,x,y):
        '''Return the distance between the drone and the target'''
        return ((self.cam_X-x)**2+(self.cam_Y-y)**2)/(1/2)

    def calcul_position_ideale(self,Drone,distance):
        '''Return the ideal position of the drone between it and an other drone'''
        norme=((self.cam_X-Drone.cam_X)**2+(self.cam_Y-Drone.cam_Y)**2)**(1/2)
        u=[(self.cam_X-Drone.cam_X)/norme, (self.cam_Y-Drone.cam_Y)/norme]
        return [Drone.cam_X+u[0]*distance,Drone.cam_Y+u[1]*distance]
            
    def calcul_position(self):
        '''This function calculate the position desire of the drone'''
        if self.leader==1:
            return [self.cam_X,self.cam_Y] #if the drone is the leader, it will stay at the same place
        else:
            objectif=[0,0,0]
            delta=self.delta
            urgence =self.urgence
            K_eloignement=self.K_eloignement
            K_urgence=self.K_urgence
            voisin_trop_proche=[]
            voisin_trop_loin=[]
            voisin_urgence=[]
            for voisin in self.voisin:
                d=self.calcul_distance(voisin[0].cam_X,voisin[0].cam_Y)
                if d<voisin[1]*(1-delta):
                    if d<urgence:
                        voisin_urgence+=[[voisin,d]]
                    else:
                        voisin_trop_proche+=[voisin]
                elif d>voisin[1]*(1+delta):
                        voisin_trop_loin+=[voisin]
            for drone in self.liste_drone:
                if drone not in self.voisin:
                    d=self.calcul_distance(drone.cam_X,drone.cam_Y)
                    if d<urgence and d!=0:
                        voisin_urgence+=[[[drone,urgence],d]]            
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
                vrai_objectif=[objectif[0]/objectif[2],objectif[1]/objectif[2]]
                self.deplacement(vrai_objectif[0],vrai_objectif[1])

    def deplacement(self,x,y):
        '''This function move the drone'''
        self.can.delete(self.target)
        self.target=self.can.create_rectangle(x-5,y-5,x+5,y+5,fill=self.col)
        norme=((self.cam_X-x)**2+(self.cam_Y-y)**2)**(1/2) ### /5 for speed
        self.can.move(self.shape,-(self.cam_X-x)/norme,-(self.cam_Y-y)/norme)
        self.set_cam_coord(self.cam_X-(self.cam_X-x)/norme,self.cam_Y-(self.cam_Y-y)/norme,0)
        self.fen.after(20,self.calcul_position)
gui()