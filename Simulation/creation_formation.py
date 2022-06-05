from tkinter import *

class gui:
    def __init__(self):
        self.fen=Tk()
        self.can=Canvas(self.fen,width=1000,height=1000)
        self.can.pack()
        self.fen.bind("<Button-1>",self.position)
        self.fen.bind("<Button-3>",self.res)
        self.liste=[]
        self.fen.mainloop()

    
    def position(self,event):
        self.liste+=[[event.x,event.y]]
        self.can.create_oval(event.x-10,event.y-10,event.x+10,event.y+10,fill="blue")
    
    def res(self,event):
        l=[]
        print("Copier dans le presse papier")
        for i in range(len(self.liste)):
            l+=[[]]
            for j in range(len(self.liste)):
                if i==0 and j==0:
                    l[i]+=["Master"]
                elif i==j:
                    l[i]+=[None]
                else :
                    l[i]+=[((self.liste[i][0]-self.liste[j][0])**2+(self.liste[i][1]-self.liste[j][1])**2)**(1/2)]
        self.fen.clipboard_append("mat="+str(l))
        


gui()