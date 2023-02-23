from playsound import playsound
from PIL import Image, ImageTk
import time
import tkinter as tk
import random
import math
import numpy as np
import sys

class Counter:
    def __init__(self,canvas):
        self.dirtCollected = 0
        self.canvas = canvas
        self.canvas.create_text(70,50,text="Dirt collected: "+str(self.dirtCollected),tags="counter")
        
    def itemCollected(self, canvas):
        self.dirtCollected +=1
        self.canvas.itemconfigure("counter",text="Dirt collected: "+str(self.dirtCollected))
        

class Bot:

    def __init__(self,namep,canvasp):
        self.x = random.randint(100,900)
        self.y = random.randint(100,900)
        self.theta = random.uniform(0.0,2.0*math.pi)
        #self.theta = 0
        self.name = namep
        self.ll = 60 #axle width
        self.vl = 0.0
        self.vr = 0.0
        self.battery = 800
        self.turning = 0
        self.moving = random.randrange(50,100)
        self.currentlyTurning = False
        #self.map = np.zeros( (10,10) )
        self.canvas = canvasp
        self.view = [0]*9

        
    def draw(self,canvas):
        self.cameraPositions = []
        for pos in range(20,-21,-5):
            self.cameraPositions.append( ( (self.x + pos*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                                 (self.y - pos*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta) ) )

        points = [ (self.x + 30*math.sin(self.theta)) - 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y - 30*math.cos(self.theta)) - 30*math.cos((math.pi/2.0)-self.theta), \
                   (self.x - 30*math.sin(self.theta)) - 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y + 30*math.cos(self.theta)) - 30*math.cos((math.pi/2.0)-self.theta), \
                   (self.x - 30*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y + 30*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta), \
                   (self.x + 30*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y - 30*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta)  \
                ]
        canvas.create_polygon(points, fill="blue", tags=self.name)

        self.sensorPositions = [ (self.x + 20*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                                 (self.y - 20*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta), \
                                 (self.x - 20*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                                 (self.y + 20*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta)  \
                            ]

        centre1PosX = self.x 
        centre1PosY = self.y
        canvas.create_oval(centre1PosX-15,centre1PosY-15,\
                           centre1PosX+15,centre1PosY+15,\
                           fill="gold",tags=self.name)
        canvas.create_text(self.x,self.y,text=str(self.battery),tags=self.name)

        wheel1PosX = self.x - 30*math.sin(self.theta)
        wheel1PosY = self.y + 30*math.cos(self.theta)
        canvas.create_oval(wheel1PosX-3,wheel1PosY-3,\
                                         wheel1PosX+3,wheel1PosY+3,\
                                         fill="red",tags=self.name)

        wheel2PosX = self.x + 30*math.sin(self.theta)
        wheel2PosY = self.y - 30*math.cos(self.theta)
        canvas.create_oval(wheel2PosX-3,wheel2PosY-3,\
                                         wheel2PosX+3,wheel2PosY+3,\
                                         fill="green",tags=self.name)

        sensor1PosX = self.sensorPositions[0]
        sensor1PosY = self.sensorPositions[1]
        sensor2PosX = self.sensorPositions[2]
        sensor2PosY = self.sensorPositions[3]
        canvas.create_oval(sensor1PosX-3,sensor1PosY-3, \
                           sensor1PosX+3,sensor1PosY+3, \
                           fill="yellow",tags=self.name)
        canvas.create_oval(sensor2PosX-3,sensor2PosY-3, \
                           sensor2PosX+3,sensor2PosY+3, \
                           fill="yellow",tags=self.name)

        for xy in self.cameraPositions:
            canvas.create_oval(xy[0]-2,xy[1]-2,xy[0]+2,xy[1]+2,fill="purple1",tags=self.name)
        for xy in self.cameraPositions:
            canvas.create_line(xy[0],xy[1],xy[0]+400*math.cos(self.theta),xy[1]+400*math.sin(self.theta),fill="light grey",tags=self.name)
        
    # cf. Dudek and Jenkin, Computational Principles of Mobile Robotics
    def move(self,canvas,registryPassives,dt):
        if self.battery>0:
            self.battery -= 1
        if self.battery==0:
            self.vl = 0
            self.vr = 0
        for rr in registryPassives:
            if isinstance(rr,Charger) and self.distanceTo(rr)<80:
                self.battery += 10
                
        if self.vl==self.vr:
            R = 0
        else:
            R = (self.ll/2.0)*((self.vr+self.vl)/(self.vl-self.vr))
        omega = (self.vl-self.vr)/self.ll
        ICCx = self.x-R*math.sin(self.theta) #instantaneous centre of curvature
        ICCy = self.y+R*math.cos(self.theta)
        m = np.matrix( [ [math.cos(omega*dt), -math.sin(omega*dt), 0], \
                        [math.sin(omega*dt), math.cos(omega*dt), 0],  \
                        [0,0,1] ] )
        v1 = np.matrix([[self.x-ICCx],[self.y-ICCy],[self.theta]])
        v2 = np.matrix([[ICCx],[ICCy],[omega*dt]])
        newv = np.add(np.dot(m,v1),v2)
        newX = newv.item(0)
        newY = newv.item(1)
        newTheta = newv.item(2)
        newTheta = newTheta%(2.0*math.pi) #make sure angle doesn't go outside [0.0,2*pi)
        self.x = newX
        self.y = newY
        self.theta = newTheta        
        if self.vl==self.vr: # straight line movement
            self.x += self.vr*math.cos(self.theta) #vr wlog
            self.y += self.vr*math.sin(self.theta)
        if self.x<0.0:
            self.x=799.0
        if self.x>800.0:
            self.x = 0.0
        if self.y<0.0:
            self.y=799.0
        if self.y>800.0:
            self.y = 0.0
        #self.updateMap()
        canvas.delete(self.name)
        self.draw(canvas)

    def look(self,registryActives):
        self.view = [0]*9
        for idx,pos in enumerate(self.cameraPositions):
            for cc in registryActives:
                if isinstance(cc,Cat):
                    dd = self.distanceTo(cc)
                    scaledDistance = max(400-dd,0)/400
                    ncx = cc.x-pos[0] #cat if robot were at 0,0
                    ncy = cc.y-pos[1]
                    #print(abs(angle-self.theta)%2.0*math.pi)
                    m = math.tan(self.theta)
                    A = m*m+1
                    B = 2*(-m*ncy-ncx)
                    r = 15 #radius
                    C = ncy*ncy - r*r + ncx*ncx 
                    if B*B-4*A*C>=0 and scaledDistance>self.view[idx]:
                        self.view[idx] = scaledDistance
        self.canvas.delete("view")
        for vv in range(9):
            if self.view[vv]==0:
                self.canvas.create_rectangle(850+vv*15,50,850+vv*15+15,65,fill="white",tags="view")
            if self.view[vv]>0:
                colour = hex(15-math.floor(self.view[vv]*16.0)) #scale to 0-15 -> hex
                fillHex = "#"+colour[2]+colour[2]+colour[2]
                self.canvas.create_rectangle(850+vv*15,50,850+vv*15+15,65,fill=fillHex,tags="view")
        return self.view

    def pickUpAndPutDown(self,xp,yp):
        self.x = xp
        self.y = yp
        self.canvas.delete(self.name)
        self.draw(self.canvas)

    # def updateMap(self):
    #     xMapPosition = int(math.floor(self.x/100))
    #     yMapPosition = int(math.floor(self.y/100))
    #     self.map[xMapPosition][yMapPosition] = 1
    #     self.drawMap()

    # def drawMap(self):
    #     for xx in range(0,10):
    #         for yy in range(0,10):
    #             print(xx,",",yy,)
    #             if self.map[xx][yy]==1:
    #                 self.canvas.create_rectangle(100*xx,100*yy,100*xx+100,100*yy+100,fill="pink",width=0,tags="map")
    #     self.canvas.tag_lower("map")
                
        
    def senseCharger(self, registryPassives):
        lightL = 0.0
        lightR = 0.0
        for pp in registryPassives:
            if isinstance(pp,Charger):
                lx,ly = pp.getLocation()
                distanceL = math.sqrt( (lx-self.sensorPositions[0])*(lx-self.sensorPositions[0]) + \
                                       (ly-self.sensorPositions[1])*(ly-self.sensorPositions[1]) )
                distanceR = math.sqrt( (lx-self.sensorPositions[2])*(lx-self.sensorPositions[2]) + \
                                       (ly-self.sensorPositions[3])*(ly-self.sensorPositions[3]) )
                lightL += 200000/(distanceL*distanceL)
                lightR += 200000/(distanceR*distanceR)
        return lightL, lightR

    def senseHubs(self, registryPassives):
        signal = []
        for pp in registryPassives:
            if isinstance(pp,WiFiHub):
                lx,ly = pp.getLocation()
                distanceL = math.sqrt( (lx-self.sensorPositions[0])*(lx-self.sensorPositions[0]) + \
                                       (ly-self.sensorPositions[1])*(ly-self.sensorPositions[1]) )
                distanceR = math.sqrt( (lx-self.sensorPositions[2])*(lx-self.sensorPositions[2]) + \
                                       (ly-self.sensorPositions[3])*(ly-self.sensorPositions[3]) )
                signal.append(200000/(distanceL*distanceL))
                signal.append(200000/(distanceR*distanceR))
        return signal
    
    def distanceTo(self,obj):
        xx,yy = obj.getLocation()
        return math.sqrt( math.pow(self.x-xx,2) + math.pow(self.y-yy,2) )

    def collectDirt(self, canvas, registryPassives, count):
        toDelete = []
        for idx,rr in enumerate(registryPassives):
            if isinstance(rr,Dirt):
                if self.distanceTo(rr)<30:
                    canvas.delete(rr.name)
                    toDelete.append(idx)
                    count.itemCollected(canvas)
        for ii in sorted(toDelete,reverse=True):
            del registryPassives[ii]
        return registryPassives

    def brain(self,chargerL,chargerR):
        # wandering behaviour
        if self.currentlyTurning==True:
            self.vl = -2.0
            self.vr = 2.0
            self.turning -= 1
        else:
            self.vl = 5.0
            self.vr = 5.0
            self.moving -= 1
        if self.moving==0 and not self.currentlyTurning:
            self.turning = random.randrange(20,40)
            self.currentlyTurning = True
        if self.turning==0 and self.currentlyTurning:
            self.moving = random.randrange(50,100)
            self.currentlyTurning = False
        #battery - these are later so they have priority
        if self.battery<600:
            if chargerR>chargerL:
                self.vl = 2.0
                self.vr = -2.0
            elif chargerR<chargerL:
                self.vl = -2.0
                self.vr = 2.0
            if abs(chargerR-chargerL)<chargerL*0.1: #approximately the same
                self.vl = 5.0
                self.vr = 5.0
            #self.vl = 5*math.sqrt(chargerR)
            #self.vr = 5*math.sqrt(chargerL)
        if chargerL+chargerR>200 and self.battery<800:
            self.vl = 0.0
            self.vr = 0.0

    def collision(self,registryActives):
        collision = False
        for rr in registryActives:
            if isinstance(rr,Cat):
                if self.distanceTo(rr)<50.0:
                    playsound("385892__spacether__262312-steffcaffrey-cat-meow1.mp3",block=False)
                    collision = True
                    rr.jump()
        return collision


class Cat:
    def __init__(self,namep,canvasp):
        self.x = random.randint(100,900)
        self.y = random.randint(100,900)
        self.theta = random.uniform(0.0,2.0*math.pi)
        self.name = namep
        self.canvas = canvasp
        self.vl = 1.0
        self.vr = 1.0
        self.turning = 0
        self.moving = random.randrange(50,100)
        self.currentlyTurning = False
        self.ll = 20
        imgFile = Image.open("cat.png")
        imgFile = imgFile.resize((30,30), Image.ANTIALIAS)
        self.image = ImageTk.PhotoImage(imgFile)
        
    def draw(self,canvas):
        body = canvas.create_image(self.x,self.y,image=self.image,tags=self.name)

    def getLocation(self):
        return self.x, self.y

    def brain(self):
        # wandering behaviour
        if self.currentlyTurning==True:
            self.vl = -2.0
            self.vr = 2.0
            self.turning -= 1
        else:
            self.vl = 1.0
            self.vr = 1.0
            self.moving -= 1
        if self.moving==0 and not self.currentlyTurning:
            self.turning = random.randrange(20,40)
            self.currentlyTurning = True
        if self.turning==0 and self.currentlyTurning:
            self.moving = random.randrange(50,100)
            self.currentlyTurning = False
            
    def move(self,canvas,registryPassives,dt):
        if self.vl==self.vr:
            R = 0
        else:
            R = (self.ll/2.0)*((self.vr+self.vl)/(self.vl-self.vr))
        omega = (self.vl-self.vr)/self.ll
        ICCx = self.x-R*math.sin(self.theta) #instantaneous centre of curvature
        ICCy = self.y+R*math.cos(self.theta)
        m = np.matrix( [ [math.cos(omega*dt), -math.sin(omega*dt), 0], \
                        [math.sin(omega*dt), math.cos(omega*dt), 0],  \
                        [0,0,1] ] )
        v1 = np.matrix([[self.x-ICCx],[self.y-ICCy],[self.theta]])
        v2 = np.matrix([[ICCx],[ICCy],[omega*dt]])
        newv = np.add(np.dot(m,v1),v2)
        newX = newv.item(0)
        newY = newv.item(1)
        newTheta = newv.item(2)
        newTheta = newTheta%(2.0*math.pi) #make sure angle doesn't go outside [0.0,2*pi)
        self.x = newX
        self.y = newY
        self.theta = newTheta        
        if self.vl==self.vr: # straight line movement
            self.x += self.vr*math.cos(self.theta) #vr wlog
            self.y += self.vr*math.sin(self.theta)
        if self.x<0.0:
            self.x=799.0
        if self.x>800.0:
            self.x = 0.0
        if self.y<0.0:
            self.y=799.0
        if self.y>800.0:
            self.y = 0.0
        #self.updateMap()
        canvas.delete(self.name)
        self.draw(canvas)

    def jump(self):
        self.x += random.randint(20,50)
        self.y += random.randint(20,50)
        if self.x<0.0:
            self.x=799.0
        if self.x>800.0:
            self.x = 0.0
        if self.y<0.0:
            self.y=799.0
        if self.y>800.0:
            self.y = 0.0
        #self.updateMap()
        self.canvas.delete(self.name)
        self.draw(self.canvas)

        
class Charger:
    def __init__(self,namep):
        self.centreX = random.randint(100,900)
        self.centreY = random.randint(100,900)
        self.name = namep
        
    def draw(self,canvas):
        body = canvas.create_oval(self.centreX-10,self.centreY-10, \
                                  self.centreX+10,self.centreY+10, \
                                  fill="gold",tags=self.name)

    def getLocation(self):
        return self.centreX, self.centreY


class WiFiHub:
    def __init__(self,namep,xp,yp):
        self.centreX = xp
        self.centreY = yp
        self.name = namep
        
    def draw(self,canvas):
        body = canvas.create_oval(self.centreX-10,self.centreY-10, \
                                  self.centreX+10,self.centreY+10, \
                                  fill="purple",tags=self.name)

    def getLocation(self):
        return self.centreX, self.centreY


class Dirt:
    def __init__(self,namep):
        self.centreX = random.randint(100,900)
        self.centreY = random.randint(100,900)
        self.name = namep

    def draw(self,canvas):
        body = canvas.create_oval(self.centreX-1,self.centreY-1, \
                                  self.centreX+1,self.centreY+1, \
                                  fill="grey",tags=self.name)

    def getLocation(self):
        return self.centreX, self.centreY


def buttonClicked(x,y,registryActives):
    for rr in registryActives:
        if isinstance(rr,Bot):
            rr.x = x
            rr.y = y

def initialise(window):
    window.resizable(False,False)
    canvas = tk.Canvas(window,width=800,height=800)
    canvas.pack()
    return canvas

def register(canvas):
    registryActives = []
    registryPassives = []
    noOfBots = 1
    noOfCats = 10
    noOfDirt = 300
    for i in range(0,noOfBots):
        bot = Bot("Bot"+str(i),canvas)
        registryActives.append(bot)
        bot.draw(canvas)
    for i in range(0,noOfCats):
        cat = Cat("Cat"+str(i),canvas)
        registryActives.append(cat)
        cat.draw(canvas)
    charger = Charger("Charger")
    registryPassives.append(charger)
    charger.draw(canvas)
    hub1 = WiFiHub("Hub1",950,50)
    registryPassives.append(hub1)
    hub1.draw(canvas)
    hub2 = WiFiHub("Hub2",50,500)
    registryPassives.append(hub2)
    hub2.draw(canvas)
    hub3 = WiFiHub("Hub3",600,800)
    registryPassives.append(hub3)
    hub3.draw(canvas)
    for i in range(0,noOfDirt):
        dirt = Dirt("Dirt"+str(i))
        registryPassives.append(dirt)
        dirt.draw(canvas)
    count = Counter(canvas)
    canvas.bind( "<Button-1>", lambda event: buttonClicked(event.x,event.y,registryActives) )
    return registryActives, registryPassives, count

def moveIt(canvas,registryActives,registryPassives,count,moves,signalStrengths):
    moves += 1
    for rr in registryActives:
        if isinstance(rr,Bot):
            rr.look(registryActives)
            chargerIntensityL, chargerIntensityR = rr.senseCharger(registryPassives)
            rr.brain(chargerIntensityL, chargerIntensityR)
        if isinstance(rr,Cat):
            rr.brain()
        rr.move(canvas,registryPassives,1.0)
        if isinstance(rr,Bot):
            registryPassives = rr.collectDirt(canvas,registryPassives, count)
            collision = rr.collision(registryActives)
        numberOfMoves = 8000
        if moves>numberOfMoves:
            print("total dirt collected in",numberOfMoves,"moves is",count.dirtCollected)
            sys.exit()
    canvas.after(50,moveIt,canvas,registryActives,registryPassives,count,moves,signalStrengths)

def training(registryActives, registryPassives,canvas):
    rr = registryActives[0]
    signalStrengths = []
    for x in range(10):
        for y in range(10):
            topCornerX = x*100.0
            topCornerY = y*100.0
            for _ in range(100):
                positionX = topCornerX + random.uniform(0.0,99.99)
                positionY = topCornerY + random.uniform(0.0,99.99)
                rr.pickUpAndPutDown(positionX,positionY)
                signalStrengths.append( (rr.senseHubs(registryPassives),x,y) )
                #canvas.update()
                #time.sleep(0.1)
    return signalStrengths


def main():
    window = tk.Tk()
    canvas = initialise(window)
    window.geometry("800x800")
    registryActives, registryPassives, count = register(canvas)

    signalStrengths = training(registryActives, registryPassives,canvas)
    
    moves = 0
    moveIt(canvas,registryActives,registryPassives, count, moves, signalStrengths)
    window.mainloop()

main()
