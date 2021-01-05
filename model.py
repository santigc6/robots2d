from mesa import Model, Agent
from mesa.time import BaseScheduler, RandomActivation
from mesa.space import SingleGrid
from mesa.datacollection import DataCollector
from scipy.optimize import minimize

# SCALE IS SET TO 1

################## FIGURES FROM BMP - START ##################
from PIL import Image
import numpy as np

WHITE=255
BLACK=0
MAX_PIXELS=21
MAX_ROBOTS1=341-4
MAX_ROBOTS2=320-4

# FIRST FIGURE
im = Image.open("shapes/figura1.bmp")
p=np.array(im)

figure2=np.zeros(shape=(MAX_PIXELS,MAX_PIXELS))

i=0
j=0
for matrix in p:
    for row in matrix:
        if row[0]==BLACK:
            figure2[j][i]=1
        elif row[0]==WHITE:
            figure2[j][i]=0

        i+=1
        if i == MAX_PIXELS:
            i=0
            j+=1

# SECOND FIGURE
im = Image.open("shapes/figura2.bmp")
p=np.array(im)
            
figure3=np.zeros(shape=(100,100))

i=0
j=0
for matrix in p:
    for row in matrix:
        if row[0]==BLACK:
            figure3[j][i]=1
        elif row[0]==WHITE:
            figure3[j][i]=0

        i+=1
        if i == MAX_PIXELS:
            i=0
            j+=1

################## FIGURE FROM BMP - END ##################

################## PARAMETERS TO DEFINE WHAT FIGURE TO USE ##################
MAX_ROBOTS=MAX_ROBOTS1
#MAX_ROBOTS=MAX_ROBOTS2

figure1=figure2
#figure1=figure3

MAX_AGENTS_MOVING=24
################## END OF PARAMETERS ##################

class RobotAgent(Agent):
    """
    Robot agent
    """
    ID=1
    def __init__(self, pos, model, robot_type, shape, loc=None, grad=-1):
        """
        Create a new robot agent.
        Args:
           x, y: Agent initial location.
           agent_type: Indicator for the robot type -> seed=0, rest=1.
        """
        super().__init__(pos, model)
        self.id=RobotAgent.ID
        RobotAgent.ID+=1
        self.pos = pos
        self.type = robot_type
        self.gradient = grad
        self.shape=shape
        self.location=[-1,-1] # computed location
        self.isMoving = False
        self.finished = False
        self.prevPos=None
        self.noMoveCount=0
        self.isTunneled=False
        
        if robot_type == 0:
            self.location=loc
            self.hasGradient = True
            self.isLocalized = True
        else:
            self.hasGradient = False
            self.isLocalized = False

    def positionToLocation(self, actualPos, actualLoc, nextPos):
        incX=nextPos[0]-actualPos[0]
        incY=nextPos[1]-actualPos[1]
        
        nextLoc=[actualLoc[0]+incX, actualLoc[1]+incY]
        
        return nextLoc
            
    def distanceMulti(self, coords, distances):
        def multilat(x, coords, distances):
            return sum([abs(np.linalg.norm(x - coords[i]) - distances[i]) for i in range(len(coords))])

        l = len(coords)
        S = sum(distances)

        # compute weight vector for initial guess
        W = [((l - 1) * S) / (S - w) for w in distances]
        # get initial guess of point location
        x0 = sum([W[i] * coords[i] for i in range(l)])
        # optimize distance from signal origin to border of spheres

        res=minimize(multilat, x0, args=(coords, distances), method='Nelder-Mead', tol=1e-30).x

        return int(round(res[0])), int(round(res[1]))
    
    def computeGradient(self):
        minGradient=99
        for neighbor in self.model.grid.neighbor_iter(self.pos): # Find the neighbor with the minimum gradient value
            if neighbor.hasGradient and neighbor.isMoving == False:
                if neighbor.gradient < minGradient:
                    minGradient=neighbor.gradient
        if minGradient == 99: # If neighbors dont have a gradient yet
            return False
        self.gradient = 1 + minGradient # Update own gradient
        self.hasGradient = True
        
        return True
    
    def computeLocation(self, locThreshHold, rad):
        nLocalized = 0
        for neighbor in self.model.grid.get_neighbors(self.pos, True, radius=rad): # Find localized neighbors
            if neighbor.isLocalized and neighbor.isMoving == False:
                nLocalized += 1
        if nLocalized >= locThreshHold: # Compute localization
            distances = []
            coords = []
            for neighbor in self.model.grid.get_neighbors(self.pos, True, radius=rad): # Find localized neighbors
                if neighbor.isLocalized and neighbor.isMoving == False:
                    distances.append(np.linalg.norm(np.array(self.pos)-np.array(neighbor.pos)))
                    coords.append(neighbor.location)

            for i in range(4):
                if self.model.seeds[i] not in coords:
                    coords.append(self.model.seeds[i].location)
                    distances.append(np.linalg.norm(np.array(self.pos)-np.array(self.model.seeds[i].pos)))
            
            distances=np.array(distances)
            coords=np.array(coords)
            
            x,y=self.distanceMulti(coords, distances) # Multilateration

            self.location=[x,y] # Computed location
            self.isLocalized = True

            return True
        
        return False
    
    def step(self):
        if self.type == 0 or self.finished: # SEED ROBOT or already finished
            return
        
        elif self.hasGradient == False:
            if self.computeGradient()==True:
                self.model.ready+=1
        
        elif self.model.ready < MAX_ROBOTS: # Check if gradient calculation has already finished
            return
        
        elif self.isLocalized == False:
            if self.computeLocation(4, 3) == True: # Incremental calculations according to number of new neighbors
                self.model.localizedRobots+=1
        
        elif self.model.localizedRobots < MAX_ROBOTS: # Check if all the robots have been located
            return
    
        ############# EDGE FOLLOWING  #############
        elif self.finished == False:
            if self.isMoving == False:
                nSeeds=0
                nNeighbors=0
                for neighbor in self.model.grid.neighbor_iter(self.pos):
                    nNeighbors+=1
                    if neighbor.type == 0:
                        nSeeds+=1
                if nSeeds >= 2 and nNeighbors >=3:
                    return
                for neighbor in self.model.grid.neighbor_iter(self.pos): # determine whether it is at the outer edge
                    if neighbor.gradient > self.gradient:
                        return
                
                if self.model.movingAgents >= MAX_AGENTS_MOVING: # We only allow a predefined number of agents to move at the same time
                    return
                
                self.isMoving=True
                self.model.movingAgents+=1
        
            if self.isMoving: # if it is at the outer edge or already moving, then start/keep moving                
                # We order the neighborhood as we need with a circular list
                orderedNeighborhood=[]
                neighborhoodList = self.model.grid.get_neighborhood(self.pos, moore=True, include_center=False)
                neighborhoodOrder=[6,5,3,0,1,2,4,7]
                if self.isTunneled:
                    neighborhoodOrder=[7,6,5,3,0,1,2,4]
                orderedNeighborhood=[neighborhoodList[i] for i in neighborhoodOrder]+[neighborhoodList[i] for i in neighborhoodOrder]

                # Check toroidal neighbors
                isNeighbor=False
                hasMoved=False
                possibleMoves=[]
                for position in orderedNeighborhood:
                    agentsInCell=self.model.grid.get_cell_list_contents(position)
                    if self.model.grid.is_cell_empty(position) == False and (len(agentsInCell) > 0 and agentsInCell[0].isMoving==False): # If cell is empty or that neighbor is moving we ignore it
                        isNeighbor=True
                    else:
                        if isNeighbor==True:
                            isNeighbor=False
                            possibleMoves.append(position)
                for position in possibleMoves:         
                    if position != self.prevPos:
                        try:
                            rollbackPos=self.pos
                            self.model.grid.move_agent(self, position)
                            self.prevPos=rollbackPos
                            hasMoved=True
                            break
                        except:
                            self.model.grid.place_agent(self, rollbackPos)
                            self.pos=rollbackPos
                            continue
                
                if hasMoved==False:
                    if self.noMoveCount < 4: # Max of 3 time units without moving to previous position
                        self.noMoveCount+=1
                    else:
                        self.prevPos=None
                        self.noMoveCount=0
                    return
                
                self.hasGradient=False
                self.computeGradient()
                
                self.isLocalized=False
                for i in [4, 5]:
                    if self.computeLocation(4,i)==True:
                        break
                
                if 0 <= self.location[0] < MAX_PIXELS and 0 <= self.location[1] < MAX_PIXELS:
                    isNeighbor=False
                    hasMoved=False
                    if self.shape[self.location[0]][self.location[1]]==1: # robot is inside the shape
                        tunneled1=[3,5,4,7]
                        tunneled2=[0,3,2,4]
                        neighborhoodList = self.model.grid.get_neighborhood(self.pos, moore=True, include_center=False)
                        self.isTunneled=True
                        for i in tunneled1:
                            agentsInCell=self.model.grid.get_cell_list_contents(neighborhoodList[i])
                            if len(agentsInCell)<=0:
                                self.isTunneled=False
                            elif len(agentsInCell)>0 and agentsInCell[0].finished==False:
                                self.isTunneled=False
                        
                        if self.isTunneled==False:
                            self.isTunneled=True
                            for i in tunneled2:
                                agentsInCell=self.model.grid.get_cell_list_contents(neighborhoodList[i])
                                if len(agentsInCell)<=0:
                                    self.isTunneled=False
                                elif len(agentsInCell)>0 and agentsInCell[0].finished==False:
                                    self.isTunneled=False
                        
                        for neighbor in self.model.grid.neighbor_iter(self.pos): # determine whether a neighbor has the same gradient
                            if neighbor.gradient == self.gradient and neighbor.isMoving == False and self.isTunneled==False:
                                self.finished = True
                                self.isMoving=False
                                self.model.movingAgents-=1
                                break    

                        # check if next move would make him leave the shape
                        orderedNeighborhood=[]
                        neighborhoodList = self.model.grid.get_neighborhood(self.pos, moore=True, include_center=False)
                        neighborhoodOrder=[6,5,3,0,1,2,4,7]
                        if self.isTunneled:
                            neighborhoodOrder=[7,6,5,3,0,1,2,4]
                        orderedNeighborhood=[neighborhoodList[i] for i in neighborhoodOrder]+[neighborhoodList[i] for i in neighborhoodOrder]
                        isNeighbor=False
                        hasMoved=False
                        new_position=None

                        for position in orderedNeighborhood:
                            agentsInCell=self.model.grid.get_cell_list_contents(position)
                            if self.model.grid.is_cell_empty(position) == False and (len(agentsInCell) > 0 and agentsInCell[0].isMoving==False):
                                isNeighbor=True
                            else:
                                if isNeighbor==True:
                                    new_position=position
                                    hasMoved=True
                                    break
                                    
                        new_location=self.positionToLocation(self.pos, self.location, new_position)
                        
                        if new_location[0]<0 or new_location[1]<0 or new_location[0]>=MAX_PIXELS or new_location[1]>=MAX_PIXELS or self.shape[new_location[0]][new_location[1]]==0 or new_position==self.prevPos: # if it is about to leave the figure
                            self.finished = True
                            self.isMoving=False
                            self.model.movingAgents-=1


class Robot2D(Model):
    """
    Model class for the 2D robot model.
    """

    def __init__(self, height=75, width=75):
        self.height = height
        self.width = width
        self.schedule = BaseScheduler(self)
        self.grid = SingleGrid(width, height, torus=True)
        self.ready=0
        self.localizedRobots=0
        self.movingAgents=0
        self.seeds=[] # Seed robots
        
        self.datacollector = DataCollector(
            model_reporters={"Ready": "ready", "Localized": "localizedRobots"},
            agent_reporters={"x": lambda a: a.pos[0], "y": lambda a: a.pos[1], "Gradient": "gradient"}
        )
        
        # Set up agents
        # We use a grid iterator that returns
        # the coordinates of a cell as well as
        # its contents. (coord_iter)
        margin1X=[i for i in range(15)]
        margin2X=[i for i in range(height-41, height)]
        margin1Y=[i for i in range(5)]
        margin2Y=[i for i in range(height-5, height)]
        nRobots=0
        
        for cell in self.grid.coord_iter():
            # Change coordinates order
            y = cell[1] 
            x = cell[2]
            
            if x in margin1X or x in margin2X or y in margin1Y or y in margin2Y:
                continue
                
            if nRobots < MAX_ROBOTS:
                agent = RobotAgent((x,y), self, 1, figure1)
                self.grid.place_agent(agent, (x, y))
                self.schedule.add(agent)
                nRobots+=1
            else: # SEED ROBOTS
                if x > 40:
                    x=37
                    y+=1
                    # SEED 1
                    agent = RobotAgent((x,y), self, 0, figure1, loc=[0,0], grad=0)
                    self.grid.place_agent(agent, (x, y))
                    self.seeds.append(agent)
                    self.schedule.add(agent)

                    # SEED 2
                    agent = RobotAgent((x+1,y), self, 0, figure1, loc=[1,0], grad=1)
                    self.grid.place_agent(agent, (x+1, y))
                    self.seeds.append(agent)
                    self.schedule.add(agent)

                    # SEED 3
                    agent = RobotAgent((x,y+1), self, 0, figure1, loc=[0,1], grad=1)
                    self.grid.place_agent(agent, (x, y+1))
                    self.seeds.append(agent)
                    self.schedule.add(agent)

                    # SEED 4
                    agent = RobotAgent((x+1,y+1), self, 0, figure1, loc=[1,1], grad=1)
                    self.grid.place_agent(agent, (x+1, y+1))
                    self.seeds.append(agent)
                    self.schedule.add(agent)
                else:
                    # SEED 1
                    x-=1
                    y+=1
                    agent = RobotAgent((x,y), self, 0, figure1, loc=[0,0], grad=0)
                    self.grid.place_agent(agent, (x, y))
                    self.seeds.append(agent)
                    self.schedule.add(agent)

                    # SEED 2
                    agent = RobotAgent((x+1,y), self, 0, figure1, loc=[1,0], grad=1)
                    self.grid.place_agent(agent, (x+1, y))
                    self.seeds.append(agent)
                    self.schedule.add(agent)

                    # SEED 3
                    agent = RobotAgent((x,y+1), self, 0, figure1, loc=[0,1], grad=1)
                    self.grid.place_agent(agent, (x, y+1))
                    self.seeds.append(agent)
                    self.schedule.add(agent)

                    # SEED 4
                    agent = RobotAgent((x+1,y+1), self, 0, figure1, loc=[1,1], grad=1)
                    self.grid.place_agent(agent, (x+1, y+1))
                    self.seeds.append(agent)
                    self.schedule.add(agent)
                
                break
        
        self.running = True

    def step(self):
        """
        Run one step of the model.
        """
        # collect data
        self.datacollector.collect(self)
        
        self.schedule.step()
