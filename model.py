from mesa import Model, Agent
from mesa.time import RandomActivation
from mesa.space import SingleGrid
from mesa.datacollection import DataCollector
from scipy.optimize import minimize

# SCALE IS SET TO 1

################## FIGURES FROM BMP - START ##################
from PIL import Image
import numpy as np

WHITE=255
BLACK=0
MAX_PIXELS=100

# FIRST FIGURE
im = Image.open("shapes/figura1.bmp")
p=np.array(im)

figure1=np.zeros(shape=(100,100))

i=0
j=0
for matrix in p:
    for row in matrix:
        if row[0]==BLACK:
            figure1[j][i]=1
        elif row[0]==WHITE:
            figure1[j][i]=0

        i+=1
        if i == MAX_PIXELS:
            i=0
            j+=1

# SECOND FIGURE
im = Image.open("shapes/figura2.bmp")
p=np.array(im)
            
figure2=np.zeros(shape=(100,100))

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

################## FIGURE FROM BMP - END ##################

class RobotAgent(Agent):
    """
    Robot agent
    """

    def __init__(self, pos, model, robot_type, shape):
        """
        Create a new robot agent.
        Args:
           x, y: Agent initial location.
           agent_type: Indicator for the robot type -> seed=0, stationary=1, moving=2.
        """
        super().__init__(pos, model)
        self.pos = pos
        self.type = agent_type
        self.gradient = 0
        self.shape=shape
        self.location=[-1,-1] # computed location
        self.isMoving = False
        self.finished = False
        
        if robot_type == 0:
            self.hasGradient = True
            self.isLocalized = True
        else:
            self.hasGradient = False
            self.isLocalized = False

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
        res=minimize(multilat, x0, args=(coords, distances), method='Nelder-Mead').x
        
        return int(round(res[0])), int(round(res[1]))
    
    def computeGradient(self):
        minGradient=99
            for neighbor in self.model.grid.neighbor_iter(self.pos): # Find the neighbor with the minimum gradient value
                if neighbor.hasGradient:
                    if neighbor.gradient < minGradient:
                        minGradient=neighbor.gradient

            self.gradient = 1 + minGradient # Update own gradient
            self.hasGradient = True
            minGradient=99
    
    def computeLocation(self):
        nLocalized = 0
        for neighbor in self.model.grid.get_neighbors(self.pos, True, radius=2): # Find localized neighbors
            if neighbor.isLocalized:
                nLocalized += 1
        if nLocalized >= 3: # Compute localization
            distances = []
            coords = []
            for neighbor in self.model.grid.get_neighbors(self.pos, True, radius=2): # Find localized neighbors
                if neighbor.isLocalized:
                    distances.append(abs(self.gradient-neighbor.gradient))
                    coords.append(neighbor.location)

            distances=np.array(distances)
            coords=np.array(coords)

            x,y=self.distanceMulti(coords, distances) # Multilateration

            self.location=[x,y] # Computed location
            self.isLocalized = True
    
    def step(self):
        if robot_type == 0 or self.finished: # SEED ROBOT or already finished
            return
        
        if self.hasGradient == False:
            self.computeGradient()
        
        elif self.isLocalized == False:
            self.computeLocation()
        
        ############# EDGE FOLLOWING  #############
        else:
            if self.isMoving = False:
                isOuterEdge=True
                for neighbor in self.model.grid.neighbor_iter(self.pos): # determine whether it is at the outer edge
                    if neighbor.gradient > self.gradient:
                        isOuterEdge = False
                        break
            if isOuterEdge or self.isMoving: # if it is at the outer edge or already moving, then start/keep moving
                self.isMoving = True
                
                # Move left to the closest neighbor
                
                if self.location #TODO THIS: # finishing position
                    self.finished = True
                    self.isMoving = False
                
                isNeighbor=False
                hasMoved=False
                for position in self.model.grid.get_neighborhood(self.pos, moore=True, include_center=False):
                    if self.model.is_cell_empty(position) == False:
                        isNeighbor=True
                    else:
                        if isNeighbor==True:
                            self.model.grid.move_agent(self, position)
                            hasMoved=True
                if hasMoved == False: # If it didnt moved we will move with the list in reverse order
                    isNeighbor = False
                    for position in list(reversed(self.model.grid.get_neighborhood(self.pos, moore=True, include_center=False))):
                        if self.model.is_cell_empty(position) == False:
                            isNeighbor=True
                        else:
                            if isNeighbor==True:
                                self.model.grid.move_agent(self, position)
                                hasMoved=True
                self.hasGradient=False
                self.computeGradient()
                
                self.isLocalized = False
                self.computeLocation()


class Robot2D(Model):
    """
    Model class for the 2D robot model.
    """

    def __init__(self, height=200, width=200):
        """"""

        self.height = height
        self.width = width
        '''
        self.density = density
        self.minority_pc = minority_pc
        self.homophily = homophily

        self.schedule = RandomActivation(self)
        self.grid = SingleGrid(width, height, torus=True)

        self.happy = 0
        self.datacollector = DataCollector(
            {"happy": "happy"},  # Model-level count of happy agents
            # For testing purposes, agent's individual x and y
            {"x": lambda a: a.pos[0], "y": lambda a: a.pos[1]},
        )
        '''
        # Set up agents
        # We use a grid iterator that returns
        # the coordinates of a cell as well as
        # its contents. (coord_iter)
        for cell in self.grid.coord_iter():
            x = cell[1]
            y = cell[2]
            if self.random.random() < self.density:
                if self.random.random() < self.minority_pc:
                    agent_type = 1
                else:
                    agent_type = 0

                agent = SchellingAgent((x, y), self, agent_type)
                self.grid.position_agent(agent, (x, y))
                self.schedule.add(agent)

        self.running = True
        self.datacollector.collect(self)

    def step(self):
        """
        Run one step of the model. If All agents are happy, halt the model.
        """
        self.happy = 0  # Reset counter of happy agents
        self.schedule.step()
        # collect data
        self.datacollector.collect(self)

        if self.happy == self.schedule.get_agent_count():
            self.running = False