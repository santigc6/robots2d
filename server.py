from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.modules import CanvasGrid, ChartModule, TextElement
from mesa.visualization.UserParam import UserSettableParameter
from model import Robot2D


class HappyElement(TextElement):
    """
    Display a text count of how many happy agents there are.
    """

    def __init__(self):
        pass

    def render(self, model):
        #return "Happy agents: " + str(model.happy)
        return " "

def robot_grid(agent):
    """
    Portrayal Method for canvas
    """
    if agent is None:
        return
    portrayal = {"Shape": "circle", "r": 1, "Filled": "true", "Layer": 0}

    if agent.type == 0:
        portrayal["Color"] = "green"
    else:
        if agent.hasGradient:
            if agent.isLocalized:
                if agent.isMoving:
                    if agent.finished:
                        portrayal["Color"]="black"
                    else:
                        portrayal["Color"]="purple"
                else:
                    if agent.finished:
                        portrayal["Color"]="black"
                    else:
                        portrayal["Color"]= "blue"
            else:
                portrayal["Color"] = "red"
        else:
            portrayal["Color"]= "grey"
    return portrayal


#happy_element = HappyElement()
canvas_element = CanvasGrid(robot_grid, 75, 75, 850, 850)
#happy_chart = ChartModule([{"Label": "happy", "Color": "Black"}])


model_params = {
    "height": 75,
    "width": 75,
}

server = ModularServer(
    Robot2D, [canvas_element], "Robot2D model", model_params
)