import math

from scara import Scara
from main import *

# This is not all of the code, in fact it is only 5% of it, showing how to use the modules as a developer

if __name__ == '__main__':
    '''
    Initiates the Scara class, a representation of a 2R Robotic Arm

    The Scara class stores the parameters of the arm, and has methods to solve inverse and forward kinematics for the arm in a specified configuration.
    It also has 

    '''
    links = [50, 50]
    scr = Scara(links)


    # Prepares the GUI
    size = (links[0]+links[1])/math.sqrt(2)*2

    # Prompts the user to draw a path
    path = draw_path(size)

    # Calculates the angles for the path
    a1_t, a2_t = path_invk(scr, path, 100)

    # Simulates the path and displays it
    simulate(
        scr, a1_t, a2_t, map_int=100, model_int=100, link_opacity=0.2,
        name='Path IK Simulation'
    )

    # Animates the path
    animate(
        Scara(links), a1_t, a2_t, model_int=100, show=True,
        name='Path IK Animation'
    )
