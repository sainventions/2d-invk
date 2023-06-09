import math

from main import *
from scara import Scara

'''
READ ME!

To start the program, click the run button on the top right of the screen, make sure to stop the program after you are done please.
'''

# This is not all of the code, in fact it accounts for a tiny fraction of it, explaining the key parts of the code at a high level.

if __name__ == '__main__':
    '''
    Initiates the Scara class, a representation of a 2R Robotic Arm with 2 linkages.
    The Scara class stores the parameters of the arm, and has methods to solve inverse and forward kinematics for the arm in a specified configuration.
    It also has methods to display the arm in a matplotlib plot.
    '''
    links = [50, 50]
    scr = Scara(links)

    '''
    Prompts the user to draw a path on the GUI, and returns the path.
    The path is returned as a function of time.
    The size of the GUI, it is set to the diagonal of the arm so the whole drawing can be calculated
    '''
    size = (links[0]+links[1])/math.sqrt(2)*2
    path = draw_path(size)

    '''
    This path is fed into the path_invk function, which calculates the inverse kinematics for the path.
    It does this by calculating the inverse kinematics for each point in the path, and then interpolating the angles between the points.
    This returns a function for each linkage angle which will be modulated to follow the path.

    This function is the main workhorse of the program, and has many sub-functions that are used to calculate the inverse kinematics and linear interpolations between points.
    '''
    a1_t, a2_t = path_invk(scr, path, 100)

    '''
    This function simulates the path, and displays it in a matplotlib plot.
    To move on to the next point, press the [x] button on the plot.
    '''
    simulate(
        scr, a1_t, a2_t, map_int=100, model_int=100, link_opacity=0.2,
        name='Path IK Simulation'
    )

    '''
    This function animates the path, and displays it in a matplotlib plot.
    To end the animation, press the [x] button on the plot.
    '''
    animate(
        Scara(links), a1_t, a2_t, model_int=100, show=True,
        name='Path IK Animation'
    )
