import math
import time
import tkinter as tk

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from scara import Scara


def lerp(y0, y1, t):
    '''
    Returns the linear interpolation between y0 and y1 at t in [0, 1]

    Parameters:
        y0 (float): the starting value
        y1 (float): the ending value
        t (float): the interpolation value in [0, 1]

    Returns:
        float: the interpolated value
    '''
    return y0 + (y1 - y0) * t


def _multi_lerp(values, t):
    '''
    Returns the linear interpolation between values at t in [0, 1]

    Parameters:
        values (list): the values to interpolate between
        t (float): the interpolation value in [0, 1]
    '''
    n = len(values)
    if n == 1:
        return values[0]
    else:
        # iterate through the intervals
        for i in range(n-1):  # skip last value
            # if t in interval
            if t >= i / (n-1) and t <= (i+1) / (n-1):
                # interpolate from y0 to y1 from t0 to t1 in the interval
                y0, y1 = values[i], values[i+1]
                t0, t1 = (i)/(n-1), (i+1)/(n-1)
                t_scaled = (t-t0) / (t1-t0)
                return lerp(y0, y1, t_scaled)


def simulate(scr: Scara, f_a1, f_a2, map_int=100, model_int=10, link_opacity=0.3, name='Scara Robot Inverse Kinematics'):
    '''
    Simulates the scara robot with the given parameters and displays the config space and output space plots

    Parameters:
        scr (Scara): the scara robot to simulate
        a1 (function): funtion for the first linkage over t
        a2 (function): funtion for the second linkage over t
        map_int (int): the number of intervals to map the config space to (default: 100)
        model_int (int): the number of intervals to model the output space with (default: 10)
        link_opacity (float): the opacity of the links in output space (default: 0.3)
        name (str): the name of the plot (default: 'Scara Robot Inverse Kinematics')

    Returns:
        None
    '''

    # Set up the figure with two subplots
    fig, axs = plt.subplots(1, 2, figsize=(
        10, 5), gridspec_kw={'width_ratios': [1, 1]})
    fig.suptitle(
        f'{name}\nL1: {scr.links[0][0]} L2: {scr.links[1][0]}  | {map_int} Config Space Intervals | {model_int} Model Intervals')

    # setup the config space
    axs[0].set_title('Config Space')
    axs[0].set_aspect(1/360, 'box')
    axs[0].set_xlim(0, 1)
    axs[0].set_ylim(-180, 180)
    axs[0].set_xlabel('Time')
    axs[0].set_ylabel('Degrees')

    # setup the output space
    axs[1].set_title('Output Space')
    axs[1].set_aspect('equal', 'box')
    axs[1].set_xlim(-100, 100)
    axs[1].set_ylim(-100, 100)
    axs[1].set_ylabel('Y-axis')
    axs[1].set_xlabel('X-axis')

    a1_t = [[], []]
    a2_t = [[], []]
    for t in range(0, map_int+1):
        print(f'config space: {t}/{map_int}')
        t = t / map_int
        a1_t[0].append(t)
        a1_t[1].append(f_a1(t))
        a2_t[0].append(t)
        a2_t[1].append(f_a2(t))

    axs[0].plot(a1_t[0], a1_t[1], 'r-')
    axs[0].plot(a2_t[0], a2_t[1], 'b-')

    for t in range(0, model_int+1):
        print(f'start/end positions: {t}/{model_int}')
        t = t / model_int
        scr.set_angles((f_a1(t), f_a2(t)))
        cum_pos = (0, 0)
        cum_angle = 0
        for link in scr.links:
            cum_angle += link[1]
            new_pos = (
                cum_pos[0] + link[0] * math.cos(math.radians(cum_angle)),
                cum_pos[1] + link[0] * math.sin(math.radians(cum_angle))
            )

            red = int(((1 - t) * 255)*link_opacity+255*(1-link_opacity))
            blue = int((t * 255)*link_opacity+255*(1-link_opacity))
            green = int(255*(1-link_opacity))
            axs[1].plot(
                [cum_pos[0], new_pos[0]],
                [cum_pos[1], new_pos[1]],
                linewidth=2, color=f'#{red:02X}{green:02X}{blue:02X}'
            )
            # axs[1].plot([cum_pos[0]], [cum_pos[1]], 'k.') # plot the joints
            cum_pos = new_pos

    xy_t = [[], []]
    for t in range(0, map_int+1):
        print(f'output space: {t}/{map_int}')
        t_float = t / map_int
        angles = (f_a1(t_float), f_a2(t_float))
        pos = scr.forward(angles)
        xy_t[0].append(pos[0])
        xy_t[1].append(pos[1])
        if t == 0 or t == map_int:
            axs[1].plot([pos[0]], [pos[1]], 'k.')

    axs[1].plot(xy_t[0], xy_t[1], 'k-')

    # Display the plot
    plt.show()


def animate(scr: Scara, f_a1, f_a2, model_int=10, link_opacity=1, show=True, name='Scara Robot Inverse Kinematics'):
    '''
    Animates the scara robot with the given parameters

    Parameters:
        scr (Scara): the scara robot to animate
        a1 (function): funtion for the first linkage over t
        a2 (function): funtion for the second linkage over t
        model_int (int): the number of intervals to model the output space with (default: 10)
        link_opacity (float): the opacity of the links in output space (default: 1)
        show (bool): whether to show the animation (default: True)
        name (str): the name of the plot (default: 'Scara Robot Inverse Kinematics')

    Returns:
        None
    '''
    fig, ax = plt.subplots()

    fig.suptitle(
        f'{name}\nL1: {scr.links[0][0]} L2: {scr.links[1][0]}  | {model_int} Model Intervals'
    )

    def frame(i, scr):
        t = (i+1) / model_int

        ax.clear()
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_aspect('equal', 'box')

        scr.set_angles((f_a1(t), f_a2(t)))
        cum_pos = (0, 0)
        cum_angle = 0
        for link in scr.links:
            cum_angle += link[1]
            new_pos = (
                cum_pos[0] + link[0] * math.cos(math.radians(cum_angle)),
                cum_pos[1] + link[0] * math.sin(math.radians(cum_angle))
            )

            red = int(((1 - t) * 255)*link_opacity+255*(1-link_opacity))
            blue = int((t * 255)*link_opacity+255*(1-link_opacity))
            green = int(255*(1-link_opacity))
            ax.plot(
                [cum_pos[0], new_pos[0]],
                [cum_pos[1], new_pos[1]],
                linewidth=2, color=f'#{red:02X}{green:02X}{blue:02X}'
            )
            ax.plot([cum_pos[0]], [cum_pos[1]], 'k.')  # plot the joints
            cum_pos = new_pos
        print(f'frame: {i+1}/{model_int} rendered')

    anim = FuncAnimation(
        fig, frame, fargs=(scr,), interval=1, frames=model_int, repeat=True
    )
    anim.save('scara.gif', writer='imagemagick', fps=60)

    if show:
        plt.show()


def basic_invk(scr: Scara, start: tuple, end: tuple, intervals: int) -> tuple:
    '''
    Returns a tuple of functions for the inverse kinematics of the scara robot
    that moves from the start position to the end position linearly

    Parameters:
        scr (Scara): the scara robot to animate
        start (tuple): the starting position
        end (tuple): the ending position
        intervals (int): the number of intervals to approximate the inverse kinematics with

    Returns:
        tuple: a tuple of functions for a1 and a2 over t
    '''
    # list of angles
    a1s = []
    a2s = []

    for i in range(intervals):
        # samples at the middle of each interval
        # t = 1/intervals*(i+.5)

        # samples at the left of each interval
        t = 1/intervals*i

        target = lerp(start[0], end[0], t), lerp(start[1], end[1], t)
        a1, a2 = scr.inverse(target)
        a1s.append(a1)
        a2s.append(a2)

    def a1(t: float) -> float:
        return _multi_lerp(a1s, t)

    def a2(t: float) -> float:
        return _multi_lerp(a2s, t)

    return a1, a2


def path_invk(scr: Scara, path, intervals: int) -> tuple:
    '''
    Returns a tuple of functions for the inverse kinematics of the scara robot
    that moves along the given path

    Parameters:
        scr (Scara): the scara robot to animate
        path (function): a parametric function that takes a float t and returns a tuple of the target position

    '''

    # list of angles
    a1s = []
    a2s = []

    for i in range(intervals):
        t = 1/intervals*(i+.5)  # samples at the middle of each interval
        target = path(t)
        a1, a2 = scr.inverse(target)
        a1s.append(a1)
        a2s.append(a2)

    def a1(t: float) -> float:
        return _multi_lerp(a1s, t)

    def a2(t: float) -> float:
        return _multi_lerp(a2s, t)

    return a1, a2


def draw_path(size):
    raw_points = []

    app = tk.Tk()
    app.title("Draw a path")
    app.geometry('1000x1000')
    canvas = tk.Canvas(app)
    canvas.pack(anchor='nw', fill='both', expand=1)

    def get_pos(event):
        global last_x, last_y
        last_x, last_y = event.x, event.y

    def draw(event):
        global last_x, last_y
        canvas.create_line(
            (last_x, last_y, event.x, event.y),
            fill='blue',
            width=6
        )
        last_x, last_y = event.x, event.y

        # limit x and y to [0, 1000]

        x = max(min(event.x, 1000), 0) / 1000 - .5
        y = max(min(event.y, 1000), 0) / 1000 - .5

        x *= size
        y *= size

        raw_points.append((x, -y))

    def finish_drawing():
        app.quit()  # Exit the main loop when the "Done" button is clicked
        app.destroy()

    canvas.bind("<Button-1>", get_pos)
    canvas.bind("<B1-Motion>", draw)

    done_button = tk.Button(app, text="Done", command=finish_drawing)
    done_button.pack(side=tk.BOTTOM)

    app.mainloop()

    if len(raw_points) < 2:
        print('Not enough points')
        exit()

    # unzip the points
    x, y = zip(*raw_points)

    def path(t):
        return (
            _multi_lerp(x, t),
            _multi_lerp(y, t)
        )

    return path


if __name__ == '__main__':
    links = [50, 50]  # 50mm linkages
    scr = Scara(links)

    start = (-40, 50)
    end = (25, -25)

    # linear IK demo
    a1_0, a2_0 = scr.inverse(start)
    a1_1, a2_1 = scr.inverse(end)

    def a1_t(t: float) -> float:
        return lerp(a1_0, a1_1, t)

    def a2_t(t: float) -> float:
        return lerp(a2_0, a2_1, t)

    simulate(
        scr, a1_t, a2_t, map_int=100, model_int=10, link_opacity=0.3,
        name='Linear Angle IK'
    )

    animate(
        Scara(links), a1_t, a2_t, model_int=120, show=True,
        name='Linear Angle IK'
    )

    # Nonlinear IK demo
    a1_t, a2_t = basic_invk(scr, start, end, 100)

    simulate(
        scr, a1_t, a2_t, map_int=100, model_int=100, link_opacity=0.2,
        name='Linear Path IK'
    )

    animate(
        Scara(links), a1_t, a2_t, model_int=120, show=True,
        name='Linear Path IK'
    )

    # Path IK demo
    # \left(150t-75,\frac{50}{1+\left(5\left(t-.5\right)\right)^{2}}\right)
    def path(t): return (150*t-75, 50/(1+(5*(t-.5))**2))

    a1_t, a2_t = path_invk(scr, path, 100)

    simulate(
        scr, a1_t, a2_t, map_int=100, model_int=100, link_opacity=0.2,
        name='Path IK'
    )

    animate(
        Scara(links), a1_t, a2_t, model_int=120, show=True,
        name='Path IK'
    )

    # Draw path IK demo
    size = (links[0]+links[1])/math.sqrt(2)*2
    path = draw_path(size)

    time.sleep(1)

    a1_t, a2_t = path_invk(scr, path, 100)

    simulate(
        scr, a1_t, a2_t, map_int=100, model_int=100, link_opacity=0.2,
        name='Path IK'
    )

    animate(
        Scara(links), a1_t, a2_t, model_int=120, show=True,
        name='Path IK'
    )
