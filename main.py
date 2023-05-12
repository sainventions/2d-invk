import matplotlib.pyplot as plt
import numpy as np
import math

import scara


def _linterp(y0, y1, x):
    return y0 + (y1 - y0) * x


def a1(t: float) -> float:
    return _linterp(-45, 172, t)


def a2(t: float) -> float:
    return _linterp(164, 50, t)


def simulate(links: list, a1, a2, map_interval=100, model_interval=10, link_opacity=0.3):
    # Set up the figure with two subplots
    fig, axs = plt.subplots(1, 2, figsize=(
        10, 5), gridspec_kw={'width_ratios': [1, 1]})
    fig.suptitle(
        f'Scara Robot Inverse Kinematics\nL1: {links[0]} L2: {links[1]}  | {map_interval} Config Space Intervals | {model_interval} Model Intervals')

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
    for t in range(0, map_interval+1):
        print(f'config space: {t}/{map_interval}')
        t = t / map_interval
        a1_t[0].append(t)
        a1_t[1].append(a1(t))
        a2_t[0].append(t)
        a2_t[1].append(a2(t))

    axs[0].plot(a1_t[0], a1_t[1], 'r-')
    axs[0].plot(a2_t[0], a2_t[1], 'b-')

    scr = scara.scara(links)

    for t in range(0, model_interval+1):
        print(f'start/end positions: {t}/{model_interval}')
        t = t / model_interval
        scr.set_angles((a1(t), a2(t)))
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
    for t in range(0, map_interval+1):
        print(f'output space: {t}/{map_interval}')
        t = t / map_interval
        angles = (a1(t), a2(t))
        pos = scr.forward(angles)
        xy_t[0].append(pos[0])
        xy_t[1].append(pos[1])

    axs[1].plot(xy_t[0], xy_t[1], 'k-')

    # Display the plot
    plt.show()


links = [50, 50]  # 50mm linkages
simulate(links, a1, a2, map_interval=100, model_interval=10, link_opacity=0.3)
