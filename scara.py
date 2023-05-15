import math
import matplotlib.pyplot as plt


class Scara:
    def __init__(self, linkages: tuple):
        angles = [0] * len(linkages)  # in degrees
        self.links = list((list(i) for i in zip(linkages, angles)))

        # link[0] = linkage
        # link[1] = angle

    def __str__(self):
        return '\n'.join(list(f'l{i+1}: {link[0]} | a{i+1}: {link[1]}' for i, link in enumerate(self.links)))

    def display(self, print=False):
        # print the linkages and angles if print=True
        if print:
            print(self)

        # setup the plot
        fig, ax = plt.subplots()
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        plt.gca().set_aspect('equal')

        # plot the linkages
        cum_pos = (0, 0)
        cum_angle = 0
        for link in self.links:
            cum_angle += link[1]
            new_pos = (
                cum_pos[0] + link[0] * math.cos(math.radians(cum_angle)),
                cum_pos[1] + link[0] * math.sin(math.radians(cum_angle))
            )
            ax.plot(
                [cum_pos[0], new_pos[0]],
                [cum_pos[1], new_pos[1]],
                linewidth=2, color='blue'
            )
            ax.plot([cum_pos[0]], [cum_pos[1]], 'b.')
            cum_pos = new_pos

        # plot the end effector
        ax.plot([cum_pos[0]], [cum_pos[1]], 'r.')

        # show the plot
        plt.show()

    def inverse(self, target: tuple) -> tuple:
        '''
        pos: tuple of position (x, y)
        returns: tuple of angles in degrees (a1, a2)
        '''
        if len(self.links) != 2:
            raise Exception('Inverse kinematics only works for 2 linkages')

        # a_{2}=\arccos\left(\frac{x_{2}^{2}+y_{2}^{2}-l_{1}^{2}-l_{2}^{2}}{2l_{1}l_{2}}\right)
        # a_{1}=\arctan\left(\frac{y_{2}}{x_{2}}\right)-\arctan\left(\frac{l_{2}\sin\left(a_{2}\right)}{l_{1}+l_{2}\cos\left(a_{2}\right)}\right)

        x, y = target
        l1 = self.links[0][0]
        l2 = self.links[1][0]

        try:
            a2_rad = math.acos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
            a1_rad = (
                math.atan(y / x) -
                math.atan(l2 * math.sin(a2_rad) / (l1 + l2 * math.cos(a2_rad)))
            )
        except ValueError:
            raise Exception('Position is out of reach')

        if x < 0:
            a1_rad += math.pi

        return (math.degrees(a1_rad), math.degrees(a2_rad))

    def inverse_angle(self, target: tuple) -> float:
        '''
        pos: tuple of position (x, y, angle)
        returns: tuple of angles in degrees (a1, a2, a3)
        WIP
        '''
        if len(self.links) != 3:
            raise Exception('Inverse kinematics with angle only works for 3 linkages')

        # a_{2}=\arccos\left(\frac{x_{2}^{2}+y_{2}^{2}-l_{1}^{2}-l_{2}^{2}}{2l_{1}l_{2}}\right)
        # a_{1}=\arctan\left(\frac{y_{2}}{x_{2}}\right)-\arctan\left(\frac{l_{2}\sin\left(a_{2}\right)}{l_{1}+l_{2}\cos\left(a_{2}\right)}\right)

        x, y, at = target
        l1 = self.links[0][0]
        l2 = self.links[1][0]
        l3 = self.links[2][0]

        # project the target to the end of the second linkage
        x, y = x + l3 * math.cos(math.radians(at)), y + l3 * math.sin(math.radians(at))

        try:
            a2_rad = math.acos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
            a1_rad = (
                math.atan(y / x) -
                math.atan(l2 * math.sin(a2_rad) / (l1 + l2 * math.cos(a2_rad)))
            )
        except ValueError:
            raise Exception('Position is out of reach')

        if x < 0:
            a1_rad += math.pi

        a1 = math.degrees(a1_rad)
        a2 = math.degrees(a2_rad)
        a3 = at - a1 - a2 # maybe wrong

        return (a1, a2, a3)

    def forward(self, angles: tuple) -> tuple:
        '''
        angles: tuple of angles in degrees
        returns: tuple of position
        '''
        cum_pos = (0, 0)
        cum_angle = 0

        for i, link in enumerate(self.links):
            cum_angle += angles[i]
            cum_pos = (
                cum_pos[0] + link[0] * math.cos(math.radians(cum_angle)),
                cum_pos[1] + link[0] * math.sin(math.radians(cum_angle))
            )
        return cum_pos

    def set_position(self, pos: tuple):
        self.set_angles(self.inverse(pos))

    def set_angles(self, angles: tuple):
        if len(angles) != len(self.links):
            raise Exception('Number of angles must match number of linkages')
        for i, angle in enumerate(angles):
            self.links[i][1] = angle