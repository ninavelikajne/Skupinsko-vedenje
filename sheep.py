import math

from matplotlib import pyplot as plt
import numpy as np

class SheepHeard:
    def __init__(self, sheepheard_size=24, max_steps=3000):
        self.sheepheard_size = sheepheard_size
        self.success = False
        self.max_steps = max_steps
        self.current_step = 0
        self.vizualize = True

        # Goal position
        self.goal = np.array([115,300])
        self.goal_radius = 45

        # dog starting position
        self.dog_pos = np.array([1.0,1.0])
        self.dog_radius = 50

        # sampling period
        self.Ts = 1


        # inter sheep distance
        self.sheep_radius = 5

        # TODO other parameters
        self.ai = 0.1
        self.wi = 0.1
        self.alpha = 7000
        self.beta = 1400
        self.gama = 140
        self.fi_r = 15
        self.fi_g = 20
        self.fi_d = 30
        self.theta_t = 2*math.pi/3
        self.theta_l = -math.pi/4
        self.theta_r = math.pi/4
        self.r_a = 40
        self.gamma_a = 450
        self.gamma_b = 375



    def run(self):
        """
            Runs the simulation of implemented sheepheard driven sheep transport
        """

        while (not self.success and self.current_step < self.max_steps):

            self.current_step += 1
