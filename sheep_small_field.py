import math

from matplotlib import pyplot as plt
import numpy as np

from helper import *


class SheepHeard:
    def __init__(self, sheepheard_size=7, max_steps=3000):
        self.N = sheepheard_size
        self.success = False
        self.max_steps = max_steps
        self.current_step = 0
        self.vizualize = True
        self.lmbda=1
        self.dog_velocity = 0

        # Goal position
        self.goal = np.array([10, 10])
        # phi_o
        self.goal_radius = 4

        # dog starting position
        self.dog_pos = np.array([-10.0, -10.0])
        # phi_n
        self.dog_radius = 20

        # sampling period
        self.Ts = 0.01

        # sheep
        self.sheep_list = [np.array([-8.5, -8.0]),np.array([-7.5, -7.0]),np.array([-7.5, -8.0]),np.array([-7, -7.5]),np.array([-6.8, -8.0]),np.array([-6.0, -7.0]),np.array([-7.0, -6.5])]


        # inter sheep distance phi_s
        self.sheep_radius = 0.2

        # TODO other parameters
        self.ai = 0.1
        self.wi = 0.1
        self.alpha = 75
        self.beta = 0.375
        self.gama = -0.5
        self.fi_r = 0.5
        self.fi_g = 2
        self.fi_d = 20
        self.theta_t = 3 * math.pi / 4
        self.theta_l = math.pi / 4
        self.theta_r = -math.pi / 4
        self.r_a = 3
        self.gamma_a = 15
        self.gamma_b = 12.5


    def run(self):
        """
            Runs the simulation of implemented sheepheard driven sheep transport
        """

        while (not self.success and self.current_step < self.max_steps):
            from matplotlib import pyplot as plt
            # plt.scatter()
            self.current_step += 1
            # u(current_step)
            self.calculate_dog_velocity()
            print(self.dog_velocity)
            visibilitiy=visible_sheep(self.dog_pos,self.dog_radius, self.goal,self.sheep_list)
            right_most_visible_from_dog(self.sheep_list, visibilitiy, self.dog_pos)
            self.dog_pos = self.dog_pos+self.Ts * self.dog_velocity

            for ii, sheep in enumerate(self.sheep_list):
                x=vector_size(sheep-self.dog_pos)
                if x>0 and x<=self.dog_radius:
                    fi =self.alpha* (1/x - 1/self.dog_radius)
                else:
                    fi=0

                v_di = fi*unit_vector(sheep-self.dog_pos)
                for sheep_ in self.sheep_list:
                    if (sheep == sheep_).all():
                        continue
                    temp_size = vector_size(sheep - sheep_)
                    if temp_size > self.sheep_radius and temp_size <= self.fi_r:
                        psi = self.beta * (1/(temp_size-self.sheep_radius)-(1/(self.fi_r-self.sheep_radius)))
                    elif temp_size > self.fi_r and temp_size <= self.fi_g:
                        psi = 0
                    elif temp_size > self.fi_g and temp_size <= self.fi_d:
                        psi = self.gama*(temp_size-self.fi_g)
                    elif temp_size > self.fi_d:
                        psi = 0

                    v_si = psi*unit_vector(sheep-sheep_)
                theta = self.ai * math.pi/180 * math.sin(self.wi*self.current_step*self.Ts)
                c, s = np.cos(self.theta_r), np.sin(theta)
                R = np.array(((c, -s), (s, c)))
                velocity_sheep = v_di +   v_si.dot(R)
                self.sheep_list[ii] = self.sheep_list[ii]+(self.Ts * velocity_sheep)



            from matplotlib import pyplot as plt
            s = np.array(self.sheep_list)
            plt.scatter(s[:, 0], s[:, 1])
            plt.scatter(self.dog_pos[0], self.dog_pos[1])
            plt.scatter(self.goal[0], self.goal[1])
            plt.show()




    def calculate_dog_velocity(self):
        sheep_at_goal = 0
        for sheep_location in self.sheep_list:
            if is_in_goal_area(sheep_location, self.goal, self.goal_radius):
                sheep_at_goal += 1

        if sheep_at_goal < self.N:
            if all_on_right(self.dog_pos, self.goal, self.sheep_list) and calculateLC(self.sheep_list, self.goal, self.dog_radius, 'left', self.dog_pos) > self.theta_t:
                self.lmbda=0
                # TODO visibility smo že zračunali
                temp =((self.dog_pos - right_most_visible_from_dog(self.sheep_list,
                                                                        visible_sheep(self.dog_pos, self.dog_radius,
                                                                                      self.goal, self.sheep_list), self.goal)))
                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c,s = np.cos(self.theta_r), np.sin(self.theta_r)
                    R = np.array(((c,-s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(unit_vector(temp)).dot(R)

            elif all_on_left(self.dog_pos, self.goal, self.sheep_list) and calculateLC(self.sheep_list, self.goal, self.dog_radius, 'right', self.dog_pos) > self.theta_t:
                self.lmbda=1
                # TODO visibility smo že zračunali
                temp =((self.dog_pos - left_most_visible_from_sheepfold(self.sheep_list,
                                                                        visible_sheep(self.dog_pos, self.dog_radius,
                                                                                      self.goal, self.sheep_list), self.goal)))


                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(unit_vector(temp)).dot(R)

            elif self.lmbda == 1:
                temp = ((self.dog_pos - left_most_visible_from_sheepfold(self.sheep_list,
                                                                                    visible_sheep(self.dog_pos,
                                                                                                  self.dog_radius,
                                                                                                  self.goal,
                                                                                                  self.sheep_list), self.goal)))

                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(unit_vector(temp)).dot(R)
            else:
                temp = ((self.dog_pos - right_most_visible_from_dog(self.sheep_list,
                                                                               visible_sheep(self.dog_pos,
                                                                                             self.dog_radius,
                                                                                             self.goal,
                                                                                             self.sheep_list), self.goal)))
                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(unit_vector(temp)).dot(R)


        else:
            self.dog_velocity=0









