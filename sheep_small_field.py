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
        self.goal = np.array([115, 300])
        # phi_o
        self.goal_radius = 45

        # dog starting position
        self.dog_pos = np.array([0.0, 0.0])
        # phi_n
        self.dog_radius = 100

        # sampling period
        self.Ts = 0.01

        # sheep
        self.sheep_list = [np.array([57.0, 50.0]),np.array([44.0, 71.0]),np.array([59.0, 64.0]),np.array([73.0, 71.0]),np.array([78.0, 60.0]),np.array([82.0, 71.0]),np.array([87.0, 58.0])]
        # self.sheep_list = np.array(self.sheep_list)
        # self.sheep_list[:,0] += 10
        # self.sheep_list[:,1] += 10
        # self.sheep_list = list(self.sheep_list)

        # inter sheep distance phi_s
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
        self.theta_t =  2*math.pi / 3
        self.theta_l = math.pi/4
        self.theta_r = -math.pi/4
        self.r_a = 50
        self.gamma_a = 450
        self.gamma_b = 375


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
            # print(self.dog_velocity)
            # print(self.dog_velocity)
            visibilitiy=visible_sheep(self.dog_pos,self.dog_radius, self.goal,self.sheep_list)
            # right_most_visible_from_dog(self.sheep_list, visibilitiy, self.dog_pos)
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
            plt.clf()
            s = np.array(self.sheep_list)
            plt.scatter(s[:, 0], s[:, 1],c='blue')
            plt.scatter(self.dog_pos[0], self.dog_pos[1],c='red')
            plt.scatter(self.goal[0], self.goal[1])
            plt.xlim([0, 250])
            plt.ylim([0, 350])
            visibilitiy = visible_sheep(self.dog_pos, self.dog_radius, self.goal, self.sheep_list)
            calculate_center_of_visible_sheep(self.sheep_list, visibilitiy)

            # left_most_visible_from_sheepfold(self.sheep_list, visibilitiy, self.goal)
            # right_most_visible_from_sheepfold(self.sheep_list, visibilitiy, self.goal)
            plt.show(block=False)




    def calculate_dog_velocity(self):
        sheep_at_goal = 0
        for sheep_location in self.sheep_list:
            if is_in_goal_area(sheep_location, self.goal, self.goal_radius):
                sheep_at_goal += 1

        if sheep_at_goal < self.N:
            if all_on_right(self.dog_pos, self.goal, self.sheep_list) and calculateLC(self.sheep_list, self.goal, self.dog_radius, 'left', self.dog_pos) > self.theta_t:
                self.lmbda=0
                # TODO visibility smo že zračunali
                temp =((right_most_visible_from_dog(self.sheep_list,
                                                                        visible_sheep(self.dog_pos, self.dog_radius,
                                                                                      self.goal, self.sheep_list), self.goal))-self.dog_pos)
                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c,s = np.cos(self.theta_r), np.sin(self.theta_r)
                    R = np.array(((c,-s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

            elif all_on_left(self.dog_pos, self.goal, self.sheep_list) and calculateLC(self.sheep_list, self.goal, self.dog_radius, 'right', self.dog_pos) > self.theta_t:
                self.lmbda=1
                # TODO visibility smo že zračunali
                temp =((left_most_visible_from_dog(self.sheep_list,
                                                                        visible_sheep(self.dog_pos, self.dog_radius,
                                                                                      self.goal, self.sheep_list), self.goal))-self.dog_pos)


                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

            elif self.lmbda == 1:
                temp = (( left_most_visible_from_dog(self.sheep_list,   visible_sheep(self.dog_pos,
                                                                                                  self.dog_radius,
                                                                                                  self.goal,
                                                                                                  self.sheep_list), self.goal))-self.dog_pos)

                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))
            else:
                temp = ((right_most_visible_from_dog(self.sheep_list,
                                                                               visible_sheep(self.dog_pos,
                                                                                             self.dog_radius,
                                                                                             self.goal,
                                                                                             self.sheep_list), self.goal))-self.dog_pos)
                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))


        else:
            self.dog_velocity=0









