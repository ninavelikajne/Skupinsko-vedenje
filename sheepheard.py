import math
import numpy as np
from matplotlib import pyplot as plt

from helper import *
import random


class SheepHeard:
    def __init__(self, sheepheard_size=40, max_steps=50000):
        self.N = sheepheard_size
        self.success = False
        self.max_steps = max_steps
        self.current_step = 0
        self.vizualize = True
        self.lmbda = 0
        self.dog_velocity = np.array([0, 0])
        # Goal position
        self.goal = np.array([115, 300])
        # phi_o
        self.goal_radius = 45

        # dog starting position
        self.dog_pos = np.array([63.0, 10.0])
        # phi_n
        self.dog_radius = 100

        # sampling period
        self.Ts = 0.001 # todo edit

        # sheep
        self.sheep_list = []
        xsize = 100
        ysize = 50
        xstart = 45
        ystart = 60

        if self.N!=24:
            for i in range(self.N):
                self.sheep_list.append(np.array([random.random() * xsize + xstart, random.random() * ysize + ystart]))

        else:
            self.sheep_list = [np.array([57, 50]), np.array([44, 71]), np.array([59, 64]), np.array([73, 71]),
                           np.array([78, 60]), np.array([82, 71]), np.array([87, 58]), np.array([96, 71]),
                           np.array([55, 76]), np.array([64, 83]), np.array([69, 79]), np.array([50, 60]),
                           np.array([87, 76]), np.array([95, 84]), np.array([100, 76]), np.array([105, 79]),
                           np.array([65, 94]), np.array([69, 90]), np.array([105, 85]), np.array([79, 95]),
                           np.array([84, 90]), np.array([90, 99]), np.array([100, 55]), np.array([105, 60])]

        # self.sheep_list = [np.array([57.0, 80.0]), np.array([44.0, 71.0]), np.array([59.0, 64.0]),
        #                    np.array([73.0, 71.0]), np.array([78.0, 60.0]), np.array([82.0, 71.0]),
        #                    np.array([87.0, 58.0])]
        # self.sheep_list = np.array(self.sheep_list)
        # self.sheep_list[:,0] += 20
        # self.sheep_list[:,1] += 20
        # self.sheep_list = list(self.sheep_list)
        # self.sheep_list+=[np.array([57.0, 80.0]), np.array([44.0, 71.0]), np.array([59.0, 64.0]), np.array([73.0, 71.0]),
        #   np.array([78.0, 60.0]), np.array([82.0, 71.0]), np.array([87.0, 58.0])]
        # self.sheep_list = np.array(self.sheep_list)
        # self.sheep_list[:, 1] += 20
        # self.sheep_list = list(self.sheep_list)
        # self.sheep_list += [np.array([57.0, 80.0]), np.array([44.0, 71.0]), np.array([59.0, 64.0]),
        #                     np.array([73.0, 71.0]),
        #                     np.array([78.0, 60.0]), np.array([82.0, 71.0]), np.array([87.0, 58.0])]
        # self.sheep_list = np.array(self.sheep_list)
        # self.sheep_list[:, 0] += 10
        # self.sheep_list = list(self.sheep_list)
        # self.sheep_list += [np.array([57.0, 80.0]), np.array([44.0, 71.0]), np.array([59.0, 64.0]),
        #                     np.array([73.0, 71.0]),
        #                     np.array([78.0, 60.0]), np.array([82.0, 71.0]), np.array([87.0, 58.0])]
        # self.sheep_list = np.array(self.sheep_list)
        # self.sheep_list[:, 0] += 10
        # self.sheep_list = list(self.sheep_list)
        # self.sheep_list += [np.array([57.0, 80.0]), np.array([44.0, 71.0]), np.array([59.0, 64.0]),
        #                     np.array([73.0, 71.0]),
        #                     np.array([78.0, 60.0]), np.array([82.0, 71.0]), np.array([87.0, 58.0])]
        # self.obstacles = []#[np.array([100,200]), np.array([-1000,-1000])]

        # # 1 poligon
        # a = np.linspace(0, 2 * np.pi, 20)
        # radius = 2
        # x = radius * np.cos(a) + 70
        # y = radius * np.sin(a) + 100
        # self.obstacles = [np.array([x[i], y[i]]) for i in range(20)]

        # # 2 poligon
        # a = np.linspace(0, 2 * np.pi, 20)
        # radius = 2
        # x = radius * np.cos(a) + 70
        # y = radius * np.sin(a) + 100
        # self.obstacles = [np.array([x[i], y[i]]) for i in range(20)]
        # radius = 3
        # x = radius * np.cos(a) + 100
        # y = radius * np.sin(a) + 200
        # self.obstacles += [np.array([x[i], y[i]]) for i in range(20)]


        # 3 poligon
        self.obstacles = np.linspace(np.array([100,200]),np.array([120,190]),30)


        # obstacle avoidance
        self.obstacle_radius = 10
        self.obstacle_effect_radius = 20

        # inter sheep distance phi_s
        self.sheep_radius = 5

        self.ai = 0.1
        self.wi = 0.1
        self.alpha = 9000
        self.beta = 1400
        self.gama = -10
        self.zeta = 300
        self.fi_r = 15
        self.fi_g = 20
        self.fi_d = 30
        self.theta_t = 2 * math.pi / 4
        # possible error + -
        self.theta_l = math.pi / 6
        self.theta_r = -math.pi / 6
        self.r_a = 40
        self.gamma_a = 750
        self.gamma_b = 575

    def run(self):
        """
            Runs the simulation of implemented sheepheard driven sheep transport
        """
        i = 0
        while (not self.success and self.current_step < self.max_steps):
            if i % 10 == 0: #todo edit
                fig = plt.figure(figsize=(7, 7))
                ax = fig.subplots()
                s = np.array(self.sheep_list)
                ax.scatter(s[:, 0], s[:, 1], c='blue')
                ax.scatter(self.dog_pos[0], self.dog_pos[1], c='red')
                circle = plt.Circle((self.goal[0], self.goal[1]), self.goal_radius, alpha=0.3, color='green')
                ax.add_artist(circle)
                for o in self.obstacles:
                    plt.scatter(o[0], o[1], c='black')

                # visibility = visible_sheep(self.dog_pos, self.dog_radius,
                #                            self.goal, self.sheep_list)
                #
                # right = (right_most_visible_from_dog(self.sheep_list, visibility
                #                                      , self.dog_pos))
                # left = (left_most_visible_from_dog(self.sheep_list, visibility
                #                                    , self.dog_pos))
                # plt.scatter(right[0], right[1], c='orange')
                # plt.scatter(left[0], left[1], c='green')

                ax.set_xlim([0, 350])
                # plt.xlim([50, 150])
                ax.set_ylim([0, 350])
                ax.set_title("Number of sheep: " + str(self.N))
                fig.savefig("./figs/" + str(str(i).zfill(9)))
                plt.close()

            i += 1
            self.current_step += 1
            self.calculate_dog_velocity()
            self.dog_pos = self.dog_pos + self.Ts * self.dog_velocity

            for ii, sheep in enumerate(self.sheep_list):
                x = vector_size(sheep - self.dog_pos)
                if x > 0 and x <= self.dog_radius:
                    fi = self.alpha * (1 / x - 1 / self.dog_radius)
                else:
                    fi = 0

                v_di = fi * unit_vector(sheep - self.dog_pos)

                v_si = 0
                for sheep_ in self.sheep_list:
                    #
                    if (sheep == sheep_).all():
                        continue
                    temp_size = vector_size(sheep_ - sheep)
                    if temp_size < self.fi_r:
                        psi = ((self.fi_r - temp_size) / self.fi_r) * self.zeta

                    elif temp_size >= self.fi_r and temp_size <= self.fi_g:
                        psi = 0
                    elif temp_size > self.fi_g and temp_size <= self.fi_d:
                        psi = self.gama * (temp_size - self.fi_g)
                    elif temp_size > self.fi_d:
                        psi = 0

                    v_si += psi * unit_vector(sheep - sheep_)

                theta = self.ai * math.pi / 180 * math.sin(self.wi * self.current_step * self.Ts)
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c)))
                velocity_sheep = v_di + R.dot(v_si)

                # obstacle avoidance
                for obstacle in self.obstacles:
                    obstacle_sheep_dist = vector_size(obstacle - sheep)
                    if obstacle_sheep_dist < self.obstacle_effect_radius and is_line_intersecting_circle(velocity_sheep,
                                                                                                         sheep - obstacle,
                                                                                                         self.obstacle_radius):
                        angle = calculate_angle_between_vectors(obstacle - sheep, velocity_sheep)
                        if math.fabs(angle) > math.pi / 3:
                            continue
                        angle_corr = (obstacle_sheep_dist / self.obstacle_effect_radius) * 3 + 0.7
                        if angle < 0:
                            # go to right
                            c, s = np.cos(self.theta_r / angle_corr), np.sin(self.theta_r / angle_corr)
                            R = np.array(((c, -s), (s, c)))
                            # mnozenje matrike z vektorjem
                            velocity_sheep = np.array(R).dot(velocity_sheep)

                        else:
                            # go to left
                            c, s = np.cos(self.theta_l / angle_corr), np.sin(self.theta_l / angle_corr)
                            R = np.array(((c, -s), (s, c)))
                            # mnozenje matrike z vektorjem
                            velocity_sheep = np.array(R).dot(velocity_sheep)

                self.sheep_list[ii] = self.sheep_list[ii] + (self.Ts * velocity_sheep)

            visibilitiy = visible_sheep(self.dog_pos, self.dog_radius, self.goal, self.sheep_list)
            calculate_center_of_visible_sheep(self.sheep_list, visibilitiy)

    def calculate_dog_velocity(self):
        sheep_at_goal = 0
        for sheep_location in self.sheep_list:
            if is_in_goal_area(sheep_location, self.goal, self.goal_radius):
                sheep_at_goal += 1

        if sheep_at_goal < self.N:
            visibility = visible_sheep(self.dog_pos, self.dog_radius,
                                       self.goal, self.sheep_list)

            if all_on_right(self.dog_pos, self.goal, self.sheep_list) and calculateLC(self.sheep_list, self.goal,
                                                                                      self.dog_radius, 'left',
                                                                                      self.dog_pos) > self.theta_t:
                self.lmbda = 0
                temp = ((right_most_visible_from_dog(self.sheep_list, visibility
                                                     , self.dog_pos)) - self.dog_pos)
                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

            elif all_on_left(self.dog_pos, self.goal, self.sheep_list) and calculateLC(self.sheep_list, self.goal,
                                                                                       self.dog_radius, 'right',
                                                                                       self.dog_pos) > self.theta_t:
                self.lmbda = 1
                temp = ((left_most_visible_from_dog(self.sheep_list,
                                                    visibility, self.dog_pos)) - self.dog_pos)

                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

            elif self.lmbda == 1:
                temp = ((left_most_visible_from_dog(self.sheep_list, visibility, self.dog_pos)) - self.dog_pos)

                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))
            else:
                temp = ((right_most_visible_from_dog(self.sheep_list, visibility, self.dog_pos)) - self.dog_pos)
                if vector_size(temp) >= self.r_a:
                    self.dog_velocity = self.gamma_a * unit_vector(temp)
                else:
                    c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))


        else:
            self.dog_velocity = np.array([0, 0])
            self.max_steps = self.current_step + 2

        for obstacle in self.obstacles:
            obstacle_sheep_dist = vector_size(obstacle - self.dog_pos)
            if obstacle_sheep_dist < self.obstacle_effect_radius and is_line_intersecting_circle(self.dog_velocity,
                                                                                                 self.dog_pos - obstacle,
                                                                                                 self.obstacle_radius):
                angle = calculate_angle_between_vectors(obstacle - self.dog_pos, self.dog_velocity)
                if math.fabs(angle) > math.pi / 3:
                    continue
                angle_corr = (obstacle_sheep_dist / self.obstacle_effect_radius) * 3 + 1
                if angle < 0:
                    # go to right
                    c, s = np.cos(self.theta_r / angle_corr), np.sin(self.theta_r / angle_corr)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = np.array(R).dot(self.dog_velocity)
                else:
                    # go to left
                    c, s = np.cos(self.theta_l / angle_corr), np.sin(self.theta_l / angle_corr)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    self.dog_velocity = np.array(R).dot(self.dog_velocity)
