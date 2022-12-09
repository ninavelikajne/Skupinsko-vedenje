import math

from matplotlib import pyplot as plt
import numpy as np
import random
from helper import *


class SheepHeard:
    def __init__(self, sheepheard_size=40, max_steps=3000):
        self.N = sheepheard_size
        self.success = False
        self.max_steps = max_steps
        self.current_step = 0
        self.vizualize = True
        self.lmbda1=1
        self.lmbda=0
        self.dog_velocity = np.array([0,0])
        # Goal position
        self.goal = np.array([115, 300])
        # phi_o
        self.goal_radius = 45

        # dog starting position
        self.dog_pos = np.array([100.0, 50.0])
        self.dog_pos1 = np.array([50.0, 50.0])
        # phi_n
        self.dog_radius = 100

        #obstacle avoidance
        self.obstacle_radius = 20
        self.obstacle_effect_radius = 50

        # sampling period
        self.Ts = 0.01

        # sheep
        self.sheep_list = []
        xsize = 100
        ysize = 50
        xstart=45
        ystart=60
        for i in range(self.N):
            self.sheep_list.append(np.array([random.random()*xsize+xstart, random.random()*ysize+ystart]))
        # self.sheep_list = [np.array([57.0, 80.0]),np.array([44.0, 71.0]),np.array([59.0, 64.0]),np.array([73.0, 71.0]),np.array([78.0, 60.0]),np.array([82.0, 71.0]),np.array([87.0, 58.0])]
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

        a = np.linspace(0, 2 * np.pi, 50)
        radius = 5
        x = radius * np.cos(a) + 100
        y = radius * np.sin(a) + 200

        self.obstacles = []# [np.array([x[i], y[i]]) for i in range(50)]
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
        self.theta_t =  2*math.pi / 4
        # possible error + -
        self.theta_l = math.pi/6
        self.theta_r = -math.pi/6
        self.r_a = 40
        self.gamma_a = 750
        self.gamma_b = 575

        # self.N = sheepheard_size
        # self.success = False
        # self.max_steps = max_steps
        # self.current_step = 0
        # self.vizualize = True
        # self.lmbda=1
        # self.dog_velocity = 0
        #
        # # Goal position
        # self.goal = np.array([115, 300])
        # # phi_o
        # self.goal_radius = 45
        #
        # # dog starting position
        # self.dog_pos = np.array([10.0, 10.0])
        # # phi_n
        # self.dog_radius = 70
        #
        # # sampling period
        # self.Ts = 0.001
        #
        # # sheep
        # self.sheep_list = [np.array([57.0, 50.0]),np.array([44.0, 71.0]),np.array([59.0, 64.0]),np.array([73.0, 71.0]),np.array([78.0, 60.0]),np.array([82.0, 71.0]),np.array([87.0, 58.0])]
        # self.sheep_list = np.array(self.sheep_list)
        # self.sheep_list[:,0] += 20
        # self.sheep_list[:,1] += 20
        # self.sheep_list = list(self.sheep_list)
        # self.sheep_list+=[np.array([57.0, 50.0]), np.array([44.0, 71.0]), np.array([59.0, 64.0]), np.array([73.0, 71.0]),
        #  np.array([78.0, 60.0]), np.array([82.0, 71.0]), np.array([87.0, 58.0])]
        #
        # # inter sheep distance phi_s
        # self.sheep_radius = 5
        #
        # self.ai = 0.1
        # self.wi = 0.1
        # self.alpha = 7000
        # self.beta = 1400
        # self.gama = -140
        # self.fi_r = 15
        # self.fi_g = 20
        # self.fi_d = 30
        # self.theta_t =  2*math.pi / 3
        # self.theta_l = math.pi/4
        # self.theta_r = -math.pi/4
        # self.r_a = 50
        # self.gamma_a = 750
        # self.gamma_b = 575


    def run(self):
        """
            Runs the simulation of implemented sheepheard driven sheep transport
        """
        i=0
        while (not self.success and self.current_step < self.max_steps):
            from matplotlib import pyplot as plt

            # if i%10==0:
            # plt.clf()
            fig = plt.figure(figsize=(7,7))
            ax = fig.subplots()
            s = np.array(self.sheep_list)
            ax.scatter(s[:, 0], s[:, 1], c='blue')
            ax.scatter(self.dog_pos[0], self.dog_pos[1], c='red')
            ax.scatter(self.dog_pos1[0], self.dog_pos1[1], c='orange')
            circle = plt.Circle((self.goal[0], self.goal[1]), self.goal_radius, alpha=0.3, color='green')
            ax.add_artist(circle)
            for o in self.obstacles:
                plt.scatter(o[0], o[1], c='green')
            # plt.scatter(self.obstacles[0][0],self.obstacles[0][1])
            ax.set_xlim([0, 350])
            # plt.xlim([50, 150])
            ax.set_ylim([0, 350])
            ax.set_title("Number of sheep: "+str(self.N))
            fig.savefig("./figs/"+str(str(i).zfill(3)))
            plt.close()
            # plt.ylim([150, 250])
            # fig.show()

            i+=1

            self.current_step += 1

            self.dog_velocity =self.calculate_dog_velocity(self.dog_pos,self.dog_pos1)
            self.dog_velocity1 =self.calculate_dog_velocity(self.dog_pos1,self.dog_pos)


            self.dog_pos = self.dog_pos+self.Ts * self.dog_velocity
            self.dog_pos1 = self.dog_pos1+self.Ts * self.dog_velocity1
            for dog in [self.dog_pos1, self.dog_pos]:
                for ii, sheep in enumerate(self.sheep_list):

                    x=vector_size(sheep-dog)
                    if x>0 and x<=self.dog_radius:
                        fi =self.alpha* (1/x - 1/self.dog_radius)
                    else:
                        fi=0

                    v_di = fi*unit_vector(sheep-dog)

                    v_si=0
                    for sheep_ in self.sheep_list:
                        #
                        if (sheep == sheep_).all():
                            continue
                        temp_size = vector_size(sheep_-sheep)
                        if temp_size < self.fi_r: #:
                            psi=((self.fi_r-temp_size)/self.fi_r)*self.zeta
                            # psi = self.beta * (1/(math.ceil(temp_size)-self.sheep_radius)-(1/(self.fi_r-self.sheep_radius)))

                        elif temp_size >= self.fi_r and temp_size <= self.fi_g:
                            psi = 0
                        elif temp_size > self.fi_g and temp_size <= self.fi_d:
                            psi = self.gama*(temp_size-self.fi_g)
                        elif temp_size > self.fi_d:
                            psi = 0

                        v_si += psi*unit_vector(sheep-sheep_)




                    theta = self.ai * math.pi/180 * math.sin(self.wi*self.current_step*self.Ts)
                    c, s = np.cos(theta), np.sin(theta)
                    R = np.array(((c, -s), (s, c)))
                    velocity_sheep = v_di + R.dot(v_si)

                    # obstacle avoidance
                    for obstacle in self.obstacles:
                        obstacle_sheep_dist = vector_size(obstacle-sheep)
                        if obstacle_sheep_dist < self.obstacle_effect_radius and \
                            is_line_intersecting_circle(velocity_sheep, sheep-obstacle, self.obstacle_radius):
                            # plt.scatter(sheep[0], sheep[1])
                            # s = sheep + velocity_sheep
                            # plt.scatter(s[0], s[1])
                            angle = calculate_angle_between_vectors(obstacle - sheep, velocity_sheep)
                            if math.fabs(angle) > math.pi/3:
                                continue
                            angle_corr = (obstacle_sheep_dist / self.obstacle_effect_radius) * 3 + 0.7
                            if angle < 0:
                                # go to right
                                c, s = np.cos(self.theta_r/angle_corr), np.sin(self.theta_r/angle_corr)
                                R = np.array(((c, -s), (s, c)))
                                # mnozenje matrike z vektorjem
                                velocity_sheep = np.array(R).dot(velocity_sheep)
                            else:
                                # go to left
                                c, s = np.cos(self.theta_l/angle_corr), np.sin(self.theta_l/angle_corr)
                                R = np.array(((c, -s), (s, c)))
                                # mnozenje matrike z vektorjem
                                velocity_sheep = np.array(R).dot(velocity_sheep)





                    self.sheep_list[ii] = self.sheep_list[ii]+(self.Ts/2 * velocity_sheep)




            visibilitiy = visible_sheep(self.dog_pos, self.dog_radius, self.goal, self.sheep_list)
            calculate_center_of_visible_sheep(self.sheep_list, visibilitiy)

            # left_most_visible_from_sheepfold(self.sheep_list, visibilitiy, self.goal)
            # right_most_visible_from_sheepfold(self.sheep_list, visibilitiy, self.goal)





    def calculate_dog_velocity(self, dog_pos, other_dog):
        sheep_at_goal = 0
        for sheep_location in self.sheep_list:
            if is_in_goal_area(sheep_location, self.goal, self.goal_radius):
                sheep_at_goal += 1

        if sheep_at_goal < self.N:
            visibility =visible_sheep(dog_pos, self.dog_radius, self.goal, self.sheep_list)
            center_of_visible_sheep =  calculate_center_of_visible_sheep(self.sheep_list, visibility)
            angle = calculate_angle_between_vectors(center_of_visible_sheep-dog_pos, other_dog-dog_pos)

            # right dog
            if angle > 0:
                center_of_mass_goal_vector = center_of_visible_sheep-self.goal
                dist = np.linalg.norm(np.cross(center_of_visible_sheep-self.goal, self.goal-dog_pos)/np.linalg.norm(center_of_visible_sheep-self.goal))
                if dist < 5:
                    self.lmbda1=0

                    temp =((right_most_visible_from_dog(self.sheep_list, visibility, dog_pos))-self.dog_pos)
                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c,s = np.cos(self.theta_r), np.sin(self.theta_r)
                        R = np.array(((c,-s), (s, c)))
                        # mnozenje matrike z vektorjem
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

                elif all_on_left(dog_pos, self.goal, self.sheep_list) and calculateLC(self.sheep_list, self.goal, self.dog_radius, 'right', self.dog_pos) > self.theta_t:
                    self.lmbda1=1

                    temp =((left_most_visible_from_dog(self.sheep_list,
                                                                            visibility, dog_pos))-self.dog_pos)


                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                        R = np.array(((c, -s), (s, c)))
                        # mnozenje matrike z vektorjem
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

                elif self.lmbda1 == 1:
                    temp = (( left_most_visible_from_dog(self.sheep_list,   visibility, dog_pos))-dog_pos)

                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                        R = np.array(((c, -s), (s, c)))
                        # mnozenje matrike z vektorjem
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))
                else:
                    temp = ((right_most_visible_from_dog(self.sheep_list,
                                                                                visibility, dog_pos))-dog_pos)
                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                        R = np.array(((c, -s), (s, c)))
                        # mnozenje matrike z vektorjem
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))
            # left dog
            else:
                center_of_mass_goal_vector = center_of_visible_sheep - self.goal
                dist = np.linalg.norm(np.cross(center_of_visible_sheep-self.goal, self.goal-dog_pos)/np.linalg.norm(center_of_visible_sheep-self.goal))

                if all_on_right(dog_pos, self.goal, self.sheep_list) and calculateLC(self.sheep_list, self.goal,
                                                                                     self.dog_radius, 'left',
                                                                                     dog_pos) > self.theta_t:
                    self.lmbda = 0

                    temp = ((right_most_visible_from_dog(self.sheep_list, visibility, dog_pos)) - dog_pos)
                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                        R = np.array(((c, -s), (s, c)))
                        # mnozenje matrike z vektorjem
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

                if dist < 5:
                    self.lmbda = 1

                    temp = ((left_most_visible_from_dog(self.sheep_list,
                                                        visibility, dog_pos)) - dog_pos)

                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                        R = np.array(((c, -s), (s, c)))
                        # mnozenje matrike z vektorjem
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

                elif self.lmbda == 1:
                    temp = ((left_most_visible_from_dog(self.sheep_list, visibility, dog_pos)) - dog_pos)

                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                        R = np.array(((c, -s), (s, c)))
                        # mnozenje matrike z vektorjem
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))
                else:
                    temp = ((right_most_visible_from_dog(self.sheep_list,
                                                         visibility, dog_pos)) - dog_pos)
                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                        R = np.array(((c, -s), (s, c)))
                        # mnozenje matrike z vektorjem
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))


        else:
            dog_velocity=np.array([0,0])
            # self.success = True
            self.max_steps = self.current_step + 3




        for obstacle in self.obstacles:
            obstacle_sheep_dist = vector_size(obstacle - dog_pos)
            if obstacle_sheep_dist < self.obstacle_effect_radius and \
                    is_line_intersecting_circle(self.dog_velocity, dog_pos - obstacle, self.obstacle_radius):

                angle = calculate_angle_between_vectors(obstacle - dog_pos, self.dog_velocity)
                if math.fabs(angle) > math.pi / 3:
                    continue
                angle_corr = (obstacle_sheep_dist / self.obstacle_effect_radius) * 3 + 1
                if angle < 0:
                    # go to right
                    c, s = np.cos(self.theta_r / angle_corr), np.sin(self.theta_r / angle_corr)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    dog_velocity = np.array(R).dot(self.dog_velocity)
                else:
                    # go to left
                    c, s = np.cos(self.theta_l / angle_corr), np.sin(self.theta_l / angle_corr)
                    R = np.array(((c, -s), (s, c)))
                    # mnozenje matrike z vektorjem
                    dog_velocity = np.array(R).dot(self.dog_velocity)
        return dog_velocity










