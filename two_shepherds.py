from matplotlib import pyplot as plt
import numpy as np
import random
from helper import *


class Shepherd:
    def __init__(self, sheepheard_size=24, max_steps=5000000):
        self.N = sheepheard_size
        self.success = False
        self.max_steps = max_steps
        self.current_step = 0
        self.vizualize = True
        self.lmbda1 = 1
        self.lmbda = 0
        self.dog_velocity = np.array([0, 0])
        # Goal position
        self.goal = np.array([115, 300])
        # phi_o
        self.goal_radius = 45

        # dog starting position
        self.dog_pos1 = np.array([63, 10])
        self.dog_pos = np.array([90, 10])
        # phi_n
        self.dog_radius = 100

        # sampling period
        self.Ts = 0.001

        # sheep
        self.sheep_list = []
        xsize = 100
        ysize = 50
        xstart = 45
        ystart = 60

        if self.N != 24:
            for i in range(self.N):
                self.sheep_list.append(np.array([random.random() * xsize + xstart, random.random() * ysize + ystart]))
        else:
            self.sheep_list = [np.array([57, 50]), np.array([44, 71]), np.array([59, 64]), np.array([73, 71]),
                               np.array([78, 60]), np.array([82, 71]), np.array([87, 58]), np.array([96, 71]),
                               np.array([55, 76]), np.array([64, 83]), np.array([69, 79]), np.array([50, 60]),
                               np.array([87, 76]), np.array([95, 84]), np.array([100, 76]), np.array([105, 79]),
                               np.array([65, 94]), np.array([69, 90]), np.array([105, 85]), np.array([79, 95]),
                               np.array([84, 90]), np.array([90, 99]), np.array([100, 55]), np.array([105, 60])]

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
        self.obstacles = np.linspace(np.array([100, 200]), np.array([120, 190]), 30)

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
            Runs the simulation of implemented shepherd driven sheep transport
        """
        i = 0
        while (not self.success and self.current_step < self.max_steps):

            if i % 10 == 0:

                fig = plt.figure(figsize=(7, 7))
                ax = fig.subplots()
                s = np.array(self.sheep_list)
                ax.scatter(s[:, 0], s[:, 1], c='blue')
                ax.scatter(self.dog_pos[0], self.dog_pos[1], c='red')
                ax.scatter(self.dog_pos1[0], self.dog_pos1[1], c='orange')
                circle = plt.Circle((self.goal[0], self.goal[1]), self.goal_radius, alpha=0.3, color='green')
                ax.add_artist(circle)
                for o in self.obstacles:
                    plt.scatter(o[0], o[1], c='black')

                ax.set_xlim([0, 350])
                ax.set_ylim([0, 350])
                ax.set_title("Number of sheep: " + str(self.N) + " Timestep: " + str(self.current_step))
                fig.savefig("./figs/" + str(str(i).zfill(8)))
                plt.close()

            i += 1

            self.current_step += 1

            self.dog_velocity = self.calculate_dog_velocity(self.dog_pos, self.dog_pos1)
            self.dog_velocity1 = self.calculate_dog_velocity(self.dog_pos1, self.dog_pos)

            self.dog_pos = self.dog_pos + self.Ts * self.dog_velocity
            self.dog_pos1 = self.dog_pos1 + self.Ts * self.dog_velocity1
            for dog in [self.dog_pos1, self.dog_pos]:
                for ii, sheep in enumerate(self.sheep_list):

                    x = vector_size(sheep - dog)
                    if x > 0 and x <= self.dog_radius:
                        fi = self.alpha * (1 / x - 1 / self.dog_radius)
                    else:
                        fi = 0

                    v_di = fi * unit_vector(sheep - dog)

                    v_si = 0
                    for sheep_ in self.sheep_list:
                        #
                        if (sheep == sheep_).all():
                            continue
                        temp_size = vector_size(sheep_ - sheep)
                        if temp_size < self.fi_r:  #:
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
                        if obstacle_sheep_dist < self.obstacle_effect_radius and \
                                is_line_intersecting_circle(velocity_sheep, sheep - obstacle, self.obstacle_radius):

                            angle = calculate_angle_between_vectors(obstacle - sheep, velocity_sheep)
                            if math.fabs(angle) > math.pi / 3:
                                continue
                            angle_corr = (obstacle_sheep_dist / self.obstacle_effect_radius) * 3 + 0.7
                            if angle < 0:
                                # go to right
                                c, s = np.cos(self.theta_r / angle_corr), np.sin(self.theta_r / angle_corr)
                                R = np.array(((c, -s), (s, c)))

                                velocity_sheep = np.array(R).dot(velocity_sheep)
                            else:
                                # go to left
                                c, s = np.cos(self.theta_l / angle_corr), np.sin(self.theta_l / angle_corr)
                                R = np.array(((c, -s), (s, c)))

                                velocity_sheep = np.array(R).dot(velocity_sheep)

                    self.sheep_list[ii] = self.sheep_list[ii] + (self.Ts / 2 * velocity_sheep)

    def calculate_dog_velocity(self, dog_pos, other_dog):
        sheep_at_goal = 0
        for sheep_location in self.sheep_list:
            if is_in_goal_area(sheep_location, self.goal, self.goal_radius):
                sheep_at_goal += 1

        if sheep_at_goal < self.N:
            visibility = visible_sheep(dog_pos, self.dog_radius, self.sheep_list)
            center_of_visible_sheep = calculate_center_of_visible_sheep(self.sheep_list, visibility)
            angle = calculate_angle_between_vectors(center_of_visible_sheep - dog_pos, other_dog - dog_pos)

            # right dog
            if angle > 0:
                dist = np.linalg.norm(
                    np.cross(center_of_visible_sheep - self.goal, self.goal - dog_pos) / np.linalg.norm(
                        center_of_visible_sheep - self.goal))
                if dist < 5:
                    self.lmbda1 = 0

                    temp = ((right_most_visible_from_dog(self.sheep_list, visibility, dog_pos)) - dog_pos)
                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                        R = np.array(((c, -s), (s, c)))
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

                elif all_on_left(dog_pos, self.goal, self.sheep_list) and calculateLC(self.sheep_list, self.goal,
                                                                                      self.dog_radius, 'right',
                                                                                      dog_pos) > self.theta_t:
                    self.lmbda1 = 1

                    temp = ((left_most_visible_from_dog(self.sheep_list,
                                                        visibility, dog_pos)) - dog_pos)

                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                        R = np.array(((c, -s), (s, c)))
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

                elif self.lmbda1 == 1:
                    temp = ((left_most_visible_from_dog(self.sheep_list, visibility, dog_pos)) - dog_pos)

                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                        R = np.array(((c, -s), (s, c)))
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))
                else:
                    temp = ((right_most_visible_from_dog(self.sheep_list,
                                                         visibility, dog_pos)) - dog_pos)
                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                        R = np.array(((c, -s), (s, c)))
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))
            # left dog
            else:
                dist = np.linalg.norm(
                    np.cross(center_of_visible_sheep - self.goal, self.goal - dog_pos) / np.linalg.norm(
                        center_of_visible_sheep - self.goal))

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
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

                elif self.lmbda == 1:
                    temp = ((left_most_visible_from_dog(self.sheep_list, visibility, dog_pos)) - dog_pos)

                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_l), np.sin(self.theta_l)
                        R = np.array(((c, -s), (s, c)))
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))
                else:
                    temp = ((right_most_visible_from_dog(self.sheep_list,
                                                         visibility, dog_pos)) - dog_pos)
                    if vector_size(temp) >= self.r_a:
                        dog_velocity = self.gamma_a * unit_vector(temp)
                    else:
                        c, s = np.cos(self.theta_r), np.sin(self.theta_r)
                        R = np.array(((c, -s), (s, c)))
                        dog_velocity = self.gamma_b * np.array(R).dot(unit_vector(temp))

        else:
            dog_velocity = np.array([0, 0])
            self.max_steps = self.current_step + 3

        for obstacle in self.obstacles:
            obstacle_sheep_dist = vector_size(obstacle - dog_pos)
            if obstacle_sheep_dist < self.obstacle_effect_radius and \
                    is_line_intersecting_circle(dog_velocity, dog_pos - obstacle, self.obstacle_radius):

                angle = calculate_angle_between_vectors(obstacle - dog_pos, dog_velocity)
                if math.fabs(angle) > math.pi / 3:
                    continue
                angle_corr = (obstacle_sheep_dist / self.obstacle_effect_radius) * 3 + 1
                if angle < 0:
                    # go to right
                    c, s = np.cos(self.theta_r / angle_corr), np.sin(self.theta_r / angle_corr)
                    R = np.array(((c, -s), (s, c)))
                    dog_velocity = np.array(R).dot(dog_velocity)
                else:
                    # go to left
                    c, s = np.cos(self.theta_l / angle_corr), np.sin(self.theta_l / angle_corr)
                    R = np.array(((c, -s), (s, c)))
                    dog_velocity = np.array(R).dot(dog_velocity)
        return dog_velocity
