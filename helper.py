import math
import numpy as np

def is_in_goal_area(pos1, goal, goal_radius):
    distance = math.sqrt(math.pow(pos1[0]- goal[0],2) + math.pow(pos1[1]- goal[1],2))
    return distance < goal_radius

def is_in_visible_area(dog, sheep, radius):
    distance = math.sqrt(math.pow(dog[0] - sheep[0], 2) + math.pow(dog[1]- sheep[1], 2))
    return distance < radius

def calculate_angle_between_vectors(v1, v2):
    return math.atan2(v1[0], v1[1]) - math.atan2(v2[0], v2[1])

def all_on_left(dog_pos, goal_pos, sheeps):
    dog_goal_vector = unit_vector(goal_pos-dog_pos)
    for sheep_pos in sheeps:
        dog_sheep_vector = unit_vector(sheep_pos-dog_pos)
        # TODO PREVERI, ČE JE USMERITEV PRAVA
        # angle = math.atan2(dog_sheep_vector[0], dog_sheep_vector[1]) - math.atan2(dog_goal_vector[0], dog_goal_vector[1])
        d = (goal_pos[0] - dog_pos[0]) * (sheep_pos[1] - dog_pos[1]) - (goal_pos[1] - dog_pos[1]) * (
                sheep_pos[0] - dog_pos[0])
        if  d < 0:
            return False
    return True

def all_on_right(dog_pos, goal_pos, sheeps):
    dog_goal_vector = unit_vector(goal_pos-dog_pos)
    for sheep_pos in sheeps:
        dog_sheep_vector = unit_vector(sheep_pos-dog_pos)
        # TODO PREVERI, ČE JE USMERITEV PRAVA
        d =(goal_pos[0] - dog_pos[0]) * (sheep_pos[1] - dog_pos[1]) - (goal_pos[1] - dog_pos[1]) * (
                    sheep_pos[0] - dog_pos[0])
        # angle = math.atan2(dog_sheep_vector[1], dog_sheep_vector[0]) - math.atan2(dog_goal_vector[0], dog_goal_vector[1])
        if d > 0:
            return False
    return True




def covered(sheep_pos, sheeps, dog_pos):
    dog_sheep_vector = unit_vector(sheep_pos - dog_pos)
    sheep_dist = euclidean(sheep_pos, dog_pos)
    for sheep_1 in sheeps:
        dog_sheep1_vector = unit_vector(sheep_1-dog_pos)
        angle = math.atan2(dog_sheep_vector[0], dog_sheep_vector[1]) - math.atan2(dog_sheep1_vector[0], dog_sheep1_vector[1])
        if angle == 0 and sheep_dist > euclidean(sheep_1, dog_pos):
            return True
    return False


def left_most_visible_from_sheepfold(sheeps, visibility, goal):
    sheeps = np.array(sheeps)
    visible_sheeps = sheeps[np.array(visibility).astype(np.bool)]
    initial_point = np.array([0.0,0.0])
    left_most = visible_sheeps[0]
    left_most_vector = unit_vector(left_most-goal)
    initial_vector = unit_vector(initial_point-goal)
    left_most_angle = math.atan2(left_most_vector[0], left_most_vector[1]) - math.atan2(initial_vector[0], initial_vector[1])
    left_most_dist = euclidean(left_most, goal)

    for sheep in visible_sheeps:
        sheep_goal_vector = unit_vector(sheep-goal)
        angle = math.atan2(sheep_goal_vector[0], sheep_goal_vector[1]) - math.atan2(initial_vector[0], initial_vector[1])
        if angle > left_most_angle:
            left_most = sheep
            left_most_angle = angle
            left_most_dist = euclidean(left_most, goal)
        elif angle == left_most_angle:
            if euclidean(sheep, goal) > left_most_dist:
                left_most = sheep
                left_most_angle = angle
                left_most_dist = euclidean(left_most, goal)

    return left_most

def right_most_visible_from_sheepfold(sheeps, visibility, goal):
    sheeps = np.array(sheeps)
    visible_sheeps = sheeps[np.array(visibility).astype(np.bool)]
    initial_point = np.array([0.0,0.0])
    right_most = visible_sheeps[0]
    right_most_vector = unit_vector(right_most-goal)
    initial_vector = unit_vector(initial_point-goal)
    right_most_angle = math.atan2(right_most_vector[0], right_most_vector[1]) - math.atan2(initial_vector[0], initial_vector[1])
    right_most_dist = euclidean(right_most, goal)


    for sheep in visible_sheeps:
        sheep_goal_vector = unit_vector(sheep-goal)
        angle = math.atan2(sheep_goal_vector[0], sheep_goal_vector[1]) - math.atan2(initial_vector[0], initial_vector[1])
        if angle < right_most_angle:
            right_most = sheep
            right_most_angle = angle
            right_most_dist = euclidean(right_most, goal)
        elif angle == right_most_angle:
            if euclidean(sheep, goal) > right_most_dist:
                left_most = sheep
                right_most_angle = angle
                right_most_dist = euclidean(left_most, goal)
    return right_most

def left_most_visible_from_dog(sheeps, visibility,dog_pos):
    sheeps = np.array(sheeps)
    visible_sheeps = sheeps[np.array(visibility).astype(np.bool)]
    initial_point = np.array([0.0,0.0])
    right_most = visible_sheeps[0]
    right_most_vector = unit_vector(right_most-dog_pos)
    initial_vector = unit_vector(initial_point-dog_pos)
    right_most_angle = math.atan2(right_most_vector[0], right_most_vector[1]) - math.atan2(initial_vector[0], initial_vector[1])

    for sheep in visible_sheeps:
        sheep_goal_vector = unit_vector(sheep-dog_pos)
        angle = math.atan2(sheep_goal_vector[0], sheep_goal_vector[1]) - math.atan2(initial_vector[0], initial_vector[1])
        if angle > right_most_angle:
            right_most = sheep
            right_most_angle = angle

    return right_most

def right_most_visible_from_dog(sheeps, visibility,dog_pos):
    sheeps = np.array(sheeps)
    visible_sheeps = sheeps[np.array(visibility).astype(np.bool)]
    initial_point = np.array([0.0,0.0])
    right_most = visible_sheeps[0]
    right_most_vector = unit_vector(right_most-dog_pos)
    initial_vector = unit_vector(initial_point-dog_pos)
    right_most_angle = math.atan2(right_most_vector[0], right_most_vector[1]) - math.atan2(initial_vector[0], initial_vector[1])

    for sheep in visible_sheeps:
        sheep_goal_vector = unit_vector(sheep-dog_pos)
        angle = math.atan2(sheep_goal_vector[0], sheep_goal_vector[1]) - math.atan2(initial_vector[0], initial_vector[1])
        if angle < right_most_angle:
            right_most = sheep
            right_most_angle = angle

    return right_most


def calculate_center_of_visible_sheep(sheeps, dog_visibility):
    sheeps = np.array(sheeps)
    visible_sheep = sheeps[np.array(dog_visibility).astype(np.bool)]
    xx = np.sum(np.array(visible_sheep), axis=0)
    N_visible, _ = np.shape(np.array(visible_sheep))
    return xx/N_visible

def calculateLC(sheeps, goal, dog_radius, left_right_sheep, dog_pos):
    if left_right_sheep == 'left':
        dog_visibility = visible_sheep(dog_pos,dog_radius,goal,sheeps)
        left_most_sheep_pos= left_most_visible_from_sheepfold(sheeps,dog_visibility,goal)
        left_most_sheep_dog_vector = (dog_pos-left_most_sheep_pos)
        # p_c
        center_of_visible_sheep = calculate_center_of_visible_sheep(sheeps, dog_visibility)
        D_cd = unit_vector(goal-center_of_visible_sheep)
        return math.acos(np.dot(D_cd, left_most_sheep_dog_vector)/(vector_size(D_cd)*vector_size(left_most_sheep_dog_vector)))

    else:
        dog_visibility = visible_sheep(dog_pos, dog_radius, goal, sheeps)
        right_most_sheep_pos = right_most_visible_from_sheepfold(sheeps, dog_visibility, goal)
        right_most_sheep_dog_vector = (dog_pos - right_most_sheep_pos)
        # p_c
        center_of_visible_sheep = calculate_center_of_visible_sheep(sheeps, dog_visibility)
        D_cd = unit_vector(goal - center_of_visible_sheep)
        return math.acos(np.dot(D_cd, right_most_sheep_dog_vector) / (vector_size(D_cd) * vector_size(right_most_sheep_dog_vector)))

""" returns list which for each sheep specifies if visible (1) or not (0)"""

def visible_sheep(dog_pos, dog_radius, goal, sheeps):
    visibility = []
    for sheep_pos in sheeps:
        if not is_in_visible_area(dog_pos, sheep_pos, dog_radius):
            visibility.append(0)
            continue
        if covered(sheep_pos, sheeps, dog_pos):
            visibility.append(0)
            continue
        visibility.append(1)

    return visibility

def distance_between_line_and_dot(line_vector, dot):
  # Use the dot product to find the orthogonal projection of the dot onto the line
  projection = np.dot(line_vector, dot) / np.dot(line_vector, line_vector) * line_vector

  # Calculate the distance between the orthogonal projection and the dot
  distance = np.linalg.norm(projection - dot)

  return distance

def is_line_intersecting_circle(velocity_vector, obstacle_sheep_vector, r):
    a = np.dot(velocity_vector, velocity_vector)
    b = 2 * np.dot(obstacle_sheep_vector, velocity_vector)
    c = np.dot(obstacle_sheep_vector, obstacle_sheep_vector) - r * r

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        # no intersection
        return False
    else:
        return True

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def vector_size(a):
    return math.sqrt(a[0]*a[0]+a[1]*a[1])
#
def euclidean(pos1, pos2):
    return math.sqrt(math.pow(pos1[0] - pos2[0], 2) + math.pow(pos1[1]- pos2[1], 2))


