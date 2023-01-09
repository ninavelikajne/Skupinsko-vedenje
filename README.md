# Sheepdog Driven Algorithm for Sheep Herd Transport

This repository contains code that is based on related article of the Sheepdog Driven Algorithm 
for Sheep Herd Transport algorithm from paper [Sheepdog Driven Algorithm 
for Sheep Herd Transport](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9549396).

## Project description
The main purpose of this study is to investigate swarming behavior, i.e., coordinated behavior exhibited by a group of individuals. Specifically, we examine the motion of individual sheep agents, which are part of a herd, in the presence of a threat, namely a dog. Based on the method described in a aforementioned article, we set two goals. The first one is adding obstacles in the field between the starting position of the herd and the goal. Thus, we needed to add an obstacle avoidance mechanism to the motion of the sheep in order to make the sheepdog-driven transport of the herd more realistic. Second goal we successfully achieved is to design an algorithm for collaborative herding using two dogs facilitating herd transport of larger groups of sheep.

# Project Structure
* *automate.py* is a script that runs the whole procedure. First it clears fig directory. The runs the simulation and saves plots into figs directory. At the end it generates an video from all the saved plots in figs folder. First parameter is the number of shepherds, and second one is the number of sheep.

* *main_one_dog.py* runs the simulation for one shepherd. Number of sheep can be set. If the number is set to 24, algorithm uses the same sheep positions as in the original paper. Max time steps can also be set.

* *main_two_dogs.py* runs the simulation for two shepherds. Number of sheep can be set. If the number is set to 24, algorithm uses the same sheep positions as in the original paper. Max time steps can also be set.
* *helper.py* contains linear algebra functions that are needed for calculating the movement of the dog and sheep.
* *make_video.py* is a script that generates an video of .avi format from all the figures in the figs directory.
* *one_shepherd.py* is a class that defines the simulation of the sheep herding algorithm with one dog.
* *two_shepherds.py* is a class that defines the simulation of the sheep herding algorithm with two dog.

Inside *one_shepherd.py*  and *two_shepherds.py* all the parameters can be set. Files also contain three predefined poligons with obtacles. This poligons can be uncommented in the code. If we dont want obtacles we can just define the *self.obstacles* array as an empty array - [].

