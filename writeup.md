# Extended Kalman Filter Project Writeup
Self-Driving Car Engineer Nanodegree Program

Things to note:
* works nice and easy under linux ubuntu vm following the instructions
* mostly used code from lesson 5
* use y = z - h(x') not y = z - Hj x for non-linear innovation.
* check that angles are in domain -pi < x < pi, especially the angle in the innovation
* make sure to update the previous timestamp when initialising
* we have a good idea of initial position and speed, so P matrix can be initialised with smaller values
* make sure to check for numerical stability, i.e. no divide by zeros - Hj, h(x).

Here is an example of running the simulator with this code:

[//]: # (Image References)
[image1]: ./Simulator.png

![alt text][image1]
