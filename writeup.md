# Particle Filter Project Writeup
Self-Driving Car Engineer Nanodegree Program

Things to note:
* works nice and easy under linux ubuntu vm following the instructions
* mostly used code from PF implementation lessons
* doesnt really solve the kidnapped problem... would need to implement e.g. particle redistribution/initialisation for that
* angles not normalised for this one, simulator complains
* init/predict/resampling easy, main challenge was getting weighting working correctly. Main problem was due to numerical/array indexing errors...
* make sure to check for numerical stability, i.e. no divide by zeros.

Here is an example of running the simulator with this code:

[//]: # (Image References)
[image1]: ./Screenshot.png
[image2]: ./Screenshot2.png

![alt text][image1]

![alt text][image2]
