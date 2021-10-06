# Master thesis at KTH 2021 in Computer science with specialisation in Computer graphics and Visualisation By Alvin Häger

This program creates a LEGO water animation that is coloured using a 'whitewater' effect, which occurs when air bubbles are trapped in water and create foam. This can occur when turbulent water such in a river suddenly change movement of direction with high velocities. 

More specifically, a dynamic fluid simulation method PIC/FLIP is used to simulate water particle movements on a 3D grid. This simulation then outputs locations of LEGO bricks and their colours, which are used in a python script in Blender to render the animation frame by frame. The fluid simulation is coded in C++ and the script used to render the animation is coded in python. A section of code in the C++ fluid simulation is run on the GPU using CUDA, which is required to run the project. Currently, the project code is still quite messy (although functional) and will be updated. 

The thesis work creatse own code and also adapts some code from other github repositories, which can be read more about in the acknowledgements section of the attached thesis report, which is included in the repository. 

[LEGO water animation](https://user-images.githubusercontent.com/25433576/136219080-f380b04d-23bb-4426-828a-d2d11b02a584.mp4)



