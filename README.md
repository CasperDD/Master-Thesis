# Master-Thesis
Visual navigation for mobile robots using ant-inspired learning walk

## Learning walk
Learning walk is a behaviour found in ants during their foraging trips. During their foraging trips ants will perform what is called a pirouette.
A pirouette is when an ant stops, then rotates around itself to scan the environment around it. The information gathered from the ant is used for localization.

## General information
This project is split up into both C++ and Python.

The Python part of the code did not get fully developed, since it was found that Python was not fast enough at reading the encoder values on the robot

The C++ part of the code have a seperate README file that explains more about the code and how to run it. 

This project is performed on a mobile robot using a Raspberry Pi.

To run the code on a similar platform a few things are needed:
* Screen, keyboard and mice to initialy get the robot started
* Being able to connect up to the robot via VNC or similar program (Not a strict requirement)
* Having a c++ compiler (used gcc 8.3.0). 
* Having opencv installed

Optional if visual studio code is used:
* Have cmake tools and cmake installed
