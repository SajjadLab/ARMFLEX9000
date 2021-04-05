# Quality Control System

## ARMFLEX9000
## April 4, 2021
## Sajjad Al-Kazzaz, ECE, University of British Columbia
## Vancouver, BC, Canada

### Abstract

ARMFLEX9000 is a 3.5 DOF arm that was developed for the purposes of quality control of marshmallows on a conveyor-belt. This paper will discuss the aspects of the design of the robot, as well as control of the robot. 

In this paper, Section 1 describes the robotics, including kinematics and path planning. Section 2 discusses robot modelling in Matlab and SimulationX. Section 3 describes the Matlab controller, this includes system control and PID implementation. Section 4 goes over PID tuning. Section 5 describes porting the controller to C.

### Nomenclature

- DOF: Degrees of freedom
- PID: Proportional, integral, and derivative

### 1. Robotics


