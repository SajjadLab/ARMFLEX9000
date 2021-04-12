# Quality Control System

## ARMFLEX9000
## April 4, 2021
## Sajjad Al-Kazzaz, ECE, University of British Columbia
## Vancouver, BC, Canada

\newpage

### Table of Contents

- [Abstract](#Abstract)
- [Nomenclature](#Nomenclature)
- [Robotics](#1-robotics)
	- Figure 1.1
- [Modelling](#2-modelling)
- [Controller](#3-controller)
- [Tuning](#4-tuning)
- [Port to C](#5-port-to-c)
- [Performance](#6-performance)

### Abstract

ARMFLEX9000 is a 3.5 DOF arm that was developed for the purposes of quality control of marshmallows on a conveyor-belt. This paper will discuss the aspects of the design of the robot, as well as control of the robot. 

In this paper, Section 1 describes the robotics, including kinematics and path planning. Section 2 discusses robot modelling in Matlab and SimulationX. Section 3 describes the Matlab controller, this includes system control and PID implementation. Section 4 goes over PID tuning. Section 5 describes porting the controller to C. Finally, Section 6 will discuss system performance.

### Nomenclature

- DOF: Degrees of freedom
- PID: Proportional, integral, and derivative


### 1. Robotics

The first thing that had to be done for the robotics of the machine was object placement and path planning. This is a crucial step that determines the length of the arm and thereby the inertia and motors used. The requirements dictated that there should be 3 cylindrical marshmallows on the stopped conveyor which the arm must be able to pick-up and discard. The chosen final positioning was determined with the help of "Mechanical" and optimized for the best results; this positioning is shown in figure 1.1. Given this positioning the optimal arm joint lengths were found to be:

$$r_a = 0.13m$$
$$r_b = 0.09m$$

*Figure 1.1*

The dots on figure 1.1 represent the path stage of the arm, to be specific the x and y coordinates of motor C. The arm starts at the calibration stage, 0. The arm then moves to the first alignment dot, also known as the garbage dot, 1. This is so that the arm doesn't swing and knock over marshmallows in its initial movement. The arm then follows a set of alignment dots as it approaches a specific marshmallow so that a general path can be followed that won't knock over or tip other marshmallows. Each marshmallow has its own set of alignment dots. Then the arm will go to the respective pick-up dot to grab the marshmallow, followed by the retraction dot and then a return to the garbage dot. From the garbage dot the controller will determine whether another marshmallow needs to be grabbed, and follow the outlined stated above, or return home and wait for further commands.

In order to translate the motor C stage coordinates into angles that can be sent to the appropriate motors, a model for inverse kinematics had to be developed. Using trigonometry the position of motor C was translated from the cartesian coordinates of the dots into polar coordinates $\alpha$ and $\beta$ for motors A and B respectively. This was done using the following formulas:

$$\beta = \arccos{\frac{x^2 + y^2 + r_a^2 + r_b^2}{2r_a r_b}}$$
$$\alpha = \arctan{\frac{y}{x}} - \arctan{\frac{r_b \sin{\beta}}{r_a + r_b \cos{\beta}}}$$

To find the necessary angle, $\gamma$, for the swivel motor C so that it remained perpendicular to the conveyor, trigonometry was also used and the following formula was calculated:
$$\gamma = \alpha + \beta - \pi/2$$

Finally, the angle $\delta$ for motor D, which controls the pincher, was found from the design of the pincher to be 0 in the release position, and $\pi/2$ in the grab position. The necessary angle $\delta$ at any time is pre-determined by the stage or dot it is in, and is part of that stages coordinates.

No forward kinematics were used in the controller since the angles were already being calculated and could be used for error calculation. Turning measured angles into measured position was unnecessary and, therefore, removed to save processing time.

### 2. Modelling

Modelling the full system was done using Simulink and SimulationX for the purposes of testing. In Simulink, 4 motor models were developed using the standard motor model and feedback systems as reference. Figure 1.2 shows the 4 motor models, each with its own feedback path. The values for the motors electrical and mechanical components were obtained from "Electrical" and "Mechanical" respectively. Using this model as a blueprint, the complete model made for tuning and demoing is shown in Figure 1.3. This model replaces the mechanical components with a communication block connected to SimulationX. SimulationX does the job of more accurately calculating mechanical components and their behaviour during simulation. Figure 1.3 also shows 4 instances of the PID Matlab functions in order to meet requirements, and a central controller that manages overall movement of the system. In SimulationX "Mechanical" developed a simplified model of the arm using the correct weights and lengths for all the components.

*Figure 1.2*

*Figure 1.3*

### 3. Controller

There are two parts to the system control. The first is the user-defined Matlab function to replace a PID block. Figure 1.4 shows the complete code for the function. Persistent arrays and variables were used to maintain a history of previous error values for the filtered derivative and to keep track of the error sum for the integral output.

*Figure 1.4*

The second part of the system control is the system controller. This is a Matlab function that has the coordinates for all the stages relevant to each of the motors and a way of keeping track of which stage the system is in at any point in time. This controller also allows the user to choose which marshmallows to pick-up during the sequence. The complete function is shown in Figure 1.5. Each clock cycle the controller calculates the error between the desired, calculated angles and the true angles being read from SimulationX. Once the error for a particular stage drops below a threshold value, the controller considers that stage complete and increments to the next stage; if the completed stage was the last stage in a marshmallows set then the next marshmallow in the list is targeted. Once the last marshmallow in the list has finished the sequence, the controller sends the arm back to the home calibration position. 

*Figure 1.5*

### 4. Tuning

To tune the system root locus and margin analysis tools were used to give a starting location. The open-loop transfer functions for each of the motors was obtained using the values and models developed in Section 2, Figure 1.2. As an estimation, the value for the controller frequency was chosen to be a very conservative 1000Hz. Thus, nhat, the number of samples used to calculate the filtered derivative, was chosen to be 10; this results in a filter pole of 400 for the purposes of testing. The root locus and margin graphs for the each of the 4 motors are shown in figures 1.6, 1.7, 1.8, and 1.9 respectively. Using this information a suitable starting location for the PID zeros was chosen for each of the motors and tested using the model from Section 2, Figure 1.3. Through trial and error each of the motors were tuned to achieve the desired results. The final PID values for all the motors are summarized in Table 1.1. 

*Figure 1.6*

*Figure 1.7*

*Figure 1.8*

*Figure 1.9*

*Table 1.1*

### 5. Port to C

Due to the similarities between Matlab and C porting the functions developed in Section 3 was fairly easy. Most of the code was copied straight from the Matlab functions and adjusted to meet the syntax requirements of C. Certain constants, like the arm lengths and their squares, are pre-calculated to reduce processing time. To further save processing time the array of cartesian coordinates for the stages of all the marshmallows are pre-translated to the polar coordinates for $\alpha$ and $\beta$ during setup so that these lengthy calculations aren't repeated during the actual interrupt routine. The complete arduino C code is shown in Figure 5.1.

*Figure 5.1*

### 6. Performance
