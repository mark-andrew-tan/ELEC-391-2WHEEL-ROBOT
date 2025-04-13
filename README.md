# ELEC 391 2WHEEL SELF-BALANCING ROBOT

## Goal

Design and fabricate a self-balancing robot on two wheels that is able to maintain balance in a vertical position while being driven by an external operator on a course track that may present sloped portions.

## Main Project Requirements
The robot must be able to balance itself on a flat surface in the vertical position by actuating the two DC motors.

The robot must be able to maintain balance while driven by an external operator on a flat surface at a reasonable speed.

The robot must be able to maintain balance while taking turns on a flat surface at a reasonable speed.
The robot must be able to move forward and backward under external control.

The robot must be controlled externally through wireless (Bluetooth) signaling. Four controls (forward, backward, turn left, turn right) must be implemented.

On a level surface, show the maximum initial pendulum deviation angle from which your robot can autonomously recover.


## Extra Feature Requirements:
Detect when approaching a wall and react accordingly to avoid hitting the wall. 
Dis-engange balancing mode into a ‘parked’ mode without touching the robot and remain in a low-power state. 

Resume balancing mode from the parked state without touching the robot.

Engage parking mode automatically when entering a parking station. Must only enter ‘parked’ mode automatically when in the desired parking location.

Grab a spool of electrical wire using the claw, and drop the spool of wire accurately into a desired location.

Play audio when a nearby wall is detected.

Play the next and previous song via the bluetooth app.

## Code
At a high level, our code is sectioned into 4 main categories:

1st section: global variables. 

2nd section: Initializations. Done in the setup() function.

3rd section: Functions called by the main loop().

4th section: Main loop(), primarily calling functions in the 3rd section. 


