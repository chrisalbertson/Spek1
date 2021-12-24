# Quad Controller
This is what runs on the computer that is inside the robot.  

The target environment is a Raspberry Pi4 rning Ubuntu Linux.  But this
software is developed on a desktop Linux PC and runs there too in "simulation
mode" where the data that would have been sent to robot hardware is written 
to a log file.

At present this software does not do much.  I am using just one leg mounted
to a workbench to suport development.     It is still very early in the 
development process.

The goal is that the robot will move under remote control but make many of 
the decisions about hove to walk and balance by itself.   The remote
control will be either web based or via a remote X11 screen of by a
gamepad controller.

To run this type "python3 main.py"  then look at
http://localhost:2222 for a simply window.  Click on some controls and see
if anything happens.

