## This repository contains all the code required for the FART 2025 robot.

There are three main files that make up the system:

  hamishCode2.py — the core control program that manages pathfinding, movement, and overall logic.
  AI_Camera_handler.py and colorsensor.py — handle communication with the camera and color sensor, respectively. These are kept separate so the main program can continue running smoothly without pausing for serial communication.

All of the other files are old testing files and are not used in the final design of the robot.
