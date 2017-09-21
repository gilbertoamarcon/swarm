# swarm

Fast Swarm simulator. C++ and OpenGL for speed. Simulation is simple for learning purposes. Code has been kept simple and minimal. There is a simple (Bounding box based) collision detector. No fancy external libraries other than OpenGL. Build is done with the one-liner build script. 

All global parameters are defined in the includes.hpp file. 

There are two visual objects: Wired (Wired.hpp, Wired.cpp) and Textured (Textured.hpp, Textured.cpp). 
  - Wired is a polygon. Initialize it as a list of 2D points.
  - Textured is a bitmap image. Initialize it with the path to the PNG image file - Alpha channels are allowed.

World coordinates are separate from screen coordinates. 

Simulation time is separate from visualization time. SIM_STEP_TIME defines (in ms) the similation step time. Visualization runs at its own rate, independent of simulation time. 

User interface is similar to the usual real-time stratage game: 
  - Move the mouse to the screen edges to move the camera.
  - Hold the middle mouse button to move the camera faster.
  - Use ASDW to move the camera.
  - Use Q and E to zoom in and out.
  - Left-Click to select robot.
  - Left-Click and drag to select multiple robots.
  - Right-Click to define robot goal position. 
  - Use V to lock camera to selected Robot.
  - Use Z to reset zoom.
  - Use H to reset camera position.
  - Use ESC to quit and exit. 
