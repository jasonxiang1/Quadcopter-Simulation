# Task
Complete development of MATLAB simulation of an aerial robot flying along a pre-defined path. Ensure the following are coded and implemented:
1. Develop a basic state machine to facilitate simulation that enables the robot to takeoff, hover, track a 
trajectory, and land
2. Introduce time-parameterized trajectories given fixed initial and final endpoint constraints and bounded 
velocity and acceleration
3. Extend the formulation to include piecewise continuous trajectories with fixed initial and final endpoint 
constraints
4. Evaluate tracking performance given different levels of flight performance and trajectory design

# Instructions to Run
1. Download source into local machine through git clone
2. Open main.m file in code/ directory in MATLAB
3. To run the simulation, the main file takes in one integer argument. The integer value of the argument corresponds to the question number in the project prompt that the program demonstrates. A breakdown of what each argument is about is shown below:
    1. 2 - Quadcopter will hover at a constant height on the z-axis while navigate along a linear path along the x-axis
    2. 3 - Quadcopter will take off vertically along the z-axis, hover for a set amount of time, then land back down vertically along the z-axis.
    3. 4 - State machine is implemented into model where quadcopter takes off vertically, hovers for a set amoung of time, tracks a pre-defined trajectory path, then hovers for a set amount time before landing

# Miscellaneous Details
For more detailed notes on the simulation, please refer to the the README.md file inside the code/ folder. The write up file also has qualitative descriptions of the simulation and its overall design.
