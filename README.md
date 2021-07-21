# Drone Racing Competition

![Alt](https://sarabakha.info/images/fuzz_ieee_2021/racing_drone.jpg)
			
## Organizers
- **Dr. Andriy Sarabakha** (Munich School of Robotics and Machine Intelligence, Technical University of Munich, Germany), andriy.sarabakha@tum.de
- **Prof. Erdal Kayacan** (Department of Engineering, Aarhus University, Denmark), erdal@ece.au.dk

## Evaluation Committee
- **Prof. ‪Jonathan M. Garibaldi**‬ (School of Computer Scince, University of Nottingham, UK)
- **Prof. Oscar Castillo** (Division of Graduate Studies, Tijuana Institute of Technology, México)
- **Dr. Hak-Keung Lam** (Department of Engineering, King's College London, UK)
- **Prof. Erdal Kayacan** (Department of Electrical and Computer Engineering, Aarhus University, Denmark)
- **Dr. Andriy Sarabakha** (Munich School of Robotics and Machine Intelligence, Technical University of Munich, Germany)

## Sponsors
- [MathWorks](https://mathworks.com/)
- [Luxembourg National Research Fund (FNR)](https://www.fnr.lu/)
			
## Motivation
Drone racing is a recreational sport in which the goal is to pass through a sequence of gates in a minimum amount of time while avoiding collisions with the environment. In autonomous drone racing, one must accomplish this task by flying fully autonomously in an unknown environment relying only on the on-board resources. Thus, autonomous drone racing is an exciting case study that aims to motivate more researchers to develop innovative ways of solving complex problems. What makes drone racing such an interesting challenge is the cumulative complexity of each sub-problem to be solved, such as _perception_, _localisation_, _path planning_ and _control_.
			
## Goal
The FUZZ IEEE 2021 Autonomous Drone Racing Competition is an aerial robotics challenge in which research groups will validate their fuzzy logic controllers in a challenging scenario – _autonomous drone racing_.
			
## Technical Challenge
The technical challenge of this competition focuses on the design of various fuzzy logic controllers which can operate on nonlinear regions of the drone's dynamical model at high speeds. The number and pose (position and orientation) of the gates will be predefined, and the drone will have to cross them in a given sequence. For the gate detection, an on-board wide-angle camera with 90° field-of-view is used. Trajectories at fixed velocities through the centres of all gates will be generated using the curve fitting method and available to the participants. However, each team is free to implement its own trajectory generation strategy to achieve better control performances. The real-time noisy localisation, which includes estimated position and attitude, will be available to simulate visual-inertial odometry which deteriorates its performances at high speeds due to motion blur and accelerometer's noise. The task of each team is to develop a fuzzy logic controller which will take the desired trajectory and current pose of the drone, as input, and will provide the commanded attitude and thrust, as control commands, as depicted in Figure 1.

![Alt](https://sarabakha.info/images/fuzz_ieee_2021/drone.jpg)

*Figure 1. Racing drone with its reference frames and control inputs.*
			
## Environment
The simulation environment of the racing track with racing gates is implemented in MATLAB. A sample environment is shown in Figure 2. The simulation package contains five files:
1. *MAIN.m*: main file for starting the simulation;
2. *trajectory.m*: trajectory generation through the center of each gate;    
3. *controller.m*: pose controller which **participants will have to implement** (a sample PD controller is provided);
4. *uav.m*: dynamical model of the drone and its low-level controller;
5. *environment.m*: visualisation of the racing arena and performance evaluation.

Please note that only the content of *controller.m* and *trajectory.m* can be modified.

![Alt](https://sarabakha.info/images/fuzz_ieee_2021/arena.gif)

*Figure 2. Racing arena simulated in Matlab.*

The position and orientation of the gates in the testing arena will be different than in the arena provided to the teams to develop their controllers.

## Execution

Replace *controller.m* (and *trajectory.m*) inside *submissions/team1* with your implemented controller and trajectory generation, and run *MAIN.m*.
			
## Marking
The controllers will be evaluated using a similar MATLAB environment but the position of the gates will be different. The main prerequisite is the real-time implementation of the controller, in other words, **the controller must run minimum at 100Hz**. Each team will be given 60 seconds to cross as many gates as possible in a predefined sequence. The evaluation rules will be as follows:
- for each successfully crossed gate: **+1 point**;
- for each skipped gate: **-0.5 points**;
- losing the next gate to cross in the field of view of the drone: **-1 point/sec**;
- after a crash with a gate/ground/ceiling/walls: **the competition will end** (the points collected till the crash will be counted);
- surviving till the end of the challenge: **+5 points**.

The Organization Committee will evaluate the developed controllers on the testing racing track. The results will be announced during [FUZZ IEEE 2021](https://attend.ieee.org/fuzzieee-2021/) conference in July 2021.
            
## Prizes
- A student MATLAB license for the **winning team**.
- A small surprise from MathWorks for the **winning team**.
- A webinar organised by MathWorks during the competition at FUZZ IEEE 2021 for **all participating teams**.
            
## Applications
Prospective participants must submit an application via the [**following form**](https://forms.gle/LHfwawWpHx5MkVVGA). Successfully registered teams will receive a package containing the simulation environment. The latest version of the simulation environment can be downloaded also from this repository.
			
## Submission
The teams must submit Matlab files containing their controller (and trajectory generator) to the Organization Committee via email by **May 31, 2021**.
