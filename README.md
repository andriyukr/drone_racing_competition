# Drone Racing Competition

			<center>
                <img src="images/fuzz_ieee_2021/racing_drone.jpg" width="800" alt="" align="middle">
            </center>
			
			<p><h4>Organizers</h4></p>
			<li><b>Dr. Andriy Sarabakha</b> (Munich School of Robotics and Machine Intelligence, Technical University of Munich, Germany), <a href="mailto: andriy.sarabakha@tum.de">andriy.sarabakha@tum.de</a>
            <li><b>Assoc. Prof. Erdal Kayacan</b> (Department of Engineering, Aarhus University, Denmark), <a href="mailto: erdal@ece.au.dk">erdal@ece.au.dk</a>
			
			<p><h4>Motivation</h4></p>
			<p align="justify">Drone racing is a recreational sport in which the goal is to pass through a sequence of gates in a minimum amount of time while avoiding collisions with the environment. In autonomous drone racing, one must accomplish this task by flying fully autonomously in an unknown environment relying only on the on-board resources. Thus, autonomous drone racing is an exciting case study that aims to motivate more researchers to develop innovative ways of solving complex problems. What makes drone racing such an interesting challenge is the cumulative complexity of each sub-problem to be solved, such as <i>perception</i>, <i>localisation</i>, <i>path planning</i> and <i>control</i>.</p>
			
			<p><h4>Goal</h4></p>
			<p align="justify">The FUZZ IEEE 2021 Autonomous Drone Racing Competition is an aerial robotics challenge in which research groups will validate their fuzzy logic controllers in a challenging scenario – <i>autonomous drone racing</i>.</p>
			
			<p><h4>Technical Challenge</h4></p>
			<p align="justify">
			    The technical challenge of this competition focuses on the design of various fuzzy logic controllers which can operate on nonlinear regions of the drone's dynamical model at high speeds. The number and pose (position and orientation) of the gates will be predefined, and the drone will have to cross them in a given sequence. For the gate detection, an on-board wide-angle camera with 90° field-of-view is used. Trajectories at fixed velocities through the centres of all gates will be generated using the curve fitting method and available to the participants. However, each team is free to implement its own trajectory generation strategy to achieve better control performances. The real-time noisy localisation, which includes estimated position and attitude, will be available to simulate visual-inertial odometry which deteriorates its performances at high speeds due to motion blur and accelerometer's noise. The task of each team is to develop a fuzzy logic controller which will take the desired trajectory and current pose of the drone, as input, and will provide the commanded attitude and thrust, as control commands, as depicted in Figure 1.</p>
			<center>
    		    <figure>
                    <img src="images/fuzz_ieee_2021/drone.jpg" width="600" alt="" align="middle">
                    <figcaption>Figure 1. Racing drone with its reference frames and control inputs.</figcaption>
                </figure>
            </center>
			
			<p><h4>Environment</h4></p>
			<p align="justify">The simulation environment of the racing track with racing gates is implemented in MATLAB. A sample environment is shown in Figure 2. The simulation package contains five files:</p>
			<li><i>main.m</i>: main file for starting the simulation;
			<li><i>trajectory.m</i>: trajectory generation through the center of each gate;    
			<li><i>controller.m</i>: pose controller which <b>participants will have to implement</b> (a sample PD controller is provided);
			<li><i>uav.m</i>: dynamical model of the drone and its low-level controller;
			<li><i>environment.m</i>: visualisation of the racing arena and performance evaluation.
            <center>
    		    <figure>
                    <img src="images/fuzz_ieee_2021/arena.gif" width="600" alt="" align="middle">
                    <figcaption>Figure 2. Racing arena simulated in Matlab.</figcaption>
                </figure>
            </center>
			
			<p><h4>Marking</h4></p>
			<p align="justify">The controllers will be evaluated using a similar MATLAB environment but the position of the gates will be different. The main prerequisite is the real-time implementation of the controller, in other words, <b>the controller must run minimum at 100Hz</b>. Each team will be given 60 seconds to cross as many gates as possible in a predefined sequence. The evaluation rules will be as follows:</p>
            <p align="justify">
                <li>for each successfully crossed gate: <b>+1 point</b>;
                <li>for each skipped gate: <b>-0.5 points</b>;
                <li>losing the next gate to cross in the field of view of the drone: <b>-1 point/sec</b>;
                <li>after a crash with a gate/ground/ceiling/walls: <b>the competition will end</b> (the points collected till the crash will be counted);
                <li>surviving till the end of the challenge: <b>+5 points</b>.
            </p>
            <p align="justify">The Organization Committee will evaluate the developed controllers on the testing racing track. The results will be announced during <a href="https://attend.ieee.org/fuzzieee-2021/" target="_blank">FUZZ IEEE 2021</a> conference in <font color="red">July 2021</font>.</p>
            
            <p><h4>Prizes</h4></p>
            <p align="justify">We may have some surprise prizes for the first, second and third teams.</p>
            
			<p><h4>Applications</h4></p>
			<p align="justify">Prospective participants must submit an application via the <a href="https://forms.gle/LHfwawWpHx5MkVVGA" target="_blank"><b>following form</b></a>. Successfully registered teams will receive a package containing the simulation environment. The latest version of the simulation environment can be downloaded also from <a href="https://github.com/andriyukr/drone_racing_competition" target="_blank"><b>GitHub</b></a> repository.</p>
			
			<p><h4>Submission</h4></p>
			<p align="justify">The teams must submit Matlab files containing their controller (and trajectory generator) to the Organization Committee via email by <font color="red"><b>May 31, 2021</b></font>.</p>
