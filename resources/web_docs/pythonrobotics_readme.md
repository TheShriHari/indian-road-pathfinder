# Source: https://github.com/AtsushiSakai/PythonRobotics

Python codes and [textbook](https://atsushisakai.github.io/PythonRobotics/index.html) for robotics algorithm.
PythonRobotics is a Python code collection and a [textbook](https://atsushisakai.github.io/PythonRobotics/index.html) of robotics algorithms.
Features:
- 
Easy to read for understanding each algorithm's basic idea.
- 
Widely used and practical algorithms are selected.
- 
Minimum dependency.
See this documentation
or this Youtube video:
or this paper for more details:
For running each sample code:
For development:
- 
[pytest](https://pytest.org/) (for unit tests)
- 
[pytest-xdist](https://pypi.org/project/pytest-xdist/) (for parallel unit tests)
- 
[mypy](https://mypy-lang.org/) (for type check)
- 
[sphinx](https://www.sphinx-doc.org/) (for document generation)
- 
[pycodestyle](https://pypi.org/project/pycodestyle/) (for code style check)
This README only shows some examples of this project.
If you are interested in other examples or mathematical backgrounds of each algorithm,
You can check the full documentation (textbook) online: [Welcome to PythonRobotics’s documentation! — PythonRobotics documentation](https://atsushisakai.github.io/PythonRobotics/index.html)
All animation gifs are stored here: [AtsushiSakai/PythonRoboticsGifs: Animation gifs of PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs)
- 
Clone this repo. git clone https://github.com/AtsushiSakai/PythonRobotics.git
- 
Install the required libraries.
- 
using conda : conda env create -f requirements/environment.yml
- 
using pip : pip install -r requirements/requirements.txt
- 
Execute python script in each directory.
- 
Add star to this repo if you like it 😃.
Reference
This is a sensor fusion localization with Particle Filter(PF).
The blue line is true trajectory, the black line is dead reckoning trajectory,
and the red line is an estimated trajectory with PF.
It is assumed that the robot can measure a distance from landmarks (RFID).
These measurements are used for PF localization.
Reference
This is a 2D localization example with Histogram filter.
The red cross is true position, black points are RFID positions.
The blue grid shows a position probability of histogram filter.
In this simulation, x,y are unknown, yaw is known.
The filter integrates speed input and range observations from RFID for localization.
Initial position is not needed.
Reference
This is a 2D Gaussian grid mapping example.
This is a 2D ray casting grid mapping example.
This example shows how to convert a 2D range measurement to a grid map.
This is a 2D object clustering with k-means algorithm.
This is a 2D rectangle fitting for vehicle detection.
Simultaneous Localization and Mapping(SLAM) examples
This is a 2D ICP matching example with singular value decomposition.
It can calculate a rotation matrix, and a translation vector between points and points.
Reference
This is a feature based SLAM example using FastSLAM 1.0.
The blue line is ground truth, the black line is dead reckoning, the red line is the estimated trajectory with FastSLAM.
The red points are particles of FastSLAM.
Black points are landmarks, blue crosses are estimated landmark positions by FastSLAM.
Reference
This is a 2D navigation sample code with Dynamic Window Approach.
This is a 2D grid based the shortest path planning with Dijkstra's algorithm.
In the animation, cyan points are searched nodes.
This is a 2D grid based the shortest path planning with A star algorithm.
In the animation, cyan points are searched nodes.
Its heuristic is 2D Euclid distance.
This is a 2D grid based the shortest path planning with D star algorithm.
The animation shows a robot finding its path avoiding an obstacle using the D* search algorithm.
Reference
This algorithm finds the shortest path between two points while rerouting when obstacles are discovered. It has been implemented here for a 2D grid.
The animation shows a robot finding its path and rerouting to avoid obstacles as they are discovered using the D* Lite search algorithm.
Refs:
This is a 2D grid based path planning with Potential Field algorithm.
In the animation, the blue heat map shows potential value on each grid.
Reference
This is a 2D grid based coverage path planning simulation.
This is a 2D path planning simulation using the Particle Swarm Optimization algorithm.
PSO is a metaheuristic optimization algorithm inspired by bird flocking behavior. In path planning, particles explore the search space to find collision-free paths while avoiding obstacles.
The animation shows particles (blue dots) converging towards the optimal path (yellow line) from start (green area) to goal (red star).
References
- 
[Particle swarm optimization - Wikipedia](https://en.wikipedia.org/wiki/Particle_swarm_optimization)
- 
[Kennedy, J.; Eberhart, R. (1995). "Particle Swarm Optimization"](https://ieeexplore.ieee.org/document/488968)
This script is a path planning code with state lattice planning.
This code uses the model predictive trajectory generator to solve boundary problem.
Reference
- 
[Optimal rough terrain trajectory generation for wheeled mobile robots](https://journals.sagepub.com/doi/pdf/10.1177/0278364906075328)
- 
[State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments](https://www.cs.cmu.edu/~alonzo/pubs/papers/JFR_08_SS_Sampling.pdf)
This PRM planner uses Dijkstra method for graph search.
In the animation, blue points are sampled points,
Cyan crosses means searched points with Dijkstra method,
The red line is the final path of PRM.
Reference
This is a path planning code with RRT*
Black circles are obstacles, green line is a searched tree, red crosses are start and goal positions.
Reference
- 
[Incremental Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/abs/1005.0416)
- 
[Sampling-based Algorithms for Optimal Motion Planning](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=bddbc99f97173430aa49a0ada53ab5bade5902fa)
Path planning for a car robot with RRT* and reeds shepp path planner.
This is a path planning simulation with LQR-RRT*.
A double integrator motion model is used for LQR local planner.
Reference
- 
[LQR-RRT*: Optimal Sampling-Based Motion Planning with Automatically Derived Extension Heuristics](https://lis.csail.mit.edu/pubs/perez-icra12.pdf)
- 
[MahanFathi/LQR-RRTstar: LQR-RRT* method is used for random motion planning of a simple pendulum in its phase plot](https://github.com/MahanFathi/LQR-RRTstar)
Motion planning with quintic polynomials.
It can calculate a 2D path, velocity, and acceleration profile based on quintic polynomials.
Reference
A sample code with Reeds Shepp path planning.
Reference
- 
[15.3.2 Reeds-Shepp Curves](http://planning.cs.uiuc.edu/node822.html)
- 
[optimal paths for a car that goes both forwards and backwards](https://pdfs.semanticscholar.org/932e/c495b1d0018fd59dee12a0bf74434fac7af4.pdf)
- 
[ghliu/pyReedsShepp: Implementation of Reeds Shepp curve.](https://github.com/ghliu/pyReedsShepp)
A sample code using LQR based path planning for double integrator model.
This is optimal trajectory generation in a Frenet Frame.
The cyan line is the target course and black crosses are obstacles.
The red line is the predicted path.
Reference
- 
[Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
- 
[Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)
This is a simulation of moving to a pose control
Reference
Path tracking simulation with Stanley steering control and PID speed control.
Reference
- 
[Stanley: The robot that won the DARPA grand challenge](http://robots.stanford.edu/papers/thrun.stanley05.pdf)
- 
[Automatic Steering Methods for Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
Path tracking simulation with rear wheel feedback steering control and PID speed control.
Reference
Path tracking simulation with LQR speed and steering control.
Reference
Path tracking simulation with iterative linear model predictive speed and steering control.
Reference
A motion planning and path tracking simulation with NMPC of C-GMRES
Reference
N joint arm to a point control simulation.
This is an interactive simulation.
You can set the goal position of the end effector with left-click on the plotting area.
In this simulation N = 10, however, you can change it.
Arm navigation with obstacle avoidance simulation.
This is a 3d trajectory following simulation for a quadrotor.
This is a 3d trajectory generation simulation for a rocket powered landing.
Reference
This is a bipedal planner for modifying footsteps for an inverted pendulum.
You can set the footsteps, and the planner will modify those automatically.
MIT
If this project helps your robotics project, please let me know with creating an issue.
Your robot's video, which is using PythonRobotics, is very welcome!!
This is a list of user's comment and references:[users_comments](https://github.com/AtsushiSakai/PythonRobotics/blob/master/users_comments.md)
Any contribution is welcome!!
Please check this document:[How To Contribute — PythonRobotics documentation](https://atsushisakai.github.io/PythonRobotics/modules/0_getting_started/3_how_to_contribute.html)
If you use this project's code for your academic work, we encourage you to cite [our papers](https://arxiv.org/abs/1808.10703)
If you use this project's code in industry, we'd love to hear from you as well; feel free to reach out to the developers directly.
If you or your company would like to support this project, please consider:
- 
[Sponsor @AtsushiSakai on GitHub Sponsors](https://github.com/sponsors/AtsushiSakai)
- 
[Become a backer or sponsor on Patreon](https://www.patreon.com/myenigma)
- 
[One-time donation via PayPal](https://www.paypal.com/paypalme/myenigmapay/)
If you would like to support us in some other way, please contact with creating an issue.
They are providing a free license of their IDEs for this OSS development.
They are providing a free license of their 1Password team license for this OSS project.