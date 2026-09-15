1 

# A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles 

Brian Paden<sup>_∗,_1</sup> , Michal Cáp<sup>ˇ</sup><sup>_∗,_1</sup><sup>_,_2</sup> , Sze Zheng Yong<sup>1</sup> , Dmitry Yershov<sup>1</sup> , and Emilio Frazzoli<sup>1</sup> 

### **Abstract** 

Self-driving vehicles are a maturing technology with the potential to reshape mobility by enhancing the safety, accessibility, efficiency, and convenience of automotive transportation. Safety-critical tasks that must be executed by a self-driving vehicle include planning of motions through a dynamic environment shared with other vehicles and pedestrians, and their robust executions via feedback control. The objective of this paper is to survey the current state of the art on planning and control algorithms with particular regard to the urban setting. A selection of proposed techniques is reviewed along with a discussion of their effectiveness. The surveyed approaches differ in the vehicle mobility model used, in assumptions on the structure of the environment, and in computational requirements. The side-by-side comparison presented in this survey helps to gain insight into the strengths and limitations of the reviewed approaches and assists with system level design choices. 

## CONTENTS 

|**I**|**Introd**|**uction**|2|
|---|---|---|---|
|**II**|**Overvi**|**ew of the Decision-Making Hierarchy used in Driverless Cars**|3|
||II-A|Route Planning . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>3|
||II-B|Behavioral Decision Making . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>3|
||II-C|Motion Planning . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>4|
||II-D|Vehicle Control . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>4|
|**III**|**Modeli**|**ng for Planning and Control**|4|
||III-A|The Kinematic Single-Track Model . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>4|
||III-B|Inertial Effects . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>6|
|**IV**|**Motion**|**Planning**|7|
||IV-A|Path Planning . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>7|
||IV-B|Trajectory Planning<br>. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>9|
||IV-C|Variational Methods . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>9|
||IV-D|Graph Search Methods . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>11|
|||IV-D1<br>Lane Graph . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>11|
|||IV-D2<br>Geometric Methods . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>12|
|||IV-D3<br>Sampling-based Methods . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>12|
|||IV-D4<br>Graph Search Strategies . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>14|
||IV-E|Incremental Search Techniques<br>. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>15|
||IV-F|Practical Deployments . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>16|
|**V**|**Vehicle**|**Control**|17|
||V-A|Path Stabilization for the Kinematic Model . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>18|
|||V-A1<br>Pure Pursuit . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>18|
|||V-A2<br>Rear wheel position based feedback . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>19|
|||V-A3<br>Front wheel position based feedback<br>. . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>19|
||V-B|Trajectory Tracking Control for the Kinematic Model . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>20|
|||V-B1<br>Control Lyapunov based design . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>20|
|||V-B2<br>Output feedback linearization<br>. . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>21|
||V-C|Predictive Control Approaches . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>21|
|||V-C1<br>Unconstrained MPC with Kinematic Models . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>22|
|||V-C2<br>Path Tracking Controllers<br>. . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>22|
|||V-C3<br>Trajectory Tracking Controllers . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>22|
||V-D|Linear Parameter Varying Controllers . . . . . . . . . . . . . . . . . . . . . . . . . . . . .|. . . . . . . . . . . . . . .<br>23|
|**VI**|**Conclu**|**sions**|23|
|**Refe**|**rences**||23|



**VI Conclusions** 

### **References** 

> _∗_ The first two authors contributed equally to this work. 

> 1 The authors are with the Laboratory for Information and Decision Systems, Massachusetts Institute of Technology, Cambridge MA, USA. email: bapaden@mit.edu, mcap@mit.edu, szyong@mit.edu, yershov@mit.edu, frazzoli@mit.edu 

> 2 Michal ˇCáp is also affiliated with Dept. of Computer Science, Faculty of Electrical Engineering, CTU in Prague, Czech Republic. 

2 

## I. INTRODUCTION 

The last three decades have seen steadily increasing research efforts, both in academia and in industry, towards developing driverless vehicle technology. These developments have been fueled by recent advances in sensing and computing technology together with the potential transformative impact on automotive transportation and the perceived societal benefit: In 2014 there were 32,675 traffic related fatalities, 2.3 million injuries, and 6.1 million reported collisions [1]. Of these, an estimated 94% are attributed to driver error with 31% involving legally intoxicated drivers, and 10% from distracted drivers [2]. Autonomous vehicles have the potential to dramatically reduce the contribution of driver error and negligence as the cause of vehicle collisions. They will also provide a means of personal mobility to people who are unable to drive due to physical or visual disability. Finally, for the 86% of the US work force that commutes by car, on average 25 minutes (one way) each day [3], autonomous vehicles would facilitate more productive use of the transit time, or simply reduce the measurable ill effects of driving stress [4]. 

Considering the potential impacts of this new technology, it is not surprising that self-driving cars have had a long history. The idea has been around as early as in the 1920s, but it was not until the 1980s that driverless cars seemed like a real possibility. Pioneering work led by Ernst Dickmanns (e.g., [5]) in the 1980s paved the way for the development of autonomous vehicles. At that time a massive research effort, the PROMETHEUS project, was funded to develop an autonomous vehicle. A notable demonstration in 1994 resulting from the work was a 1,600 km drive by the VaMP driverless car, of which 95% was driven autonomously [6]. At a similar time, the CMU NAVLAB was making advances in the area and in 1995 demonstrated further progress with a 5,000 km drive across the US of which 98% was driven autonomously [7]. 

The next major milestone in driverless vehicle technology was the first DARPA Grand Challenge in 2004. The objective was for a driverless car to navigate a 150-mile off-road course as quickly as possible. This was a major challenge in comparison to previous demonstrations in that there was to be no human intervention during the race. Although prior works demonstrated nearly autonomous driving, eliminating human intervention at critical moments proved to be a major challenge. None of the 15 vehicles entered into the event completed the race. In 2005 a similar event was held; this time 5 of 23 teams reached the finish line [8]. Later, in 2007, the DARPA Urban Challenge was held, in which vehicles were required to drive autonomously in a simulated urban setting. Six teams finished the event demonstrating that fully autonomous urban driving is possible [9]. 

Numerous events and major autonomous vehicle system tests have been carried out since the DARPA challenges. Notable examples include the Intelligent Vehicle Future Challenges from 2009 to 2013 [10], Hyundai Autonomous Challenge in 2010 [11], the VisLab Intercontinental Autonomous Challenge in 2010 [12], the Public Road Urban DriverlessCar Test in 2013 [13], and the autonomous drive of the 

Bertha-Benz historic route [14]. Simultaneously, research has continued at an accelerated pace in both the academic setting as well as in industry. The Google self-driving car [15] and Tesla’s Autopilot system [16] are two examples of commercial efforts receiving considerable media attention. 

The extent to which a car is automated can vary from fully human operated to fully autonomous. The SAE J3016 standard [17] introduces a scale from 0 to 5 for grading vehicle automation. In this standard, the level 0 represents a vehicle where all driving tasks are the responsibility of a human driver. Level 1 includes basic driving assistance such as adaptive cruise control, anti-lock braking systems and electronic stability control [18]. Level 2 includes advanced assistance such as hazard-minimizing longitudinal/lateral control [19] or emergency braking [20], [21], often based upon set-based formal control theoretic methods to compute ‘worst-case’ sets of provably collision free (safe) states [22]–[24]. At level 3 the system monitors the environment and can drive with full autonomy under certain conditions, but the human operator is still required to take control if the driving task leaves the autonomous system’s operational envelope. A vehicle with level 4 automation is capable of fully autonomous driving in certain conditions and will safely control the vehicle if the operator fails to take control upon request to intervene. Level 5 systems are fully autonomous in all driving modes. 

The availability of on-board computation and wireless communication technology allows cars to exchange information with other cars and with the road infrastructure giving rise to a closely related area of research on connected intelligent vehicles [25]. This research aims to improve the safety and performance of road transport through information sharing and coordination between individual vehicles. For instance, connected vehicle technology has a potential to improve throughput at intersections [26] or prevent formation of traffic shock waves [27]. 

To limit the scope of this survey, we focus on aspects of decision making, motion planning, and control for self-driving cars, in particular, for systems falling into the automation level of 3 and above. For the same reason, the broad field of perception for autonomous driving is omitted and instead the reader is referred to a number of comprehensive surveys and major recent contributions on the subject [28]–[31]. 

The decision making in contemporary autonomous driving systems is typically hierarchically structured into route planning, behavioral decision making, local motion planning and feedback control. The partitioning of these levels are, however, rather blurred with different variations of this scheme occurring in the literature. This paper provides a survey of proposed methods to address these core problems of autonomous driving. Particular emphasis is placed on methods for local motion planning and control. 

The remainder of the paper is structured as follows: In Section II, a high level overview of the hierarchy of decision making processes and some of the methods for their design are presented. Section III reviews models used to approximate the mobility of cars in urban settings for the purposes of motion planning and feedback control. Section IV surveys the rich literature on motion planning and discusses its applicability for 

3 

self-driving cars. Similarly, Section V discusses the problems of path and trajectory stabilization and specific feedback control methods for driverless cars. Lastly, Section VI concludes with remarks on the state of the art and potential areas for future research. 

## II. OVERVIEW OF THE DECISION-MAKING HIERARCHY USED IN DRIVERLESS CARS 

In this section we describe the decision making architecture of a typical self-driving car and comment on the responsibilities of each component. Driverless cars are essentially autonomous decision-making systems that process a stream of observations from on-board sensors such as radars, LIDARs, cameras, GPS/INS units, and odometry. These observations, together with prior knowledge about the road network, rules of the road, vehicle dynamics, and sensor models, are used to automatically select values for controlled variables governing the vehicle’s motion. Intelligent vehicle research aims at automating as much of the driving task as possible. The commonly adopted approach to this problem is to partition and organize perception and decision-making tasks into a hierarchical structure. The prior information and collected observation data are used by the perception system to provide an estimate of the state of the vehicle and its surrounding environment; the estimates are then used by the decisionmaking system to control the vehicle so that the driving objectives are accomplished. 

The decision making system of a typical self-driving car is hierarchically decomposed into four components (cf. Figure II.1): At the highest level a route is planned through the road network. This is followed by a behavioral layer, which decides on a local driving task that progresses the car towards the destination and abides by rules of the road. A motion planning module then selects a continuous path through the environment to accomplish a local navigational task. A control system then reactively corrects errors in the execution of the planned motion. In the remainder of the section we discuss the responsibilities of each of these components in more detail. 

## _A. Route Planning_ 

At the highest level, a vehicle’s decision-making system must select a route through the road network from its current position to the requested destination. By representing the road network as a directed graph with edge weights corresponding to the cost of traversing a road segment, such a route can be formulated as the problem of finding a minimum-cost path on a road network graph. The graphs representing road networks can however contain millions of edges making classical shortest path algorithms such as Dijkstra [32] or A* [33] impractical. The problem of efficient route planning in transportation networks has attracted significant interest in the transportation science community leading to the invention of a family of algorithms that after a one-time pre-processing step return an optimal route on a continent-scale network in milliseconds [34], [35]. For a comprehensive survey and comparison of practical algorithms that can be used to efficiently plan routes for both human-driven and self-driving vehicles, see [36]. 



<!-- Start of picture text -->
User specifiied<br>destination<br>Route Planning<br>Road network data<br>Sequence of waypoints through road network<br>Behavioral Layer<br>Perceived agents,<br>obstacles, and  IntersectionNegotiate<br>signage<br>Parking Lane Change<br>Maneuver Following Lanes<br>Unstructured<br>Environment<br>Motion Specif i cation<br>Motion Planning<br>Estimated pose and<br>collision free space<br>Reference path or trajectory<br>Local Feedback<br>Control<br>Estimate of vehicle<br>state<br>Steering, throttle and brake commands<br><!-- End of picture text -->

Figure II.1: Illustration of the hierarchy of decision-making processes. A destination is passed to a route planner that generates a route through the road network. A behavioral layer reasons about the environment and generates a motion specification to progress along the selected route. A motion planner then solves for a feasible motion accomplishing the specification. A feedback control adjusts actuation variables to correct errors in executing the reference path. 

## _B. Behavioral Decision Making_ 

After a route plan has been found, the autonomous vehicle must be able to navigate the selected route and interact with other traffic participants according to driving conventions and rules of the road. Given a sequence of road segments specifying the selected route, the behavioral layer is responsible for selecting an appropriate driving behavior at any point of time based on the perceived behavior of other traffic participants, road conditions, and signals from infrastructure. For example, when the vehicle is reaching the stop line before an intersection, the behavioral layer will command the vehicle 

4 

to come to a stop, observe the behavior of other vehicles, bikes, and pedestrians at the intersection, and let the vehicle proceed once it is its turn to go. 

Driving manuals dictate qualitative actions for specific driving contexts. Since both driving contexts and the behaviors available in each context can be modeled as finite sets, a natural approach to automating this decision making is to model each behavior as a state in a finite state machine with transitions governed by the perceived driving context such as relative position with respect to the planned route and nearby vehicles. In fact, finite state machines coupled with different heuristics specific to considered driving scenarios were adopted as a mechanism for behavior control by most teams in the DARPA Urban Challenge [9]. 

Real-world driving, especially in an urban setting, is however characterized by uncertainty over the intentions of other traffic participants. The problem of intention prediction and estimation of future trajectories of other vehicles, bikes and pedestrians has also been studied. Among the proposed solution techniques are machine learning based techniques, e.g., Gaussian mixture models [37], Gaussian process regression [38], the learning techniques reportedly used in Google’s self-driving system for intention prediction [39], and modelbased approaches for directly estimating intentions from sensor measurements [40], [41]. 

This uncertainty in the behavior of other traffic participants is commonly considered in the behavioral layer for decision making using probabilistic planning formalisms, such as Markov Decision Processes (MDPs) and their generalizations. For example, [42] formulates the behavioral decision-making problem in the MDP framework. Several works [43]–[46] model unobserved driving scenarios and pedestrian intentions explicitly using a partially-observable Markov decision process (POMDP) framework and propose specific approximate solution strategies. 

## _C. Motion Planning_ 

When the behavioral layer decides on the driving behavior to be performed in the current context, which could be, e.g., cruise-in-lane, change-lane, or turn-right, the selected behavior has to be translated into a path or trajectory that can be tracked by the low-level feedback controller. The resulting path or trajectory must be dynamically feasible for the vehicle, comfortable for the passenger, and avoid collisions with obstacles detected by the on-board sensors. The task of finding such a path or trajectory is a responsibility of the motion planning system. 

The task of motion planning for an autonomous vehicle corresponds to solving the standard motion planning problem as discussed in the robotics literature. Exact solutions to the motion planning problem are in most cases computationally intractable. Thus, numerical approximation methods are typically used in practice. Among the most popular numerical approaches are variational methods that pose the problem as non-linear optimization in a function space, graph-search approaches that construct graphical discretization of the vehicle’s state space and search for a shortest path using graph search 

methods, and incremental tree-based approaches that incrementally construct a tree of reachable states from the initial state of the vehicle and then select the best branch of such a tree. The motion planning methods relevant for autonomous driving are discussed in greater detail in Section IV. 

## _D. Vehicle Control_ 

In order to execute the reference path or trajectory from the motion planning system a feedback controller is used to select appropriate actuator inputs to carry out the planned motion and correct tracking errors. The tracking errors generated during the execution of a planned motion are due in part to the inaccuracies of the vehicle model. Thus, a great deal of emphasis is placed on the robustness and stability of the closed loop system. 

Many effective feedback controllers have been proposed for executing the reference motions provided by the motion planning system. A survey of related techniques are discussed in detail in Section V. 

## III. MODELING FOR PLANNING AND CONTROL 

In this section we will survey the most commonly used models of mobility of car-like vehicles. Such models are widely used in control and motion planning algorithms to approximate a vehicle’s behavior in response to control actions in relevant operating conditions. A high-fidelity model may accurately reflect the response of the vehicle, but the added detail may complicate the planning and control problems. This presents a trade-off between the accuracy of the selected model and the difficulty of the decision problems. This section provides an overview of general modeling concepts and a survey of models used for motion planning and control. 

Modeling begins with the notion of the vehicle _configuration_ , representing its pose or position in the world. For example, configuration can be expressed as the planar coordinate of a point on the car together with the car’s heading. This is a _coordinate system_ for the configuration space of the car. This coordinate system describes planar rigid-body motions (represented by the Special Euclidean group in two dimensions, SE(2)) and is a commonly used configuration space [47]–[49]. Vehicle motion must then be planned and regulated to accomplish driving tasks and while respecting the constraints introduced by the selected model. 

## _A. The Kinematic Single-Track Model_ 

In the most basic model of practical use, the car consists of two wheels connected by a rigid link and is restricted to move in a plane [48]–[52]. It is assumed that the wheels do not slip at their contact point with the ground, but can rotate freely about their axes of rotation. The front wheel has an added degree of freedom where it is allowed to rotate about an axis normal to the plane of motion. This is to model steering. These two modeling features reflect the experience most passengers have where the car is unable to make lateral displacement without simultaneously moving forward. More formally, the limitation on maneuverability is referred to as a _nonholonomic_ constraint [47], [53]. The nonholonomic constraint is 

5 

expressed as a differential constraint on the motion of the car. This expression varies depending on the choice of coordinate system. Variations of this model have been referred to as the car-like robot, bicycle model, kinematic model, or single track model. 

The following is a derivation of the differential constraint in several popular coordinate systems for the configuration. In reference to Figure III.1, the vectors _pr_ and _pf_ denote the location of the rear and front wheels in a stationary or inertial coordinate system with basis vectors (ˆ _ex,_ ˆ _ey,_ ˆ _ez_ ). The heading _θ_ is an angle describing the direction that the vehicle is facing. This is defined as the angle between vectors _e_ ˆ _x_ and _pf − pr_ . 

Differential constraints will be derived for the coordinate systems consisting of the angle _θ_ , together with the motion of one of the points _pr_ as in [54], and _pf_ as in [55]. 



Figure III.1: Kinematics of the single track model. _pr_ and _pf_ are the ground contact points of the rear and front tire respectively. _θ_ is the vehicle heading. Time derivatives of _pr_ and _pf_ are restricted by the nonholonomic constraint to the direction indicated by the blue arrows. _δ_ is the steering angle of the front wheel. 

The motion of the points _pr_ and _pf_ must be collinear with the wheel orientation to satisfy the no-slip assumption. Expressed as an equation, this constraint on the rear wheel is 



and for the front wheel: 



This expression is usually rewritten in terms of the componentwise motion of each point along the basis vectors. The motion of the rear wheel along the _e_ ˆ _x_ -direction is _xr_ := _pr · e_ ˆ _x_ . Similarly, for _e_ ˆ _y_ -direction, _yr_ := _pr ·_ ˆ _ey_ . The forward speed is _vr_ := _p_ ˙ _r ·_ ( _pf − pr_ ) _/∥_ ( _pf − pr_ ) _∥_ , which is the magnitude of _p_ ˙ _r_ with the correct sign to indicate forward or reverse driving. In terms of the scalar quantities _xr_ , _yr_ , and _θ_ , the differential constraint is 



Alternatively, the differential constraint can be written in terms 

the motion of _pf_ , 



where the front wheel forward speed _vf_ is now used. The front wheel speed, _vf_ , is related to the rear wheel speed by 



The planning and control problems for this model involve selecting the steering angle _δ_ within the mechanical limits of the vehicle _δ ∈_ [ _δmin, δmax_ ], and forward speed _vr_ within an acceptable range, _vr ∈_ [ _vmin, vmax_ ] _._ 

A simplification that is sometimes utilized, e.g. [56], is to select the heading rate _ω_ instead of steering angle _δ_ . These quantities are related by 



simplifying the heading dynamics to 



In this situation, the model is sometimes referred to as the unicycle model since it can be derived by considering the motion of a single wheel. 

An important variation of this model is the case when _vr_ is fixed. This is sometimes referred to as the Dubins car, after Lester Dubins who derived the minimum time motion between to points with prescribed tangents [57]. Another notable variation is the Reeds-Shepp car for which minimum length paths are known when _vr_ takes a single forward and reverse speed [58]. These two models have proven to be of some importance to motion planning and will be discussed further in Section IV. 

The kinematic models are suitable for planning paths at low speeds (e.g. parking maneuvers and urban driving) where inertial effects are small in comparison to the limitations on mobility imposed by the no-slip assumption. A major drawback of this model is that it permits instantaneous steering angle changes which can be problematic if the motion planning module generates solutions with such instantaneous changes. 

Continuity of the steering angle can be imposed by augmenting (III.4), where the steering angle integrates a commanded rate as in [49]. Equation (III.4) becomes 



In addition to the limit on the steering angle, the steering rate can now be limited: _vδ ∈ δ_ ˙ _min, δ_ ˙ _max_ . The same problem � � can arise with the car’s speed _vr_ and can be resolved in the same way. The drawback to this technique is the increased dimension of the model which can complicate motion planning and control problems. 

The choice of coordinate system is not limited to using one of the wheel locations as a position coordinate. For models 

6 

derived using principles from classical mechanics it can be convenient to use the center of mass as the position coordinate as in [59], [60], or the center of oscillation as in [61], [62]. 

## _B. Inertial Effects_ 

When the acceleration of the vehicle is sufficiently large, the no-slip assumption between the tire and ground becomes invalid. In this case a more accurate model for the vehicle is as a rigid body satisfying basic momentum principles. That is, the acceleration is proportional to the force generated by the ground on the tires. Taking _pc_ to be the vehicles center of mass, and a coordinate of the configuration (cf. Figure III.2), the motion of the vehicle is governed by 



where _Fr_ and _Ff_ are the forces applied to the vehicle by the ground through the ground-tire interaction, _m_ is the vehicles total mass, and _Izz_ is the polar moment of inertia in the _e_ ˆ _z_ direction about the center of mass. In the following derivations we tacitly neglect the motion of _pc_ in the _e_ ˆ _z_ direction with the assumptions that the road is level, the suspension is rigid and vehicle remains on the road. 

The expressions for _Fr_ and _Ff_ vary depending on modeling assumptions [18], [59], [60], [62], but in any case the expression can be tedious to derive. Equations (III.10)-(III.15) therefore provide a detailed derivation as a reference. 

The force between the ground and tires is modeled as being dependent on the rate that the tire slips on the ground. Although the center of mass serves as a coordinate for the configuration, the velocity of each wheel relative to the ground is needed to determine this relative speed. The kinematic relations between these three points are 



These kinematic relations are used to determine the velocities of the point on each tire in contact with the ground, _sr_ and _sf_ . The velocity of these points are referred to as the tire slip velocity. In general, _sr_ and _sf_ differ from _p_ ˙ _r_ and _p_ ˙ _f_ through the angular velocity of the wheel. The kinematic relation is 



The angular velocities of the wheels are given by 



and _R_ = �0 _,_ 0 _, −r_<sup>�</sup><sup>_T_</sup> . The wheel radius is the scalar quantity _r_ , and Ω _{r,f }_ are the angular speeds of each wheel relative to the car. This is illustrated for the rear wheel in Figure III.3. 

Under static conditions, or when the height of the center of mass can be approximated as _pc ·_ ˆ _ez ≈_ 0, the component of the force normal to the ground, _F{r,f } ·_ ˆ _ez_ can be computed from a static force-torque balance as 



The normal force is then used to compute the traction force on each tire together with the slip and a friction coefficient model, _µ_ , for the tire behavior. The traction force on the rear tire is given component-wise by 



The same expression describes the front tire with the _r_ - subscript replaced by an _f_ -subscript. The formula above models the traction force as being anti-parallel to the slip with magnitude proportional to the normal force with a nonlinear 



Figure III.2: Illustration of single track model kinematics without the no-slip assumption. _ω{r,f }_ are relative angular velocities of the wheels with respect to the vehicle. 



Figure III.3: Illustration of the rear wheel kinematics _in two dimensions_ showing the wheel slip, _sr_ , in relation to the rear wheel velocity, _p_ ˙ _r_ , and angular speed, Ω _r_ . In general, _sr_ and _p_ ˙ _r_ are not collinear and may have nonzero components normal to the plane depicted. 

7 

dependence on the slip ratio (the magnitude of the slip normalized by Ω _rr_ for the rear and Ω _f r_ for the front). Combining (III.10)-(III.15) yields expressions for the net force on each wheel of the car in terms of the control variables, generalized coordinates, and their velocities. Equation (III.14), together with the following model for _µ_ , 



are a frequently used model for tire interaction with the ground. Equation (III.15) is a simplified version of the well known model due to Pacejka [63]. 

The rotational symmetry of (III.14) together with the peak in (III.15) lead to a maximum norm force that the tire can exert in any direction. This peak is referred to as the friction circle depicted in Figure III.4. 



Figure III.4: A zoomed in view of the wheel slip to traction force map at each tire (top) and a zoomed out view emphasizing the peak that defines the friction circle (bottom); cf. Equation (III.15). 

The models discussed in this section appear frequently in the literature on motion planning and control for driverless cars. They are suitable for the motion planning and control tasks discussed in this survey. However, lower level control tasks such as electronic stability control and active suspension systems typically use more sophisticated models for the chassis, steering and, drive-train. 

## IV. MOTION PLANNING 

The motion planning layer is responsible for computing a safe, comfortable, and dynamically feasible trajectory from the vehicle’s current configuration to the goal configuration provided by the behavioral layer of the decision making hierarchy. Depending on context, the goal configuration may differ. For example, the goal location may be the center point of the current lane a number of meters ahead in the direction 

of travel, the center of the stop line at the next intersection, or the next desired parking spot. The motion planning component accepts information about static and dynamic obstacles around the vehicle and generates a collision-free trajectory that satisfies dynamic and kinematic constraints on the motion of the vehicle. Oftentimes, the motion planner also minimizes a given objective function. In addition to travel time, the objective function may penalize hazardous motions or motions that cause passenger discomfort. In a typical setup, the output of the motion planner is then passed to the local feedback control layer. In turn, feedback controllers generate an input signal to regulate the vehicle to follow this given motion plan. 

A motion plan for the vehicle can take the form of a path or a trajectory. Within the path planning framework, the solution path is represented as a function _σ_ ( _α_ ) : [0 _,_ 1] _→X_ , where _X_ is the configuration space of the vehicle. Note that such a solution does not prescribe how this path should be followed and one can either choose a velocity profile for the path or delegate this task to lower layers of the decision hierarchy. Within the trajectory planning framework, the control execution time is explicitly considered. This consideration allows for direct modeling of vehicle dynamics and dynamic obstacles. In this case, the solution trajectory is represented as a timeparametrized function _π_ ( _t_ ) : [0 _, T_ ] _→X_ , where _T_ is the planning horizon. Unlike a path, the trajectory prescribes how the configuration of the vehicle evolves over time. 

In the following two sections, we provide a formal problem definition of the path planning and trajectory planning problems and review the main complexity and algorithmic results for both formulations. 

## _A. Path Planning_ 

The path planning problem is to find a path _σ_ ( _α_ ) : [0 _,_ 1] _→ X_ in the configuration space _X_ of the vehicle (or more generally, a robot) that starts at the initial configuration and reaches the goal region while satisfying given global and local constraints. Depending on whether the quality of the solution path is considered, the terms _feasible_ and _optimal_ are used to describe this path. Feasible path planning refers to the problem of determining a path that satisfies some given problem constraints without focusing on the quality of the solution; whereas optimal path planning refers to the problem of finding a path that optimizes some quality criterion subject to given constraints. 

The optimal path planning problem can be formally stated as follows. Let _X_ be the configuration space of the vehicle and let Σ( _X_ ) denote the set of all continuous functions [0 _,_ 1] _→X_ . The initial configuration of the vehicle is **x** init _∈X_ . The path is required to end in a goal region _X_ goal _⊆X_ . The set of all allowed configurations of the vehicle is called the free configuration space and denoted _X_ free. Typically, the free configurations are those that do not result in collision with obstacles, but the free-configuration set can also represent other holonomic constraints on the path. The differential constraints on the path are represented by a predicate _D_ ( **x** _,_ **x**<sup>_′_</sup> _,_ **x**<sup>_′′_</sup> _, . . ._ ) and can be used to enforce some degree of smoothness of the path for the vehicle, such as the bound on the path curvature and/or 

8 

the rate of curvature. For example, in the case of _X ⊆_ R<sup>2</sup> , the differential constraint may enforce the maximum curvature _κ_ of the path using Frenet-Serret formula as follows: 



Further, let _J_ ( _σ_ ) : Σ( _X_ ) _→_ R be the cost functional. Then, the optimal version of the path planning problem can be generally stated as follows. 

**Problem IV.1** (Optimal path planning) **.** Given a 5-tuple ( _X_ free _,_ **x** init _, X_ goal _, D, J_ ) find _σ_<sup>_∗_</sup> = 



The problem of feasible and optimal path planning has been studied extensively in the past few decades. The complexity of this problem is well understood, and many practical algorithms have been developed. 

_Complexity:_ A significant body of literature is devoted to studying the complexity of motion planning problems. The following is a brief survey of some of the major results regarding the computational complexity of these problems. 

The problem of finding an optimal path subject to holonomic and differential constraints as formulated in Problem IV.1 is known to be PSPACE-hard [64]. This means that it is at least as hard as solving any NP-complete problem and thus, assuming P _̸_ = NP, there is no efficient (polynomialtime) algorithm able to solve all instances of the problem. Research attention has since been directed toward studying approximate methods, or approaches to subsets of the general motion planning problem. 

Initial research focused primarily on feasible (i.e., nonoptimal) path planning for a holonomic vehicle model in polygonal/polyhedral environments. That is, the obstacles are assumed to be polygons/polyhedra and there are no differential constraints on the resulting path. In 1970, Reif [64] found that an obstacle-free path for a holonomic vehicle, whose footprint can be described as a single polyhedron, can be found in polynomial time in both 2-D and 3-D environments. Canny [65] has shown that the problem of feasible path planning in a free space represented using polynomials is in PSPACE, which rendered the decision version of feasible path planning without differential constraints as a PSPACEcomplete problem. 

For the optimal planning formulation, where the objective is to find the _shortest_ obstacle-free path. It has been long known that a shortest path for a holonomic vehicle in a 2- D environment with polygonal obstacles can be found in polynomial time [66], [67]. More precisely, it can be computed in time _O_ ( _n_<sup>2</sup> ), where _n_ is the number of vertices of the polygonal obstacles [68]. This can be solved by constructing and searching the so-called visibility graph [69]. In contrast, Lazard, Reif and Wang [70] established that the problem of finding a shortest curvature-bounded path in a 2-D plane 

amidst polygonal obstacles (i.e., a path for a car-like robot) is NP-hard, which suggests that there is no known polynomial time algorithm for finding a shortest path for a car-like robot among polygonal obstacles. A related result is that, that the existence of a curvature constrained path in polygonal environment can be decided in EXPTIME [71]. 

A special case where a solution can be efficiently computed is the shortest curvature bounded path in an obstacle free environment. Dubins [57] has shown that the shortest path having curvature bounded by _κ_ between given two points _p_ 1 _, p_ 2 and with prescribed tangents _θ_ 1 _, θ_ 2 is a curve consisting of at most three segments, each one being either a circular arc segment or a straight line. Reeds and Shepp [58] extended the method for a car that can move both forwards and backwards. Another notable case due to Agraval et al. [72] is an _O_ ( _n_<sup>2</sup> log _n_ ) algorithm for finding a shortest path with bounded curvature inside a convex polygon. Similarly, Boissonnat and Lazard [73] proposed a polynomial time algorithm for finding an exact curvature-bounded path in environments where obstacles have bounded-curvature boundary. 

Since for most problems of interest in autonomous driving, exact algorithms with practical computational complexity are unavailable [70], one has to resort to more general, numerical solution methods. These methods generally do not find an exact solution, but attempt to find a satisfactory solution or a sequence of feasible solutions that converge to the optimal solution. The utility and performance of these approaches are typically quantified by the class of problems for which they are applicable as well as their guarantees for converging to an optimal solution. The numerical methods for path planning can be broadly divided in three main categories: 

_Variational methods_ represent the path as a function parametrized by a finite-dimensional vector and the optimal path is sought by optimizing over the vector parameter using non-linear continuous optimization techniques. These methods are attractive for their rapid convergence to _locally_ optimal solutions; however, they typically lack the ability to find globally optimal solutions unless an appropriate initial guess in provided. For a detailed discussion on variational methods, see Section IV-C. 

_Graph-search methods_ discretize the configuration space of the vehicle as a graph, where the vertices represent a finite collection of vehicle configurations and the edges represent transitions between vertices. The desired path is found by performing a search for a minimum-cost path in such a graph. Graph search methods are not prone to getting stuck in local minima, however, they are limited to optimize only over a finite set of paths, namely those that can be constructed from the atomic motion primitives in the graph. For a detailed discussion about graph search methods, see Section IV-D. 

_Incremental search methods_ sample the configuration space and incrementally build a reachability graph (oftentimes a tree) that maintains a discrete set of reachable configurations and feasible transitions between them. Once the graph is large enough so that at least one node is in the goal region, the desired path is obtained by tracing the edges that lead to that node from the start configuration. In contrast to more basic graph search methods, sampling-based methods incrementally 

9 

increase the size of the graph until a satisfactory solution is found within the graph. For a detailed discussion about incremental search methods, see Section IV-E. 

Clearly, it is possible to exploit the advantages of each of these methods by combining them. For example, one can use a coarse graph search to obtain an initial guess for the variational method as reported in [74] and [75]. A comparison of key properties of select path planning methods is given in Table I. In the remainder of this section, we will discuss the path planning algorithms and their properties in detail. 

## _B. Trajectory Planning_ 

The motion planning problems in dynamic environments or with dynamic constraints may be more suitably formulated in the trajectory planning framework, in which the solution of the problem is a trajectory, i.e. a time-parametrized function _π_ ( _t_ ) : [0 _, T_ ] _→X_ prescribing the evolution of the configuration of the vehicle in time. 

Let Π( _X , T_ ) denote the set of all continuous functions [0 _, T_ ] _→X_ and _x_ init _∈X_ . be the initial configuration of the vehicle. The goal region is _X_ goal _⊆X_ . The set of all allowed configurations at time _t ∈_ [0 _, T_ ] is denoted as _X_ free( _t_ ) and used to encode holonomic constraints such as the requirement on the path to avoid collisions with static and, possibly, dynamic obstacles. The differential constraints on the trajectory are represented by a predicate _D_ ( **x** _,_ **x**<sup>_′_</sup> _,_ **x**<sup>_′′_</sup> _, . . ._ ) and can be used to enforce dynamic constraints on the trajectory. Further, let _J_ ( _π_ ) : Π( _X , T_ ) _→_ R be the cost functional. Under these assumptions, the optimal version of the trajectory planning problem can be very generally stated as: 

**Problem IV.2** (Optimal trajectory planning) **.** Given a 6-tuple ( _X_ free _,_ **x** init _, X_ goal _, D, J, T_ ) find _π_<sup>_∗_</sup> = 



_Complexity:_ Since trajectory planning in a dynamic environment is a generalization of path planning in static environments, the problem remains PSPACE-hard. Moreover, trajectory planning in dynamic environments has been shown to be harder than path planning in the sense that some variants of the problem that are tractable in static environments become intractable when an analogical problem is considered in a dynamic environment. In particular, recall that a shortest path for a point robot in a static 2-D polygonal environment can be found efficiently in polynomial time and contrast it with the result of Canny and Reif [76] establishing that finding velocitybounded collision-free trajectory for a holonomic point robot amidst moving polygonal obstacles<sup>1</sup> is NP-hard. Similarly, while path planning for a robot with a fixed number of degrees 

> 1In fact, the authors considered an even more constrained _2-D asteroid avoidance problem_ , where the task is to find a collision-free trajectory for a point robot with a bounded velocity in a 2-D plane with convex polygonal obstacles moving with a fixed linear speed. 

of freedom in 3-D polyhedral environments is tractable, Reif and Sharir [85] established that trajectory planning for robot with 2 degrees of freedom among translating and rotating 3-D polyhedral obstacles is PSPACE-hard. 

Tractable exact algorithms are not available for non-trivial trajectory planning problems occurring in autonomous driving, making the numerical methods a popular choice for the task. Trajectory planning problems can be numerically solved using some variational methods directly in the time domain or by converting the trajectory planning problem to path planning in a configuration space with an added time-dimension [86]. 

The conversion from a trajectory planning problem ( _X_ free<sup>_T,_</sup><sup>**x**</sup> init<sup>_T, X_</sup> goal<sup>_T, DT , JT , T_)toapathplanningproblem</sup> ( _X_ free<sup>_P,_</sup><sup>**x**</sup> init<sup>_P, X_</sup> goal<sup>_P, DP , JP_)isusuallydoneasfollows.The</sup> configuration space where the path planning takes place is defined as _X_<sup>_P_</sup> := _X_<sup>_T_</sup> _×_ [0 _, T_ ]. For any **y** _∈X_<sup>_P_</sup> , let _t_ ( **y** ) _∈_ [0 _, T_ ] denote the time component and _c_ ( **y** ) _∈X_<sup>_T_</sup> denote the "configuration" component of the point **y** . A path _σ_ ( _α_ ) : [0 _,_ 1] _→X_<sup>_P_</sup> can be converted to a trajectory _π_ ( _t_ ) : [0 _, T_ ] _→X_<sup>_T_</sup> if the time component at start and end point of the path is constrained as 



and the path is monotonically-increasing, which can be enforced by a differential constraint 



Further, the free configuration space, initial configuration, goal region and differential constraints is mapped to their path planning counterparts as follows: 



A solution to such a path planning problem is then found using a path planning algorithm that can handle differential constraints and converted back to the trajectory form. 

## _C. Variational Methods_ 

We will first address the trajectory planning problem in the framework of non-linear continuous optimization. In this context, the problem is often referred to as trajectory optimization. Within this subsection we will adopt the trajectory planning formulation with the understanding that doing so does not affect generality since path planning can be formulated as trajectory optimization over the unit time interval. To leverage existing nonlinear optimization methods, it is necessary to project the infinite-dimensional function space of trajectories to a finite-dimensional vector space. In addition, most nonlinear programming techniques require the trajectory optimization problem, as formulated in Problem IV.2, to be 

10 

||Model assumptions|Completeness|Optimality|Time Complexity|Anytime|
|---|---|---|---|---|---|
|||Geometric Metho|ds|||
|Visibility graph [33]|2-D polyg. conf.<br>space,<br>no diff. constraints|Yes|Yes <sup>_a_</sup>|_O_(_n_<sup>2</sup>) [68] <sup>_b_</sup>|No|
|Cyl. algebr. decomp. [76]|No diff. constraints|Yes|No|Exp. in dimension. [65]|No|
|||Variational Meth|ods|||
|Variational methods<br>(Sec IV-C)|Lipschitz-continuous<br>Jacobian|No|Locally optimal|_O_(1_/ϵ_) [77] <sup>_k,l_</sup>|Yes|
|||Graph-search Met|hods|||
|Road lane graph +<br>Dijkstra (Sec IV-D1)|Arbitrary|No <sup>_c_</sup>|No <sup>_d_</sup>|_O_(_n_+_m_log_m_) [78] <sup>_e,f_</sup>|No|
|Lattice/tree of motion prim.<br>+ Dijkstra (Sec IV-E)|Arbitrary|No <sup>_c_</sup>|No <sup>_d_</sup>|_O_(_n_+_m_log_m_) [78] <sup>_e,f_</sup>|No|
|PRM [79] <sup>_g_ </sup>+ Dijkstra|Exact steering<br>procedure available|Probabilistically<br>complete [80] <sup>_∗_</sup>|Asymptotically<br>optimal <sup>_∗_</sup>[80]|_O_(_n_<sup>2</sup>) [80] <sup>_h,f,∗_</sup>|No|
|PRM* [80], [81] + Dijkstra|Exact steering<br>procedure available|Probabilistically<br>complete<br>[80]–[82] <sup>_∗,†_</sup>|Asymptotically<br>optimal<br>[80]–[82] <sup>_∗,†_</sup>|_O_(_n_ log_n_) [80],<br>[82] <sup>_h,f,∗,†_</sup>|No|
|RRG [80] + Dijkstra|Exact steering<br>procedure available|Probabilistically<br>complete [80] <sup>_∗_</sup>|Asymptotically<br>optimal [80] <sup>_∗_</sup>|_O_(_n_ log_n_) [80] <sup>_h,f,∗_</sup>|Yes|
|||Incremental Sear|ch|||
|RRT [83]|Arbitrary|Probabilistically<br>complete [83] <sup>_i,∗_</sup>|Suboptimal [80] <sup>_∗_</sup>|_O_(_n_ log_n_) [80] <sup>_h,f,∗_</sup>|Yes|
|RRT* [80]|Exact steering<br>procedure available|Probabilistically<br>complete [80],<br>[82] <sup>_∗,†_</sup>|Asymptotically<br>optimal [80],<br>[82] <sup>_∗,†_</sup>|_O_(_n_ log_n_) [80],<br>[82] <sup>_h,f,∗,†_</sup>|Yes|
|SST* [84]|Lipschitz-continuous<br>dynamics|Probabilistically<br>complete [84] <sup>_†_</sup>|Asymptotically<br>optimal [84] <sup>_†_</sup>|N/A <sup>_j_</sup>|Yes|



Table I: Comparison of path planning methods. **Legend:** _a_ : for the shortest path problem; _b_ : _n_ is the number of points defining obstacles; _c_ : complete only w.r.t. the set of paths induced by the given graph; _d_ : optimal only w.r.t. the set of paths induced by the given graph; _e_ : _n_ and _m_ are the number of edges and vertices in the graph respectively; _f_ : assuming _O_ (1) collision checking; _g_ : batch version with fixed-radius connection strategy; _h_ : _n_ is the number of samples/algorithm iterations; _i_ : for certain variants; _j_ : not explicitly analyzed; _k_ : _ϵ_ is the required distance from the optimal cost; _l_ : faster rates possible with additional assumptions; _∗_ : shown for systems without differential constraints; _†_ : shown for some class of nonholonomic systems. 

## converted into the following form 



where the holonomic and differential constraints are represented as a system of equality and inequality constraints. 

In some applications the constrained optimization problem is relaxed to an unconstrained one using penalty or barrier functions. In both cases, the constraints are replaced by an augmented cost functional. With the penalty method, the cost functional takes the form 



Similarly, barrier functions can be used in place of inequality constraints. The augmented cost functional in this case takes 



where the barrier function satisfies _g_ ( _π_ ) _<_ 0 _⇒ h_ ( _π_ ) _< ∞_ , _g_ ( _π_ ) _≥_ 0 _⇒ h_ ( _π_ ) = _∞_ , and lim _g_ ( _π_ ) _→_ 0 _{h_ ( _π_ ) _}_ = _∞_ . The intuition behind both of the augmented cost functionals is that, by making _ε_ small, minima in cost will be close to minima of the original cost functional. An advantage of barrier functions is that local minima remain feasible, but must be initialized with a feasible solution to have finite augmented cost. Penalty methods on the other hand can be initialized with any trajectory and optimized to a local minima. However, local minima may violate the problem constraints. A variational formulation using barrier functions is proposed in [60] where a change of coordinates is used to convert the constraint that the vehicle remain on the road into a linear constraint. A logarithmic barrier is used with a Newton-like method in a similar fashion to interior point methods. The approach effectively computes minimum time trajectories for a detailed vehicle model over a segment of roadway. 

11 

Next, two subclasses of variational methods are discussed: Direct and indirect methods. 

_Direct Methods:_ A general principle behind direct variational methods is to restrict the approximate solution to a finite-dimensional subspace of Π( _X , T_ ). To this end, it is usually assumed that 



where _πi_ is a coefficient from R, and _φi_ ( _t_ ) are _basis_ functions of the chosen subspace. A number of numerical approximation schemes have proven useful for representing the trajectory optimization problem as a nonlinear program. We mention here the two most common schemes: Numerical integrators with collocation and pseudospectral methods. 

_1) Numerical Integrators with Collocation:_ With collocation, it is required that the approximate trajectory satisfies the constraints in a set of discrete points _{tj}_<sup>_M_</sup> _j_ =1<sup>.This</sup> requirement results in two systems of discrete constraints: A system of nonlinear equations which approximates the system dynamics 



and a system of nonlinear inequalities which approximates the state constraints placed on the trajectory 



Numerical integration techniques are used to approximate the trajectory between the collocation points. For example, a piecewise linear basis 



together with collocation gives rise to the Euler integration method. Higher order polynomials result in the Runge-Kutta family of integration methods. Formulating the nonlinear program with collocation and Euler’s method or one of the RungeKutta methods is more straightforward than some other methods making it a popular choice. An experimental system which successfully uses Euler’s method for numerical approximation of the trajectory is presented in [14]. 

In contrast to Euler’s method, the Adams approximation, is investigated in [87] for optimizing the trajectory for a detailed vehicle model and is shown to provide improved numerical accuracy and convergence rates. 

_2) Pseudospectral Methods:_ Numerical integration techniques utilize a discretization of the time interval with an interpolating function between collocation points. Pseudospectral approximation schemes build on this technique by additionally representing the interpolating function with a basis. Typical basis functions interpolating between collocation points are finite subsets of the Legendre or Chebyshev polynomials. These methods typically have improved convergence rates over basic collocation methods, which is especially true when adaptive methods for selecting collocation points and basis functions are used as in [88]. 

_Indirect Methods:_ Pontryagin’s minimum principle [89], is a celebrated result from optimal control which provides optimality conditions of a solution to Problem IV.2. Indirect methods, as the name suggests, solve the problem by finding solutions satisfying these optimality conditions. These optimality conditions are described as an augmented system of ordinary differential equations (ODEs) governing the states and a set of co-states. However, this system of ODEs results in a two point boundary value problem and can be difficult to solve numerically. One technique is to vary the free initial conditions of the problem and integrate the system forward in search of the initial conditions which leads to the desired terminal states. This method is known as the _shooting_ method, and a version of this approach has been applied to planning parking maneuvers in [90]. The advantage of indirect methods, as in the case of the shooting method, is the reduction in dimensionality of the optimization problem to the dimension of the state space. 

The topic of variational approaches is very extensive and hence, the above is only a brief description of select approaches. See [91], [92] for dedicated surveys on this topic. 

## _D. Graph Search Methods_ 

Although useful in many contexts, the applicability of variational methods is limited by their convergence to only local minima. In this section, we will discuss the class of methods that attempts to mitigate the problem by performing global search in the discretized version of the path space. These socalled graph search methods discretize the configuration space _X_ of the vehicle and represent it in the form of a graph and then search for a minimum cost path on such a graph. 

In this approach, the configuration space is represented as a graph _G_ = ( _V, E_ ), where _V ⊂X_ is a discrete set of selected configurations called vertices and _E_ = _{_ ( _oi, di, σi_ ) _}_ is the set of edges, where _oi ∈ V_ represents the origin of the edge, _di_ represents the destination of the edge and _σi_ represents the path segments connecting _oi_ and _di_ . It is assumed that the path segment _σi_ connects the two vertices: _σi_ (0) = _oi_ and _σi_ (1) = _di._ Further, it is assumed that the initial configuration **x** init is a vertex of the graph. The edges are constructed in such a way that the path segments associated with them lie completely in _X_ free and satisfy differential constraints. As a result, any path on the graph can be converted to a feasible path for the vehicle by concatenating the path segments associated with edges of the path through the graph. 

There is a number of strategies for constructing a graph discretizing the free configuration space of a vehicle. In the following subsections, we discuss three common strategies: Hand-crafted lane graphs, graphs derived from geometric representations and graphs constructed by either control or configuration sampling. 

_1) Lane Graph:_ When the path planning problem involves driving on a structured road network, a sufficient graph discretization may consist of edges representing the path that the car should follow within each lane and paths that traverse intersections. 

Road lane graphs are often partly algorithmically generated 

12 

from higher-level street network maps and partly human edited. An example of such a graph is in Figure IV.1. 



Figure IV.1: Hand-crafted graph representing desired driving paths under normal circumstances. 

Although most of the time it is sufficient for the autonomous vehicle to follow the paths encoded in the road lane graph, occasionally it must be able to navigate around obstacles that were not considered when the road network graph was designed or in environments not covered by the graph. Consider for example a faulty vehicle blocking the lane that the vehicle plans to traverse – in such a situation a more general motion planning approach must be used to find a collision-free path around the detected obstacle. 

The general path planning approaches can be broadly divided into two categories based on how they represent the obstacles in the environment. So-called geometric or combinatorial methods work with geometric representations of the obstacles, where in practice the obstacles are most commonly described using polygons or polyhedra. On the other hand, socalled sampling-based methods abstract away from how the obstacles are internally represented and only assumes access to a function that determines if any given path segment is in collision with any of the obstacles. 

_2) Geometric Methods:_ In this section, we will focus on path planning methods that work with geometric representations of obstacles. We will first concentrate on path planning without differential constraints because for this formulation, efficient exact path planning algorithms exist. Although not being able to enforce differential constraints is limiting for path planning for traditionally-steered cars because the constraint on minimum turn radius cannot be accounted for, these methods can be useful for obtaining the lower- and upperbounds<sup>2</sup> on the length of a curvature-constrained path and for path planning for more exotic car constructions that can turn on the spot. 

In path planning, the term roadmap is used to describe a graph discretization of _X_ free that describes well the connectivity of the free configuration space and has the property that any point in _X_ free is trivially reachable from some vertices of the roadmap. When the set _X_ free can be described geometrically 

> 2Lower bound is the length of the path without curvature constraint, upper bound is the length of a path for a large robot that serves as an envelope within which the car can turn in any direction. 

using a linear or semi-algebraic model, different types of roadmaps for _X_ free can be algorithmically constructed and subsequently used to obtain complete path planning algorithms. Most notably, for _X_ free _⊆_ R<sup>2</sup> and polygonal models of the configuration space, several efficient algorithms for constructing such roadmaps exists such as the vertical cell decomposition [93], generalized Voronoi diagrams [94], [95], and visibility graphs [33], [96]. For higher dimensional configuration spaces described by a general semi-algebraic model, the technique known as cylindrical algebraic decomposition can be used to construct a roadmap in the configuration space [47], [97] leading to complete algorithms for a very general class of path planning problems. The fastest of this class is an algorithm developed by Canny [65] that has (single) exponential time complexity in the dimension of the configuration space. The result is however mostly of a theoretical nature without any known implementation to date. 

Due to its relevance to path planning for car-like vehicles, a number of results also exist for the problem of path planning with a constraint on maximum curvature. Backer and Kirkpatrick [98] provide an algorithm for constructing a path with bounded curvature that is polynomial in the number of features of the domain, the precision of the input and the number of segments on the simplest obstacle-free Dubins path connecting the specified configurations. Since the problem of finding a _shortest_ path with bounded curvature amidst polygonal obstacles is NP-hard, it is not surprising that no exact polynomial solution algorithm is known. An approximation algorithm for finding shortest curvature-bounded path amidst polygonal obstacles has been first proposed by Jacobs and Canny [99] and later improved by Wang and Agarwal [100] with time complexity _O_ (<sup>_<u>n</u>_</sup> _ϵ_<sup>42log</sup><sup>_n_),where</sup><sup>_n_isthenumberof</sup> vertices of the obstacles and _ϵ_ is the approximation factor. For the special case of so-called moderate obstacles that are characterized by smooth boundary with curvature bounded by _κ_ , an exact polynomial algorithm for finding a path with curvature bounded by at most _κ_ have been developed by Boissonnat and Lazard [73]. 

_3) Sampling-based Methods:_ In autonomous driving, a geometric model of _X_ free is usually not directly available and it would be too costly to construct from raw sensoric data. Moreover, the requirements on the resulting path are often far more complicated than a simple maximum curvature constraint. This may explain the popularity of sampling-based techniques that do not enforce a specific representation of the free configuration set and dynamic constraints. Instead of reasoning over a geometric representation, the sampling based methods explore the reachability of the free configuration space using _steering_ and _collision checking_ routines: 

The steering function steer( **x** _,_ **y** ) returns a path segment starting from configuration **x** going towards configuration **y** (but not necessarily reaching **y** ) ensuring the differential constraints are satisfied, i.e., the resulting motion is feasible for the vehicle model in consideration. The exact manner in which the steering function is implemented depends on the context in which it is used. Some typical choices encountered in the literature are: 

1) Random steering: The function returns a path that results 

13 

from applying a random control input through a forward model of the vehicle from state **x** for either a fixed or variable time step [101]. 

- 2) Heuristic steering: The function returns a path that results from applying control that is heuristically constructed to guide the system from **x** towards **y** [102]– [104]. This includes selecting the maneuver from a predesigned discrete set (library) of maneuvers. 

- 3) Exact steering: The function returns a feasible path that guides the system from **x** to **y** . Such a path corresponds to a solution of a 2-point boundary value problem. For some systems and cost functionals, such a path can be obtained analytically, e.g., a straight line for holonomic systems, a Dubins curve for forward-moving unicycle [57], or a Reeds-Shepp curve for bi-directional unicycle [58]. An analytic solution also exists for differentially flat systems [61], while for more complicated models, the exact steering can be obtained by solving the twopoint boundary value problem. 

- 4) Optimal exact steering: The function returns an optimal exact steering path with respect to the given cost functional. In fact, the straight line, the Dubins curve, and the Reeds-Shepp curve from the previous point are optimal solutions assuming that the cost functional is the arc-length of the path [57], [58]. 

The collision checking function col-free( _σ_ ) returns true if path segment _σ_ lies entirely in _X_ free and it is used to ensure that the resulting path does not collide with any of the obstacles. 

Having access to steering and collision checking functions, the major challenge becomes how to construct a discretization that approximates well the connectivity of _X_ free without having access to an explicit model of its geometry. We will now review sampling-based discretization strategies from literature. 

A straightforward approach is to choose a set of motion primitives (fixed maneuvers) and generate the search graph by recursively applying them starting from the vehicle’s initial configuration **x** init, e.g., using the method in Algorithm 1. For path planning without differential constraints, the motion primitives can be simply a set of straight lines with different directions and lengths. For a car-like vehicle, such motion primitive might by a set of arcs representing the path the car would follow with different values of steering. A variety of techniques can be used for generating motion primitives for driverless vehicles. A simple approach is to sample a number of control inputs and to simulate forwards in time using a vehicle model to obtain feasible motions. In the interest of having continuous curvature paths, clothoid segments are also sometimes used [105]. The motion primitives can be also obtained by recording the motion of a vehicle driven by an expert driver [106]. 

Observe that the recursive application of motion primitives may generate a tree graph in which in the worst-case no two edges lead to the same configuration. There are, however, sets of motion primitives, referred to as lattice-generating, that result in regular graphs resembling a lattice. See Figure IV.2a for an illustration. The advantage of lattice generating primitives is that the vertices of the search graph cover the configuration 

space uniformly, while trees in general may have a high density of vertices around the root vertex. Pivtoraiko et al. use the term "state lattice" to describe such graphs in [107] and point out that a set of lattice-generating motion primitives for a system in hand can be obtained by first generating regularly spaced configurations around origin and then connecting the origin to such configurations by a path that represents the solution to the two-point boundary value problem between the two configurations. 

## **Algorithm 1:** Recursive Roadmap Construction 



An effect that is similar to recursive application of latticegenerating motion primitives from the initial configuration can be achieved by generating a discrete set of samples covering the (free) configuration space and connecting them by feasible path segments obtained using an exact steering procedure. 

Most sampling-based roadmap construction approaches follow the algorithmic scheme shown in Algorithm 2, but differ in the implementation of the sample-points( _X , n_ ) and neighbors( **x** _, V_ ) routines. The function samplepoints( _X , n_ ) represents the strategy for selecting _n_ points from the configuration space _X_ , while the function neighbors( **x** _, V_ ) represents the strategy for selecting a set of neighboring vertices _N ⊆ V_ for a vertex **x** , which the algorithm will attempt to connect to **x** by a path segment using an exact steering function, steerexact( **x** _,_ **y** ). 

## **Algorithm 2:** Sampling-based Roadmap Construction 



The two most common implementations of samplepoints( _X , n_ ) function are 1) return _n_ points arranged in a regular grid and 2) return _n_ randomly sampled points from _X_ . While random sampling has an advantage of being generally applicable and easy to implement, so-called Sukharev grids 

14 





<!-- Start of picture text -->
(a) Lattice graph<br><!-- End of picture text -->





<!-- Start of picture text -->
(b) Non-lattice graph<br><!-- End of picture text -->

Figure IV.2: Lattice and non-lattice graph, both with 5000 edges. (a) The graph resulting from recursive application of 90<sup>_◦_</sup> left circular arc, 90<sup>_◦_</sup> right circular arc, and a straight line. (b) The graph resulting from recursive application of 89<sup>_◦_</sup> left circular arc, 89<sup>_◦_</sup> right circular arc, and a straight line. The recursive application of those primitives does form a tree instead of a lattice with many branches looping in the neighborhood of the origin. As a consequence, the area covered by the right graph is smaller. 

have been shown to achieve optimal _L∞_ -dispersion in unit hypercubes, i.e. they minimize the radius of the largest empty ball with no sample point inside. For in depth discussion of relative merits of random and deterministic sampling in the context of sampling-based path planning, we refer the reader to [108]. The two most commonly used strategies for implementing neighbors( **x** _, V_ ) function are to take 1) the set of _k_ -nearest neighbors to **x** or 2) the set of points lying within the ball centered at **x** with radius _r_ . 

In particular, samples arranged deterministically in a _d_ - dimensional grid with the neighborhood taken as 4 or 8 nearest neighbors in 2-D or the analogous pattern in higher dimensions represents a straightforward deterministic discretization of the free configuration space. This is in part because they arise naturally from widely used bitmap representations of free and occupied regions of robots’ configuration space [109]. 

Kavraki et al. [110] advocate the use of random sampling within the framework of Probabilistic Roadmaps (PRM) in order to construct roadmaps in high-dimensional configuration spaces, because unlike grids, they can be naturally run in an anytime fashion. The batch version of PRM [79] follows the scheme in Algorithm 2 with random sampling and neighbors selected within a ball with fixed radius _r_ . Due to the general formulation of PRMs, they have been used for path planning for a variety of systems, including systems with differential constraints. However, the theoretical analyses of the algorithm have primarily been focused on the performance of the algorithm for systems without differential constraints, i.e. when a straight line is used to connect two configurations. Under such an assumption, PRMs have been shown in [80] to be probabilistically complete and asymptotically optimal. That is, the probability that the resulting graph contains a valid solution (if it exists) converges to one with increasing size of the graph and the cost of the shortest path in the graph converges to the optimal cost. Karaman and Frazzoli [80] proposed an adapta- 

tion of batch PRM, called PRM*, that instead only connects neighboring vertices in a ball with a logarithmically shrinking radius with increasing number of samples to maintain both asymptotic optimality and computational efficiency. 

In the same paper, the authors propose Rapidly-exploring Random Graphs (RRG*), which is an incremental discretization strategy that can be terminated at any time while maintaining the asymptotic optimality property. Recently, Fast Marching Tree (FMT*) [111] has been proposed as an asymptotically optimal alternative to PRM*. The algorithm combines discretization and search into one process by performing a lazy dynamic programming recursion over a set of sampled vertices that can be subsequently used to quickly determine the path from initial configuration to the goal region. 

Recently, the theoretical analysis has been extended also to differentially constrained systems. Schmerling et al. [81] propose differential versions of PRM* and FMT* and prove asymptotic optimality of the algorithms for driftless controlaffine dynamical systems, a class that includes models of nonslipping wheeled vehicles. 

_4) Graph Search Strategies:_ In the previous section, we have discussed techniques for the discretization of the free configuration space in the form of a graph. To obtain an actual optimal path in such a discretization, one must employ one of the graph search algorithms. In this section, we are going to review the graph search algorithms that are relevant for path planning. 

The most widely recognized algorithm for finding shortest paths in a graph is probably the Dijkstra’s algorithm [32]. The algorithm performs the best first search to build a tree representing shortest paths from a given source vertex to all other vertices in the graph. When only a path to a single vertex is required, a heuristic can be used to guide the search process. The most prominent heuristic search algorithm is A* developed by Hart, Nilsson and Raphael [112]. If the provided 

15 

heuristic function is admissible (i.e., it never overestimates the cost-to-go), A* has been shown to be optimally efficient and is guaranteed to return an optimal solution. For many problems, a bounded suboptimal solution can be obtained with less computational effort using Weighted A* [113], which corresponds to simply multiplying the heuristic by a constant factor _ϵ >_ 1. It can be shown that the solution path returned by A* with such an inflated heuristics is guaranteed to be no worse than (1 + _ϵ_ ) times the cost of an optimal path. 

Often, the shortest path from the vehicle’s current configuration to the goal region is sought repeatedly every time the model of the world is updated using sensory data. Since each such update usually affects only a minor part of the graph, it might be wasteful to run the search every time completely from scratch. The family of real-time replanning search algorithms such as D* [114], Focussed D* [115] and D* Lite [116] has been designed to efficiently recompute the shortest path every time the underlying graph changes, while making use of the information from previous search efforts. 

Anytime search algorithms attempt to provide a first suboptimal path quickly and continually improve the solution with more computational time. Anytime A* [117] uses a weighted heuristic to find the first solution and achieves the anytime behavior by continuing the search with the cost of the first path as an upper bound and the admissible heuristic as a lower bound, whereas Anytime Repairing A* (ARA*) [118] performs a series of searches with inflated heuristic with decreasing weight and reuses information from previous iterations. On the other hand, Anytime Dynamic A* (ADA*) [119] combines ideas behind D* Lite and ARA* to produce an anytime search algorithm for real-time replanning in dynamic environments. 

A clear limitation of algorithms that search for a path on a graph discretization of the configuration space is that the resulting optimal path on such graph may be significantly longer than the true shortest path in the configuration space. Any-angle path planning algorithms [120]–[122] are designed to operate on grids, or more generally on graphs representing cell decomposition of the free configuration space, and try to mitigate this shortcoming by considering "shortcuts" between the vertices on the graph during search. In addition, Field D* introduces linear-interpolation to the search procedure to produce smooth paths [123]. 

## _E. Incremental Search Techniques_ 

A disadvantage of the techniques that search over a fixed graph discretization is that they search only over the set of paths that can be constructed from primitives in the graph discretization. Therefore, these techniques may fail to return a feasible path or return a noticeably suboptimal one. 

The incremental _feasible_ motion planners strive to address this problem and provide a feasible path to any motion planning problem instance, if one exists, given enough computation time. Typically, these methods incrementally build increasingly finer discretization of the configuration space while concurrently attempting to determine if a path from initial configuration to the goal region exists in the discretization at each step. If the instance is “easy”, the solution is 

provided quickly, but in general the computation time can be unbounded. Similarly, incremental _optimal_ motion planning approaches on top of finding a feasible path fast attempt to provide a sequence of solutions of increasing quality that converges to an optimal path. 

The term _probabilistically complete_ is used in the literature to describe algorithms that find a solution, if one exists, with probability approaching one with increasing computation time. Note that probabilistically complete algorithm may not terminate if the solution does not exist. Similarly, the term _asymptotically optimal_ is used for algorithms that converge to optimal solution with probability one. 

A naïve strategy for obtaining completeness and optimality in the limit is to solve a sequence of path planning problems on a fixed discretization of the configuration space, each time with a higher resolution of the discretization. One disadvantage of this approach is that the path planning processes on individual resolution levels are independent without any information reuse. Moreover, it is not obvious how fast the resolution of the discretization should be increased before a new graph search is initiated, i.e., if it is more appropriate to add a single new configuration, double the number of configuration, or double the number of discrete values along each configuration space dimension. To overcome such issues, incremental motion planning methods interweave incremental discretization of configuration space with search for a path within one integrated process. 

An important class of methods for incremental path planning is based on the idea of incrementally growing a tree rooted at the initial configuration of the vehicle outwards to explore the reachable configuration space. The "exploratory" behavior is achieved by iteratively selecting a random vertex from the tree and by expanding the selected vertex by applying the steering function from it. Once the tree grows large enough to reach the goal region, the resulting path is recovered by tracing the links from the vertex in the goal region backwards to the initial configuration. The general algorithmic scheme of an incremental tree-based algorithm is described in Algorithm 3. 

## **Algorithm 3:** Incremental Tree-based Algorithm 

|_V ←{_**x**init_} ∪_sample-points(_X, n_); _E ←∅_;<br>**while** _not interrupted_ **do**|
|---|
|**x**selected _←_select(_V_);|
|_σ ←_extend(**x**selected_, V_);<br>**if** col-free(_σ_) **then**|
|**x**new =_σ_(1);|
|_V ←V ∪{_**x**new_}_; _E ←E∪{_(**x**selected_,_**x**new_, σ_)_}_;|
|**return** (V,E)|



One of the first randomized tree-based incremental planners was the expansive spaces tree (EST) planner proposed by Hsu et al. [124]. The algorithm selects a vertex for expansion, **x** selected, randomly from _V_ with a probability that is inversely proportional to the number of vertices in its neighborhood, which promotes growth towards unexplored regions. During expansion, the algorithm samples a new vertex **y** within a 

16 

neighborhood of a fixed radius around **x** selected, and use the same technique for biasing the sampling procedure to select a vertex from the region that is relatively less explored. Then it returns a straight line path between **x** selected and **y** . A generalization of the idea for planning with kinodynamic constraints in dynamic environments was introduced in [125], where the capabilities of the algorithm were demonstrated on different non-holonomic robotic systems and the authors use an idealized version of the algorithm to establish that the probability of failure to find a feasible path depends on the expansiveness property of the state space and decays exponentially with the number of samples. 

Rapidly-exploring Random Trees (RRT) [101] have been proposed by La Valle as an efficient method for finding feasible trajectories for high-dimensional non-holonomic systems. The rapid exploration is achieved by taking a random sample **x** rnd from the free configuration space and extending the tree in the direction of the random sample. In RRT, the vertex selection function select( _V_ ) returns the nearest neighbor to the random sample **x** rnd according to the given distance metric between the two configurations. The extension function extend() then generates a path in the configuration space by applying a control for a fixed time step that minimizes the distance to **x** rnd. Under certain simplifying assumptions (random steering is used for extension), the RRT algorithm has been shown to be probabilistic complete [83]. We remark that the result on probabilistic completeness does not readily generalize to many practically implemented versions of RRT that often use heuristic steering. In fact, it has been recently shown in [126] that RRT using heuristic steering with fixed time step is not probabilistically complete. 

Moreover, Karaman and Frazzoli [127] demonstrated that the RRT converges to a suboptimal solution with probability one and designed an asymptotically optimal adaptation of the RRT algorithm, called RRT*. As shown in Algorithm 4, the RRT* at every iteration considers a set of vertices that lie in the neighborhood of newly added vertex **x** new and a) connects **x** new to the vertex in the neighborhood that minimizes the cost of path from **x** init to **x** new and b) rewires any vertex in the neighborhood to **x** new if that results in a lower cost path from **x** init to that vertex. An important characteristic of the algorithm is that the neighborhood region is defined as the ball centered at **x** new with radius being function of the size of the tree: _r_ = _γ_<sup>�</sup><sup>_d_</sup> (log _n_ ) _/n,_ where _n_ is the number of vertices in the tree, _d_ is the dimension of the configuration space, and _γ_ is an instance-dependent constant. It is shown that for such a function, the expected number of vertices in the ball is logarithmic in the size of the tree, which is necessary to ensure that the algorithm almost surely converges to an optimal path while maintaining the same asymptotic complexity as the suboptimal RRT. 

Sufficient conditions for asymptotic optimality of RRT* under differential constraints are stated in [80] and demonstrated to be satisfiable for Dubins vehicle and double integrator systems. In a later work, the authors further show in the context of small-time locally attainable systems that the algorithm can be adapted to maintain not only asymptotic optimality, but also computational efficiency [82]. Other related works focus 

**Algorithm 4:** RRT* Algorithm. The cost-to-come to vertex **x** is denoted as _c_ ( **x** ), the cost of path segment _σ_ is denoted as _c_ ( _σ_ ) and the parent vertex of vertex **x** is denoted by _p_ <u>(</u> **x** <u>).</u> 



**while** _not interrupted_ **do** 





// find best parent **x** par _←_ arg min **x** _∈X_ near _c_ ( **x** ) + _c_ (connect( **x** _,_ **x** new)) subj. to col-free(steerexact( **x** _,_ **x** new)); _σ_<sup>_′_</sup> = steerexact( **x** par _,_ **x** new); _E ← E ∪{_ ( **x** par _,_ **x** new _, σ_<sup>_′_</sup> ) _}_ ; 



## **return** <u>(V,E)</u> 

on deriving distance and steering functions for non-holonomic systems by locally linearizing the system dynamics [128] or by deriving a closed-form solution for systems with linear dynamics [129]. On the other hand, RRT<sup>X</sup> is an algorithm that extends RRT<sup>_∗_</sup> to allow for real-time incremental replanning when the obstacle region changes, e.g., in the face of new data from sensors [130]. 

New developments in the field of sampling-based algorithms include algorithms that achieve asymptotic optimality without having access to an exact steering procedure. In particular, Li at al. [84] recently proposed the Stable Sparse Tree (SST) method for asymptotically (near-)optimal path planning, which is based on building a tree of randomly sampled controls propagated through a forward model of the dynamics of the system such that the locally suboptimal branches are pruned out to ensure that the tree remains sparse. 

## _F. Practical Deployments_ 

Three categories of path planning methodologies have been discussed for self-driving vehicles: variational methods, graph- 

17 

searched methods and incremental tree-based methods. The actual field-deployed algorithms on self-driving systems come from all the categories described above. For example, even among the first four successful participants of DARPA Urban Challenge, the approaches used for motion planning significantly differed. The winner of the challenge, CMU’s Boss vehicle used variational techniques for local trajectory generation in structured environments and a lattice graph in 4-dimensional configuration space (consisting of position, orientation, and velocity) together with Anytime D* to find a collision-free paths in parking lots [131]. The runner-up vehicle developed by Stanford’s team reportedly used a search strategy coined Hybrid A* that during search, lazily constructs a tree of motion primitives by recursively applying a finite set of maneuvers. The search is guided by a carefully designed heuristic and the sparsity of the tree is ensured by only keeping a single node within a given region of the configuration space [132]. Similarly, the vehicle arriving third developed by the VictorTango team from Virginia Tech constructs a graph discretization of possible maneuvers and searches the graph with the A* algorithm [133]. Finally, the vehicle developed by MIT used a variant of RRT algorithm called closed-loop RRT with biased sampling [134]. 

## V. VEHICLE CONTROL 

Solutions to Problem IV.1 or IV.2 are provided by the motion planning process. The role of the feedback controller is to stabilize to the reference path or trajectory in the presence of modeling error and other forms of uncertainty. Depending on the reference provided by the motion planner, the control objective may be _path stabilization_ or _trajectory stabilization_ . More formally, the path stabilization problem is stated as follows: 

**Problem V.1.** (Path stabilization) Given a controlled differential equation _x_ ˙ = _f_ ( _x, u_ ), reference path _xref_ : R _→_ R<sup>_n_</sup> , and velocity _vref_ : R _→_ R, find a feedback law, _u_ ( _x_ ), such that solutions to _x_ ˙ = _f_ ( _x, u_ ( _x_ )) satisfy the following: _∀ε >_ 0 and _t_ 1 _< t_ 2, there exists a _δ >_ 0 and a differentiable _s_ : R _→_ R such that 



Qualitatively, these conditions are that (1) a small initial tracking error will remain small, (2) the tracking error must converge to zero, and (3) progress along the reference path tends to a nominal rate. 

Many of the proposed vehicle control laws, including several discussed in this section, use a feedback law of the form 



where the feedback is a function of the nearest point on the reference path. An important issue with controls of this form is that the closed loop vector field _f_ ( _x, u_ ( _x_ )) will not be continuous. If the path is self intersecting or not differentiable 

at some point, a discontinuity in which _f_ ( _x, u_ ( _x_ )) is will lie directly on the path. This leads to unpredictable behavior if the executed trajectory encounters the discontinuity. This discontinuity is illustrated in Figure V.1. A backstepping control design which does not use a feedback law of the form (V.1) is presented in [135]. 



Figure V.1: Visualization of (V.5) for a sample reference path shown in black. The color indicates the value of _s_ for each point in the plane and illustrates discontinuities in (V.5). 

The trajectory stabilization problem is more straightforward, but these controllers are prone to performance limitations [136]. 

**Problem V.2.** (Trajectory stabilization) Given a controlled differential equation _x_ ˙ = _f_ ( _x, u_ ) and a reference trajectory _xref_ ( _t_ ), find _π_ ( _x_ ) such that solutions to _x_ ˙ = _f_ ( _x, π_ ( _x_ )) satisfy the following: _∀ε >_ 0 and _t_ 1 _< t_ 2, there exists a _δ >_ 0 such that 



In many cases, analyzing the stability of trajectories can be reduced to determining the origin’s stability in a time varying system. The basic form of Lyapunov’s theorem is only applicable to time invariant systems. However, stability theory for time varying systems is also well established (e.g. [137, Theorem 4.9]). 

Some useful qualifiers for various types of stability include: 

- _Uniform asymptotic stability_ for a time varying system which asserts that _δ_ in condition 1 of the above problem is independent of _t_ 1. 

- _Exponential stability_ asserts that the rate of convergence is bounded above by an exponential decay. 

A delicate issue that should be noted is that controller specifications are usually expressed in terms of the asymptotic tracking error as time tends to infinity. In practice, reference trajectories are finite so there should also be consideration for the transient response of the system. 

The remainder of this section is devoted to a survey of select control designs which are applicable to driverless cars. An overview of these controllers is provided in Table II. 

18 

|Controller||Model|Stability|Time Complexity|Comments/Assumptions|
|---|---|---|---|---|---|
|Pure Pursuit|(V-A1)|Kinematic|LES<sup>_⋆_</sup>to<br>ref. path|_O_(_n_)<sup>_∗_</sup>|No path curvature|
|Rear wheel<br>based feedback|(V-A2)|Kinematic|LES<sup>_⋆_</sup>to<br>ref. path|_O_(_n_)<sup>_∗_</sup>|_C_<sup>2</sup>(R<sup>_n_</sup>) ref. paths|
|Front wheel<br>based feedback|(V-A3)|Kinematic|LES<sup>_⋆_</sup>to<br>ref. path|_O_(_n_)<sup>_∗_</sup>|_C_<sup>1</sup>(R<sup>_n_</sup>) ref. paths;<br>Forward driving only|
|Feedback<br>linearization|(V-B2)|Steering rate<br>controlled kinematic|LES<sup>_⋆_</sup><br>to ref. traj.|_O_(1)|_C_<sup>1</sup>(R<sup>_n_</sup>) ref. traj.;<br>Forward driving only|
|Control Lyapunov<br>design|(V-B1)|Kinematic|LES<sup>_⋆_</sup>to<br>ref. traj.|O(1)|Stable for constant path<br>curvature and velocity|
|Linear MPC|(V-C)|_C_<sup>1</sup>(R<sup>_n_</sup> _×_R<sup>_m_</sup>)<br>model<sup>_♯_</sup>|LES<sup>_⋆_</sup>to ref.<br>or path|_O_<br>�_√_<br>_N_ln<br>�_N_<br>_ε_<br>�<sup>�</sup><sup>_†_</sup>|Stability depends<br>on horizon length|
|Nonlinear MPC|(V-C)|_C_<sup>1</sup>(R<sup>_n_</sup> _×_R<sup>_m_</sup>)<br>model<sup>_♯_</sup>|Not guaranteed|_O_( <sup>1</sup><br>_ε_<sup>)</sup><sup>_‡_</sup>|Works well in practice|



Table II: Overview of controllers discussed within this section. **Legend:** _⋆_ : local exponential stability (LES); _∗_ : assuming (V.1) is evaluated by a linear search over an _n_ -point discretization of the path or trajectory; _†_ : assuming the use of an interior-point method to solve (V.28) with a time horizon of _n_ and solution accuracy of _ε_ ; _‡_ : based on asymptotic convergence rate to local minimum of (V.25) using steepest descent. Not guaranteed to return solution or find global minimum.; _♯_ : vector field over the state space R<sup>_n_</sup> defined by each input in R<sup>_m_</sup> is a continuously differentiable function so that the gradient of the cost or linearization about the reference is defined. 

Subsection V-A details a number of effective control strategies for path stabilization of the kinematic model, and subsection V-B2 discusses trajectory stabilization techniques. Predictive control strategies, discussed in subsection V-C, are effective for more complex vehicle models and can be applied to path and trajectory stabilization. 

## _A. Path Stabilization for the Kinematic Model_ 

_1) Pure Pursuit:_ Among the earliest proposed path tracking strategies is pure pursuit. The first discussion appeared in [138], and was elaborated upon in [52], [139]. This strategy and its variations (e.g. [140], [141]) have proven to be an indispensable tool for vehicle control owing to its simple implementation and satisfactory performance. Numerous publications including two vehicles in the DARPA Grand Challenge [8] and three vehicles in the DARPA Urban challenge [9] reported using the pure pursuit controller. 

The control law is based on fitting a semi-circle through the vehicle’s current configuration to a point on the reference path ahead of the vehicle by a distance _L_ called the lookahead distance. Figure V.2 illustrates the geometry. The circle is defined as passing through the position of the car and the point on the path ahead of the car by one lookahead distance with the circle tangent to the car’s heading. The curvature of the circle is given by 



For a vehicle speed _vr_ , the commanded heading rate is 



In the original publication of this controller [138], the angle _α_ is computed directly from camera output data. However, _α_ 



Figure V.2: Geometry of the pure pursuit controller. A circle (blue) is fit between rear wheel position and the reference path (brown) such that the chord length (green) is the look ahead distance _L_ and the circle is tangent to the current heading direction. 

can be expressed in terms of the inertial coordinate system to define a state feedback control. Consider the configuration ( _xr, yr, θ_ )<sup>_T_</sup> and the points on the path, ( _xref_ ( _s_ ) _, yref_ ( _s_ )), such that _∥_ ( _xref_ ( _s_ ) _, yref_ ( _s_ )) _−_ ( _xr, yr_ ) _∥_ = _L_ . Since there is generally more than one such point on the reference, take the one with the greatest value of the parameter _s_ to uniquely define a control. Then _α_ is given by 



Assuming that the path has no curvature and the vehicle speed is constant (potentially negative), the pure pursuit controller solves Problem V.1. For a fixed nonzero curvature, pure pursuit has a small steady state tracking error. 

In the case where the vehicle’s distance to the path is greater than _L_ , the controller output is not defined. Another consideration is that changes in reference path curvature can lead to 

19 

the car deviating from the reference trajectory. This may be acceptable for driving along a road, but can be problematic for tracking parking maneuvers. Lastly, the heading rate command _ω_ becomes increasingly sensitive to the feedback angle _α_ as the vehicle speed increases. A common fix for this issue is to scale _L_ with the vehicle speed. 

_2) Rear wheel position based feedback:_ The next approach uses the rear wheel position as an output to stabilize a nominal rear wheel path [56]. The controller assigns 



Detailed assumptions on the reference path and a finite domain containing the reference path where (V.5) is a continuous function are described in [56]. The unit tangent to the path at _s_ ( _t_ ) is given by 



and the tracking error vector is 



These values are used to compute a transverse error coordinate from the path _e_ which is a cross product between the two vectors 



with the subscript denoting the component indices of the vector. The control uses the angle _θe_ between the vehicle’s heading vector and the tangent vector to the path. 



The geometry is illustrated in Figure V.3. 



Figure V.3: Feedback variables for the rear wheel based feedback control. _θe_ is the difference between the tangent at the nearest point on the path to the rear wheel and the car heading. The magnitude of the scalar value _e_ is illustrated in red. As illustrated _e >_ 0, and for the case where the car is to the left of the path, _e <_ 0. 

A change of coordinates to ( _s, e, θe_ ) yields 



where _κ_ ( _s_ ) denotes the curvature of the path at _s_ . The following heading rate command provides local asymptotic convergence to twice continuously differentiable paths: 



with _g_ 1( _e, θe, t_ ) _>_ 0, _k_ 2 _>_ 0, and _vr̸_ = 0 which is verified with the Lyapunov function _V_ ( _e, θe_ ) = _e_<sup>2</sup> + _θe_<sup>2</sup><sup>_/k_2in[56]</sup> using the coordinate system (V.10). The requirement that the path be twice differentiable comes from the appearance of the curvature in the feedback law. An advantage of this control law is that stability is unaffected by the sign of _vr_ making it suitable for reverse driving. 



_3) Front wheel position based feedback:_ This approach was proposed and used in Stanford University’s entry to the 2005 DARPA Grand Challenge [55], [8]. The approach is to take the front wheel position as the regulated variable. The control uses the variables _s_ ( _t_ ) _, e_ ( _t_ ) _,_ and _θe_ ( _t_ ) as in the previous subsections, with the modification that _e_ ( _t_ ) is computed with the front wheel position as opposed to the rear wheel position. Taking the time derivative of the transverse error reveals 



The error rate in (V.13) can be directly controlled by the steering angle for error rates with magnitude less than _vf_ . Solving for the steering angle such that _e_ ˙ = _−ke_ drives _e_ ( _t_ ) to zero exponentially fast. 



The term _θe_ in this case is not interpreted as heading error since it will be nonzero even with perfect tracking. It is more appropriately interpreted as a combination of a feed-forward term of the nominal steering angle to trace out the reference path and a heading error term. 

The drawback to this control law is that it is not defined when _|ke/vf | >_ 1. The exponential convergence over a finite domain can be relaxed to local exponential convergence with the feedback law 



which, to first order in _e_ , is identical to the previous equation. This is illustrated in Figure V.4. 

Like the control law in (V.14) this controller locally exponentially stabilizes the car to paths with varying curvature with the condition that the path is continuously differentiable. The condition on the path arises from the definition of _θe_ in the feedback policy. A drawback to this controller is that it is not stable in reverse making it unsuitable for parking. 

20 



Figure V.4: Front wheel output based control. The control strategy is to point the front wheel towards the path so that the component of the front wheel’s velocity normal to the path is proportional to the distance to the path. This is achieved locally and yields local exponential convergence. 

_Comparison of path tracking controllers for kinematic models:_ The advantages of controllers based on the kinematic model with the no-slip constraint on the wheels is that they have low computational requirements, are readily implemented, and have good performance at moderate speeds. Figure V.5 provides a qualitative comparison of the path stabilizing controllers of this sections based on, and simulated with, (III.3) for a lane change maneuver. In the simulation of the front wheel output based controller, the rear wheel reference path is replaced by the front wheel reference path satisfying 



The parameters of the simulation are summarized in Table III. 

In reference to Figure V.5, the pure pursuit control tracks the reference path during periods with no curvature. In the region where the path has high curvature, the pure pursuit control causes the system to deviate from the reference path. In contrast, the latter two controllers converge to the path and track it through the high curvature regions. 

In both controllers using (V.5) in the feedback policy, local exponential stability can only be proven if there is a neighborhood of the path where (V.5) is continuous. Intuitively, this means the path cannot cross over itself and must be differentiable. 



Figure V.5: Tracking performance comparison for the three path stabilizing control laws discussed in this section. (a) Pure pursuit deviates from reference when curvature is nonzero. (b) The rear wheel output based controller drives the rear wheel to the rear wheel reference path. Overshoot is a result of the second order response of the system. (c) The front wheel output based controller drives the front wheel to the reference path with a first order response and tracks the path through the maneuver. 

||Example|Parameters||
|---|---|---|---|
|Reference path|Wheel-<br>base|Steering<br>limit|Initial<br>Configuration|
|(_xref_(_s_)_, yref_(_s_)) =<br>(_s,_4_·_tanh<br>�_s−_40<br>4<br>�|_L_= 5|_|δ| ≤π/_4|(_xr_(0)_, yr_(0)_, θ_(0))<br>=(0_, −_2_,_0)|
|C|ontroller|Parameter|s|
|Pure pursuit<br>Re|ar wheel|feedback|Front wheel<br>feedback|
|_L_= 5_, vr_ = 1<br>_ke_|= 0_._25_, _<br>_vr_ =|_kθ_ = 0_._75_,_<br>1|_k_ = 0_._5_, vr_ = 1|



Table III: Simulation and controller parameters used to generate Figure V.5. 

## _B. Trajectory Tracking Control for the Kinematic Model_ 

_1) Control Lyapunov based design:_ A control design based on a control Lyapunov function is described in [142]. The approach is to define the tracking error in a coordinate frame fixed to the car. The configuration error can be expressed by a change of basis from the inertial coordinate frame using the reference trajectory, and velocity, ( _xref , yref , θref , vref , ωref_ ), 



The evolution of the configuration error is then 



With the control assignment, 



21 

the closed loop error dynamics become 

_x_ ˙ _e_ =( _ωref_ + _vref_ ( _k_ 2 _ye_ + _k_ 3 sin( _θe_ ))) _ye − k_ 1 _xe,_ 

_y_ ˙ _e_ = _−_ ( _ωref_ + _vref_ ( _k_ 2 _ye_ + _k_ 3 sin( _θe_ ))) _xe_ + _vref_ sin ( _θe_ ) _, θ_ ˙ _e_ = _ωref − ω._ 

Stability is verified for _k_ 1 _,_ 2 _,_ 3 _>_ 0, _ω_ ˙ _ref_ = 0, and _v_ ˙ _ref_ = 0 by the Lyapunov function 



with negative semi-definite time derivative, 



A local analysis shows that the control law provides local exponential stability. However, for the system to be time invariant, _ωref_ and _vref_ are required to be constant. 

A related controller is proposed in [143] which utilizes a backstepping design to achieve uniform local exponential stability for a finite domain with time varying references. 

_2) Output feedback linearization:_ For higher vehicle speeds, it is appropriate to constrain the steering angle to have continuous motion as in (III.8). With the added state, it becomes more difficult to design a controller from simple geometric considerations. A good option in this case is to output-linearize the system. This is not easily accomplished using the front or rear wheel positions. An output which simplifies the feedback linearization is proposed in [144], where a point ahead of the vehicle by any distance _d̸_ = 0, aligned with the steering angle is selected. 

Let _xp_ = _xf_ + _d_ cos( _θ_ + _δ_ ) and _yp_ = _yf_ + _d_ sin( _θ_ + _δ_ ) be the output of the system. Taking the derivative of these outputs and substituting the dynamics of III.8 yields 





Then, defining the right hand side of (V.17) as auxiliary control variables _ux_ and _uy_ yields 



which makes control straightforward. From _ux_ and _uy_ , the original controls _vf_ and _vδ_ are recovered by using the inverse of the matrix in (V.17), provided below: 



From the input-output linear system, local trajectory stabilization can be accomplished with the controls 



To avoid confusion, note that in this case, the output position ( _xp, yp_ ) and controlled speed _vf_ are not collocated as in the previously discussed controllers. 

## _C. Predictive Control Approaches_ 

The simple control laws discussed above are suitable for moderate driving conditions. However, slippery roads or emergency maneuvers may require a more accurate model, such as the one introduced in Section III-B. The added detail of more sophisticated models complicates the control design making it difficult to construct controllers from intuition and geometry of the configuration space. 

Model predictive control [145] is a general control design methodology which can be very effective for this problem. Conceptually, the approach is to solve the motion planning problem over a short time horizon, take a short interval of the resulting open loop control, and apply it to the system. While executing, the motion planning problem is re-solved to find an appropriate control for the next time interval. Advances in computing hardware as well as mathematical programming algorithms have made predictive control feasible for real-time use in driverless vehicles. MPC is a major field of research on its own and this section is only intended to provide a brief description of the technique and to survey results on its application to driverless vehicle control. 

Since model predictive control is a very general control technique, the model takes the form of a general continuous time control system with control, _u_ ( _t_ ) _∈_ R<sup>_m_</sup> , and state, _x_ ( _t_ ) _∈_ R<sup>_n_</sup> , _x_ ˙ = _f_ ( _x, u, t_ ) (V.21) 

A feasible reference trajectory _xref_ ( _t_ ), and for some motion planners _uref_ ( _t_ ), are provided satisfying (V.21). The system is then discretized by an appropriate choice of numerical approximation so that (V.21) is given at discrete time instances by 



One of the simplest discretization schemes is Euler’s method with a zero order hold on control: 



The state and control are discretized by their approximation at times _tk_ = _k ·_ ∆ _t_ . Solutions to the discretized system are approximate and will not match the continuous time equation exactly. Similarly, the reference trajectory and control sampled at the discrete times _tk_ will not satisfy the discrete time equation. For example, the mismatch between solutions to (V.23) and (V.21) will be _O_ (∆ _t_ ) and the reference trajectory sampled at time _tk_ will result in 



To avoid over-complicating the following discussion, we assume the discretization is exact for the remainder of the 

22 

section. The control law typically takes the form 





The function _gn_ penalizes deviations from the reference trajectory and control at each time step, while the function _h_ is a terminal penalty at the end of the time horizon. The set _Xn_ is the set of allowable states which can restrict undesirable positions or velocities, e.g., excessive tire slip or obstacles. The set _Un_ encodes limits on the magnitude of the input signals. Important considerations are whether the solutions to the right hand side of (V.25) exist, and when they do, the stability and robustness of the closed loop system. These issues are investigated in the predictive control literature [146], [147]. 

To implement an MPC on a driverless car, (V.25) must be solved several times per second which is a major obstacle to its use. In the special case that _h_ and _gn_ are quadratic, _Un_ and _Xn_ are polyhedral, and _F_ is linear, the problem becomes a quadratic program. Unlike a general nonlinear programming formulation, interior point algorithms are available for solving quadratic programs in polynomial time. To leverage this, the complex vehicle model is often linearized to obtain an approximate linear model. Linearization approaches typically differ in the reference about which the linearization is computed— current operating point [148]–[150], reference path [151] or more generally, about a reference trajectory, which results in the following approximate linear model: 





where _ξ_ := _x − xref_ and _η_ := _u − uref_ are the deviations of the state and control from the reference trajectory. This first order expansion of the perturbation dynamics yields a linear time varying (LTV) system, 



Then using a quadratic objective, and expressing the polyhedral constraints algebraically, we obtain 



subject to 



where _Rk_ and _Qk_ are positive semi-definite. If the states and inputs are unconstrained, i.e., _Un_ = R<sup>_m_</sup> , _Xn_ = R<sup>_n_</sup> , a semi-closed form solution can be obtained by dynamic programming requiring only the calculation of an _N_ step matrix recursion [152]. Similar closed form recursive solutions have also been explored when vehicle models are represented by controlled auto-regressive integrated moving average (CARIMA) with no state and input constraints [153], [154]. 

A further variation in the model predictive control approach is to replace state constraints (e.g., obstacles) and input constraints by penalty functions in the performance functional. Such a predictive control approach based on a dynamic model with nonlinear tire behavior is presented in [155]. In addition to penalizing control effort and deviation from a reference path to a goal which does not consider obstacles, the performance functional penalizes input constraint violations and collisions with obstacles over the finite control horizon. In this sense it is similar to potential field based motion planning, but is demonstrated to have improved performance. 

The following are some variations of the model predictive control framework that are found in the literature of car controllers: 

_1) Unconstrained MPC with Kinematic Models:_ The earliest predictive controller in [153] falls under this category, in which the model predictive control framework is applied without input or state constraints using a CARIMA model. The resulting semi-closed form solution has minimal computational requirements and has also been adopted in [154]. Moreover, the time-varying linear quadratic programming approach with no input or state constraints was considered in [154] using a linearized kinematic model. 

_2) Path Tracking Controllers:_ In [151], a predictive control is investigated using a center of mass based linear dynamic model (assuming constant velocity) for path tracking and an approximate steering model. The resulting integrated model is validated with a detailed automatic steering model and a 27 degree-of-freedom CarSim vehicle model. 

_3) Trajectory Tracking Controllers:_ A predictive controller using a tire model similar to Section III was investigated in [148]. The full nonlinear predictive control strategy was carried out in simulation and shown to stabilize a simulated emergency maneuver in icy conditions with a control frequency of 20 Hz. However, with a control horizon of just two time steps the computation time was three times the sample time of the controller making experimental validation impossible. A linearization based approach was also investigated in [148]–[150] based on a single linearization about the state of the vehicle at the current time step. The reduced complexity of solving the quadratic program resulted in acceptable computation time, and successful experimental results are reported for driving in icy conditions at speeds up to 21m/s. The promising simulation and experimental results of this approach are improved upon in [150] by providing 

23 

conditions for the uniform local-asymptotic stability of the time varying system. 

## _D. Linear Parameter Varying Controllers_ 

Many controller design techniques are available for linear systems making a linear model desirable. However, the broad range of operating points encountered under normal driving conditions make it difficult to rely on a model linearized about a single operating point. To illustrate this, consider the lateral error dynamics in (V.10). If the tracking error is assumed to remain small a linearization of the dynamics around the operating point _θe_ = 0 and _e_ = 0 yields 



Introducing a new control variable incorporating a feedforward _u_ = _ω_ + _vrκ_ ( _s_ ) simplifies the discussion. The dynamics are now 



Observe that the model is indeed linear, but the forward speed _vr_ appears in the linear model. A simple proportional plus derivative control with gains _kp_ and _kd_ will stabilize the lateral dynamics but the poles of the closed loop system are given by _−kd ±_ ~~�~~ _kd_<sup>2</sup><sup>_−_4</sup><sup>_kpvr_</sup> _/_ 2 _._ At higher speeds the poles move � <u>�</u> into the complex plane leading to an oscillatory response. In contrast, a small _kp_ gain leads to a poor response at low speed. A very intuitive and widely used remedy to this challenge is gain scheduling. In this example, parameterizing _kp_ as a function of _vr_ fixes the poles to a single value for each speed. This technique falls into the category of control design for linear parameter varying (LPV) models [156]. Gain scheduling is a classical approach to this type of controller design. Tools from robust control and convex optimization are readily applied to address more complex models. 

LPV control designs for lateral control are presented in [157]–[159]. LPV models are used in [160], [161] together with predictive control approaches for path and trajectory stabilization. At a lower level of automation, LPV control techniques have been proposed for integrated system control. In these designs, several subsystems are combined under a single controller to achieve improved handling performance. LPV control strategies for actuating active and semi-active suspension systems are developed in [162], [163], while integrated suspension and braking control systems are developed in [164], [165]. 

## VI. CONCLUSIONS 

The past three decades have seen increasingly rapid progress in driverless vehicle technology. In addition to the advances in computing and perception hardware, this rapid progress has been enabled by major theoretical progress in the computational aspects of mobile robot motion planning and feedback control theory. Research efforts have undoubtedly been spurred 

by the improved utilization and safety of road networks that driverless vehicles would provide. 

Driverless vehicles are complex systems which have been decomposed into a hierarchy of decision making problems, where the solution of one problem is the input to the next. The breakdown into individual decision making problems has enabled the use of well developed methods and technologies from a variety of research areas. The task is then to integrate these methods so that their interactions are semantically valid, and the combined system is computationally efficient. A more efficient motion planning algorithm may only be compatible with a computationally intensive feedback controller such as model predictive control. Conversely, a simple control law may require less computation to execute, but is also less robust and requires using a more detailed model for motion planning. 

This paper has provided a survey of the various aspects of driverless vehicle decision making problems with a focus on motion planning and feedback control. The survey of performance and computational requirements of various motion planning and control techniques serves as a reference for assessing compatibility and computational tradeoffs between various choices for system level design. 

## REFERENCES 

- [1] “2014 crash data key findings.” National Highway Traffic Safety Administration, Report No. DOT HS 812 219, 2014. 

- [2] S. Singh, “Critical reasons for crashes investigated in the national motor vehicle crash causation survey.” National Highway Traffic Safety Administration, Report No. DOT HS 812 115, 2014. 

- [3] B. McKenzie and M. Rapino, “Commuting in the United States: 2009, American Community Survey Reports,” _U.S. Census Bureau_ , 2011. ACS-15, 2011. 

- [4] D. A. Hennessy and D. L. Wiesenthal, “Traffic congestion, driver stress, and driver aggression,” _Aggressive behavior_ , pp. 409–423, 1999. 

- [5] E. D. Dickmanns and V. Graefe, “Dynamic monocular machine vision,” _Machine vision and applications_ , vol. 1, pp. 223–240, 1988. 

- [6] E. D. Dickmanns _et al._ , “Vehicles capable of dynamic vision,” in _IJCAI_ , pp. 1577–1592, 1997. 

- [7] “No Hands Across America Journal.” http://www _._ cs _._ cmu _._ edu/afs/cs/ usr/tjochem/www/nhaa/Journal _._ html. Accessed: 2015-12-14. 

- [8] M. Buehler, K. Iagnemma, and S. Singh, _The 2005 DARPA Grand Challenge: The great robot race_ , vol. 36. Springer Science & Business Media, 2007. 

- [9] M. Buehler, K. Iagnemma, and S. Singh, _The DARPA Urban Challenge: Autonomous vehicles in city traffic_ , vol. 56. springer, 2009. 

- [10] J. Xin, C. Wang, Z. Zhang, and N. Zheng, “China future challenge: Beyond the intelligent vehicle,” _IEEE Intell. Transp. Syst. Soc. Newslett_ , vol. 16, pp. 8–10, 2014. 

- [11] P. Cerri, G. Soprani, P. Zani, J. Choi, J. Lee, D. Kim, K. Yi, and A. Broggi, “Computer vision at the Hyundai autonomous challenge,” in _International Conference on Intelligent Transportation Systems_ , pp. 777–783, IEEE, 2011. 

- [12] A. Broggi, P. Cerri, M. Felisa, M. C. Laghi, L. Mazzei, and P. P. Porta, “The vislab intercontinental autonomous challenge: an extensive test for a platoon of intelligent vehicles,” _International Journal of Vehicle Autonomous Systems_ , vol. 10, pp. 147–164, 2012. 

- [13] A. Broogi, P. Cerri, S. Debattisti, M. C. Laghi, P. Medici, D. Molinari, M. Panciroli, and A. Prioletti, “Proud-public road urban driverlesscar test,” _Transactions on Intelligent Transportation Systems_ , vol. 16, pp. 3508–3519, 2015. 

- [14] J. Ziegler, P. Bender, M. Schreiber, H. Lategahn, T. Strauss, C. Stiller, T. Dang, U. Franke, N. Appenrodt, C. G. Keller, _et al._ , “Making Bertha drive—an autonomous journey on a historic route,” _Intelligent Transportation Systems Magazine_ , vol. 6, pp. 8–20, 2014. 

- [15] “Google Self-Driving Car Project.” https://www _._ google _._ com/ selfdrivingcar/. Accessed: 2015-12-14. 

- [16] “Tesla Motors: Model S Press Kit.” https://www _._ teslamotors _._ com/ presskit/autopilot. Accessed: 2016-3-15. 

24 

- [17] S. O.-R. A. V. S. Committee _et al._ , “Taxonomy and definitions for terms related to on-road motor vehicle automated driving systems,” 2014. 

- [18] R. Rajamani, _Vehicle dynamics and control_ . Springer Science & Business Media, 2011. 

- [19] J. C. Gerdes and E. J. Rossetter, “A unified approach to driver assistance systems based on artificial potential fields,” _Journal of Dynamic Systems, Measurement, and Control_ , vol. 123, pp. 431–438, 2001. 

- [20] M. Brännström, E. Coelingh, and J. Sjöberg, “Model-based threat assessment for avoiding arbitrary vehicle collisions,” _Transactions on Intelligent Transportation Systems_ , vol. 11, pp. 658–669, 2010. 

- [21] A. Vahidi and A. Eskandarian, “Research advances in intelligent collision avoidance and adaptive cruise control,” _Transactions on Intelligent Transportation Systems_ , vol. 4, pp. 143–153, 2003. 

- [22] M. R. Hafner, D. Cunningham, L. Caminiti, and D. Del Vecchio, “Cooperative collision avoidance at intersections: Algorithms and experiments,” _Transactions on Intelligent Transportation Systems_ , vol. 14, pp. 1162–1175, 2013. 

- [23] A. Colombo and D. Del Vecchio, “Efficient algorithms for collision avoidance at intersections,” in _Proceedings of the 15th ACM international conference on Hybrid Systems: Computation and Control_ , pp. 145–154, ACM, 2012. 

- [24] H. Kowshik, D. Caveney, and P. Kumar, “Provable systemwide safety in intelligent intersections,” _Vehicular Technology, IEEE Transactions on_ , vol. 60, pp. 804–818, 2011. 

- [25] A. Eskandarian, _Handbook of intelligent vehicles_ . Springer London, UK, 2012. 

- [26] D. Miculescu and S. Karaman, “Polling-systems-based control of highperformance provably-safe autonomous intersections,” in _Decision and Control (CDC), 2014 IEEE 53rd Annual Conference on_ , pp. 1417– 1423, IEEE, 2014. 

- [27] F. Zhou, X. Li, and J. Ma, “Parsimonious shooting heuristic for trajectory control of connected automated traffic part I: Theoretical analysis with generalized time geography,” _arXiv preprint arXiv:1511.04810_ , 2015. 

- [28] D. Geronimo, A. M. Lopez, A. D. Sappa, and T. Graf, “Survey of pedestrian detection for advanced driver assistance systems,” _Transactions on Pattern Analysis & Machine Intelligence_ , pp. 1239–1258, 2009. 

- [29] G. Ros, A. Sappa, D. Ponsa, and A. M. Lopez, “Visual SLAM for driverless cars: A brief survey,” in _Intelligent Vehicles Symposium (IV) Workshops_ , 2012. 

- [30] A. Geiger, J. Ziegler, and C. Stiller, “Stereoscan: Dense 3d reconstruction in real-time,” in _Intelligent Vehicles Symposium (IV), 2011 IEEE_ , pp. 963–968, IEEE, 2011. 

- [31] P. Liu, A. Kurt, K. Redmill, and U. Ozguner, “Classification of highway lane change behavior to detect dangerous cut-in maneuvers,” in _The Transportation Research Board (TRB) 95th Annual Meeting_ , 2015. 

- [32] E. W. Dijkstra, “A note on two problems in connexion with graphs,” _Numerische mathematik_ , vol. 1, pp. 269–271, 1959. 

- [33] N. J. Nilsson, “A mobile automaton: An application of artificial intelligence techniques,” tech. rep., DTIC Document, 1969. 

- [34] A. V. Goldberg and C. Harrelson, “Computing the shortest path: A search meets graph theory,” in _Proceedings of the sixteenth annual ACM-SIAM symposium on Discrete algorithms_ , pp. 156–165, Society for Industrial and Applied Mathematics, 2005. 

- [35] R. Geisberger, P. Sanders, D. Schultes, and C. Vetter, “Exact routing in large road networks using contraction hierarchies,” _Transportation Science_ , vol. 46, pp. 388–404, 2012. 

- [36] H. Bast, D. Delling, A. Goldberg, M. Müller-Hannemann, T. Pajor, P. Sanders, D. Wagner, and R. F. Werneck, “Route planning in transportation networks,” _arXiv preprint arXiv:1504.05140_ , 2015. 

- [37] F. Havlak and M. Campbell, “Discrete and continuous, probabilistic anticipation for autonomous robots in urban environments,” _Transactions on Robotics_ , vol. 30, pp. 461–474, 2014. 

- [38] Q. Tran and J. Firl, “Modelling of traffic situations at urban intersections with probabilistic non-parametric regression,” in _Intelligent Vehicles Symposium (IV), 2013 IEEE_ , pp. 334–339, IEEE, 2013. 

- [39] A. C. Madrigal, “The trick that makes Google’s self-driving cars work,” _The Atlantic_ , 2015. http://www _._ theatlantic _._ com/technology/ archive/2014/05/all-the-world-a-track-the-trick-that-makes-googlesself-driving-cars-work/370871/. 

- [40] R. Verma and D. D. Vecchio, “Semiautonomous multivehicle safety,” _Robotics & Automation Magazine, IEEE_ , vol. 18, pp. 44–54, 2011. 

- [41] S. Z. Yong, M. Zhu, and E. Frazzoli, “Generalized innovation and inference algorithms for hidden mode switched linear stochastic systems 

   - with unknown inputs,” in _Decision and Control (CDC), 2014 IEEE 53rd Annual Conference on_ , pp. 3388–3394, IEEE, 2014. 

- [42] S. Brechtel, T. Gindele, and R. Dillmann, “Probabilistic MDP-behavior planning for cars,” in _14th International Conference on Intelligent Transportation Systems_ , pp. 1537–1542, IEEE, 2011. 

- [43] S. Ulbrich and M. Maurer, “Probabilistic online POMDP decision making for lane changes in fully automated driving,” in _16th International Conference on Intelligent Transportation Systems_ , pp. 2063– 2067, IEEE, 2013. 

- [44] S. Brechtel, T. Gindele, and R. Dillmann, “Probabilistic decisionmaking under uncertainty for autonomous driving using continuous POMDPs,” in _17th International Conference on Intelligent Transportation Systems_ , pp. 392–399, IEEE, 2014. 

- [45] E. Galceran, A. G. Cunningham, R. M. Eustice, and E. Olson, “Multipolicy decision-making for autonomous driving via changepoint-based behavior prediction,” in _Proceedings of Robotics: Science & Systems Conference_ , p. 2, 2015. 

- [46] T. Bandyopadhyay, K. S. Won, E. Frazzoli, D. Hsu, W. S. Lee, and D. Rus, “Intention-aware motion planning,” in _Algorithmic Foundations of Robotics X_ , pp. 475–491, Springer, 2013. 

- [47] S. M. LaValle, _Planning algorithms_ . Cambridge university press, 2006. [48] A. De Luca, G. Oriolo, and C. Samson, “Feedback control of a nonholonomic car-like robot,” in _Robot motion planning and control_ , pp. 171–253, Springer, 1998. 

- [49] R. M. Murray and S. S. Sastry, “Nonholonomic motion planning: Steering using sinusoids,” _Transactions on Automatic Control_ , vol. 38, pp. 700–716, 1993. 

- [50] T. Fraichard and R. Mermond, “Path planning with uncertainty for carlike robots,” in _International Conference on Robotics and Automation_ , vol. 1, pp. 27–32, IEEE, 1998. 

- [51] M. Egerstedt, X. Hu, H. Rehbinder, and A. Stotsky, “Path planning and robust tracking for a car-like robot,” in _Proceedings of the 5th symposium on intelligent robotic systems_ , pp. 237–243, Citeseer, 1997. 

- [52] R. C. Coulter, “Implementation of the pure pursuit path tracking algorithm,” tech. rep., DTIC Document, 1992. 

- [53] H. Goldstein, _Classical mechanics_ . Pearson Education India, 1965. [54] Y. Kuwata, S. Karaman, J. Teo, E. Frazzoli, J. P. How, and G. Fiore, “Real-time motion planning with applications to autonomous urban driving,” _Transactions on Control Systems Technology_ , vol. 17, pp. 1105–1118, 2009. 

- [55] M. D. Ventures, “Stanley: The robot that won the DARPA Grand Challenge,” _Journal of field Robotics_ , vol. 23, pp. 661–692, 2006. 

- [56] C. Samson, “Path following and time-varying feedback stabilization of a wheeled mobile robot,” in _2nd Int. Conf. on Automation, Robotics and Computer Vision_ , 1992. 

- [57] L. E. Dubins, “On curves of minimal length with a constraint on average curvature, and with prescribed initial and terminal positions and tangents,” _American Journal of Mathematics_ , vol. 79, pp. 497– 516, 1957. 

- [58] J. Reeds and L. Shepp, “Optimal paths for a car that goes both forwards and backwards,” _Pacific journal of mathematics_ , vol. 145, pp. 367–393, 1990. 

- [59] E. Velenis and P. Tsiotras, “Minimum time vs maximum exit velocity path optimization during cornering,” in _International symposium on industrial electronics_ , pp. 355–360, 2005. 

- [60] A. Rucco, G. Notarstefano, and J. Hauser, “Computing minimum laptime trajectories for a single-track car with load transfer,” in _Conference on Decision and Control_ , pp. 6321–6326, IEEE, 2012. 

- [61] J. H. Jeon, R. V. Cowlagi, S. C. Peters, S. Karaman, E. Frazzoli, P. Tsiotras, and K. Iagnemma, “Optimal motion planning with the halfcar dynamical model for autonomous high-speed driving,” in _American Control Conference_ , pp. 188–193, IEEE, 2013. 

- [62] S. C. Peters, E. Frazzoli, and K. Iagnemma, “Differential flatness of a front-steered vehicle with tire force control,” in _International Conference on Intelligent Robots and Systems (IROS)_ , pp. 298–304, IEEE, 2011. 

- [63] E. Bakker, L. Nyborg, and H. B. Pacejka, “Tyre modelling for use in vehicle dynamics studies,” tech. rep., SAE Technical Paper, 1987. 

- [64] J. H. Reif, “Complexity of the mover’s problem and generalizations,” in _20th Annual Symposium on Foundations of Computer Science_ , SFCS ’79, (Washington, DC, USA), pp. 421–427, IEEE, 1979. 

- [65] J. Canny, _The complexity of robot motion planning_ . MIT press, 1988. [66] T. Lozano-Pérez and M. A. Wesley, “An algorithm for planning collision-free paths among polyhedral obstacles,” _Communications of the ACM_ , vol. 22, pp. 560–570, 1979. 

- [67] J. A. Storer and J. H. Reif, “Shortest paths in the plane with polygonal obstacles,” _Journal of the ACM_ , vol. 41, pp. 982–1012, 1994. 

25 

- [68] M. H. Overmars and E. Welzl, “New methods for computing visibility graphs,” in _Proceedings of the fourth annual symposium on Computational geometry_ , pp. 164–171, ACM, 1988. 

- [69] M. De Berg, M. Van Kreveld, M. Overmars, and O. C. Schwarzkopf, _Computational geometry_ . Springer, 2000. 

- [70] S. Lazard, J. Reif, and H. Wang, “The complexity of the two dimensional curvatureconstrained shortest-path problem,” in _Proceedings of the Third International Workshop on the Algorithmic Foundations of Robotics_ , pp. 49–57, 1998. 

- [71] S. Fortune and G. Wilfong, “Planning constrained motion,” _Annals of Mathematics and Artificial Intelligence_ , vol. 3, pp. 21–82, 1991. 

- [72] P. K. Agarwal, T. Biedl, S. Lazard, S. Robbins, S. Suri, and S. Whitesides, “Curvature-constrained shortest paths in a convex polygon,” _SIAM Journal on Computing_ , vol. 31, pp. 1814–1851, 2002. 

- [73] J.-D. Boissonnat and S. Lazard, “A polynomial-time algorithm for computing a shortest path of bounded curvature amidst moderate obstacles,” in _Proceedings of the twelfth annual symposium on Computational geometry_ , pp. 242–251, ACM, 1996. 

- [74] F. Lamiraux, E. Ferre, and E. Vallee, “Kinodynamic motion planning: Connecting exploration trees using trajectory optimization methods,” in _International Conference on Robotics and Automation_ , vol. 4, pp. 3987–3992, IEEE, 2004. 

- [75] F. Boyer and F. Lamiraux, “Trajectory deformation applied to kinodynamic motion planning for a realistic car model,” in _International Conference on Robotics and Automation_ , pp. 487–492, IEEE, 2006. 

- [76] J. Canny and J. Reif, “New lower bound techniques for robot motion planning problems,” in _28th Symposium on Foundations of Computer Science_ , pp. 49–60, IEEE, 1987. 

- [77] D. P. Bertsekas, _Nonlinear programming_ . Athena scientific, 1999. [78] M. L. Fredman and R. E. Tarjan, “Fibonacci heaps and their uses in improved network optimization algorithms,” _Journal of the ACM (JACM)_ , vol. 34, pp. 596–615, 1987. 

- [79] L. E. Kavraki, M. N. Kolountzakis, and J.-C. Latombe, “Analysis of probabilistic roadmaps for path planning,” _Transactions on Robotics and Automation_ , vol. 14, pp. 166–171, 1998. 

- [80] S. Karaman and E. Frazzoli, “Sampling-based algorithms for optimal motion planning,” _The International Journal of Robotics Research_ , vol. 30, pp. 846–894, 2011. 

- [81] E. Schmerling, L. Janson, and M. Pavone, “Optimal sampling-based motion planning under differential constraints: the driftless case,” _International Conference on Robotics and Automation_ , 2015. 

- [82] S. Karaman and E. Frazzoli, “Sampling-based optimal motion planning for non-holonomic dynamical systems,” in _International Conference on Robotics and Automation_ , pp. 5041–5047, IEEE, 2013. 

- [83] S. M. LaValle and J. J. Kuffner, “Randomized kinodynamic planning,” _The International Journal of Robotics Research_ , vol. 20, pp. 378–400, 2001. 

- [84] Y. Li, Z. Littlefield, and K. E. Bekris, “Sparse methods for efficient asymptotically optimal kinodynamic planning,” in _Algorithmic Foundations of Robotics XI_ , pp. 263–282, Springer, 2015. 

- [85] J. Reif and M. Sharir, “Motion planning in the presence of moving obstacles,” _Journal of the ACM_ , vol. 41, pp. 764–790, 1994. 

- [86] T. Fraichard, “Trajectory planning in a dynamic workspace: a’state-time space’approach,” _Advanced Robotics_ , vol. 13, pp. 75–94, 1998. 

- [87] J. Kasac, J. Deur, B. Novakovic, I. Kolmanovsky, and F. Assadian, “A conjugate gradient-based BPTT-like optimal control algorithm with vehicle dynamics control application,” _Transactions on Control Systems Technology_ , vol. 19, pp. 1587–1595, 2011. 

- [88] C. L. Darby, W. W. Hager, and A. V. Rao, “An hp-adaptive pseudospectral method for solving optimal control problems,” _Optimal Control Applications and Methods_ , vol. 32, pp. 476–502, 2011. 

- [89] L. S. Pontryagin, _Mathematical theory of optimal processes_ . CRC Press, 1987. 

- [90] Y. Tassa, N. Mansard, and E. Todorov, “Control-limited differential dynamic programming,” in _International Conference on Robotics and Automation_ , pp. 1168–1175, IEEE, 2014. 

- [91] J. T. Betts, “Survey of numerical methods for trajectory optimization,” _Journal of Guidance, Control, and Dynamics_ , vol. 21, pp. 193–207, 1998. 

- [92] E. Polak, “An historical survey of computational methods in optimal control,” _SIAM Review_ , vol. 15, pp. 553–584, 1973. 

- [93] B. Chazelle, “Approximation and decomposition of shapes,” _Advances in Robotics_ , vol. 1, pp. 145–185, 1987. 

- [94] C. Ó’Dúnlaing and C. K. Yap, “A "retraction" method for planning the motion of a disc,” _Journal of Algorithms_ , vol. 6, pp. 104–111, 1985. 

   - [96] J.-C. Latombe, _Robot motion planning_ , vol. 124. Springer Science & Business Media, 2012. 

   - [97] J. T. Schwartz and M. Sharir, “On the "piano movers" problem. ii. general techniques for computing topological properties of real algebraic manifolds,” _Advances in applied Mathematics_ , vol. 4, pp. 298–351, 1983. 

   - [98] J. Backer and D. Kirkpatrick, “Finding curvature-constrained paths that avoid polygonal obstacles,” in _Proceedings of the twenty-third annual symposium on Computational geometry_ , pp. 66–73, ACM, 2007. 

   - [99] P. Jacobs and J. Canny, “Planning smooth paths for mobile robots,” in _Nonholonomic Motion Planning_ , pp. 271–342, Springer, 1993. 

   - [100] H. Wang and P. K. Agarwal, “Approximation algorithms for curvatureconstrained shortest paths.,” in _SODA_ , vol. 96, pp. 409–418, 1996. 

   - [101] S. M. La Valle, “Rapidly-exploring random trees a new tool for path planning,” tech. rep., Computer Science Dept., Iowa State University, 1998. 

   - [102] S. Petti and T. Fraichard, “Safe motion planning in dynamic environments,” in _Intelligent Robots and Systems, 2005.(IROS 2005). 2005 IEEE/RSJ International Conference on_ , pp. 2210–2215, IEEE, 2005. 

   - [103] A. Bhatia and E. Frazzoli, “Incremental search methods for reachability analysis of continuous and hybrid systems,” in _Hybrid Systems: Computation and Control: 7th International Workshop, HSCC 2004, Philadelphia, PA, USA, March 25-27, 2004. Proceedings_ (R. Alur and G. J. Pappas, eds.), pp. 142–156, Springer Berlin Heidelberg, 2004. 

   - [104] E. Glassman and R. Tedrake, “A quadratic regulator-based heuristic for rapidly exploring state space,” in _Robotics and Automation (ICRA), 2010 IEEE International Conference on_ , pp. 5021–5028, IEEE, 2010. 

   - [105] S. Fleury, P. Soueres, J.-P. Laumond, and R. Chatila, “Primitives for smoothing mobile robot trajectories,” _Transactions on Robotics and Automation_ , vol. 11, pp. 441–448, 1995. 

   - [106] E. Velenis, P. Tsiotras, and J. Lu, “Aggressive maneuvers on loose surfaces: Data analysis and input parametrization,” in _Mediterranean Conference on Control Automation_ , pp. 1–6, 2007. 

   - [107] M. Pivtoraiko, R. A. Knepper, and A. Kelly, “Differentially constrained mobile robot motion planning in state lattices,” _Journal of Field Robotics_ , vol. 26, pp. 308–333, 2009. 

   - [108] S. M. LaValle, M. S. Branicky, and S. R. Lindemann, “On the relationship between classical grid search and probabilistic roadmaps,” _The International Journal of Robotics Research_ , vol. 23, pp. 673–692, 2004. 

   - [109] J. Lengyel, M. Reichert, B. R. Donald, and D. P. Greenberg, _Real-time robot motion planning using rasterizing computer graphics hardware_ , vol. 24. ACM, 1990. 

   - [110] L. E. Kavraki, P. Švestka, J.-C. Latombe, and M. H. Overmars, “Probabilistic roadmaps for path planning in high-dimensional configuration spaces,” _Transactions on Robotics and Automation_ , vol. 12, pp. 566– 580, 1996. 

   - [111] L. Janson, E. Schmerling, A. Clark, and M. Pavone, “Fast marching tree: A fast marching sampling-based method for optimal motion planning in many dimensions,” _The International Journal of Robotics Research_ , 2015. 

   - [112] P. E. Hart, N. J. Nilsson, and B. Raphael, “A formal basis for the heuristic determination of minimum cost paths,” _Transactions on Systems Science and Cybernetics_ , vol. 4, pp. 100–107, 1968. 

   - [113] I. Pohl, “First results on the effect of error in heuristic search,” _Machine Intelligence_ , pp. 219–236, 1970. 

   - [114] A. Stentz, “Optimal and efficient path planning for partially-known environments,” in _International Conference Robotics and Automation_ , pp. 3310–3317, IEEE, 1994. 

   - [115] S. Anthony, “The focussed D* algorithm for real-time replanning.,” in _IJCAI_ , vol. 95, pp. 1652–1659, 1995. 

   - [116] S. Koenig and M. Likhachev, “Fast replanning for navigation in unknown terrain,” _Transactions on Robotics_ , vol. 21, pp. 354–363, 2005. 

   - [117] E. A. Hansen and R. Zhou, “Anytime heuristic search.,” _J. Artif. Intell. Res.(JAIR)_ , vol. 28, pp. 267–297, 2007. 

   - [118] M. Likhachev, G. J. Gordon, and S. Thrun, “ARA*: Anytime A* with provable bounds on sub-optimality,” in _Advances in Neural Information Processing Systems_ , p. None, 2003. 

   - [119] M. Likhachev, D. I. Ferguson, G. J. Gordon, A. Stentz, and S. Thrun, “Anytime dynamic A*: An anytime, replanning algorithm.,” in _ICAPS_ , pp. 262–271, 2005. 

   - [120] K. Daniel, A. Nash, S. Koenig, and A. Felner, “Theta*: Any-angle path planning on grids,” _Journal of Artificial Intelligence Research_ , pp. 533–579, 2010. 

- [95] O. Takahashi and R. J. Schilling, “Motion planning in a plane using generalized voronoi diagrams,” _Transactions on Robotics and Automation_ , vol. 5, pp. 143–150, 1989. 

26 

- [121] A. Nash, S. Koenig, and C. Tovey, “Lazy theta*: Any-angle path planning and path length analysis in 3d,” in _Third Annual Symposium on Combinatorial Search_ , 2010. 

- [122] P. Yap, N. Burch, R. C. Holte, and J. Schaeffer, “Block a*: Databasedriven search with applications in any-angle path-planning.,” in _AAAI_ , 2011. 

- [123] D. Ferguson and A. Stentz, “Using interpolation to improve path planning: The field d* algorithm,” _Journal of Field Robotics_ , vol. 23, pp. 79–101, 2006. 

- [124] D. Hsu, J.-C. Latombe, and R. Motwani, “Path planning in expansive configuration spaces,” in _International Conference on Robotics and Automation_ , vol. 3, pp. 2719–2726, IEEE, 1997. 

- [125] D. Hsu, R. Kindel, J.-C. Latombe, and S. Rock, “Randomized kinodynamic motion planning with moving obstacles,” _The International Journal of Robotics Research_ , vol. 21, pp. 233–255, 2002. 

- [126] T. Kunz and M. Stilman, “Kinodynamic RRTs with fixed time step and best-input extension are not probabilistically complete,” in _Algorithmic Foundations of Robotics XI_ , pp. 233–244, Springer, 2015. 

- [127] S. Karaman and E. Frazzoli, “Optimal kinodynamic motion planning using incremental sampling-based methods,” in _Conference on Decision and Control_ , pp. 7681–7687, IEEE, 2010. 

- [128] A. Perez, R. Platt Jr, G. Konidaris, L. Kaelbling, and T. LozanoPerez, “LQR-RRT*: Optimal sampling-based motion planning with automatically derived extension heuristics,” in _International Conference on Robotics and Automation_ , pp. 2537–2542, IEEE, 2012. 

- [129] D. J. Webb and J. van den Berg, “Kinodynamic RRT*: Asymptotically optimal motion planning for robots with linear dynamics,” in _International Conference on Robotics and Automation_ , pp. 5054–5061, IEEE, 2013. 

- [130] M. Otte and E. Frazzoli, “RRT-X: Real-time motion planning/replanning for environments with unpredictable obstacles,” in _International Workshop on the Algorithmic Foundations of Robotics_ , 2014. 

- [131] C. Urmson, J. Anhalt, D. Bagnell, C. Baker, R. Bittner, M. Clark, J. Dolan, D. Duggins, T. Galatali, C. Geyer, _et al._ , “Autonomous driving in urban environments: Boss and the Urban Challenge,” _Journal of Field Robotics_ , vol. 25, pp. 425–466, 2008. 

- [132] D. Dolgov, S. Thrun, M. Montemerlo, and J. Diebel, “Practical search techniques in path planning for autonomous driving,” _Ann Arbor_ , vol. 1001, p. 48105, 2008. 

- [133] A. Bacha, C. Bauman, R. Faruque, M. Fleming, C. Terwelp, C. Reinholtz, D. Hong, A. Wicks, T. Alberi, D. Anderson, _et al._ , “Odin: Team victorTango’s entry in the DARPA Urban Challenge,” _Journal of Field Robotics_ , vol. 25, pp. 467–492, 2008. 

- [134] J. Leonard, J. How, S. Teller, M. Berger, S. Campbell, G. Fiore, L. Fletcher, E. Frazzoli, A. Huang, S. Karaman, _et al._ , “A perceptiondriven autonomous urban vehicle,” _Journal of Field Robotics_ , vol. 25, pp. 727–774, 2008. 

- [135] J. P. Hespanha _et al._ , “Trajectory-tracking and path-following of underactuated autonomous vehicles with parametric modeling uncertainty,” _Transactions on Automatic Control_ , vol. 52, pp. 1362–1379, 2007. 

- [136] A. P. Aguiar, J. P. Hespanha, and P. V. Kokotovi´c, “Path-following for nonminimum phase systems removes performance limitations,” _Transactions on Automatic Control_ , vol. 50, pp. 234–239, 2005. 

- [137] H. K. Khalil, _Nonlinear systems_ , vol. 3. Prentice hall New Jersey, 1996. [138] R. Wallace, A. Stentz, C. E. Thorpe, H. Maravec, W. Whittaker, and T. Kanade, “First results in robot road-following.,” in _IJCAI_ , pp. 1089– 1095, 1985. 

- [139] O. Amidi and C. E. Thorpe, “Integrated mobile robot control,” in _Fibers’ 91, Boston, MA_ , pp. 504–523, International Society for Optics and Photonics, 1991. 

- [140] A. L. Rankin, C. D. Crane III, D. G. Armstrong II, A. D. Nease, and H. E. Brown, “Autonomous path-planning navigation system for site characterization,” in _Aerospace/Defense Sensing and Controls_ , pp. 176– 186, International Society for Optics and Photonics, 1996. 

- [141] J. Wit, C. D. Crane, and D. Armstrong, “Autonomous ground vehicle path tracking,” _Journal of Robotic Systems_ , vol. 21, pp. 439–449, 2004. 

- [142] Y. Kanayama, Y. Kimura, F. Miyazaki, and T. Noguchi, “A stable tracking control method for an autonomous mobile robot,” in _International Conference on Robotics and Automation_ , pp. 384–389, IEEE, 1990. 

- [143] Z.-P. Jiang and H. Nijmeijer, “Tracking control of mobile robots: a case study in backstepping,” _Automatica_ , vol. 33, pp. 1393–1399, 1997. 

- [144] B. d’Andréa Novel, G. Campion, and G. Bastin, “Control of nonholonomic wheeled mobile robots by state feedback linearization,” _The International journal of robotics research_ , vol. 14, pp. 543–559, 1995. 

- [145] C. E. Garcia, D. M. Prett, and M. Morari, “Model predictive control: theory and practice-a survey,” _Automatica_ , vol. 25, pp. 335–348, 1989. 

- [146] E. F. Camacho and C. B. Alba, _Model predictive control_ . Springer Science & Business Media, 2013. 

- [147] D. Q. Mayne, J. B. Rawlings, C. V. Rao, and P. O. Scokaert, “Constrained model predictive control: Stability and optimality,” _Automatica_ , vol. 36, pp. 789–814, 2000. 

- [148] P. Falcone, F. Borrelli, J. Asgari, H. E. Tseng, and D. Hrovat, “Predictive active steering control for autonomous vehicle systems,” _Transactions on Control Systems Technology_ , vol. 15, pp. 566–580, 2007. 

- [149] P. Falcone, M. Tufo, F. Borrelli, J. Asgari, and H. E. Tseng, “A linear time varying model predictive control approach to the integrated vehicle dynamics control problem in autonomous systems,” in _46th Conference on Decision and Control_ , pp. 2980–2985, IEEE, 2007. 

- [150] P. Falcone, F. Borrelli, H. E. Tseng, J. Asgari, and D. Hrovat, “Linear time-varying model predictive control and its application to active steering systems: Stability analysis and experimental validation,” _International journal of robust and nonlinear control_ , vol. 18, pp. 862– 875, 2008. 

- [151] E. Kim, J. Kim, and M. Sunwoo, “Model predictive control strategy for smooth path tracking of autonomous vehicles with steering actuator dynamics,” _International Journal of Automotive Technology_ , vol. 15, pp. 1155–1164, 2014. 

- [152] D. E. Kirk, _Optimal control theory: an introduction_ . Courier Corporation, 2012. 

- [153] A. Ollero and O. Amidi, “Predictive path tracking of mobile robots. application to the CMU Navlab,” in _5th International Conference on Advanced Robotics_ , vol. 91, pp. 1081–1086, 1991. 

- [154] G. V. Raffo, G. K. Gomes, J. E. Normey-Rico, C. R. Kelber, and L. B. Becker, “A predictive controller for autonomous vehicle path tracking,” _Transactions on Intelligent Transportation Systems_ , vol. 10, pp. 92–102, 2009. 

- [155] Y. Yoon, J. Shin, H. J. Kim, Y. Park, and S. Sastry, “Model-predictive active steering and obstacle avoidance for autonomous ground vehicles,” _Control Engineering Practice_ , vol. 17, pp. 741–750, 2009. 

- [156] O. Sename, P. Gaspar, and J. Bokor, _Robust control and linear parameter varying approaches: application to vehicle dynamics_ , vol. 437. Springer, 2013. 

- [157] P. Gáspár, Z. Szabó, and J. Bokor, “LPV design of fault-tolerant control for road vehicles,” _International Journal of Applied Mathematics and Computer Science_ , vol. 22, pp. 173–182, 2012. 

- [158] J. Huang and M. Tomizuka, “LTV controller design for vehicle lateral control under fault in rear sensors,” _Transactions on Mechatronics_ , vol. 10, pp. 1–7, 2005. 

- [159] S. Ç. Baslamisli,<sup>˙</sup> I. E. Köse, and G. Anla¸s, “Gain-scheduled integrated active steering and differential control for vehicle handling improvement,” _Vehicle System Dynamics_ , vol. 47, pp. 99–119, 2009. 

- [160] T. Besselmann and M. Morari, “Autonomous vehicle steering using explicit LPV-MPC,” in _Control Conference (ECC), 2009 European_ , pp. 2628–2633, IEEE, 2009. 

- [161] T. Besselmann, P. Rostalski, and M. Morari, “Hybrid parameter-varying model predictive control for lateral vehicle stabilization,” in _European Control Conference_ , pp. 1068–1075, IEEE, 2007. 

- [162] P. Gáspár, Z. Szabó, and J. Bokor, “The design of an integrated control system in heavy vehicles based on an LPV method,” in _Conference on Decision and Control_ , pp. 6722–6727, IEEE, 2005. 

- [163] C. Poussot-Vassal, O. Sename, L. Dugard, P. Gaspar, Z. Szabo, and J. Bokor, “A new semi-active suspension control strategy through LPV technique,” _Control Engineering Practice_ , vol. 16, pp. 1519–1534, 2008. 

- [164] M. Doumiati, O. Sename, L. Dugard, J.-J. Martinez-Molina, P. Gaspar, and Z. Szabo, “Integrated vehicle dynamics control via coordination of active front steering and rear braking,” _European Journal of Control_ , vol. 19, pp. 121–143, 2013. 

- [165] P. Gaspar, Z. Szabo, J. Bokor, C. Poussot-Vassal, O. Sename, and L. Dugard, “Toward global chassis control by integrating the brake and suspension systems,” in _5th IFAC Symposium on Advances in Automotive Control, IFAC AAC 2007_ , p. 6, IFAC, 2007. 

27 

**Brian Paden** received B.S. and M.S. degrees in Mechanical Engineering in 2011 and 2013 respectively from UC Santa Barbara. Concurrently from 2011 to 2013 he was an Engineer at LaunchPoint Technologies developing electronically controlled automotive valve-train systems. His research interests are in the areas of control theory and motion planning with a focus on applications for autonomous vehicles. His current affiliation is with the Laboratory for Information and Decision systems and he is a Doctoral Candidate in the department of Mechanical Engineering at MIT. 

**Michal Cáp**<sup>**ˇ**</sup> received his Bc. degree in Information Technology from Brno University of Technology, the Czech Republic and MSc degree in Agent Technologies from Utrecht University, the Netherlands. He is currently pursuing PhD degree in Artificial Intelligence at CTU in Prague, Czech Republic. At the moment, Michal Cap holds the position of Fulbright-funded visiting researcher at Massachusetts Institute of Technology, USA. His research interests include motion planning, multi-robot trajectory coordination and autonomous transportation systems in general. 

**Sze Zheng Yong** is currently a postdoctoral associate in the Laboratory for Information and Decision Systems at Massachusetts Institute of Technology. He obtained his Dipl.-Ing.(FH) degree in automotive engineering with a specialization in mechatronics and control systems from the Esslingen University of Applied Sciences, Germany in 2008 and his S.M. and Ph.D. degrees in mechanical engineering from MIT in 2010 and 2016, respectively. His research interests lie in the broad area of control and estimation of hidden mode hybrid systems, with applications to intention-aware autonomous systems and resilient cyber-physical systems. 

**Dmitry Yershov** was born in Kharkov, Ukraine, in 1983. He received his B.S. and M.S. degree in applied mathematics from Kharkov National University in 2004 and 2005, respectively. In 2013, he graduated with a Ph.D. degree from the the Department of Computer Science at the University of Illinois at Urbana-Champaign. He joined the Laboratory for Information and Decision Systems at the Massachusetts Institute of Technology in years 2013–2015 where he was a postdoctoral associate. Currently, he is with nuTonomy inc. 



His research is focused primarily on the feedback planning problem in robotics, which extends to planning in information spaces and planning under uncertainties. 

**Emilio Frazzoli** is a Professor of Aeronautics and Astronautics with the Laboratory for Information and Decision Systems at the Massachusetts Institute of Technology. He received a Laurea degree in Aerospace Engineering from the University of Rome, “Sapienza" , Italy, in 1994, and a Ph. D. degree from the Department of Aeronautics and Astronautics of the Massachusetts Institute of Technology, in 2001. Before returning to MIT in 2006, he held faculty positions at the University of 



Illinois, Urbana-Champaign, and at the University of California, Los Angeles. He was the recipient of a NSF CAREER award in 2002, and of the IEEE George S. Axelby award in 2015. His current research interests focus primarily on autonomous vehicles, mobile robotics, and transportation systems. 

