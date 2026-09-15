# Relevant AV Literature & Resources for Unstructured Indian Roads
*Curated for SIH PS 26037: Adaptive Path Planning, Dynamic Obstacle Avoidance & Behavioral Control*

---

## 1. Datasets for Unstructured ODDs & Trajectory Forecasting

These datasets provide empirical ground truth for unstructured road geometry, missing lane markers, non-lane-based traffic, and multi-agent trajectory prediction:

* **[Intel's Dataset for AD Conditions in India (IDD)](https://idd.insaan.iiit.ac.in/)**  
  *Context:* 10k finely annotated images across 34 classes from 182 drive sequences on Indian roads (by Intel & IIIT Hyderabad). Essential benchmark for auto-rickshaws, stray animals, and unregulated traffic mix.
* **[CSSAD Dataset (Camera-based Stereo for Developing Countries)](http://aplicaciones.cimat.mx/Personal/jbhayet/ccsad-dataset)**  
  *Context:* Specifically collected in developing regions lacking lane markings, with abundant potholes, speed humps, and irregular pedestrian flows.
* **[Argoverse Motion Forecasting Dataset](https://www.argoverse.org/)**  
  *Context:* 324,557 mined scenarios containing trajectory rollouts of dynamic actors; ideal for benchmarking multi-object Kalman filters and trajectory predictors.
* **[Oxford Road Boundaries Dataset](https://oxford-robotics-institute.github.io/road-boundaries-dataset/)**  
  *Context:* 62,605 samples containing raw and segmented masks for road boundaries in scenes without formal lane striping.

---

## 2. Unstructured Road Detection & Boundary Modeling Papers

Foundational methods for navigating drivable corridors without relying on lane markers:

* **[2015]** *Automatic Driving on Ill-defined Roads: An Adaptive, Shape-constrained, Color-based Method* — Ososinski & Labrosse. [[Paper](https://www.semanticscholar.org/paper/Automatic-Driving-on-Ill-defined-Roads-An-Adaptive-Ososinski-Labrosse/36cfe2e94b7b99653e6565642236e0127d43ef5a)]  
  *Relevance:* Boundary extraction when pavement edges degrade into gravel or dirt shoulders.
* **[2013]** *Road model prediction based unstructured road detection* — Zuo & Yao. [[Paper](https://www.semanticscholar.org/paper/Road-model-prediction-based-unstructured-road-Zuo-Yao/b8b2d3da341042d148ed2988216dbb3ddb6081ed)]  
  *Relevance:* Estimating forward drivable corridors on roads lacking painted demarcation.
* **[2012]** *Road Tracking Method Suitable for Both Unstructured and Structured Roads* — Procházka. [[Paper](https://www.semanticscholar.org/paper/International-Journal-of-Advanced-Robotic-Systems-Proch%C3%A1zka/4819fda4bc778454701f2a4b30db46ec56aa45bc)]  
  *Relevance:* Hybrid road tracking robust to sudden transitions between paved and unpaved terrain.
* **[2012]** *Fast Vanishing-Point Detection in Unstructured Environments* — Moghadam & Starzyk. [[Paper](https://www.semanticscholar.org/paper/Fast-Vanishing-Point-Detection-in-Unstructured-Moghadam-Starzyk/c02f52b8b80db037f92facbb605c5715513935fb)]  
  *Relevance:* Orientation and heading stabilization on unmarked single-lane rural roads.

---

## 3. Obstacle Representation & Dynamic Tracking Papers

Lightweight obstacle representation and multi-agent state estimation:

* **[2014]** *Classification and Tracking of Dynamic Objects with Multiple Sensors for Autonomous Driving in Urban Environments* — Darms & Rybski. [[Paper](https://www.semanticscholar.org/paper/Classification-and-Tracking-of-Dynamic-Objects-Darms-Rybski/6c9ce40060fa3efea7d04a4a0e36609592ed6ddf)]  
  *Relevance:* Core principles behind multi-class dynamic state tracking using Kalman filters.
* **[2012]** *The Stixel World* — Pfeiffer et al. [[Paper](https://www.semanticscholar.org/paper/The-Stixel-World-N-Im/5307f5e2ff2f0403a92b63418ca5812965dcfb90)]  
  *Relevance:* Compact representation of obstacles as vertical columns, minimizing 2D occupancy grid construction latency.
* **[2014]** *Extending the Stixel World with online self-supervised color modeling for road-versus-obstacle segmentation* — Sanberg & Dubbelman. [[Paper](https://www.semanticscholar.org/paper/Extending-the-Stixel-World-with-online-self-Sanberg-Dubbelman/6dd60e0484931b284f49ab8204b011d153ff4967)]  
  *Relevance:* Fast obstacle vs. drivable surface separation under variable outdoor illumination.
* **[2013]** *Avoiding moving obstacles during visual navigation* — Cherubini & Grechanichenko. [[Paper](https://www.semanticscholar.org/paper/Avoiding-moving-obstacles-during-visual-navigation-Cherubini-Grechanichenko/7c0e580c0f914086e9c918aef1df561253a71044)]  
  *Relevance:* Reactive avoidance bounds against moving agents in confined spaces.

---

## 4. Motion Planning, Behavioral FSM & Control Papers

Theoretical foundations directly corresponding to the `adaptive_path_planner`, `universal_bottleneck_decider`, and `pure_pursuit_controller`:

* **[2009]** *Automatic Steering Methods for Autonomous Automobile Path Tracking* — Snider (CMU-RI-TR-09-08). [[Paper](https://www.semanticscholar.org/paper/Automatic-Steering-Methods-for-Autonomous-Snider/18520721525ed81a6ffa6d8b1c7dcbd771e4a64b)]  
  *Relevance:* The foundational mathematical derivation for speed-adaptive lookahead Pure Pursuit and Stanley control formulation.
* **[2016]** *A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles* — Paden et al. [[Paper](https://arxiv.org/abs/1604.07446)]  
  *Relevance:* Taxonomy of hierarchical motion planning (FSM decision making $\rightarrow$ spatiotemporal planning $\rightarrow$ tracking control).
* **[2016]** *A Convex Optimization Approach to Smooth Trajectories for Motion Planning with Car-Like Robots* — Zhu & Schmerling. [[Paper](https://www.semanticscholar.org/paper/A-Convex-Optimization-Approach-to-Smooth-Zhu-Schmerling/785b22bbdb04f2ddd4233a4c40d798ed3194374f)]  
  *Relevance:* Smooth spline optimization ensuring lateral acceleration and jerk stay within physical comfort bounds ($< 1.0 \text{ m/s}^3$).
* **[2010]** *Vision-Based Autonomous Navigation System Using ANN and FSM Control* — Sales & Shinzato. [[Paper](https://www.semanticscholar.org/paper/Vision-Based-Autonomous-Navigation-System-Using-Sales-Shinzato/e1fcccdbc373c9bbd5bd970c34368e7e1aa56424)]  
  *Relevance:* State machine arbitration for switching between cruising, yielding, and reactive detour modes.
* **[2008]** *Planning Long Dynamically Feasible Maneuvers for Autonomous Vehicles* — Likhachev & Ferguson. [[Paper](https://www.semanticscholar.org/paper/Planning-Long-Dynamically-Feasible-Maneuvers-for-Likhachev-Ferguson/1f8ca38a1fa455db3388c617697cc91300c59bc6)]  
  *Relevance:* Theoretical formulation of search spaces incorporating vehicle non-holonomic turning limits (kinematic bicycle model constraints).

---

## 5. Technical Talks & Foundational Video References

Key engineering lectures covering sensing limits, sensor fusion, and motion planning architecture:

* **Chris Urmson (Carnegie Mellon / Aurora):** *How a driverless car sees the road*  
  [Watch Video](https://www.youtube.com/watch?v=tiwVMrTLUWg)  
  *Takeaway:* Real-world handling of dynamic actor occlusions and unexpected obstacles in complex environments.
* **Amnon Shashua (Mobileye):** *Autonomous Driving, Computer Vision and Machine Learning (CVPR Keynote)*  
  [Watch Video](https://www.youtube.com/watch?v=n8T7A3wqH3Q)  
  *Takeaway:* Formalizing safety guarantees, Responsibility-Sensitive Safety (RSS), and bottleneck clearance envelopes.
* **Sensing Architectures:** *What goes into sensing for autonomous driving?*  
  [Watch Video](https://www.youtube.com/watch?v=GCMXXXmxG-I)  
  *Takeaway:* Range gating, field-of-view considerations, and handling sensor degradation/dropout.
* **Core Philosophy:** *The Three Pillars of Autonomous Driving*  
  [Watch Video](https://www.youtube.com/watch?v=GZa9SlMHhQc)  
  *Takeaway:* Clean decomposition between Perception, Decision/Planning, and Actuation Control.

---

## 6. Open Source Architecture Benchmarks

Software repositories used to cross-reference modular vehicle autonomy stacks:

* **[Autoware](https://github.com/CPFL/Autoware)** — Modular, open-source architecture for planning, control, and behavioral state management.
* **[Comma.ai Openpilot](https://github.com/commaai/openpilot)** — Production-grade longitudinal speed deciders and lateral path tracking.
* **[argoverse-api](https://github.com/argoai/argoverse-api)** — Evaluation toolkits for trajectory prediction errors and tracking evaluation.