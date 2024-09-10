## Adaptation of the Residual Vector of Unknown Dynamics

### Overview
In physical human-robot interaction, safety is crucial. This project implements a model-based collision detection method that adaptively updates a robot’s dynamic parameters in real time, ensuring accurate detection of external forces. This method utilizes the residual between expected and actual momentum to detect collisions and allows for continuous refinement of parameters like joint friction and inertia.

### Features
- Momentum-based residual detection: Detects collisions by comparing expected and actual momentum.
- Adaptive dynamic parameters: Updates dynamic parameters (mass, inertia, joint friction) in real time to ensure detection accuracy.
- Real-time simulation: Demonstrates robot behavior under both collision-free and collision conditions.
- Controller implementation: Uses a feedback linearization controller for accurate trajectory tracking.
- Visual results: Graphical representations of joint positions, velocities, accelerations, and momentum residuals.

### Project structure
``` bash
├── data.mat
├── trajectory_data.mat
├── main.m
├── library/
│   ├── DHmatrix.m
│   ├── createRobot.m
│   ├── dynamic_model.m
│   ├── dynamics.m
│   ├── excitation_trajectory.m
│   ├── friction.m
│   ├── get_J.m
│   ├── get_J_dot.m
│   ├── get_end_eff_pos.m
│   ├── inv_diff_kin.m
│   ├── plot_results.m
│   ├── regressor_matrix.m
│   └── save_mat.m
├── .gitignore
├── report.pdf
└── README.md
```

### Simulation Parameters
The robot being simulated is a 3R elbow-type manipulator. The dynamic parameters used in the simulation are as follows:

- Link 1 mass: 15 kg, radius: 0.2 m
- Link 2 mass: 10 kg, radius: 0.1 m
- Link 3 mass: 5 kg, radius: 0.1 m

The trajectory for the end-effector is designed to follow an elliptical path in 3D space.

#### Controller parameters
- kp = 80, kd = 15: Gain parameters for the feedback linearization controller
- Residual gain (k): 0.2
- λ1 = 1.5, λ2 = 1.1: Adaptive rate parameters for updating the dynamic parameters

### Results
The results of the simulation include:

1.	Contact-free test:
  - The dynamic parameters remain stable and the momentum residual stays close to zero.
  - Plots showing the smooth motion of joints and the end-effector.
2.	Collision test:
  - Detects collisions at specified time intervals.
  - Momentum residual peaks during collisions, confirming accurate detection.
  - Graphs showing deviations in joint positions, velocities, and accelerations during the collision events.
