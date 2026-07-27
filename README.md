# RTKS Time Violation Analysis

<img width="155" align="center" alt="unipd-logo" src="https://github.com/user-attachments/assets/1e4e14cf-9d9c-4c5a-aaf7-64cb6b64a0c8" />


*Real-Time Kernels and Systems course - University of Padua*

Contributors:

* Lorenzo Albertin;
* Andrea Santi;
* Caterina Vallotto.

This repository contains the source code, experimental configurations, documentation, and results produced for the Real-Time Kernels and Systems course project on performance interference and timing violations in Cyber-Physical Systems.

The project is inspired by the work presented in "*An Empirical Study of Performance Interference: Timing Violation Patterns and Impacts"*. This project's objective is not to reproduce the complete TimeTrap framework, but to investigate, on a smaller and controlled scale, if different forms of timing and resource interference can produce measurable effects on the timing behavior and physical-control performance of autonomous systems.

The study is divided into two case studies:

* **ArduPilot**: evaluated through the ArduCopter Software-In-The-Loop environment;
* **TurtleBot3**: evaluated through ROS and Gazebo-based simulation.

Each case study contains a baseline configuration and multiple experimental branches representing different timing, scheduling, delay-injection, or resource-interference conditions.

## Repository organization

The repository uses separate Git branches to keep the two case studies and their experimental configurations isolated.

The following tree represents the **logical organization** of the branches. 

```text
main
│
├── ardupilot/main-baseline
│   ├── exp/E0-baseline
│   ├── exp/E1-direct-dos
│   ├── exp/E2-separated-sensor
│   ├── exp/E3-delay-injection
│   └── exp/E4-multicore-interference
│
└── turtlebot3/main-baseline
    ├── turtlebot3/E0
    ├── turtlebot3/E1
    ├── turtlebot3/E2
    ├── turtlebot3/E3
    ├── turtlebot3/E4
    └── turtlebot3/E5
```

### Main branch

The `main` branch contains the common project material, including:

* this general repository README;
* the project report.

Experiment-specific source code is maintained in the corresponding ArduPilot or TurtleBot3 branches.

### ArduPilot branches

The `ardupilot/main-baseline` branch contains the ArduPilot baseline and the instrumentation shared by the experiments.

The experimental configurations are stored in the following branches:

* `exp/E0-baseline`
* `exp/E1-direct-dos`
* `exp/E2-separated-sensor`
* `exp/E3-delay-injection`
* `exp/E4-multicore-interference`

Each branch contains the source-code required for that particular configuration, together with a specific README describing the experiment and its execution procedure.

### TurtleBot3 branches

The `turtlebot3/main-baseline` branch contains the TurtleBot3 simulation baseline.

The experimental configurations are stored in the following branches:

* `turtlebot3/E0`
* `turtlebot3/E1`
* `turtlebot3/E2`
* `turtlebot3/E3`
* `turtlebot3/E4`
* `turtlebot3/E5`

Each branch contains the source-code required for that particular configuration, together with a specific README describing the experiment and its execution procedure.

## Directory structure

The directories visible in the repository depend on the currently selected branch.

The general structure is:

```text
RTKS_Time_Violation_Analysis/
├── README.md
├── project report
│
├── ArduPilot/
│   └── ArduPilot Copter-4.0 source code and experiment-specific changes
│
├── ArduPilot_experiments_results/
│   ├── analysis scripts
│   └── results collected from the ArduPilot experimental runs
│
├── TurtleBot3/
│   └── catkin_ws/
│       └── src/
│           ├── navigation/
│           └── turtlebot3_simulations/
│
└── TurtleBot3_experiments_results/
    ├── analysis scripts
    └── results collected from the TurtleBot3 experimental runs
```

Not every directory is present in every branch.

## Using the experimental branches

To inspect or reproduce a specific configuration, first select its branch.

For example:

```bash
git switch exp/E3-delay-injection
```

or:

```bash
git switch turtlebot3/E3
```

After switching branches, read the specific README before building or executing the experiment.
