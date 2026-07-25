# Main Baseline: TurtleBot3 Timing-Violation Experiments

This branch contains the common baseline used for the TurtleBot3 timing-violation experiments.

The case study is implemented as a ROS Noetic catkin workspace and uses Gazebo to simulate the TurtleBot3 mobile robot. The workspace includes the ROS Navigation stack and the TurtleBot3 simulation packages required by the experimental configurations.

This branch provides the source code and directory structure from which the TurtleBot3 experiments are derived. It does not contain an experiment-specific perturbation. Each experimental configuration is maintained in a dedicated branch containing its own code modifications and a customized `README.md` with the corresponding execution instructions.

## Experiment branches

The TurtleBot3 case study is organized into the following branches:

* `turtlebot3/E0`
* `turtlebot3/E1`
* `turtlebot3/E2`
* `turtlebot3/E3`
* `turtlebot3/E4`
* `turtlebot3/E5`

Each branch represents a separate experimental configuration.

## Workspace structure

The TurtleBot3 source code is organized as a ROS catkin workspace:

```text
TurtleBot3/
└── catkin_ws/
    └── src/
        ├── navigation/
        └── turtlebot3_simulations/
```

### `navigation/`

This directory contains the ROS Navigation stack used by the case study.

The imported source code is based on the `noetic-devel` branch of:

[ros-planning/navigation](https://github.com/ros-planning/navigation/tree/noetic-devel)

The stack provides the ROS packages used for localization, path planning, costmap management, and mobile-robot navigation.

### `turtlebot3_simulations/`

This directory contains the Gazebo simulation packages used to execute the TurtleBot3 experiments.

The imported source code is based on the `noetic` branch of:

[ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/noetic)

The package provides the TurtleBot3 Gazebo models, simulation worlds, launch files, and related simulation components.

## Prerequisites

The baseline requires a ROS Noetic environment with the dependencies needed by the Navigation stack and TurtleBot3 simulation packages.

Before building the workspace, make sure that:

* ROS Noetic is installed and sourced;
* the required TurtleBot3 packages are installed;
* Gazebo is available;
* all ROS package dependencies have been installed;
* the `TURTLEBOT3_MODEL` environment variable is set to the model used by the experiment.

Source ROS Noetic with:

```bash
source /opt/ros/noetic/setup.bash
```

The TurtleBot3 model can be selected with:

```bash export TURTLEBOT3_MODEL=<model> ```

Replace `<model>` with the TurtleBot3 model used by the experimental configuration  (e.g., ```bash export TURTLEBOT3_MODEL=<model> ```).

## How to build the workspace

1. **Open the catkin workspace:**

   From the repository root, run:

   ```bash
   cd TurtleBot3/catkin_ws
   ```

2. **Install missing ROS dependencies, when required:**

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**

   ```bash
   catkin_make
   ```

4. **Source the workspace:**

   ```bash
   source devel/setup.bash
   ```

The workspace must be sourced in every terminal used to launch a TurtleBot3 node or simulation.

## How to run the baseline simulation

After building and sourcing the workspace, the standard TurtleBot3 Gazebo world can be launched with:

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

This command launches the common simulation environment.

The complete launch procedure used for a specific experiment—including navigation setup, goal configuration, interference processes, logging nodes, and termination conditions—is documented in the corresponding experimental branch.

## How to use the repository

1. **Select an experiment branch:**

   ```bash
   git switch turtlebot3/E0
   ```

   Replace `turtlebot3/E0` with the branch corresponding to the experiment that you want to execute.

2. **Read the branch-specific instructions:**

   Each experimental branch contains a dedicated `README.md` describing:

   * the purpose of the configuration;
   * the changes with respect to the baseline;
   * the applied interference or timing perturbation;
   * the required build procedure;
   * the commands used to launch the simulation;
   * the logging and data-collection procedure.

3. **Build the selected version:**

   ```bash
   cd TurtleBot3/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Run the experiment:**

   Follow the instructions contained in the selected branch rather than the instructions from another experimental branch.

## Experimental results

The experimental results are maintained separately from the source-code configurations.

The results collected from the final TurtleBot3 runs, together with the corresponding analysis scripts, will be stored in:

```text
TurtleBot3_experiments_results/
```

This separation keeps generated data and analysis outputs distinct from the source code required to reproduce each experiment.

## Repository rules

The `turtlebot3/main-baseline` branch must contain only the code and documentation shared by the TurtleBot3 experiments.

Experiment-specific changes are in their corresponding branches:

```text
turtlebot3/E0
turtlebot3/E1
turtlebot3/E2
turtlebot3/E3
turtlebot3/E4
turtlebot3/E5
```

## Third-party software

This branch contains source code originating from the following open-source projects:

* ROS Navigation;
* TurtleBot3 Simulations.

The original license files, package metadata, copyright notices, and source-code headers must be retained.

Refer to the repository-level `LICENSE.md` and to the license information included in each upstream package for the applicable terms.
