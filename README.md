# T-E0 Experiment: Clean Navigation Baseline

Experiment T-E0 represents the clean navigation baseline of the TurtleBot3 case study.

It uses the unmodified ROS Navigation Stack and does not introduce any intentional software delay or source-level timing instrumentation.

The purpose of this experiment is to characterize the nominal timing and navigation behavior of the simulated TurtleBot3 before introducing the timing perturbations implemented in experiments T-E1 -- T-E5.

T-E0 also provides an independent reference for evaluating whether the instrumentation used by the zero-delay configurations of the other experiments changes the normal behavior of the navigation stack.

## Experimental configuration

The experiment uses:

* ROS Noetic;
* Gazebo;
* TurtleBot3 Burger;
* the `turtlebot3_world` simulation environment;
* the ROS Navigation Stack;
* the same static map, initial pose, navigation goal, and termination criteria used by the other TurtleBot3 experiments;
* no intentional delay injection;
* no modification of the selected localization, costmap, or controller injection points.

The navigation task requires the robot to move through the simulated environment, pass close to obstacles, and perform multiple turns before reaching the assigned goal.

The simulated environment remains static during the experiment. No dynamic obstacle is intentionally introduced.

## Workspace structure

The experiment is contained in the following catkin workspace:

```text
TurtleBot3/
└── catkin_ws/
    └── src/
        ├── navigation/
        └── turtlebot3_simulations/
```

The `navigation/` directory contains the ROS Navigation Stack, while `turtlebot3_simulations/` contains the Gazebo models, worlds, and launch files required by the simulation.

## How to build the experiment

1. **Select the  branch:**

   From the repository root, run:

   ```bash
   git switch turtlebot3/E0
   ```

2. **Source ROS Noetic:**

   ```bash
   source /opt/ros/noetic/setup.bash
   ```

3. **Select the TurtleBot3 Burger model:**

   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

4. **Open the catkin workspace:**

   ```bash
   cd TurtleBot3/catkin_ws
   ```

5. **Install missing dependencies, when required:**

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

6. **Build the workspace:**

   ```bash
   catkin_make
   ```

7. **Source the workspace:**

   ```bash
   source devel/setup.bash
   ```

## How to run the experiment

### 1. Start the Gazebo simulation

Open a terminal, source the environment, and launch the TurtleBot3 world:

```bash
source /opt/ros/noetic/setup.bash
source TurtleBot3/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Wait until Gazebo has completely loaded the robot and the simulation world.

### 2. Start the navigation stack

Open a second terminal and source the same environment:

```bash
source /opt/ros/noetic/setup.bash
source TurtleBot3/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
```

Launch the TurtleBot3 navigation configuration using the map adopted for the project:

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

### 3. Set the initial pose

In RViz, use **2D Pose Estimate** to assign the predefined initial position and orientation of the robot.

The same initial pose must be used for every T-E0 run and for the corresponding runs of the other TurtleBot3 experiments.

Wait until the AMCL particle cloud and the estimated robot pose have stabilized before starting the recording.

### 4. Start ROS bag recording

Open a third terminal and create a directory for the current run:

```bash
mkdir -p TurtleBot3_experiments_results/E0
```

Start the ROS bag recording:

```bash
rosbag record \
  -O TurtleBot3_experiments_results/E0/E0_run1.bag \
  /cmd_vel \
  /odom \
  /scan \
  /amcl_pose \
  /tf \
  /tf_static \
  /move_base/status \
  /move_base/result \
  /initialpose \
  /move_base_simple/goal \
  /clock
```

Change `E0_run1.bag` according to the current repetition:

```text
E0_run1.bag
E0_run2.bag
E0_run3.bag
E0_run4.bag
E0_run5.bag
```

### 5. Send the navigation goal

In RViz, use **2D Nav Goal** to send the predefined project goal.

The goal must be identical in every run.

Giving an example, we inserted the same goal by using the following command in a new terminal:
```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{ header: { frame_id: 'map' }, pose: { position: { x: 1.3350000381469727, y: 1.3899997472763062, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: -0.10608347709795334, w: 0.9943572275026757 } } }"
```

The effective mission interval begins when the `/move_base_simple/goal` message is published and ends when the corresponding `/move_base/result` message is received.

### 6. End the recording

After the robot reaches the goal or the run reaches the defined termination condition, stop `rosbag record` with:

```text
Ctrl+C
```

Record if the run:

* reached the goal successfully;
* entered a recovery behavior;
* timed out;
* collided with an obstacle;
* showed any other irregular behavior.

## Number of runs

If you want to execute N independent runs:

```text
E0_run1
E0_run2
E0_run3
...
E0_runN
```

before each run:

1. restart or reset the simulation;
2. restore the same robot model and simulation world;
3. set the same initial pose;
4. use the same map and navigation goal;
5. start a new ROS bag recording;
6. apply the same run-termination criteria.

The runs must be treated as independent repetitions.

## Recorded topics

The common ROS bag records the following topics:

```text
/cmd_vel
/odom
/scan
/amcl_pose
/tf
/tf_static
/move_base/status
/move_base/result
/initialpose
/move_base_simple/goal
/clock
```

These topics make it possible to reconstruct:

* the beginning and end of the navigation task;
* the final action result;
* the velocity-command stream;
* the odometric trajectory;
* the robot localization estimate;
* the laser measurements;
* the TF transformation history;
* the minimum observed laser range;
* recovery or navigation-status events.

## Evaluation metrics

Since T-E0 does not contain internal source-level timing instrumentation, its timing behavior is evaluated through externally observable ROS messages.

The main metrics include:

* mission success or failure;
* mission duration;
* interval between consecutive `/cmd_vel` messages;
* effective velocity-command frequency;
* fraction of almost-zero velocity commands;
* odometric path length;
* commanded linear velocity;
* commanded angular velocity;
* minimum valid laser range;
* recovery activity;
* observed collisions;
* timeouts or missing navigation results.

The nominal controller frequency is 10 Hz, corresponding to an expected interval of approximately 100 ms between consecutive velocity commands.

## Expected role of T-E0

T-E0 is not a zero-delay version of an instrumented experiment.

It is an independent clean baseline in which:

* the ROS Navigation Stack remains unmodified;
* no `usleep()` operation is added;
* no experiment-specific timing logger is inserted into the selected source-level injection points;
* only external ROS bag recording is maintained.

T-E0 must therefore be used to determine whether the instrumentation present in the 0-ms configurations of T-E1 -- T-E5 introduces measurable overhead or changes the nominal navigation behavior.

## Experimental results

The generated ROS bag files and the corresponding run-level metadata are experimental outputs and should be stored separately from the source code.

The final T-E0 results will be maintained under:

```text
TurtleBot3_experiments_results/E0/
```

Generated bag files must not be committed directly to the source-code branch unless they are intentionally selected for publication in the results directory.

## Third-party software

This branch contains source code originating from:

* [ROS Navigation](https://github.com/ros-planning/navigation/tree/noetic-devel);
* [TurtleBot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/noetic).

The original license files, package metadata, copyright notices, and source-code headers must be retained.

Refer to the repository-level `LICENSE.md` and to the license information included in each upstream package for the applicable terms.
