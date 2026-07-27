# T-E4 Experiment: Delay Before Local-Planner Execution

Experiment T-E4 evaluates the effects of a controlled software delay introduced inside the `move_base` control cycle immediately before invocation of the local planner.

The purpose of the experiment is to determine if delaying the start of local-planner computation increases scheduling displacement and command-publication latency, if this internal timing violation propagates to the `/cmd_vel` stream, and if it affects mission-level navigation behavior.

The delay is inserted inside:

```cpp
MoveBase::executeCycle()
```

immediately before:

```cpp
tc_->computeVelocityCommands(cmd_vel);
```

The experiment postpones the invocation of the local planner. It does not intentionally increase the computational cost of `computeVelocityCommands()` after the planner starts executing.

## Experimental configuration

The experiment uses:

* ROS Noetic;
* Gazebo;
* TurtleBot3 Burger;
* the `turtlebot3_world` simulation environment;
* the ROS Navigation Stack;
* the same static map, initial pose, navigation goal, and termination criteria used by the other TurtleBot3 experiments;
* source-level timing instrumentation inside `MoveBase::executeCycle()`;
* a controlled delay applied immediately before local-planner invocation;
* a custom CSV logger for internal controller and command-publication timing measurements;
* ROS bag recording for externally observable navigation behavior.

The tested delay conditions are:

```text
0 ms
20 ms
50 ms
100 ms
200 ms
500 ms
1000 ms
```

The 0-ms configuration retains the complete source-level instrumentation and logging code without introducing an intentional suspension. It is the matched instrumented baseline for the delayed conditions.

## Injection point

The modified execution sequence is conceptually:

```text
MoveBase::executeCycle()
    |
    ├── begin the control cycle
    ├── record the control-cycle start time
    ├── prepare the local-planner input state
    ├── apply the configured delay
    ├── record the planner-invocation time
    ├── tc_->computeVelocityCommands(cmd_vel)
    ├── record planner completion and command validity
    ├── publish the resulting velocity command
    ├── record command-publication timing
    └── write the internal timing measurements
```

The intentional suspension occurs after the `move_base` control cycle has started and immediately before the local planner is invoked.

While `MoveBase::executeCycle()` is suspended:

* localization, odometry, laser, and costmap data may continue evolving;
* the local planner has not yet computed the next velocity command;
* the previously published command may remain the latest command available to the robot;
* publication of the next `/cmd_vel` message is postponed;
* the planner computation itself is not intentionally delayed after it starts.

Hence, the experiment distinguishes a planner that starts late from a planner whose internal computation becomes more expensive.

## Workspace structure

The experiment is contained in the following catkin workspace:

```text
TurtleBot3/
└── catkin_ws/
    └── src/
        ├── navigation/
        └── turtlebot3_simulations/
```

The T-E4 source-level modification is located inside the ROS Navigation code responsible for `MoveBase::executeCycle()`.

## How to build the experiment

1. **Select the E4 branch:**

   From the repository root, run:

   ```bash
   git switch turtlebot3/E4
   ```

2. **Source ROS Noetic:**

   ```bash
   source /opt/ros/noetic/setup.bash
   ```

3. **Select the TurtleBot3 Burger model:**

   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

4. **Select the delay condition:**

   Configure the E4 delay to one of the following values:

   ```text
   0 ms
   20 ms
   50 ms
   100 ms
   200 ms
   500 ms
   1000 ms
   ```

5. **Open the catkin workspace:**

   ```bash
   cd TurtleBot3/catkin_ws
   ```

6. **Install missing dependencies, when required:**

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

7. **Build the workspace:**

   ```bash
   catkin_make
   ```

8. **Source the workspace:**

   ```bash
   source devel/setup.bash
   ```

Rebuild the workspace anytime the delay value is changed directly in the source code.

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

Launch the TurtleBot3 navigation configuration using the project map:

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

Wait until RViz, AMCL, `move_base`, the local and global costmaps, and the navigation plugins have initialized.

### 3. Set the initial pose

In RViz, use **2D Pose Estimate** to assign the predefined initial position and orientation of the robot.

Use the same initial pose for every run and every delay condition.

Wait until the estimated robot pose has stabilized before starting the recording.

### 4. Prepare the output directory

Open a third terminal and create a directory for the selected delay condition.

For example, for the 200-ms condition:

```bash
mkdir -p TurtleBot3_experiments_results/E4/200ms
```

Use separate directories for all conditions:

```text
TurtleBot3_experiments_results/E4/0ms/
TurtleBot3_experiments_results/E4/20ms/
TurtleBot3_experiments_results/E4/50ms/
TurtleBot3_experiments_results/E4/100ms/
TurtleBot3_experiments_results/E4/200ms/
TurtleBot3_experiments_results/E4/500ms/
TurtleBot3_experiments_results/E4/1000ms/
```

### 5. Start ROS bag recording

Start the ROS bag recording before sending the navigation goal.

For example:

```bash
rosbag record \
  -O TurtleBot3_experiments_results/E4/200ms/E4_200ms_run1.bag \
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

Change the output path according to the selected delay and repetition.

Examples include:

```text
E4_0ms_run1.bag
E4_20ms_run1.bag
E4_50ms_run1.bag
E4_100ms_run1.bag
E4_200ms_run1.bag
E4_500ms_run1.bag
E4_1000ms_run1.bag
```

### 6. Preserve the internal timing CSV

The T-E4 instrumentation generates a source-level CSV containing one row for each instrumented `move_base` control cycle.

Before starting the navigation task:

* verify where the T-E4 logger creates the CSV;
* make sure an old result file will not be mistaken for the current run;
* remove or rename any previous CSV when required;
* verify that the CSV contains the expected header;
* verify that the configured delay is the intended one.

After the run, move or copy the CSV into the directory associated with the current delay and repetition.

Use a descriptive name such as:

```text
E4_200ms_run1_internal.csv
```

Do not reuse one result filename for multiple runs.

### 7. Send the navigation goal

In RViz, use **2D Nav Goal** to send the predefined project goal.

Alternatively, publish the same goal used by the clean baseline:

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"{ header: { frame_id: 'map' }, pose: { position: { x: 1.3350000381469727, y: 1.3899997472763062, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: -0.10608347709795334, w: 0.9943572275026757 } } }"
```

The same goal must be used for every run and delay condition.

The effective mission interval begins when the `/move_base_simple/goal` message is published and ends when the corresponding `/move_base/result` message is received.

For a run without a terminal result, record the observation duration from the goal timestamp to the end of the bag. Do not treat this value as a successful mission-completion time.

### 8. End the recording

After the robot reaches the goal or the run reaches the defined termination condition, stop `rosbag record` with:

```text
Ctrl+C
```

Preserve both:

* the ROS bag;
* the source-level timing CSV.

Record whether the run:

* reached the goal successfully;
* entered a recovery behavior;
* was aborted;
* timed out;
* produced no terminal result;
* collided with an obstacle;
* showed irregular motion;
* showed alternating rotations or reverse movements;
* presented any other unexpected behavior.

## Number of runs

The final study used the following repetitions:

| Delay condition | Independent runs |
| --------------- | ---------------- |
| 0 ms            | 5                |
| 20 ms           | 5                |
| 50 ms           | 5                |
| 100 ms          | 5                |
| 200 ms          | 5                |
| 500 ms          | 5                |
| 1000 ms         | 5                |
| **Total**       | **35**           |

When reproducing the experiment with a different number of repetitions, use:

```text
E4_<delay>_run1
E4_<delay>_run2
E4_<delay>_run3
...
E4_<delay>_runN
```

Before each independent run:

1. restart or reset the simulation;
2. restore the same robot model and simulation world;
3. set the same initial pose;
4. wait until the localization estimate has stabilized;
5. use the same static map and navigation goal;
6. select and verify the intended delay condition;
7. start a new ROS bag recording;
8. generate a separate internal timing CSV;
9. apply the same termination criteria.

## Recorded internal measurements

The `move_base` logger records timing information for each instrumented local-control cycle.

The measurements are used to derive:

* requested software delay;
* measured software delay;
* delay error;
* control-cycle start time;
* planner-invocation time;
* planner-completion time;
* command-ready time;
* command-publication time;
* scheduling displacement;
* planner-computation time;
* command-ready latency;
* command-publication latency;
* interval between consecutive command publications;
* effective command-publication frequency;
* valid-command ratio;
* nominal-period exceedance or deadline-miss proxy, when defined.

The primary scheduling displacement is measured from the beginning of the control cycle to invocation of the local planner:

```text
scheduling displacement =
    planner-invocation time
    -
    control-cycle start time
```

This metric represents the delay accumulated before `computeVelocityCommands()` starts.

Planner-computation time is:

```text
planner-computation time =
    planner-completion time
    -
    planner-invocation time
```

This interval must include only execution of `computeVelocityCommands()` and must exclude the intentional pre-planner suspension.

Command-ready latency can be defined as:

```text
command-ready latency =
    command-ready time
    -
    control-cycle start time
```

Command-publication latency is:

```text
command-publication latency =
    command-publication time
    -
    control-cycle start time
```

It represents the time required for a new velocity command to become externally observable after the control cycle begins.

The measured delay must use a monotonic wall clock around the intentional suspension.

Logging and file output should remain outside the intervals used to calculate the primary timing metrics whenever possible, so that CSV disk operations are not included in:

* measured delay;
* scheduling displacement;
* planner-computation time;
* command-publication latency.

The CSV should contain an explicit header and use consistent measurement units across all runs.

## Recorded ROS topics

The common ROS bag records:

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
* the navigation result;
* the velocity-command stream;
* the odometric trajectory;
* localization estimates;
* laser measurements;
* TF transformations;
* obstacle proximity;
* recovery and navigation-status events.

The `/cmd_vel` topic is particularly important for T-E4 because it makes it possible to verify whether internal scheduling displacement and command-publication latency propagate to the externally observable control-output stream.

## Evaluation metrics

The source-level T-E4 measurements are analyzed together with the common end-to-end navigation metrics.

The primary internal metrics are:

* requested delay;
* measured delay;
* delay error;
* scheduling displacement;
* planner-computation time;
* command-ready latency;
* command-publication latency;
* command-publication interval;
* effective command-publication frequency;
* valid-command ratio;
* nominal-period exceedance or deadline-miss proxy, when defined.

The externally observable metrics include:

* mission success or failure;
* mission duration;
* observation duration for runs without a terminal result;
* mean, median, 95th-percentile, and maximum interval between consecutive `/cmd_vel` messages;
* effective command frequency;
* fraction of almost-zero commands;
* odometric path length;
* commanded linear and angular velocities;
* minimum valid laser range;
* recovery activity;
* collision observations;
* aborted missions;
* timeouts or missing navigation results.

An almost-zero velocity command is a `/cmd_vel` message satisfying both:

```text
|linear.x| < 0.01 m/s
|angular.z| < 0.05 rad/s
```

The median command gap represents typical output periodicity, while the 95th percentile and maximum characterize occasional long interruptions.

The 20-ms and 50-ms conditions evaluate whether the additional latency can be absorbed inside the nominal 100-ms controller period.

The 100-ms condition represents a transition in which the intentional delay becomes comparable to the nominal controller period.

The 200-ms, 500-ms, and 1000-ms conditions evaluate progressively stronger propagation to the `/cmd_vel` stream and to mission-level navigation availability.

Planner-computation time must be analyzed separately from scheduling displacement. A large scheduling displacement together with comparatively stable planner-computation time indicates that the planner starts late but does not become computationally slower.

Mission success alone is not considered sufficient evidence of nominal behavior. A run may reach the goal after:

* large command-publication delays;
* reduced command frequency;
* intermittent stopping;
* increased path length;
* recovery activity;
* substantially increased completion time.

For runs showing irregular motion immediately after the goal, the first five seconds should also be analyzed using:

* travelled distance;
* net displacement;
* path efficiency;
* changes in the sign of the angular command;
* reverse-command count;
* time to first relevant motion.

Every metric should first be calculated separately for each independent run.

Condition-level summaries should then report:

* number of valid runs;
* number of successful runs;
* mean or median, depending on the metric;
* sample standard deviation where appropriate;
* minimum;
* 95th percentile;
* maximum;
* anomalous outcomes and missing data.

The 0-ms T-E4 condition is the direct reference for evaluating every delayed condition.

## Experimental results

Generated ROS bags, timing CSV files, and run-level metadata are experimental outputs and must be stored separately from the source-code branch.

The final T-E4 results should be maintained under:

```text
TurtleBot3_experiments_results/E4/
```

A recommended organization is:

```text
TurtleBot3_experiments_results/
└── E4/
    ├── 0ms/
    ├── 20ms/
    ├── 50ms/
    ├── 100ms/
    ├── 200ms/
    ├── 500ms/
    └── 1000ms/
```

Each delay directory should contain:

* one ROS bag for each independent run;
* one internal timing CSV for each independent run;
* optional run-level notes or metadata;
* no files belonging to another delay condition.

A more detailed organization can use:

```text
TurtleBot3_experiments_results/
└── E4/
    ├── 0ms/
    │   ├── E4_0ms_run1.bag
    │   ├── E4_0ms_run1_internal.csv
    │   └── ...
    ├── 20ms/
    ├── 50ms/
    ├── 100ms/
    ├── 200ms/
    ├── 500ms/
    └── 1000ms/
```

Raw ROS bags and source-level timing CSV files should not be overwritten after the final dataset has been validated.