# T-E1 Experiment: AMCL Pose-Publication Delay

Experiment T-E1 evaluates the effects of a controlled software delay introduced before publication of the pose estimate produced by AMCL.

The purpose of the experiment is to determine if delaying the delivery of the localization result generates a measurable pose temporal displacement and if this internal timing violation propagates to velocity-command generation and navigation behavior.

The delay is inserted inside the AMCL laser-processing callback, immediately before:

```cpp
pose_pub_.publish(...);
```

The experiment affects only the publication time of the AMCL pose estimate. The pose values are not intentionally modified.

## Experimental configuration

The experiment uses:

* ROS Noetic;
* Gazebo;
* TurtleBot3 Burger;
* the `turtlebot3_world` simulation environment;
* the ROS Navigation Stack;
* the same static map, initial pose, navigation goal, and termination criteria used by the other TurtleBot3 experiments;
* source-level timing instrumentation in the AMCL pose-publication path;
* a controlled delay applied immediately before the AMCL pose is published;
* a custom CSV logger for internal pose-publication timing measurements;
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
AMCL laser-processing callback
    |
    ├── receive and process the source laser scan
    ├── calculate the new pose estimate
    ├── preserve the source scan timestamp
    ├── collect timing information
    ├── apply the configured delay
    ├── publish the AMCL pose
    ├── record the publication time
    └── write the internal timing measurements
```

The intentional suspension occurs after AMCL has calculated the pose estimate and before the pose is published.

While the AMCL callback is suspended:

* the robot may continue moving;
* odometry and laser data may continue evolving;
* downstream components continue using the most recently published localization result;
* the newly computed pose becomes progressively older before it is made available;
* costmaps and the local controller may continue operating with stale localization information.

Hence, the experiment investigates the temporal freshness of the localization result rather than intentionally corrupting the estimated pose.

## Workspace structure

The experiment is contained in the following catkin workspace:

```text
TurtleBot3/
└── catkin_ws/
    └── src/
        ├── navigation/
        └── turtlebot3_simulations/
```

The T-E1 source-level modification is located inside the AMCL code path responsible for publishing the estimated pose.

## How to build the experiment

1. **Select the E1 branch:**

   From the repository root, run:

   ```bash
   git switch turtlebot3/E1
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

   Configure the E1 delay to one of the following values:

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

### 3. Set the initial pose

In RViz, use **2D Pose Estimate** to assign the predefined initial position and orientation of the robot.

Use the same initial pose for every run and every delay condition.

Wait until the estimated robot pose has stabilized before starting the recording.

### 4. Prepare the output directory

Open a third terminal and create a directory for the selected delay condition.

For example, for the 200-ms condition:

```bash
mkdir -p TurtleBot3_experiments_results/E1/200ms
```

Use separate directories for all conditions:

```text
TurtleBot3_experiments_results/E1/0ms/
TurtleBot3_experiments_results/E1/20ms/
TurtleBot3_experiments_results/E1/50ms/
TurtleBot3_experiments_results/E1/100ms/
TurtleBot3_experiments_results/E1/200ms/
TurtleBot3_experiments_results/E1/500ms/
TurtleBot3_experiments_results/E1/1000ms/
```

### 5. Start ROS bag recording

Start the ROS bag recording before sending the navigation goal.

For example:

```bash
rosbag record \
  -O TurtleBot3_experiments_results/E1/200ms/E1_200ms_run1.bag \
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
E1_0ms_run1.bag
E1_20ms_run1.bag
E1_50ms_run1.bag
E1_100ms_run1.bag
E1_200ms_run1.bag
E1_500ms_run1.bag
E1_1000ms_run1.bag
```

### 6. Preserve the internal timing CSV

The T-E1 instrumentation generates a source-level CSV containing one row for each instrumented AMCL pose-publication event.

Before starting the navigation task:

* verify where the T-E1 logger creates the CSV;
* make sure an old result file will not be mistaken for the current run;
* remove or rename any previous CSV when required;
* verify that the CSV contains the expected header.

After the run, move or copy the CSV into the directory associated with the current delay and repetition.

Use a descriptive name such as:

```text
E1_200ms_run1_internal.csv
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
E1_<delay>_run1
E1_<delay>_run2
E1_<delay>_run3
...
E1_<delay>_runN
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

The AMCL pose-publication logger records timing information for each instrumented pose-publication event.

The measurements are used to derive:

* requested software delay;
* measured software delay;
* delay error;
* timestamp of the source laser scan or corresponding pre-publication reference;
* time at which the resulting AMCL pose is published;
* pose temporal displacement;
* number of instrumented pose-publication events.

The primary internal metric is the pose temporal displacement:

```text
pose temporal displacement =
    pose publication time
    -
    source laser-scan timestamp
```

This value represents the age of the localization result when it becomes available to downstream components.

It includes:

* normal AMCL processing latency;
* the intentional software delay;
* scheduling and wake-up overhead;
* the remaining publication overhead.

The measured delay must use a monotonic wall clock around the intentional suspension.

Logging and file output should remain outside the interval used to measure the intentional delay whenever possible, so that CSV disk operations are not included in the measured delay.

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
* the source laser measurements;
* the localization estimates;
* the TF transformations;
* obstacle proximity;
* recovery and navigation-status events.

## Evaluation metrics

The source-level T-E1 measurements are analyzed together with the common end-to-end navigation metrics.

The primary internal metrics are:

* requested delay;
* measured delay;
* delay error;
* pose temporal displacement.

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

The median command gap represents typical output periodicity, while the 95th percentile and maximum characterize occasional long interruptions that may not be visible in the median.

Mission success alone is not considered sufficient evidence of nominal behavior. A run may reach the goal after substantial pose temporal displacement, intermittent stopping, trajectory inefficiency, recovery activity, or a large increase in mission duration.

For runs showing irregular motion immediately after the goal, the first five seconds should also be analyzed using:

* travelled distance;
* net displacement;
* path efficiency;
* changes in the sign of the angular command;
* reverse-command count;
* time to first relevant motion.

Every metric should first be calculated separately for each independent run. Condition-level summaries should then report the number of valid runs and suitable aggregate statistics, such as median, mean, sample standard deviation, minimum, 95th percentile, and maximum.

## Experimental results

Generated ROS bags, timing CSV files, and run-level metadata are experimental outputs and must be stored separately from the source-code branch.

The final T-E1 results should be maintained under:

```text
TurtleBot3_experiments_results/E1/
```

A recommended organization is:

```text
TurtleBot3_experiments_results/
└── E1/
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

Raw ROS bags and source-level timing CSV files should not be overwritten after the final dataset has been validated.