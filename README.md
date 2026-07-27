# T-E3 Experiment: Local-Costmap Update Delay

Experiment T-E3 evaluates the effects of a controlled software delay introduced into the local-costmap producer task of the ROS Navigation Stack.

The purpose of the experiment is to determine if delaying the production of the local costmap generates a measurable update temporal displacement and if this internal timing violation propagates to velocity-command generation and navigation behavior.

The delay is inserted inside:

```cpp
Costmap2DROS::mapUpdateLoop()
```

immediately before:

```cpp
updateMap();
```

The experiment affects only the local costmap.

## Experimental configuration

The experiment uses:

* ROS Noetic;
* Gazebo;
* TurtleBot3 Burger;
* the `turtlebot3_world` simulation environment;
* the ROS Navigation Stack;
* the same static map, initial pose, navigation goal, and termination criteria used by the other TurtleBot3 experiments;
* source-level timing instrumentation in the local-costmap update loop;
* a controlled delay applied immediately before the local costmap is updated;
* a custom CSV logger for internal local-costmap timing measurements;
* ROS bag recording for externally observable navigation behavior.

The tested delay conditions are:

```text
0 ms
200 ms
500 ms
1000 ms
2000 ms
```

The 0-ms configuration retains the complete source-level instrumentation and logging code without introducing an intentional suspension. It is the matched instrumented baseline for the delayed conditions.

## Injection point

The modified execution sequence is conceptually:

```text
Costmap2DROS::mapUpdateLoop()
    |
    ├── begin local-costmap update iteration
    ├── collect timing information
    ├── apply the configured delay
    ├── updateMap()
    ├── record update completion
    └── write the internal timing measurements
```

The intentional suspension occurs before `updateMap()`.

While the local-costmap thread is suspended:

* the robot may continue moving;
* odometry and laser data may continue evolving;
* the latest completed local costmap remains available;
* the next local-costmap refresh is postponed;
* the local controller may continue operating using the most recently completed costmap.

Hence, the experiment investigates the temporal freshness of the obstacle representation rather than directly delaying velocity-command computation.

## Workspace structure

The experiment is contained in the following catkin workspace:

```text
TurtleBot3/
└── catkin_ws/
    └── src/
        ├── navigation/
        └── turtlebot3_simulations/
```

The T-E3 source-level modification is located inside the ROS Navigation code responsible for the `Costmap2DROS` update loop.

## How to build the experiment

1. **Select the E3 branch:**

   From the repository root, run:

   ```bash
   git switch turtlebot3/E3
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

   Configure the E3 delay to one of the following values:

   ```text
   0 ms
   200 ms
   500 ms
   1000 ms
   2000 ms
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

Wait until the estimated robot pose have stabilized before starting the recording.

### 4. Prepare the output directory

Open a third terminal and create a directory for the selected delay condition.

For example, for the 200-ms condition:

```bash
mkdir -p TurtleBot3_experiments_results/E3/200ms
```

Use separate directories for all conditions:

```text
TurtleBot3_experiments_results/E3/0ms/
TurtleBot3_experiments_results/E3/200ms/
TurtleBot3_experiments_results/E3/500ms/
TurtleBot3_experiments_results/E3/1000ms/
TurtleBot3_experiments_results/E3/2000ms/
```

### 5. Start ROS bag recording

Start the ROS bag recording before sending the navigation goal.

For example:

```bash
rosbag record \
  -O TurtleBot3_experiments_results/E3/200ms/E3_200ms_run1.bag \
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
E3_0ms_run1.bag
E3_200ms_run1.bag
E3_500ms_run1.bag
E3_1000ms_run1.bag
E3_2000ms_run1.bag
```

### 6. Preserve the internal timing CSV

The T-E3 instrumentation generates a source-level CSV containing one row for each instrumented local-costmap update iteration.

Before starting the navigation task:

* verify where the T-E3 logger creates the CSV;
* make sure an old result file will not be mistaken for the current run;
* remove or rename any previous CSV when required.

After the run, move or copy the CSV into the directory associated with the current delay and repetition.

Use a descriptive name such as:

```text
E3_200ms_run1_internal.csv
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
* timed out;
* collided with an obstacle;
* showed irregular motion;
* showed alternating rotations or reverse movements;
* presented any other unexpected behavior.

## Number of runs

The final study used the following repetitions:

| Delay condition | Independent runs |
| --------------- | ---------------- |
| 0 ms            | 3                |
| 200 ms          | 3                |
| 500 ms          | 3                |
| 1000 ms         | 5                |
| 2000 ms         | 3                |

The 1000-ms condition includes two additional runs because irregular startup motion was observed during an initial execution.

When reproducing the experiment with a different number of repetitions, use:

```text
E3_<delay>_run1
E3_<delay>_run2
E3_<delay>_run3
...
E3_<delay>_runN
```

Before each independent run:

1. restart or reset the simulation;
2. restore the same robot model and simulation world;
3. set the same initial pose;
4. use the same static map and navigation goal;
5. select and verify the intended delay condition;
6. start a new ROS bag recording;
7. generate a separate internal timing CSV;
8. apply the same termination criteria.

## Recorded internal measurements

The local-costmap logger records timing information for each instrumented update iteration.

The measurements are used to derive:

* requested software delay;
* measured software delay;
* delay error;
* execution duration of `updateMap()`;
* interval between consecutive completed local-costmap updates;
* age of the previous completed update immediately before the next refresh;
* local-costmap update temporal displacement.

The measured delay must use a monotonic wall clock around the intentional suspension.

Logging and file output should remain outside the interval used to calculate the primary timing metric whenever possible, so that CSV disk operations are not included in the measured execution time.

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

## Evaluation metrics

The source-level T-E3 measurements are analyzed together with the common end-to-end navigation metrics.

The primary internal metrics are:

* requested delay;
* measured delay;
* delay error;
* local-costmap update-completion interval;
* local-costmap update temporal displacement;
* previous-update age;
* `updateMap()` execution duration.

The externally observable metrics include:

* mission success or failure;
* mission duration;
* interval between consecutive `/cmd_vel` messages;
* effective command frequency;
* fraction of almost-zero commands;
* odometric path length;
* commanded linear and angular velocities;
* minimum valid laser range;
* recovery activity;
* collision observations;
* timeouts or missing navigation results.

For runs showing irregular motion immediately after the goal, the first five seconds should also be analyzed using:

* travelled distance;
* net displacement;
* path efficiency;
* changes in the sign of the angular command;
* reverse-command count;
* time to first relevant motion.

## Experimental results

Generated ROS bags, timing CSV files, and run-level metadata are experimental outputs and must be stored separately from the source-code branch.

The final T-E3 results should be maintained under:

```text
TurtleBot3_experiments_results/E3/
```

A recommended organization is:

```text
TurtleBot3_experiments_results/
└── E3/
    ├── 0ms/
    ├── 200ms/
    ├── 500ms/
    ├── 1000ms/
    └── 2000ms/
```
