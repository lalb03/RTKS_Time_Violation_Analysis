# E4 Experiment: External Cache Interference

Experiment E4 builds upon the E2 architecture. It replaces the explicit software delay used in E3 with an external cache aggressor.

The aggressor is activated when the drone starts performing the second curve of the mission `way.txt`.

When the current mission navigation index changes from waypoint 2 to waypoint 3, ArduPilot creates the following trigger file:

```text
/dev/shm/timetrap_e4.trigger
```

An external script waits for this file and starts a stress-ng cache worker for 6 seconds.

The aggressor configuration is:
```bash
Scheduling policy: SCHED_FIFO
Real-time priority: 20
CPU affinity: virtual CPU 0
Cache workers: 1
Target cache level: L3
Attack duration: 6 seconds
Trigger: NAV_INDEX transition from 2 to 3
```
The resulting priority hierarchy is:
```bash
Sensor thread:     SCHED_FIFO 80
Control thread:    SCHED_FIFO 60
Cache aggressor:   SCHED_FIFO 20
```

## How to run the experiment
1. **Start the External Aggressor:**
   Open a terminal in the `ArduPilot` root directory and run:
```bash
sudo ./tt_aggressor.sh | tee e4_aggressor_output.txt
```
The script should display:
```bash
[TimeTrap E4] Waiting for ArduPilot trigger...
```
At this point, the aggressor is inactive and is only waiting for the trigger file.

2. **Start the Simulator:**
   Open another terminal in the ardupilot root directory and run:
```bash
sudo ./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```
3. **Start the Mission:**
   In the MAVProxy terminal, type:
```bash
wp load way.txt
mode guided
arm throttle
takeoff 20
mode auto
```
4. **Wait for the Cache Aggressor:**
   When ArduPilot changes its current navigation index from waypoint 2 to waypoint 3, it creates the trigger file.

   The external script detects the file and starts the following workload:
```bash
taskset -c 0 chrt -f 20 stress-ng \
    --cache 1 \
    --cache-level 3 \
    --timeout 6s \
    --metrics-brief
```
   The script should display:
```bash
[TimeTrap E4] Trigger received
[TimeTrap E4] Starting L3 aggressor: FIFO/20, 6 seconds
```
After 6 seconds, it should display:
```bash
[TimeTrap E4] Aggressor stopped
```
