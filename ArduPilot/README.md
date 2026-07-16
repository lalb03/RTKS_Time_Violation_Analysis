# E0 Experiment: Baseline

The experiment E0 is a simple run of the unmodified ArduPilot SITL (Software In The Loop) simulator. 
The goal is to collect the baseline telemetry and performance data in ideal conditions (logged in `exp_log.csv`) to serve as a ground truth for others experiments.

## How to run the experiment

1. **Start the Simulator:**
   Open a terminal in the `ardupilot` root directory and run:
   ```bash
   ./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
   ```
2. **Start the Mission:**
   Wait for the MAVProxy console to initialize and the EKF to report "EKF3 is using GPS". Then, type the following commands in the MAVProxy terminal:
   ```bash
   wp load way.txt
   mode guided
   arm throttle
   takeoff 20
   mode auto
   ```
3. **Collect Data:**
   The drone will take off and follow the waypoints. Once the mission is complete, disarm the drone or close the simulator. The telemetry data will be saved in `exp_log.csv`.
