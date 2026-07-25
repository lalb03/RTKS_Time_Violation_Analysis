# E3 Experiment: Software Fault Injection
Experiment E3 builds upon the E2 architecture. It introduces a synthetic delay in the control loop when the drone starts performing the second curve of the mission `way.txt`.

The experiment is activated when the current mission navigation index changes from waypoint 2 to waypoint 3. At that point, a delay of 30 ms is injected for 200 consecutive control cycles. A second experiment in done by setting the delay to 50 ms.

## How to run the experiment

1. **Start the Simulator (as root):**
   Open a terminal in the `ArduPilot` root directory and run:
   ```bash
   sudo ./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
   ```
2. **Start the Mission:**
   In the MAVProxy terminal, type:
   ```bash
   wp load way.txt
   mode guided
   arm throttle
   takeoff 20
   mode auto
   ```
3. **Wait for the Fault Injection:**
   The delay is automatically activated when ArduPilot changes its current navigation index from waypoint 2 to waypoint 3.

   The injected delay is implemented inside ModeAuto::wp_run() before the waypoint navigation controller is updated.
