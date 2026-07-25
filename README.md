# E1 Experiment: Direct Denial of Service (DoS)

Experiment E1 preserves the original ArduPilot SITL execution architecture and the common logging instrumentation used in E0. It adds an external `stress-ng` workload that generates broad CPU, cache, and memory pressure inside the virtual machine. 
The goal is to demonstrate that a global CPU overload does not cause control instability due to the "Synthetic Clock" effect in SITL, but merely dilates the simulation time.
The objective is to evaluate whether indiscriminate resource saturation mainly affects the time required to execute the mission or also produces measurable velocity-tracking and trajectory deviations.

## How to run the experiment

1. **Start the Aggressor (DoS):**
   Open a terminal and start a CPU-intensive stress test on all available cores:
   ```bash
   stress-ng --cpu 4 --cache 4 --vm 2 --vm-bytes 1G
   ```
2. **Start the Simulator:**
   Open a second terminal in the `ArduPilot` root directory and run:
   ```bash
   ./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
   ```
3. **Start the Mission:**
   In the MAVProxy terminal, type:
   ```bash
   wp load way.txt
   mode guided
   arm throttle
   takeoff 10
   mode auto
   ```

To see if the aggressor is working, run `htop` on a new terminal.
