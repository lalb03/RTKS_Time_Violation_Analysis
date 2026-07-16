# E1 Experiment: Direct Denial of Service (DoS)

The experiment E1 uses the unmodified ArduPilot SITL simulator. 
The goal is to demonstrate that a global CPU overload does not cause control instability due to the "Synthetic Clock" effect in SITL, but merely dilates the simulation time.

## How to run the experiment

1. **Start the Aggressor (DoS):**
   Open a terminal and start a CPU-intensive stress test on all available cores:
   ```bash
   stress-ng --cpu 4 --cache 4 --vm 2 --vm-bytes 1G
   ```
2. **Start the Simulator:**
   Open a second terminal in the ardupilot root directory and run:
   ```bash
   ./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
   ```
3. **Start the Mission:**
   In the MAVProxy terminal, type:
   ```bash
   wp load way.txt
   mode auto
   arm throttle
   ```
