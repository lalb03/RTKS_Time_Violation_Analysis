# E2 Experiment: Separated Architecture
In experiment E2, the ArduPilot architecture is modified. The sensor acquisition and the control logic are split into two different threads using `SCHED_FIFO` real-time scheduling. The sensor thread runs at priority 80, while the control thread runs at priority 60.

**Note:** Root privileges are strictly required to request `SCHED_FIFO` policies from the Linux Kernel.

## How to run the experiment

1. **Start the Simulator:**
   Open a terminal in the `ardupilot` root directory and run:
   ```bash
   sudo ./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
   ```
2. **Start the Mission:**
   In the MAVProxy terminal, type:
   ```bash
   wp load way.txt
   mode guided
   arm throttle
   takeoff 10
   mode auto
   ```

To see if the priorities are setted, look at the SITL Console and search for the output messages. You can also run the following command: `ps -T -p "$(pgrep -n arducopter)" \ -o pid,tid,comm,cls,rtprio,pri,psr`
