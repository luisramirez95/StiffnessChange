README Luis stiffness attempt

#run this to start the containers
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh

#download codex 
curl -fsSL https://chatgpt.com/codex/install.sh | sh
export PATH="/home/admin/.local/bin:$PATH"
codex --version
cd /workspaces/isaac_ros-dev/src/turtle_ros/src/motor_srv
codex


# Terminal 1 — Dynamixel motor driver
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run motor_srv motor_33

# Terminal 2 — Arduino pressure bridge
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run motor_srv pressure_cmd_bridge

# Terminal 3 — keyboard/flipper controller
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run motor_srv motor3_path

Keyboard Servo Teleop (6 motors) controls:
  Right: R1 u/j, R2 i/k, R3 o/l
  Left : L1 q/a, L2 w/s, L3 e/d
  space: reset all to neutral
  1/2/3: trajectories (mirrored)
  x    : exit

    b    → soft
    n    → stiff
    m    → release






    #how to run the stiffness change test 
     docs/pneumatic_experiment.md — The detailed experiment guide, including exact commands,
    parameters, telemetry format, CSV outputs, metric definitions, plots, and limitations.

    You’ll use three terminals inside the same Isaac ROS development container. You
  do not need to run the motor controller r keyboard controller for this test.

  This code has passed simulated tests, but this will be its first hardware test.
  Stay beside the system and be ready to stop it.

  ### 1. Prepare the hardware

  - Connect the Arduino USB cable.
  - Connect the pneumatic system and its normal power supply.

  - Close Arduino Serial Monitor.
  - Stop any existing pressure bridge, pressure logger, or keyboard-control
    program with Ctrl-C. Only one program should access the Arduino.

  ### 2. Build the updated code

  In Terminal 1, paste:

  source /opt/ros/humble/setup.bash
  cd /workspaces/isaac_ros-dev
  colcon build --packages-select motor_srv --symlink-install

  Wait until it finishes.

  You may see the previously identified indentation error in
  robot_motion_tracker.py. That is an unrelated existing file. If you see other
  errors or Failed <<< motor_srv, stop and share the output.

  ### 3. Start the Arduino bridge

  Still in Terminal 1, paste:

  source /workspaces/isaac_ros-dev/install/setup.bash
  ros2 run motor_srv pressure_cmd_bridge --ros-args -p port:=/dev/ttyACM0 -p baud:=9600

  Leave this terminal running. The bridge connects ROS to the Arduino. Opening USB may reset the Arduino.

  ### 4. Check that measurements are arriving

  Open Terminal 2 and paste:

   source /workspaces/isaac_ros-dev/install/setup.bash
  ros2 topic echo /pressure/telemetry --once

  It should print one measurement and exit.
  Look for:

  - "valid": true
  - psi: the measured pressure
  - system_v: approximately your supply
    voltage

  - system_a and system_w: numerical
    readings

  If it reports "valid": false, shows NA,
  or prints nothing after several seconds,
  do not start the experiment. Share that
  output first.

  ### 5. Start the additional recording

  Open Terminal 3 and paste:

  source /workspaces/isaac_ros-dev/install/setup.bash
  mkdir -p "$HOME/pneumatic_runs/bags"

 ros2 bag record \
    -o "$HOME/pneumatic_runs/bags/pneumatic_$(date -u +%Y%m%dT%H%M%SZ)" \
    /pressure_cmd /pressure/raw /pressure/telemetry \
    /pressure/bridge_status /pressure/command_sent \
    /pressure/experiment_heartbeat /pressure/experiment_events

  Leave it running. This saves an
  additional ROS recording. The experiment
  also saves CSV files independently.

  ### 6. Start the physical test

  Return to Terminal 2.

  The following command starts operating
  the pump and valves:

  ros2 run motor_srv pneumatic_experiment

  You do not need to press the manual
  pressure keys. The program automatically
  runs:

  Release → −2 PSI → −10 PSI → −2 PSI →
  release

  It repeats this five times, with settling
  checks and two-second holds. Do not issue
  other pressure commands during the test.

  ### 7. Finish—or stop early

  On successful completion, Terminal 2
  reports complete and prints the output
  directory.

  To stop early, press Ctrl-C in Terminal
  2. The experiment requests release and
  saves the data collected so far.

  Keep Terminal 1 running while pressure
  returns near zero. Then:

  1. Press Ctrl-C in Terminal 3 to finish
     the recording.

  2. Stop Terminal 1 after verifying
     release.

  If communication fails, a software
  release request may not reach the
  Arduino. Use your hardware’s independent
  release procedure if needed.

  ### 8. Generate the plots

  In Terminal 2, paste:

  RUN="$(ls -dt "$HOME"/pneumatic_runs/*Z | head -n 1)"
  echo "$RUN"

  ros2 run motor_srv plot_pneumatic "$RUN/telemetry.csv"

  ls "$RUN"
  ls "$RUN/plots"

  This selects the newest experiment
  directory and generates the plots.

  Your main results are:

   File      telemetry.csv
   Contains  Recorded pressure and
             electrical measurements
  ─────────────────────────────────────────
   File      summary.csv
   Contains  Transition times, power, and
             energy by trial/phase
  ─────────────────────────────────────────
   File      plots/response.png
   Contains  Pressure, power, and
             accumulated energy
  ─────────────────────────────────────────
   File      plots/trial_comparison.png
   Contains  Comparisons between trials
  ─────────────────────────────────────────
   File      plots/statistics.csv
   Contains  Averages and standard
             deviations

  The additional ROS recording is under ~/
  pneumatic_runs/bags/.