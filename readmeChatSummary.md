# Robot Control System — Chat/Codex Persistent Summary

## 1. System Overview

ROS 2 Humble keyboard node `motor3_path.py` publishes six position targets in ticks and separate pressure commands. `motor_33.py` sends the position targets to six Dynamixel motors and publishes motor feedback. `pressure_cmd_bridge.py` converts pressure commands to serial text for an Arduino. The Arduino firmware is not stored in this package; the user reports that it controls the pump, two valves, and pressure sensor locally. The 2026-09-22 update below records its current behavior and the added characterization experiment.

```text
keyboard -> motor3_path.py -- /robot/motor_commands --> motor_33.py --> Dynamixel IDs 1–6
                         \-- pressure_cmd ----------> pressure_cmd_bridge.py
                                                        | /dev/ttyACM0
                                                        v
                                                     Arduino
                                                        |-- pump
                                                        |-- two valves
                                                        `-- pressure sensor
```

**Location:** This summary and the three inspected source files are in `/workspaces/isaac_ros-dev/src/turtle_ros/src/motor_srv/motor_srv/`. The summary was initially created at the user's first requested path, then moved here at the user's request. Use this location for future work.

## 2. Relevant Files

### `motor3_path.py`

- **Purpose / node:** `KeyboardServoTeleop`, ROS node `keyboard_servo_teleop`; reads a terminal keyboard, publishes motor targets, and runs trajectories in worker threads. Requires an interactive stdin terminal. No direct hardware serial interface.
- **Publishers:** `Float32MultiArray` `/robot/motor_commands`, six ticks ordered `[R1, R2, R3, L1, L2, L3]`, reliable/volatile depth 10; `Int8` relative topic `pressure_cmd`, depth 10. No subscribers.
- **Motor mapping:** R1/R2/R3 are right shoulder/elbow/pitch; L1/L2/L3 are left shoulder/elbow/pitch. Downstream array indices map to Dynamixel IDs 1–6 in that order.
- **Keyboard:** R1 `u/j`, R2 `i/k`, R3 `o/l`; L1 `q/a`, L2 `w/s`, L3 `e/d` increase/decrease stored degrees. `space` sends reset ticks only when no trajectory is active; `1`, `2`, `3` start sequences; `x` shuts down. `b` publishes pressure `0` (soft), `n` publishes `1` (stiff), `m` publishes `3` (release), including while a trajectory is active. The printed help omits `b/n/m`.
- **Defaults / conversion:** Initial one-time publish and space reset use `[2048,2048,2048,1797,1888,2048]`. Separate `midticks` reference is `[2048,2048,1979,1797,1888,2139]`; it is not the reset pose. `angles_deg` starts as six zeros, so the 50 Hz timer's first later publish can replace the initial offset pose with six 2048 ticks. Conversion is approximately `tick = 2048 - 2048 × radians/π`; integer conversion truncates. Manual movement starts from the stored angles, not measured motor positions.
- **Settings:** `step_deg=2`, manual clamp `[-150,150]` degrees, `hz_timer=50`, trajectory `pub_hz=120`, `ticks_per_second=600`, minimum segment `0.10 s`, slow multiplier `2.5`. `alpha=1.0` disables optional publish smoothing. The manual clamp does not constrain trajectory tick values. `hz_timer` is clamped to 5–2000 Hz.
- **Key `1`, `_run_smooth_trajectory`:** One Catmull–Rom cycle through four right-side tick keyframes A `[1592,1500,2048]`, B `[1592,1500,3072]`, C `[1592,2600,3072]`, D `[2048,2600,2048]`, mirrored by copying all three targets to the left. Segment duration depends on tick distance; B→C and D→A use the slow multiplier. No pressure command. Direct A publish at start can jump from the prior pose.
- **Key `2`, `_run_elbow_sweep_trajectory`:** Four gated phases: publish pressure `1`, wait for another `2`; sweep right elbow `+30°` to `-30°` with right pitch held at `3072`, then wait; publish pressure `1` again, then wait; sweep right elbow `-30°` to `+30°` with right pitch held at `2048`. Left elbow/pitch are reflected about `midticks[4]=1888` and `midticks[5]=2139`; shoulders stay at the `midticks` derived angles unless `elbow_j1_deg_fixed` is set. Defaults: elbow range `[-30,30]`, speed `60°/s`, descending/ascending pitch ticks `3072/2048`. Phase 1 and 3 contain no motor sweep. The sequence itself sends no release or soft command.
- **Key `3`, `_run_shoulder_pitch_coupled_trajectory`:** Two consecutive shoulder sweeps, `-20°→+60°` with right pitch `2320` (`1979+341`), then `+60°→-20°` with right pitch `1638` (`1979-341`), at `40°/s`. Holds each elbow at its stored angle. Left shoulder and pitch reflect around `midticks` reference offsets; left elbow holds independently. No phase prompts or pressure commands. Defaults are parameters and can be overridden.
- **Trajectory state / modification notes:** `sequence_active` allows one trajectory at a time and suppresses the periodic timer and manual joint keys; `1`/`3` warn if busy, while `2` also resumes its own paused phases through `_elbow_pause_event`. Each worker clears active state in `finally` and, if a final tick set exists, converts it to stored angles. `_angles_lock` and `_sequence_lock` protect parts of this state; some active/waiting reads and writes occur outside locks. `space` is ignored during a sequence. `_publish_ticks`, `send_pressure_cmd`, `_on_timer`, and the three `_run_*_trajectory` methods are the main control paths.

### `motor_33.py`

- **Purpose / node:** `DynamixelControlNode`, ROS node `dynamixel_control_node`; Dynamixel SDK Protocol 2.0 over `/dev/ttyUSB0` at **4,000,000 baud**. IDs `[1,2,3,4,5,6]` map directly to the six array positions above.
- **Subscribers:** `Float32MultiArray` `/robot/motor_commands` and relative `goal_positions`, both depth 10. Only six-element messages are accepted. Once a valid `/robot/motor_commands` arrives, `motor_commands_available=True` permanently selects `target_positions` and further `goal_positions` messages are ignored. Incoming floats are truncated to integers. No tick-range validation.
- **Publishers:** Relative `dynamixel_status` as repeated `[ID, position, signed velocity]`, and `dynamixel_current` as repeated `[ID, signed current]`; both best-effort/volatile depth 10. No ROS pressure interface.
- **Defaults / operation:** `goal_positions` and `target_positions` start `[2048,2048,2048,1797,1888,2048]`. On startup the node opens serial, enables torque on all six motors, attempts position P gain **800** and D gain **15** for motors in operating mode 3 or 4, then starts a 20 Hz timer that sends six positions by group sync write and reads position, velocity, and current. The gain function's own fallback arguments are `kp=640,kd=0`, but initialization explicitly passes `800,15`; its final log does not individually prove every write succeeded. Registers: torque 64, goal position 116, present current 126, velocity 128, position 132, operating mode 11, D gain 80, P gain 84. `move_to_zero_position()` exists but is not called by startup/control flow; it commands all IDs to 2048. `ready_to_move` is set but does not gate the timer.
- **Important functions:** `enable_torque`, `set_motor_gains`, both command callbacks, `control_loop`, `convert_to_signed`. The receive log says “3 motor commands” despite accepting six. Torque is enabled before the periodic goal writes; running the node can actuate hardware.

### `pressure_cmd_bridge.py` — original baseline (superseded by 2026-09-22 entry below)

- **Purpose / node:** `PressureCmdBridge`, ROS node `pressure_cmd_bridge`; bridges ROS `Int8` commands from relative `pressure_cmd` (depth 10) to Arduino serial. No publishers.
- **Hardware / settings:** Default `/dev/ttyACM0` at **9600 baud**; parameters `port`, `baud`, `write_newline=True`, `serial_timeout=0.2`, and declared `reconnect_period=0.5` (not read or used). Serial write timeout is `0.2 s`. A background worker reconnects after errors, waits 2 s after opening for Arduino reset, and drains Arduino output without interpreting sensor data. Open failures retry after a fixed 1 s; commands received while disconnected are dropped, not queued.
- **Mapping / functions:** `cmd_map`: `0→"2"`, `1→"10"`, `2→"t"` (commented unused toggle), `3→"release"`, `4→"4"` (commented unused recalibrate). A newline is appended by default. `cmd_callback` validates and writes; `serial_worker` handles connection/read draining; `destroy_node` closes serial. The numeric pressure meaning comes from the user and comments; this Python bridge does not control or read pump, valves, or pressure sensor directly. The worker and callback share `self.ser`, though worker access is not covered by the callback's lock.

## 3. How To Run The Robot

Run in three interactive terminals in the Isaac ROS development container. These commands are the user's physically tested startup procedure; each node can actuate physical hardware.

**Terminal 1 — Dynamixel controller:** opens the bus, enables torque, applies gains, sends motor targets, and publishes feedback.

```sh
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run motor_srv motor_33
```

**Terminal 2 — Arduino pressure bridge:** opens Arduino serial and forwards pressure commands.

```sh
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run motor_srv pressure_cmd_bridge
```

**Terminal 3 — Keyboard / trajectory controller:** reads keyboard input and publishes motor and pressure commands.

```sh
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run motor_srv motor3_path
```

## 4. Current Known Working State

**Confirmed on the physical robot by the user:** `motor_33` communicates with IDs 1–6 and all six torque-enable; it reports Kp=800/Kd=15. `pressure_cmd_bridge` connects to `/dev/ttyACM0` at 9600. `motor3_path` starts. Keyboard `b`, `n`, `m` were physically tested as low vacuum/soft, high vacuum/stiff, and release. The user reports a multi-phase sequence produced “Phase 1 complete. Press '2' for Phase 2.”, “Continuing elbow 4-phase sequence.”, and “Phase 2 complete. Press '2' for Phase 3.”; starting another trajectory while it was active produced “Trajectory already running.”

**Discrepancy:** The user called that tested sequence “trajectory/sequence 3”; the current code emits those quoted prompts only from **key `2`** (`_run_elbow_sweep_trajectory`). Key `3` is the shoulder/pitch sweep. The exact key pressed during that physical test needs confirmation. The user-described neutral `2048` for all motors also differs from the offset startup/reset array and separate `midticks` reference above.

**Inferred from code, not separately confirmed on the robot:** Individual trajectory geometry/timing, status/current publication, six-element command precedence, Arduino mappings `0→2`, `1→10`, `3→release`, and the first-timer-publish startup behavior.

**Still needs physical testing or confirmation:** Which key was used for the reported phase test; full end-to-end results and safe travel for keys `1`, `2`, and `3` (including each phase); exact real-world effect of each pressure serial command; individual gain-write success if needed. No physical test is implied by this documentation change.

## 5. Change Log / Persistent Memory

### 2026-09-20 — Initial baseline documentation

- **Files changed:** `readmeChatSummary.md` only; no Python source changed.
- **What / why:** Recorded the three-node ROS control path, actual code behavior, user-reported physical test state, hardware cautions, and workflow so future sessions can avoid rediscovery.
- **Previous behavior / new behavior:** Robot-control behavior unchanged; persistent documentation now exists at the requested path.
- **Physically tested:** Documentation change **NOT PHYSICALLY TESTED**. The historical physical observations above were supplied by the user; no robot was run for this task.
- **Test result:** Static reading of the three named Python files only.
- **Unresolved issue:** Requested summary path differs from actual source package path; reported “sequence 3” conflicts with code's key `2` phase prompts; startup timer can overwrite the initial offset pose.
- **Recommended next step:** Confirm the sequence key and intended summary location when convenient; before any control change, review the relevant code and request user-led physical validation.

### 2026-09-20 — Move summary beside control source

- **Files changed:** `readmeChatSummary.md` moved from `/workspaces/isaac_ros-dev/src/motor_srv/motor_srv/` to `/workspaces/isaac_ros-dev/src/turtle_ros/src/motor_srv/motor_srv/`; location references and current issues updated. No Python source changed.
- **What / why:** Placed persistent documentation in the same directory as the three control files, as requested.
- **Previous behavior / new behavior:** Documentation was separate from the actual package; it is now beside the source. Robot-control behavior is unchanged.
- **Physically tested:** **NOT PHYSICALLY TESTED**; no robot run was needed for a documentation move.
- **Test result:** Confirmed the destination file exists.
- **Unresolved issue:** Reported “sequence 3” still conflicts with key `2` phase prompts; startup timer can overwrite the initial offset pose.
- **Recommended next step:** Confirm the sequence key before modifying trajectory behavior.

**Future entries:** Append a dated entry after every relevant control-code modification. Include date, files, change and reason, previous/new behavior, physical-test status and user-provided result, unresolved issues, and next step. Preserve useful earlier entries. Never mark a change physically tested until the user reports that test. If the user later reports a result, update the matching entry.

## 6. Current Issues / Next Steps

- Pneumatic experiment setup is currently blocked by absent `/dev/ttyACM*` devices inside the container (user-reported). Restore USB visibility and verify live telemetry before starting trials; see the 2026-09-23 entry below.

- Confirm whether the reported multi-phase physical test used key `2` or key `3`.
- Review the initial offset pose versus the timer's all-2048 first publish before changing startup behavior.
- User-led physical validation remains needed for complete trajectories, pressure-command effects, and any future changes.

## 7. Important Safety / Hardware Notes

- This code controls physical Dynamixel motors and pneumatic hardware. Starting `motor_33` enables torque and writes goals; starting `motor3_path` publishes motor goals.
- Do not automatically run motor trajectories or pneumatic actuation to test a code modification. Physical actuation stays under the user's control unless explicitly requested. Building, linting, inspection, and analysis are okay.
- A successful build or static check does not establish that a change works on the robot. Mark every relevant code change **NOT PHYSICALLY TESTED** until the user supplies an actual robot test result.
- For future changes, read this summary first, then only the specific relevant source files. Avoid recursive repository scans and unrelated packages. After any relevant code modification, update affected technical descriptions, append to section 5, and maintain section 6. Keep this file concise and preserve useful history.


## 8. 2026-09-22 — Negative-pressure telemetry and characterization experiment

**Current objective:** Characterize atmospheric→-2 PSI, -2→-10 PSI (jamming), and -10→-2 PSI (unjamming) pressure transitions and whole-branch electrical power/energy. Pressure response is only a proxy for jamming state, **not a mechanical stiffness measurement**; that needs force–displacement data.

### User-reported current Arduino hardware and behavior

- Pump IN1 pin 3, IN2 pin 2, PWM pin 5, motor standby pin 4; valve A pin 7, valve B pin 6; pressure sensor A0. Valve HIGH=open, LOW=closed.
- Negative gauge pressure is correct. Calibration applies exactly one inversion: `return -((voltage - zeroVoltage) / V_SPAN) * PSI_SPAN;`. Actual -14 PSI should read approximately -14 PSI. ROS adds neither an absolute value nor another sign inversion.
- Serial `2` targets **-2 PSI**, `10` targets **-10 PSI**, and `release` releases toward 0 PSI and cancels fast inflation. The historical physical-effect uncertainty above is superseded by this user report.
- Deadband is ±0.3 PSI. SUCKING (`psi > targetPSI + 0.3`): A closed, B closed, pump PWM 200. Normal passive RELEASING (`psi < targetPSI - 0.3`): A open, B open, pump off. HOLDING inside the band: A closed, B open, pump off.
- A direct -10→-2 target switch enables one-time fast inflation: A open, B open, pump PWM 200 while pressure is below -2.3 PSI. On reaching the target deadband, fast inflation disables and original pressure maintenance resumes. Release cancels it. **No Arduino controller behavior was changed.** No sketch was available here to edit or verify.
- INA260: I2C `0x40`, 16-sample averaging; high-side sensing in the complete 12 V pneumatic actuation branch (supply → IN+ → IN− → branch). Measurements include pump, valve A, valve B and motor/valve driver losses. SDA/SCL use board-specific Arduino I2C pins; common ground and compatible logic supply are required. The specific Arduino/breakout board is not documented. These are **SystemV/SystemA/SystemW**, even if diagnostic labels still say PumpV/PumpA/PumpW.
- Telemetry contract: PSI, target, controller state, pump PWM, A/B state, system volts/amps/watts, optionally Arduino millis. Host receipt ROS and monotonic timestamps are added by the bridge. Legacy power labels are accepted as whole-system values. Values must be V/A/W, not mV/mA/mW.

### Implemented ROS additions and preserved controls

`pressure_cmd_bridge` remains the sole pneumatic serial owner at `/dev/ttyACM0`, **9600 baud unchanged**. Existing `pressure_cmd` Int8 mappings and keyboard `b/n/m` remain intact, including legacy `t`/`4` mappings. Existing launch files and motor/trajectory code are unchanged. Do not run the old serial-owning `pressure_logger` or Arduino Serial Monitor against the same port.

The bridge now publishes raw lines, validated structured JSON telemetry, connection faults and serial-write acknowledgements on `pressure/raw`, `pressure/telemetry`, `pressure/bridge_status` and `pressure/command_sent`. The separate `pneumatic_experiment` publishes commands through `pressure_cmd`; it never opens serial. Additional `pressure/experiment_events` and `pressure/experiment_heartbeat` topics support bagging and an experiment-only bridge watchdog. After heartbeat loss, the bridge requests release; failed releases are retained for reconnection. Ordinary manual use does not arm this watchdog.

Five trials by default: release and stabilize near 0 → -2 and settle → hold 2 s → -10 and settle → hold 2 s → -2 and settle → hold 2 s → release and stabilize near 0. Each transition requires ±0.3 PSI for 0.5 continuous seconds and has a 10 s timeout. Parameters expose trials, hold/settling durations, tolerance, timeout, output directory, communication timeout (2 s) and startup timeout (10 s). Bridge and experiment must run on the same host for monotonic freshness checks.

On timeout, invalid data, communication loss, conflicting command or Ctrl-C/SIGTERM, the experiment requests release, saves partial data and reports failure. **Software cannot guarantee physical release across a broken serial connection or a failed bridge/Jetson.** Keep the bridge alive during experiment shutdown and verify atmospheric pressure yourself. No firmware watchdog was added.

Outputs in `~/pneumatic_runs/<UTC timestamp>/`: `telemetry.csv`, per-trial/per-phase long-format `summary.csv`, `metadata.json`, `events.jsonl`; headless `plot_pneumatic` generates pressure/target, system power, cumulative observed energy, phase annotations, transition time/energy comparisons and mean/sample-SD statistics under `plots/`. Separate optional rosbag2 recording goes under `~/pneumatic_runs/bags/`.

Transition duration includes the settling confirmation interval; settling time is the start of the final continuously in-band interval. Energy uses trapezoidal integration of observed power samples within each phase, without extrapolating missing edges. Sample counts and observed duration expose data coverage. Holding rows report mean holding power. No idle-baseline correction is claimed.

**Exact build/source/start/record/stop/plot commands, telemetry syntax, metric definitions and limitations:** [Pneumatic experiment run guide](../docs/pneumatic_experiment.md).

### Change-log entry and outstanding validation

- **Changed:** `pressure_cmd_bridge.py`, `setup.py` (two added entry points), `package.xml` (runtime dependencies), this summary. **Added:** `pneumatic_data.py`, `pneumatic_experiment.py`, `plot_pneumatic.py`, `test/test_pneumatic.py`, `docs/pneumatic_experiment.md`. Existing local edits were preserved.
- **Before/after:** Bridge discarded diagnostics; now it exposes telemetry alongside the original manual command interface. Sequencing and analysis are separate opt-in tools. No baud, pressure calibration, firmware, launch or manual-control mapping changed.
- **Physical status:** **NOT PHYSICALLY TESTED.** No hardware nodes were started and no pressure commands were sent to hardware.
- **Non-hardware checks:** 20 passing tests, including parser faults, settling reset, five simulated trials, energy integration, unchanged serial command strings, watchdog, communication loss, interruption and conflicting commands. Changed Python modules compile; the headless plotting smoke test produced both PNGs and statistics from simulated CSVs. Isolated colcon build completes but reports an existing unrelated `robot_motion_tracker.py` indentation error (empty `else` around line 73); left untouched.
- **Unresolved / next step:** The exact user-supplied diagnostic line is now covered by tests. Verify live V/A/W units, release target reporting and sample cadence; perform user-led trials with the run guide. At 9600 baud diagnostic length limits timing resolution. If higher baud is needed, update Arduino and ROS together after firmware is available. Historical trajectory-key and startup-pose issues remain unrelated and unresolved.


### 2026-09-22 — Exact diagnostic format confirmed by user

```text
PSI:-1.40 Target:-10.00 State:SUCKING Pump:200 ValveA:CLOSED ValveB:CLOSED PumpV:12.080 PumpA:1.240 PumpW:14.980
```

Fields are space separated with no space after colons. `Pump` is commanded integer PWM; PSI/Target have two decimals, power fields three. States are SUCKING/RELEASING/HOLDING; valves OPEN/CLOSED. There is **no Arduino millis timestamp**, so that CSV column remains blank. The parser accepts this exact format and interprets PumpV/A/W as whole-branch SystemV/A/W. INA260 unavailable output `PumpV:NA PumpA:NA PumpW:NA` causes an invalid-sensor abort and release request, never zero-filled energy data. During the user's confirmed fast -10→-2 transition, firmware reports `State:RELEASING Pump:200 ValveA:OPEN ValveB:OPEN`; preserve those fields to distinguish it from passive release. This confirms firmware diagnostic syntax/behavior, **not physical validation of the new ROS experiment**.


### 2026-09-23 — Experiment startup troubleshooting and current handoff

- **Documentation status:** Section 8 already records the new experiment code, hardware contract, telemetry parser, preserved manual controls, default five-trial sequence, failure handling, outputs, metrics and offline validation. The linked run guide contains the complete build/run/record/stop/plot procedure.
- **User-reported progress:** The user attempted to start the updated bridge. Splitting `source /workspaces/isaac_ros-dev/install/setup.bash` between the directory and filename caused shell errors. Typing a backslash followed by a space before `-p` caused `UnknownROSArgsError: [' -p', 'port:=/dev/ttyACM0']`. Removing that backslash fixed the command-line error; it did not establish a successful serial connection.
- **Current blocker:** Neither `/pressure/telemetry` nor `/pressure/raw` produced output in the user's echo checks. Running `ls -l /dev/ttyACM*` inside the container returned “No such file or directory”. The user reported connecting the Arduino after starting the container. Missing container USB access is a possible cause, not yet a confirmed diagnosis; host USB detection and the Arduino's current device name remain unverified.
- **Port distinction:** `/dev/ttyACM0` is the configured pneumatic Arduino port; the Dynamixel joint-motor node is configured for `/dev/ttyUSB0`. USB names can change. Confirm the actual Arduino using its expected pressure diagnostics rather than assuming its device name proves identity.
- **Next step:** Keep Arduino USB connected, restart the container using the user's normal procedure, then check `ls -l /dev/ttyACM*` inside it. If still absent, run `ls -l /dev/ttyACM* /dev/ttyUSB*` on the Jetson host outside the container and inspect the result. A different device name must be verified before changing the bridge's port parameter. No successful restart or recovered telemetry has been reported yet.
- **Physical validation status:** **EXPERIMENT NOT PHYSICALLY TESTED.** User startup attempts are recorded, but no completed trial, confirmed live ROS telemetry, or experimental results have been reported. Historical manual-control tests and the user's confirmed Arduino fast-inflation behavior do not validate the new ROS experiment.
- **Files changed / behavior:** This summary only; code, firmware and manual controls unchanged. Documentation checked against the existing summary and conversation; no hardware command or experiment was run for this update.

For the user's terminal workflow, copy each command below as one complete line. Do not split `setup.bash` from its path or insert a backslash before `-p`.

Bridge terminal, after confirming `/dev/ttyACM0` exists:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run motor_srv pressure_cmd_bridge --ros-args -p port:=/dev/ttyACM0 -p baud:=9600
```

In a second terminal inside the same container:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 topic echo /pressure/telemetry --once
```

This echo waits for a message. If nothing arrives after about 10 seconds, stop only the echo with Ctrl-C, inspect the bridge terminal, and use `ros2 node list` and `ros2 topic echo /pressure/raw --once` to distinguish node/serial visibility from parsing issues. **Do not start `pneumatic_experiment` until live valid telemetry is confirmed.** The experiment command actuates the pump and valves automatically; the normal motor driver and keyboard node are unnecessary for this stationary characterization.
