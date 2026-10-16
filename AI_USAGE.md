# AI Usage Log

This file records how AI tools were used on this case study: what they suggested, what I decided, and where I corrected or overruled them. Entries are added as the work progresses. The summary for the write-up will be drawn from this log.

**Tools:** Claude Code (Claude Opus 5.5) in VS Code.

**How I use it:** to read the brief and brainstorm, to research sensors and estimation techniques, to draft the design, and to generate and review code. I make every design decision and review every line.

---

## Log

### 2026-10-15: Reading the brief and first design discussion

**AI contribution**
- Read the brief and framed the problem as estimating 1D position along the road on a known map. Proposed a ladder of approaches:
  - A: keep the last speed
  - B: integrate the phone's IMU
  - C: map-aided Kalman filter
  - D: particle filter on the road network
  - E: C/D plus infrastructure or vehicle data
- Proposed the simulator structure: ground truth, sensor models, plug-in estimators, Monte Carlo evaluation. Also proposed config-driven scenarios, so parameters can be changed live during the interview.
- Recommended Java + JavaFX (Python and TypeScript were the alternatives considered).

**My decisions**
- Main approach: **D (particle filter on the road network)**, with **E** as an extra feature.
- Default mode is **phone only**. BLE beacons are an optional infrastructure layer.
- Stack: Java + JavaFX.
- Accepted the framing: simulate the road in 2D for visualisation, but run the estimator in 1D along the road, with a hypothesis for each branch.

**Corrections and caveats**
- The AI first dismissed the phone camera, assuming that a mounted phone faces the driver. On review, the *rear* camera of a windshield- or dash-mounted phone faces the road, so visual odometry is physically possible. It is now listed as out of scope (permissions, battery, low tunnel light, not standard for navigation apps), not as impossible.
- The AI's sensor noise and bias figures are ballpark values. They must be checked against real IMU and barometer datasheets before being used as simulator defaults.

### 2026-10-15: Phone sensor survey

**AI contribution**
- Listed common smartphone sensors (motion, radio, environmental, audio/video, plus phone-side data such as maps and traffic) and rated how useful each could be inside a tunnel.
- Flagged differences between Android and iOS: iOS has no raw GNSS, no ambient light API and no cell info for apps.
- Proposed matching measured road shape against the map as the main phone-only correction: curvature from the gyroscope, elevation profile from the barometer.

**My decisions**
- Accepted a three-tier sensor plan: realistic models for the core sensors, simple models for secondary ones, and write-up-only for the rest.

### 2026-10-16: Design document (`docs/DESIGN.md`)

**AI contribution**
- Drafted the architecture: record-then-estimate, compute-then-play UI, separate random streams per component. Also drafted the particle filter's state and update steps, the aiding modules, the scenario format, metrics, UI layout and milestones.

**My decisions**
- Approved the design as written, including the iOS profile toggle and YAML scenarios.

### 2026-10-16: Implementation

**AI contribution**
- Wrote the code for every milestone: the road network and driver model, sensor models, the
  alignment front-end, estimators A–D (+E), navigation layer, Monte Carlo runner and studies,
  JavaFX simulator, tests, README.
- Debugged the estimators against the simulator. Every fix was driven by a trace or a Monte
  Carlo result, not by intuition. Each change is listed in `docs/DESIGN.md` §14.

**Where the AI's first version was wrong** (found by the simulator, then fixed)

| First version | What exposed it | Fix |
|---|---|---|
| A and B froze between fixes during GNSS | Constant ~12 m lag in the trace while GNSS was good | Propagate continuously |
| The particle filter learned the accelerometer bias itself | D drifted more than the EKF; a known weakness of particle filters for static parameters | Kalman bias calibrator before the tunnel |
| Speed clamped at zero inside the filters | Systematic under-estimation after each stop in the S4 jam | Allow slightly negative speed in the state |
| Stop detector released slowly when the car pulled away | Same S4 trace: ~1 m/s lost per stop-and-go cycle | Release on departure of the acceleration vector |
| Forward-axis regression used every fix | Alignment error jumped 0.3° → 4° at tunnel entry on a curve | Straight road, good fixes only |
| Barometer offset and gyro bias per particle | Ablation showed the result changing from 3 to 40 m with the random draws ("genetic drift") | Calibrate them globally |
| Barometer treated as independent at 2 Hz | Removing one error source at a time: only removing the baro fixed over-confidence | Down-weight by correlation time |
| Lateral acceleration aiding on | Body roll biases it by ~8% | Off by default |
| Monte Carlo kept all runs in memory | Out-of-memory on the 7-minute scenarios | Keep a per-run digest |
| Tunnel-length study plotted error *at the exit* | C and D looked flat with length: the light-sensor portal fix resets them just before the exit | Plot the largest in-tunnel error instead |

**Still open** (reported in the write-up, not hidden)
- OBD fusion (D+E) is still over-confident in stop-and-go traffic.
- A phone shift mid-tunnel (S7) is only partly handled.
- The stop detector relies on road vibration; with none at all it fails badly.

**My review and overrides**
- *To be filled in by me after reviewing the code and results.*
