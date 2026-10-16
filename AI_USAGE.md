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
- *Pending review.*

**Assumptions the AI made that still need my confirmation**
- Added an iOS device profile toggle, which I had not confirmed yet.
- Chose YAML over JSON for scenario files.
