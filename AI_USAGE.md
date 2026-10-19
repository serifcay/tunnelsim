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

### 2026-10-18: My question about speed in stop-and-go traffic

**My input**
- I noticed that in stop-and-go traffic the estimated speed drifted far from the real speed, and
  asked how speed is computed from the accelerometer and whether it could be improved.

**AI contribution**
- Explained the pipeline. Then, instead of guessing, it measured the speed error with each error
  source removed in turn. Alignment dominated.
- Traced the "up" axis through the jam: it was re-estimated at every stop and jumped by up to 0.4°.
- Predicted, and then confirmed with a new scenario (S11), that a stop on a slope corrupts the
  alignment: 2.1 km errors. This was a design flaw in the AI's own aligner that none of the
  original scenarios exposed, because none of them stopped on a slope.
- Fixes, on my go-ahead:
  1. Freeze "up" after the initial alignment.
  2. A per-car accelerometer scale estimate.
- Its first scale estimator (in the Kalman calibrator) was biased: the pull-away happens while the
  forward axis is still converging. It moved the estimate into the alignment regression, corrected
  for GNSS noise in the regressor, and added uncertainty-based shrinkage. Each step was compared
  in Monte Carlo before keeping it.

**Result**
- D's in-tunnel rms in the jam (S4) went from 53 to 27 m.
- The incident stop (S5) went from 37 to 11 m.
- Stops on slopes (S11) went from 574 to 17 m.

### 2026-10-19: My question about the sign of S7's errors, and new requests

**My input**
- I noticed that in S7 (phone shift) both C and D always ended up behind the car, and asked why.
- I asked for scenarios with the phone shift in an uphill and a downhill tunnel.
- I asked for the exit announcements to be shown on the simulator map.

**AI contribution**
- Explained the mechanism: a leftover tilt after the shift leaks gravity into the forward axis.
- Showed that the tracking error persisted with a perfect gyro, which pointed to the simulator
  rather than the estimator. The simulator's gyro model counted some mount-rotation steps twice and
  others not at all (floating-point step times). It confirmed this with a temporary fix: D's error
  after the shift went from −88/−361 m to ±2 m.
- Separated a second, independent effect: C drifts behind on that road even without a shift. A
  small forward misalignment leaks the curve's lateral acceleration, and C applies the map slope at
  a slightly wrong position, with no curvature or barometer matching to correct either.
- Fixed the simulator, reduced the bias widening after handling, added a regression test, added
  S12/S13, and drew the planned and actual announcement points plus an announcement banner on the
  map.

- When the study re-run showed S11 and S10 getting *worse* (scenarios without a phone shift), it
  bisected the change against the last commit and found a second simulator artefact. The driver
  model ended braking in one step, so the body pitch snapped back at ~1.4 rad/s. The aligner took
  that for phone handling at every stop. Fixed by giving body pitch and roll a 0.3 s suspension lag.

**Result (D, median in-tunnel rms)**
- S7: 44.5 → 13 m, consistency NEES 73 → 0.8.
- S12 uphill ~19 m; S13 downhill ~14 m.
- The suspension fix also improved S11 (17 → 16 m) and S10 (13 → 10 m), with NEES around 1.

### 2026-10-19: Multi-exit scenarios

**My input**
- Asked for an uphill and a downhill curved tunnel with several exits.

**AI contribution**
- Checked that the code handles several forks before writing the scenarios, and found a real gap.
  The announcement logic counted any off-route hypothesis as "past the exit", so hypotheses on a
  wrong *earlier* exit would trigger the announcements for the planned exit early. Fixed it. A
  driver who takes an earlier exit now ends the planned exit's announcements instead of scoring
  them as missed.
- Added S14/S15: three exits; the route takes the middle one, in the curve; a second car takes
  exit 1 by mistake. Added numbered exit signs on the map (on the ramps, so they don't collide with
  announcement points) and a regression test.

**Result**
- D tells the exits apart: the wrong exit is recognised after 1.0 s (A and C: 13.5 s). This
  number came from the flattering metric corrected on 2026-10-20; the honest figure is 1.5 s.
- Announcement timing for exit 2: 0.3 s.

### 2026-10-20: Remaining error on slopes and after phone shifts; calibrating sensors while running

**My input**
- I felt the phone-shift and slope scenarios still had extra error, and asked whether there was
  room for improvement.
- I asked whether sensors could be calibrated against each other while running.
- I asked for the expected gain per scenario, so I could choose. I chose vertical speed and
  slope-aware re-alignment.

**AI contribution**
- Measured before proposing. Two of its own hypotheses were ruled out: accelerometer scale on
  gravity, and tunnel pressure disturbances. The real limit on long constant slopes is
  information: the barometer is the only anchor.
- Prototyped barometric vertical speed and measured every scenario.
- Bounded gyro-feedback calibration with a perfect-gyro "oracle" (≤ 2 m gain), and recommended
  skipping it.
- Pointed out a latent weak spot: after phone handling, the next stop on a slope re-introduced the
  slope-counted-twice error.
- Implemented both chosen changes, added S16 (phone adjusted at a stop on a slope) and tests.

**Result (D, median in-tunnel rms, 40 runs)**
- With vertical speed: S3 18 → 13 m, S7 13 → 10 m, S12 19 → 15 m (worst 5%: 93 → 53 m),
  S13 13 → 11 m.
- S16 with slope-aware re-alignment: 13 m, against 31 m with the old behaviour; a phone adjusted
  at a stop now costs about 1 m.

- When the full study showed the S6 branch decision "slowing" from 0.5 to 2.5 s with vertical speed,
  the AI did not tune vertical speed away. A softer setting did not change it, which pointed at the
  metric. The metric took the first moment P(true branch) ≥ 0.9, which a particle cloud lagging
  behind the car reaches trivially. It was replaced by "≥ 0.9 for good, per fork". Honest numbers:
  S6 3.9 s (vertical speed actually helps: 4.7 s without it); S14/S15 1.5 s.

**Still open** (reported in the write-up, not hidden)
- OBD fusion (D+E) is still over-confident in stop-and-go traffic.
- The stop detector relies on road vibration; with none at all it fails badly.

## My review and overrides

These are my decisions and the places where I did not accept the AI's output as it was. All of
them are visible in the log above. The pattern: the AI's first answers were plausible and
well-presented, but three times its "honest limitation" was in fact a flaw that my questions
exposed.

### How I steered the work

- **Discussion before code.** I asked for a review of the brief and a design discussion before any
  implementation, and approved the design document before coding started.
- **Language.** I chose Java (with JavaFX) because I need to explain and change every line live.
  The AI noted that Python is more common for positioning R&D.
- **Scope.** I chose the particle filter on the road network (D) as the main approach.
  Infrastructure and vehicle data (BLE beacons, OBD speed) are an optional extra (E). The default
  is "phone only".
- **Sensors.** I asked for a full survey of phone sensors before deciding what to simulate.
- **Evidence before choosing.** When the AI proposed further improvements, I asked for the
  expected gain per scenario before deciding. I chose barometric vertical speed and slope-aware
  re-alignment, and accepted its advice to skip gyro-feedback calibration (measured best case:
  about 2 m).
- **Publishing.** Private repository first, public only after my review.

### Where my questions overturned the AI's conclusions

1. **Speed in stop-and-go traffic.** The AI had written the jam scenario up as a limitation. I
   asked why the estimated speed drifted so far from the real speed and whether that could be
   improved. Its measurements then showed the phone's "up" axis being re-estimated at every stop.
   A new scenario confirmed the worst case: a stop on a slope counted the slope twice, giving
   2 km errors. Fixing it (plus a per-car accelerometer scale) halved the jam error and removed
   the slope failure.
2. **The sign of the phone-shift errors.** The AI had described a phone moving in its holder as
   "only partly handled". I asked why C and D always ended up *behind* the car. That exposed two
   bugs in the simulator, not in the estimator:
   - the simulated gyro missed part of the rotation (floating-point timing);
   - the car's braking dive snapped back instantly when stopping, which looked like phone
     handling.

   After the fixes, a phone shift costs D about 1–3 m instead of ~100 m.
3. **Remaining error on slopes and after phone shifts.** I felt these cases still had extra error.
   The AI's two first explanations were ruled out by its own tests. The measured cause was limited
   information on long constant slopes. This led to barometric vertical speed: −10 to −27% median
   error on every slope scenario.

### What I added beyond the AI's plan

- **Scenarios:**
  - phone shift on an uphill and on a downhill tunnel (S12, S13);
  - curved uphill and downhill tunnels with several exits (S14, S15). Building these exposed a
    callout bug with multiple forks: hypotheses on an earlier, wrong exit would have fired the
    planned exit's announcements early.
  - a phone adjusted while stopped on a slope (S16).
- **Visualisation:**
  - exit announcements on the simulator map: planned points, where each estimator actually
    announced, and a banner with what the driver hears;
  - numbered exit signs.

### What I take from it

- The AI wrote the code quickly and debugged carefully once given a concrete symptom.
- Asking "why does this number look like that?" was worth more than accepting a well-written
  limitation.
- The simulator itself needed as much scrutiny as the estimator: it was wrong twice, and one
  metric (branch decision time) was flattering until it was redefined.
- This is also why my first next step is validating on real phone recordings.
