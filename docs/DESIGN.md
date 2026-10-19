# Through the Tunnel: Design

Status: **implemented**. Sections 1–12 are the design as reviewed before coding. Section 14 lists
what changed during implementation and why.

## 1. Goal

Estimate a car's position while GPS is unavailable in a tunnel 500 m to 5 km long. Show the estimate next to the true position in a visual simulator. Evaluate where the approach holds and where it breaks.

Three things decide whether the system is good:

1. **Along-road error.** The car is on a known single-lane road, so the error that matters is how far along the road the estimate is off. Sideways error is removed by the map.
2. **Exit announcement timing.** The navigation app must call out an exit inside the tunnel at the right moment. A late announcement is worse than an early one.
3. **Clean recovery.** When GPS returns, the estimate must converge without the marker jumping, and the first bad GPS fixes after the tunnel must not corrupt it.

## 2. Assumptions

| # | Assumption | Why / how it can be relaxed |
|---|---|---|
| A1 | The world is a flat local plane (x east, y north, z up, in metres). Earth's curvature and rotation are ignored. | Earth rotation (about 0.004 °/s) is far below a phone gyro's leftover bias. Tunnels are at most 5 km long. |
| A2 | The road is single-lane, with no lane changes or overtaking. The car follows the centreline. | Given in the brief. Sideways offset is ignored. |
| A3 | The navigation app has a map with road geometry, curvature, elevation, speed limits and tunnel extents. | Standard in navigation maps. Option: the estimator's map can be given errors relative to the true road (realism knob). |
| A4 | The phone sits in a fixed mount at an **unknown** orientation relative to the car. | The orientation is estimated, not given. Scenario S7 moves the phone in the middle of the tunnel. |
| A5 | GNSS (satellite positioning) is unavailable inside the tunnel and degraded within a configurable zone around each tunnel mouth. | Configurable per scenario. |
| A6 | Default device: a generic modern phone with an accelerometer/gyroscope, barometer, GNSS (with raw measurements), ambient light sensor and cellular radio. | An iOS profile removes raw GNSS, the light sensor and cell info. A low-end profile removes the barometer. |
| A7 | Daytime lighting unless the scenario says otherwise. | Affects only the light sensor model. |
| A8 | Sensor timestamps share one clock, with optional jitter. | Jitter can be configured to test its effect. |
| A9 | Option E only: BLE beacons in the tunnel at known positions, and/or car speed from OBD-II or Android Automotive. | Off by default. The default mode is **phone only**. |

All sensor noise figures used as defaults are **ballpark values to check against datasheets** (for example the Bosch BMI270 and ST LSM6DSV for the accelerometer/gyroscope, and the Bosch BMP390 for the barometer) before they are used in the final results.

## 3. Architecture

```
            ┌──────────────────────────────────────────────────────────┐
 scenario   │                       sim-core                            │
 .yaml ───▶ │  map ──▶ vehicle (truth) ──▶ sensors ──▶ SensorLog         │
            │                                         │                 │
            │                     ┌───────────────────┘                 │
            │                     ▼                                     │
            │   alignment front-end ──▶ estimators (A, B, C, D [+E])    │
            │                                         │                 │
            │                                         ▼                 │
            │                     nav (callout, reroute, smoothing)      │
            │                                         │                 │
            │                                         ▼                 │
            │                                      metrics              │
            └───────────────┬───────────────────────────┬──────────────┘
                            ▼                           ▼
                     sim-app (JavaFX)           sim-batch (CLI)
                     run + scrub + tweak        Monte Carlo, ablation,
                                                CSV + PNG figures
```

### Key design decisions

- **Record first, then estimate.** A run first generates the full truth trajectory and a `SensorLog` from the seed. The estimators then replay that log in timestamp order. Every estimator sees **identical** data, and the log can be exported as CSV. Later, the same log format can be filled from a **real phone recording**, which links directly to the "next steps with real devices" section.
- **Compute the whole run, then play it back.** A full run (5 minutes, all estimators, 1,000+ particles) should take well under a second. The UI computes the run up front and then plays and scrubs a timeline, including backwards. When a parameter changes, the run is recomputed and the playhead stays where it was. This makes live tweaks in the interview instant and repeatable.
- **Deterministic seeding with independent random streams.** A master seed produces a separate random stream for each component (car, each sensor, each estimator). Changing one sensor's parameter does not change the noise in any other sensor, so A/B comparisons stay fair.
- **The map is the reference.** In phone-only mode the main source of correction is matching measured road **shape** against the map: curvature from the gyroscope, elevation profile from the barometer. The approach is similar to aircraft navigating by matching the terrain below them.

## 4. Modules and packages

Gradle multi-project build, Java 21 (records, sealed interfaces, pattern matching).

```
pointr/
├── README.md, AI_USAGE.md
├── docs/            DESIGN.md, WRITEUP.md (later), figures/
├── scenarios/       *.yaml (scenario library)
├── sim-core/        pure Java, no UI dependency
├── sim-app/         JavaFX visual simulator
└── sim-batch/       command-line Monte Carlo runner → results/
```

`sim-core`, root package `tunnelsim` (name to be decided):

| Package | Contents |
|---|---|
| `map` | `RoadNetwork` (directed graph), `Edge`, `Node`, `RoadGeometry` (segments: straight, arc, clothoid), road profiles (curvature, grade and elevation as functions of distance along the road), `Tunnel`, `MapPosition(edgeId, s)`, conversion between road distance and x/y |
| `scenario` | `ScenarioConfig` (records), YAML loader (Jackson), validation, built-in scenario library |
| `vehicle` | `DriverModel` (target speed, acceleration/jerk limits, traffic events), `TruthState`, `TruthTrajectory` |
| `sensor` | Sealed `SensorEvent` hierarchy, one model per sensor, noise building blocks, `DeviceProfile` |
| `align` | Estimation of the phone-to-car rotation; turns phone-frame IMU data into car-frame signals |
| `estimator` | `Estimator` interface, estimators A–D, shared "aiding" modules (§6.3), option E aiding modules |
| `nav` | `Route`, `CalloutPolicy`, `RerouteDetector`, `DisplaySmoother` |
| `metrics` | Per-run metrics, Monte Carlo aggregation, CSV writers |
| `engine` | `SimulationRun` (truth → log → estimators → nav → metrics), random stream management |

## 5. Simulation model

### 5.1 Road network

- The road is a directed graph. Each edge carries a centreline built from **straight / arc / clothoid** segments, so curvature changes continuously as on real roads. Each segment also has a **grade** (slope), and elevation is accumulated from the grades.
- Tunnels are intervals of road distance on one or more edges. Each tunnel can have a configurable **degraded GNSS zone** at either mouth, and optional entrance lighting (real tunnels have brighter lighting just inside the entrance).
- An **exit inside the tunnel** is a node where the main edge splits into the continuing main road and a ramp. The ramp's first few metres run almost alongside the main road and then curve away, which makes telling the branches apart a realistic problem.
- The estimator can use either the true map or a **perturbed map**: geometry, curvature and elevation with added errors. This lets us measure how sensitive the system is to map quality.

### 5.2 Vehicle and driver

- A kinematic point mass moving along the path: road distance, speed, acceleration, with acceleration and jerk limits.
- `DriverModel`: follows a target speed (speed limit × a personal factor), plus scripted **traffic events**: slow down to a speed, stop for N seconds, stop-and-go pattern, hard brake.
- The car's attitude is derived from the motion: heading follows the road, pitch = road grade + a small pitch from acceleration (suspension), roll from sideways acceleration (the car leaning in curves).
- The route says which branch the navigation expects. The driver follows it with probability `p_follow` (scenario S6 forces the wrong branch).
- Multiple cars per scenario: each has its own driver, phone, sensor log and estimators.

### 5.3 Sensor models (phone only)

| Sensor | Rate | Model |
|---|---|---|
| **Accelerometer** | 100 Hz | Specific force in the phone frame, i.e. car acceleration + gravity, which leaks into the forward axis on slopes. Includes white noise (about 0.01 m/s² per sample), a random power-on bias (up to about 0.2 m/s² per axis), slow bias drift, scale error (±1%), cross-axis coupling, quantisation, temperature-driven bias drift, road vibration that grows with speed, and an engine-idle vibration signature when stopped. |
| **Gyroscope** | 100 Hz | Angular rate in the phone frame. White noise (about 0.05 °/s per sample), leftover bias (about 0.1 °/s), slow bias drift, scale error, vibration. |
| **Mount** | n/a | Fixed rotation from phone frame to car frame, random within configurable limits. Optional step change partway through the run (S7). |
| **GNSS position fix** | 1 Hz | Position error correlated over time (a first-order Gauss–Markov process with a 30–60 s time constant, σ about 3 m in open sky), plus latency. The reported accuracy is sometimes **optimistic**, as on real phones. |
| **GNSS raw measurements** | 1 Hz | Doppler speed (σ about 0.1–0.2 m/s), number of satellites used, mean signal strength (C/N0). Signal strength fades through the degraded zone at the tunnel mouth. |
| **GNSS on exit** | n/a | A reacquisition delay of 1–5 s, then a few fixes with inflated error and a reflection-induced bias that decays over time. |
| **Barometer** | 25 Hz | Pressure from elevation (about 8.3 m per hPa near sea level), noise σ about 0.03 hPa, slow drift. Tunnel effects: a pressure step at entry, slow variation from traffic pushing air through the tunnel, random spikes (the car's ventilation). |
| **Ambient light** | about 5 Hz, reported on change | Daylight outside, brighter entrance zone, then dim interior, with sensor lag. Optional periodic ceiling-light pattern. |
| **Magnetometer** | 50 Hz | Earth's field plus strong random disturbance inside the tunnel. Modelled so we can show it being **rejected**. |
| **Cellular** | events | Handovers between serving cells at configured positions, with ±50–100 m jitter and missed events. |

### 5.4 Option E sensors

| Sensor | Model |
|---|---|
| **BLE beacons** | Beacons along the tunnel with configurable spacing and transmit power. Signal strength falls off with distance (exponent n ≈ 2–3, possibly lower in a tunnel because it guides the signal), plus random fading (σ 4–6 dB). Also modelled: scan windows, detection probability, a fraction of **failed beacons**, and beacons placed slightly off their mapped positions. |
| **OBD / car speed** | 1–10 Hz. Speed with a scale error (1–3%, from tyre wear and pressure), 1 km/h quantisation and 100–300 ms latency. |

### 5.5 Device profiles

| Profile | Differences |
|---|---|
| `GENERIC` (default) | All of §5.3 |
| `IOS` | No raw GNSS (only fix, speed and speed accuracy), no light sensor, no cell info |
| `LOW_END` | No barometer, about 2× IMU noise and bias |

Comparing profiles answers the question "how much is each sensor worth?" directly.

## 6. Estimation

### 6.1 Common interface

```java
public interface Estimator {
    String id();                                  // "A-const-vel", "D-pf", "D-pf+ble", ...
    void reset(EstimatorContext ctx);             // map, route, device profile, config, random stream
    void onEvent(SensorEvent e);                  // every sensor event, in timestamp order
    Estimate current();                           // state after the last event
}

public record Estimate(
    double t,
    MapPosition position,                         // best estimate (edge, distance along it)
    double sigmaAlong,                            // 1σ along-road uncertainty [m]
    double speed, double sigmaSpeed,
    Map<EdgeId, Double> branchProbability,        // sums to 1 over the edges currently possible
    Mode mode,                                    // GNSS, DEAD_RECKONING, RECOVERING
    Optional<ParticleSnapshot> particles          // particle filter only, for display
) {}
```

### 6.2 Alignment front-end (shared)

This step turns phone-frame IMU data into **car-frame** signals:

- `a_fwd`: forward acceleration
- `a_lat`: sideways acceleration
- `ω_yaw`: rotation rate about the car's vertical axis
- `θ`: pitch

It estimates the phone-to-car rotation in three ways:

1. **Gravity direction** while stopped or driving at steady speed gives the phone's roll and pitch relative to the car.
2. **The forward axis** comes from correlating horizontal acceleration with changes in GNSS Doppler speed during acceleration and braking.
3. **Cars don't slide sideways**: average sideways and vertical acceleration should be about zero on straight roads, which refines the result.

A config switch `alignment: ESTIMATED | TRUTH` lets an evaluation separate alignment errors from estimator errors.

### 6.3 Aiding modules (shared by C, D and E)

Each module turns sensor data into a likelihood, i.e. a score for how well a hypothesis (road position, speed, biases) explains the measurement. The same module code serves the EKF (as a linearised measurement) and the particle filter (as a weight).

| Module | Measurement | Predicted value | Notes |
|---|---|---|---|
| `GnssPositionAiding` | Position fix | x/y position of the hypothesis on the road | Heavy-tailed (Student-t) likelihood so outliers don't dominate. Outlier gate. Inflated noise for the first fixes after exit. |
| `GnssSpeedAiding` | Doppler speed | v | Main source of bias calibration before the tunnel |
| `CurvatureAiding` | ω_yaw − gyro bias | v · κ(edge, s), where κ is the map's curvature | **Main phone-only correction inside the tunnel**, and the branch detector. Averaged over a window, and the likelihood is softened because errors are correlated. |
| `LateralAccelAiding` | a_lat | v² · κ(edge, s) | A second, independent view of curvature |
| `BaroAiding` | Barometric altitude − baro offset | z_map(edge, s) | Heavy-tailed likelihood to survive tunnel pressure spikes. Run at about 2 Hz. |
| `ZeroVelocityAiding` | Stop detector (low motion + idle vibration signature) | v = 0 | A strong correction when stopped. Accelerometer bias becomes observable while stopped. |
| `SpeedPriorAiding` | Speed limit / traffic speed at s | v | Weak. Scenario S9 makes the traffic data stale. |
| `PortalAiding` | Tunnel entry/exit event from light or GNSS signal strength | s ≈ s_portal | σ grows with speed × sensor lag |
| `CellAiding` | Cell handover event | s within the handover zone | Coarse |
| `BleAiding` (E) | Signal strength per beacon | Path-loss model from the beacon's mapped position | Robust to missing and failed beacons |
| `ObdSpeedAiding` (E) | OBD speed | k · v, with scale factor k calibrated before the tunnel | |

Each aiding module can be **switched on or off** individually. That is what the ablation study in §8 relies on.

### 6.4 Estimators

| ID | Name | Description |
|---|---|---|
| **A** | Constant velocity | Holds the last GNSS speed and advances along the route. The baseline most apps roughly use. |
| **B** | IMU dead reckoning | Integrates `a_fwd` with the bias calibrated before the tunnel. No map aiding beyond following the road. |
| **C** | Map-aided EKF | Extended Kalman filter with state `[s, v, b_a]`. Uses GNSS, stop detection, speed prior, portal events, grade correction from the map. No curvature matching, because that is strongly nonlinear and has multiple possible answers. Takes the route's branch by default. Shows **why** D is needed. |
| **D** | **Particle filter on the road network** | The main approach, described in §6.5 |
| **D+E** | D with BLE and/or OBD | Same filter with the extra aiding modules switched on |

### 6.5 Particle filter (D) in detail

**State per particle:** `(edge, s, v, b_a, b_g, b_h)`

| Symbol | Meaning |
|---|---|
| edge, s | Which road edge, and distance along it |
| v | Speed |
| b_a | Forward accelerometer bias |
| b_g | Gyroscope yaw-rate bias |
| b_h | Barometer altitude offset |

All biases are learned **while GNSS is still available**. Calibration is simply the filter running with GNSS, not a separate step.

**Propagation** at 10 Hz. Each step uses the IMU data summed over the 10 samples since the last step (dt = 0.1 s).

```
a      = mean(a_fwd) − b_a − g · grade_map(edge, s)     // remove slope gravity using this particle's map grade
v'     = max(0, v + a·dt + w_v)
s'     = s + v·dt + ½·a·dt²
b_*'   = b_* + w_b                                      // slow random-walk drift
if s' > length(edge): move to a successor edge
                      (route edge with probability p_follow, otherwise the other branch)
```

Using **each particle's own map grade** is deliberate. A phone cannot tell road slope apart from forward acceleration, so the slope correction depends on the position hypothesis. A particle in the wrong place corrects for the wrong slope and is penalised by the other aiding modules.

**Measurement update:** multiply each weight by the likelihood from every active aiding module that has new data.

**Resampling:** systematic resampling when the effective number of particles drops below N/2. Small random jitter is added to s, v and the biases so the particles don't all collapse onto the same values. To keep **both branches alive**, a minimum share of particles (for example 5%) stays on each possible branch for a few seconds after the fork, so the correct branch isn't lost before the curvature evidence arrives.

**Output:**
- Branch probability = total weight of particles on each edge.
- Position = weighted mean and spread on the most probable edge.
- If the top branch has less than 0.9 probability, the estimate is reported as **ambiguous**.

**Size:** default N = 1,000. A slider allows 100–5,000 to show the cost/accuracy trade-off.

**Known risks** (discussed in the write-up):
- Over-confidence from correlated curvature and barometer errors; mitigated by softening the likelihoods.
- Particle depletion on long straight tunnels.
- A filter that is wrong but confident.

### 6.6 Navigation layer

- **Callout policy.** The maneuver sits at the fork's road position. Callouts fire at configured distances before it (default 1,000 m, 400 m and "now" at 50 m). Two policies:
  - `MEAN`: fire when the estimated distance to the maneuver drops to D.
  - `CONSERVATIVE`: fire when there is at least probability α (default 0.2) that the remaining distance is already ≤ D. Because being late is worse, this deliberately errs early.
- **Reroute detector.** If the total probability of off-route edges exceeds 0.9, raise a reroute event. Its latency is measured.
- **Display smoother.** The marker never moves backwards along the route and blends toward the estimate over about 2 s with a speed limit. The smoother is kept separate from the estimator, so metrics are reported for both the estimate and the displayed marker.

## 7. Scenario configuration

Scenarios are YAML files, chosen over JSON because YAML allows comments, which helps when editing live. Any value can be overridden from the UI or the command line.

```yaml
name: curved-3km-with-exit
seed: 42
duration: 400               # seconds
device: GENERIC             # GENERIC | IOS | LOW_END
options: { ble: false, obd: false }

road:
  main:
    - { type: straight, length: 1500, grade: 0.0 }
    - { type: clothoid, length: 150, curvatureEnd: 0.002 }
    - { type: arc,      length: 800, grade: -0.03 }
    - { type: clothoid, length: 150, curvatureEnd: 0.0 }
    - { type: straight, length: 2500, grade: 0.02 }
  tunnels:
    - { from: 1800, to: 4800, gnssDegradedZone: 60, lighting: day }
  branches:
    - at: 3900               # distance along the main road
      ramp:
        - { type: clothoid, length: 80, curvatureEnd: -0.01 }
        - { type: arc,      length: 300, grade: 0.04 }
        - { type: straight, length: 600 }
      tunnelUntil: 250      # the ramp stays in the tunnel for its first 250 m

route: { takeBranchAt: 3900 }   # the navigation expects the exit

cars:
  - id: car1
    driver: { speedFactor: 1.0, pFollowRoute: 1.0 }
    events:
      - { at: 2500, type: slowTo, speed: 8 }
      - { at: 2700, type: stop, duration: 30 }

sensors:                    # overrides of device-profile defaults
  accel: { biasSigma: 0.2, noise: 0.01 }
  gyro:  { biasSigma: 0.1 }
  gnss:  { reacquireDelay: [1, 5] }

estimators: [A, B, C, D]
estimatorConfig:
  D: { particles: 1000, pFollow: 0.8, aiding: { curvature: true, baro: true } }
```

### Scenario library

| ID | Scenario | What it tests |
|---|---|---|
| S1 | Straight, flat 2 km tunnel, steady 80 km/h | Sanity check. The **worst case** for map matching, because the road has no shape. |
| S2 | Curved 3 km tunnel with an exit at 2.1 km, free-flowing traffic | **Headline scenario**: callout and branch choice |
| S3 | 5 km underwater tunnel, V-shaped profile (−4% then +4%) | Barometer and slope correction |
| S4 | 5 km straight flat tunnel, stop-and-go traffic jam | Stop detection. Worst case for A. |
| S5 | 60 s full stop inside the tunnel (incident), then moving again | Stop detection, bias observability |
| S6 | Driver misses the exit, or takes an exit not on the route | Branch detection, reroute latency |
| S7 | Phone shifts in its mount in the middle of the tunnel | Alignment robustness |
| S8 | Two 800 m tunnels separated by 150 m of open road | Partial recovery, a short GNSS window |
| S9 | Traffic data says free flow, but there is a jam | A wrong speed prior |
| E1–E3 | S2 with BLE / S4 with OBD / S2 with BLE and 30% of beacons failed | Option E value and robustness |

## 8. Evaluation

### Metrics per run, per estimator

**Along-road error.** Defined as e(t) = road-path distance from the true position to the estimated position, with a sign. If the estimate is on the wrong branch, the error is measured through the fork node and flagged. Reported as:
- RMS inside the tunnel
- maximum inside the tunnel
- value at the tunnel exit
- error as a function of time spent in the tunnel

**Callouts:**
- true distance to the maneuver when each callout fires
- timing error in seconds
- `late` flag: true distance < D − tolerance

**Branch:**
- time from the fork until the correct branch reaches probability > 0.9
- time spent displaying the wrong branch
- reroute latency

**Recovery:**
- time from GNSS reacquisition until |e| < 10 m
- largest jump in the estimate and in the displayed marker

**Consistency** (is the claimed uncertainty honest?):
- fraction of time |e| < 2σ (should be about 95%)
- NEES, the standard normalised check of error against claimed uncertainty

**Runtime:** filter cost per step.

### Batch studies (`sim-batch`)

- **Monte Carlo:** 200 or more seeds per scenario. Report median and 95th percentile; plot error against time in the tunnel as bands, and the error distribution (CDF) at the exit.
- **Tunnel length sweep:** 0.5 to 5 km, for straight-flat versus curvy-hilly tunnels.
- **Ablation:** D with each aiding module switched off in turn, giving the contribution of each sensor.
- **Device profiles:** GENERIC vs IOS vs LOW_END.
- **Sensitivity:** to IMU bias, map error and particle count.

Outputs go to `results/<study>/`: CSV files plus PNG figures (XChart), reused in the write-up.

## 9. Visual simulator (JavaFX)

```
┌──────────────────────────────────────────────────────────────────────┐
│ Scenario ▼  Seed [42]  Device ▼  ☑A ☑B ☑C ☑D  ☐BLE ☐OBD  ▶ ⏸ ⏭ 1x▼  │
├───────────────────────────────────────────────┬──────────────────────┤
│  Map view (pan/zoom)                          │ Parameters           │
│  road, tunnel shading, beacons, branch        │  tunnel length ──●── │
│  true car ●   estimates ◆ (one colour each)   │  accel bias    ──●── │
│  particle cloud, ±2σ band along the road      │  particles     ──●── │
│  branch probabilities at the fork             │  traffic       ▼     │
├───────────────────────────────────────────────┤  [Apply → re-run]    │
│  Straight-line strip view: 0 ─────────── L    │──────────────────────│
│  true │ estimates with ±2σ │ callout markers  │ Event log            │
├───────────────────────────────────────────────┤  12.0s tunnel entry  │
│  Charts (tabs): error±2σ | speed | a_fwd |     │  98.4s callout 400m  │
│  yaw rate vs map | baro vs map | GNSS C/N0 |  │   (true: 371 m)      │
│  light | BLE signal                           │  ...                 │
├───────────────────────────────────────────────┴──────────────────────┤
│  Timeline ═══════════●════════════════════════  t = 143.2 s          │
└──────────────────────────────────────────────────────────────────────┘
```

- The **straight-line strip view** shows the 1D along-road error far more clearly than the map can at tunnel scale.
- Charts are drawn with a lightweight custom Canvas plotter that reduces the number of points to draw. JavaFX's built-in `LineChart` is too slow for 100 Hz series.
- Clicking **Apply** reruns the scenario with the same seed, and the timeline keeps its position.

## 10. Technology

| Concern | Choice |
|---|---|
| Language | Java 21 |
| Build | Gradle (Kotlin DSL) with wrapper; `org.openjfx.javafxplugin` |
| UI | JavaFX 21 |
| Config | Jackson + `jackson-dataformat-yaml` |
| Linear algebra | Hand-written small matrices (the EKF is 3×3). EJML only if needed. |
| Batch plots | XChart (PNG export) |
| Tests | JUnit 5 + AssertJ |

Run commands (target): `./gradlew :sim-app:run` and `./gradlew :sim-batch:run --args="--study montecarlo --scenario S2"`.

## 11. Testing strategy

- **Geometry:** converting road distance to x/y and back gives the same point; curvature and heading stay continuous across clothoids; elevation accumulates correctly from grades.
- **Sensor models:** statistical tests (noise σ, bias drift, Doppler error) on long synthetic runs.
- **Estimators:** with zero noise every estimator should have about zero error. With GNSS always on, D should match GNSS accuracy. On a constant-speed straight road, A should be exact.
- **Determinism:** the same seed and config must give identical logs and estimates.
- **Regression:** threshold checks on key metrics for S1–S4, so changes don't silently make things worse.

## 12. Milestones

| # | Milestone | Done when |
|---|---|---|
| M0 | Build skeleton, README, empty JavaFX window | `./gradlew :sim-app:run` opens a window |
| M1 | Map, scenario loader, true car motion, map view and timeline | A car drives S2 in the UI |
| M2 | Sensor models, SensorLog, CSV export, sensor charts | All §5.3 sensors are visible in charts |
| M3 | Alignment, A/B/C, metrics, error charts, strip view | Baselines are evaluated on S1–S5 |
| M4 | Particle filter D, branches, callout, reroute, display smoother | S2 and S6 work end to end |
| M5 | Option E (BLE, OBD), device profiles | E1–E3 and profile comparison |
| M6 | Batch studies, figures | `results/` is filled |
| M7 | Write-up, screen recording, live-demo rehearsal | Ready to submit |

**Live-demo rehearsal list** (likely interviewer requests): change tunnel length; add a jam or a stop; make the driver miss the exit; increase IMU bias; disable the barometer; kill beacons; switch to the iOS profile; reduce particles; move the exit closer to the tunnel entrance.

## 13. Open questions (resolved)

1. Root package and repository name: `tunnelsim`.
2. Map errors: the estimators use the true map. A map-error study is listed as future work.
3. Night lighting: modelled (`road.lighting: night`).

## 14. As built: changes from the design

Each change below was driven by a failure seen in the simulator or the Monte Carlo runs.

| Area | Design | As built | Why |
|---|---|---|---|
| Particle state | `(edge, s, v, b_a, b_g, b_h)` | `(edge, s, v, b_a, k_g)` per particle (+ OBD scale residual with option E). Gyro bias, lateral accelerometer bias and barometer offset are calibrated **globally** by a small Kalman filter / low-pass while GNSS is available (`BiasCalibrator`). | As per-particle nuisance states, the baro offset and gyro bias made weights vary even on a road with no information. Frequent resampling on such weights makes the cloud's mean speed random-walk ("genetic drift"); on S1 this cost ~1.5 m/s of speed error. Static parameters are also a known weak spot of particle filters: resampling discards what was learned. |
| Accelerometer bias | Learned by the particle filter during the GNSS phase | Kalman calibration before the tunnel; particles inherit the calibrated value and its uncertainty at tunnel entry | The particle filter calibrated the bias worse than the EKF (C) and drifted further in the tunnel |
| Gyro scale | Not modelled | Per-particle gyro scale factor with a 1% prior | A scale error biases the speed inferred from curvature matching consistently, so it cannot be treated as white noise |
| Speed in the filter state | Clamped at ≥ 0 | May go slightly negative in the state; only position is forced forward and the output is clamped | Clamping made particles with a too-high accelerometer bias look perfectly stopped, so every stop selected a biased bias. In stop-and-go traffic this cost up to 1 m/s per cycle. |
| Stop detector | Variance of specific force | Plus **immediate release** when the specific-force vector departs from its value at rest | A late release told the filters "stopped" for 0.3–0.5 s while the car was already accelerating at 2 m/s² |
| Alignment (forward axis) | Regression of horizontal accel on GNSS speed change | The same, but only on straight road (low yaw rate) and with good fixes | In curves, lateral acceleration × random speed changes rotated the forward estimate by 0.5° per fix; near portals noisy Doppler did the same. 2.5–4° of misalignment leaks curve acceleration into the forward axis. |
| "Up" axis | Gravity whenever stopped, plus slow refinement while cruising with GNSS | Estimated once at the start (first stop, or steady cruising if the car never stops), then **frozen**; re-estimated only after phone handling | Re-estimating at every stop added ±0.4° of tilt (deceleration still in the low-pass). That is 0.07 m/s² of fake forward acceleration after each stop. Worse, a stop on a slope absorbed the slope into "up", which the estimators then subtracted again from the map: a jam on a 3% climb gave **2.1 km** errors (S11). A constant tilt is harmless because the bias calibration absorbs it. |
| Accelerometer scale | Not planned | Effective forward scale (sensor scale + car pitch under acceleration, ~+3%) from the magnitude of the alignment regression. Corrected for GNSS speed noise in the regressor, shrunk towards 0 by its own uncertainty, and used by B, C and D. | Stop-and-go traffic integrates scale errors every cycle. Estimating the scale in the Kalman calibrator instead was biased low, because during the initial pull-away the forward axis is still converging and its misalignment cancels the scale. |
| Phone handling | Not planned | Gyro spikes (> 20°/s) propagate the alignment; estimators widen the accelerometer bias by 0.05 m/s² afterwards | S7, S12, S13: a 27° phone rotation. Tracking is now accurate to within the gyro's ~1% scale error. The first version widened the bias by 0.1 m/s² to cover a ~4° tracking error, which turned out to be a simulator bug (next row). |
| Body pitch and roll (simulator) | Follow acceleration instantly | Follow through a 0.3 s suspension lag | When the car stopped, the driver model ended the deceleration in one 10 ms step, so the braking dive snapped back at ~1.4 rad/s. The aligner took that gyro spike for phone handling and rotated its alignment by the car's own pitch change: about 0.5° of tilt, 0.1 m/s² of gravity leak, after every stop. This had been there since the start, masked by the large bias widening after "handling". It surfaced as a regression in S11/S10 when the widening was reduced. |
| Simulated gyro during a phone shift | Rate by central difference of the stored mount | Each stored mount step counted exactly once (difference over t ± dt/2, tolerant time comparison) | The central difference over t ± dt, with floating-point step times, counted some steps twice and others not at all. The simulated gyro under-reported up to ~20% of the rotation, depending on the seed. The leftover tilt leaked gravity backwards and both C and D drifted behind (always negative, because the shift is identical in every run). Found because the user asked why S7's errors were always negative. |
| Barometer likelihood | Student-t at 2 Hz | Down-weighted by its correlation time (20 s) | Tunnel pressure disturbances are correlated for ~25 s. Treated as independent, they made the filter 7× over-confident and caused the worst tails. |
| Lateral acceleration aiding | On | **Off by default** (available as an option) | Body roll in curves adds gravity to the lateral axis (~8%), which biases speed by ~4% |
| OBD | Scale-corrected speed | Also latency-compensated, with the scale residual carried per particle | The 0.2 s latency is systematic during every acceleration in stop-and-go traffic |
| Several exits in one tunnel | Off-route = past the maneuver | A position off the route counts as past the maneuver only if it left the route at the maneuver's fork. A driver who took an earlier exit ends the callouts for the planned one (not scored as missed). | With several exits, particles on a wrong *earlier* exit would otherwise make the callouts for the planned exit fire early. Found when adding S14/S15. |
| Monte Carlo | Keep runs | Keep only a per-run digest; no particle snapshots in batch | Memory: 40 parallel 7-minute runs exhausted the heap |
