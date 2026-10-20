package tunnelsim.estimator;

import tunnelsim.align.VehicleImu;
import tunnelsim.estimator.Estimate.ParticleSnapshot;
import tunnelsim.map.Edge;
import tunnelsim.map.Features.Beacon;
import tunnelsim.map.Features.CellZone;
import tunnelsim.map.Features.Portal;
import tunnelsim.map.MapPosition;
import tunnelsim.map.RoadNetwork;
import tunnelsim.scenario.EstimatorConfig;
import tunnelsim.scenario.SensorsConfig;
import tunnelsim.sensor.SensorEvent;
import tunnelsim.sensor.SensorEvent.BaroSample;
import tunnelsim.sensor.SensorEvent.BleScan;
import tunnelsim.sensor.SensorEvent.CellChange;
import tunnelsim.sensor.SensorEvent.GnssFix;
import tunnelsim.sensor.SensorEvent.LightSample;
import tunnelsim.sensor.SensorEvent.ObdSpeed;
import tunnelsim.util.MathUtil;
import tunnelsim.util.Rng;

import java.util.HashMap;
import java.util.Map;

/**
 * Approach D: a particle filter on the road network. Each particle is a hypothesis
 * (edge, s, v, accel bias, gyro scale). Particles move along the road using the
 * forward accelerometer minus the gravity component of <em>their own</em> map grade, and choose a
 * branch at forks. Weights come from matching what the phone measures against what the map
 * predicts at each hypothesis: yaw rate (v times curvature), lateral acceleration (v^2 times
 * curvature), barometric altitude (map elevation), stops, portals, cells, plus GNSS outside
 * tunnels and BLE/OBD with option E.
 * <p>
 * Gyro bias, lateral accelerometer bias and barometer offset are calibrated globally while GNSS
 * is available ({@link BiasCalibrator}) instead of per particle: as per-particle nuisance states
 * they made weights vary even where the road carries no information, and frequent resampling on
 * such weights makes the cloud's mean speed random-walk ("genetic drift").
 */
public final class RoadParticleFilter implements Estimator {
    private static final double SIGMA_ZUPT = 0.1;    // m/s
    private static final double PRIOR_DECORRELATION = 30;
    private static final int BLOCK = 10;             // IMU samples per propagation step
    private static final int SNAPSHOT = 150;

    private final EstimatorContext ctx;
    private final RoadNetwork net;
    private final EstimatorConfig cfg;
    private final EstimatorConfig.AidingConfig aid;
    private final EstimatorConfig.ParticleFilterConfig pf;
    private final int window;
    private final Rng rng;
    private final Edge[] edges;
    private final int n;
    private final String id;
    private final GnssTracker gnss = new GnssTracker();
    private final PortalDetector portals = new PortalDetector();

    // The particle cloud, stored as parallel arrays (one entry per particle) for speed:
    // edge/s = where it is on the road, v = speed, ba = accelerometer bias, ks = gyro scale,
    // ko = OBD scale residual (option E), accel = last acceleration, logw = log of its weight.
    private int[] edge;
    private double[] s, v, ba, ks, ko, accel, logw;
    private final BiasCalibrator calibrator;
    // Per particle: yaw rate and lateral acceleration the map predicts, summed over the current
    // curvature window, to be compared with what the gyro and accelerometer measured.
    private double[] predYaw, predLat;
    /** Per particle: altitude change predicted since the start of the vertical-speed window [m]. */
    private double[] predDz;
    private double windowStartAltitude = Double.NaN;
    private double windowStartT;
    private boolean initialised;
    private double t;

    // Block and window accumulators.
    private double blockAcc, blockYaw, blockLat, lastBlockT = Double.NaN;
    private double lastAccelScale;
    private int blockCount, blockStationary;
    private double winYaw, winLat;
    private int winBlocks;
    private int movingBlocks;
    private double lastPriorT = Double.NEGATIVE_INFINITY;
    private double baroSum, lastBaroT = Double.NEGATIVE_INFINITY;
    private int baroCount;
    // OBD scale calibration (option E).
    private double obdSum, gnssSpeedSum, lastObd = Double.NaN;
    // Fork guard: first time noticeable mass passed each fork.
    private final Map<Integer, Double> forkPassed = new HashMap<>();
    private final Map<Integer, Beacon> beaconsById = new HashMap<>();
    private int reinitialisations;
    private boolean snapshots = true;

    public RoadParticleFilter(EstimatorContext ctx, String id) {
        this.ctx = ctx;
        this.net = ctx.net();
        this.cfg = ctx.config();
        this.aid = cfg.aiding;
        this.pf = cfg.pf;
        // Number of 0.1 s propagation blocks in one curvature-matching window (e.g. 1 s -> 10 blocks).
        this.window = Math.max(1, (int) Math.round(pf.curvatureWindow / (BLOCK * 0.01)));
        this.rng = ctx.rng();
        this.id = id;
        this.n = Math.max(10, cfg.particles);
        this.edges = net.edges().toArray(new Edge[0]);
        edge = new int[n];
        s = new double[n];
        v = new double[n];
        ba = new double[n];
        ks = new double[n];
        ko = new double[n];
        accel = new double[n];
        calibrator = new BiasCalibrator(net);
        logw = new double[n];
        predYaw = new double[n];
        predLat = new double[n];
        predDz = new double[n];
        // Index beacons by id so each BLE reading can find its beacon's surveyed position quickly.
        for (Beacon b : net.beacons()) {
            beaconsById.put(b.id(), b);
        }
    }

    @Override
    public String id() {
        return id;
    }

    public int reinitialisations() {
        return reinitialisations;
    }

    /** Disables particle snapshots in estimates (batch runs do not draw them). */
    public void setSnapshots(boolean enabled) {
        this.snapshots = enabled;
    }

    // ---------------------------------------------------------------------------------------
    // Sensor input
    // ---------------------------------------------------------------------------------------

    @Override
    public void onSensor(SensorEvent event) {
        t = event.t();
        // Each sensor type has its own update; IMU samples arrive separately via onVehicleImu.
        switch (event) {
            case GnssFix fix -> onGnss(fix);
            case BaroSample b -> onBaro(b);
            case LightSample l -> {
                // A sudden jump in light level means the car just passed a tunnel portal.
                if (portals.onLight(l) && initialised && aid.portal) {
                    portalUpdate();
                }
            }
            case CellChange c -> {
                // The phone changed serving cell: the car is near that cell's handover zone.
                if (initialised && aid.cell) {
                    cellUpdate(c.cellId());
                }
            }
            case BleScan scan -> {
                if (initialised && ctx.useBle()) {
                    bleUpdate(scan);
                }
            }
            case ObdSpeed o -> {
                lastObd = o.speed();
                // Only used inside tunnels, and only once the speedometer scale has been learned
                // against GNSS (enough speed summed up to make the ratio reliable).
                if (initialised && ctx.useObd() && gnssSpeedSum > 200 && !gnss.fresh(t)) {
                    // Calibrated speedometer scale; its residual error (ko, per particle) is
                    // systematic, so it is carried as a nuisance state rather than treated as noise.
                    double scale = obdSum / gnssSpeedSum;
                    double sigma = 0.25;
                    double latency = ctx.scenario().sensors.obd.latency; // assumed calibrated per car
                    for (int i = 0; i < n; i++) {
                        // The OBD reading is a little old: compare with this particle's speed back then.
                        double vThen = v[i] - accel[i] * latency;
                        double e = vThen - o.speed() / (scale * (1 + ko[i]));
                        logw[i] += -0.25 * e * e / (sigma * sigma); // tempered x0.5: quantisation is correlated
                    }
                    normaliseAndResample();
                }
            }
            default -> {
            }
        }
    }

    private void onGnss(GnssFix fix) {
        // Ignore GNSS if it is switched off for this run, or if the fix is too inaccurate.
        if (!aid.gnss || !gnss.accept(fix)) {
            return;
        }
        calibrator.onFix(fix);
        // Learn the OBD speedometer scale: ratio of summed OBD speed to summed GNSS speed.
        if (!Double.isNaN(lastObd) && fix.speed() > 5) {
            obdSum += lastObd;
            gnssSpeedSum += fix.speed();
        }
        // Reported accuracy is a 68% radius; divide by 1.5 to get a per-axis standard deviation.
        double sigma = Math.max(1.5, fix.accuracy() / 1.5);
        if (!initialised) {
            initialise(fix, sigma);
            return;
        }
        // The first fixes after a tunnel suffer from reflections: trust them less.
        double inflate = gnss.fixesSinceOutage() <= 5 ? 2.5 : 1;
        sigma *= inflate;

        // If no particle is anywhere near the fix, the filter is lost: re-seed half of it.
        double minD = Double.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            minD = Math.min(minD, Math.hypot(x(i) - fix.x(), y(i) - fix.y()));
        }
        if (minD > Math.max(50, 5 * sigma)) {
            reseed(fix, sigma, 0.5);
            reinitialisations++;
        }

        // Score every particle by how well its position and speed agree with the fix. The position
        // term is heavy-tailed (Student-t, nu = 4), so one bad fix cannot wipe out the cloud.
        double nu = 4;
        double sv = Math.max(0.15, fix.speedAccuracy()) * inflate;
        for (int i = 0; i < n; i++) {
            double dx = x(i) - fix.x(), dy = y(i) - fix.y();
            double d2 = (dx * dx + dy * dy) / (sigma * sigma);
            logw[i] += -(nu + 2) / 2 * Math.log1p(d2 / nu);
            double e = (v[i] - fix.speed()) / sv;
            logw[i] += -0.5 * e * e;
        }
        // While GNSS is available the biases come from the calibrator; at tunnel entry the
        // particles carry the calibrated values with their uncertainty.
        if (calibrator.ready()) {
            double sa = Math.max(0.003, calibrator.accelBiasSigma());
            for (int i = 0; i < n; i++) {
                ba[i] = calibrator.accelBias() + rng.gaussian(sa);
            }
        }
        normaliseAndResample();
    }

    /** Creates the cloud around the first GNSS fix: spread along the road by the fix uncertainty. */
    private void initialise(GnssFix fix, double sigma) {
        double[] near = net.nearest(fix.x(), fix.y());
        for (int i = 0; i < n; i++) {
            edge[i] = (int) near[0];
            s[i] = MathUtil.clamp(near[1] + rng.gaussian(sigma), 0, edges[edge[i]].length);
            v[i] = Math.max(0, fix.speed() + rng.gaussian(0.3));
            ba[i] = rng.gaussian(0.2);                        // bias unknown until calibrated
            ks[i] = 1 + rng.gaussian(pf.gyroScaleSigma);      // gyro scale near 1, about 1% spread
            ko[i] = rng.gaussian(0.005);
            logw[i] = -Math.log(n);                           // all particles equally likely
        }
        initialised = true;
    }

    /** Moves a random share of the particles back to the GNSS fix (recovery when the filter is lost). */
    private void reseed(GnssFix fix, double sigma, double fraction) {
        double[] near = net.nearest(fix.x(), fix.y());
        for (int i = 0; i < n; i++) {
            if (rng.chance(fraction)) {
                edge[i] = (int) near[0];
                s[i] = MathUtil.clamp(near[1] + rng.gaussian(sigma), 0, edges[edge[i]].length);
                v[i] = Math.max(0, fix.speed() + rng.gaussian(0.5));
            }
        }
    }

    private void onBaro(BaroSample b) {
        if (!aid.baro) {
            return;
        }
        // Average the 25 Hz pressure samples and process them once per baroInterval (1 s).
        baroSum += b.pressure();
        baroCount++;
        if (!initialised || t - lastBaroT < pf.baroInterval) {
            return;
        }
        lastBaroT = t;
        double h = MathUtil.pressureToAltitude(baroSum / baroCount);
        baroSum = 0;
        baroCount = 0;
        boolean fresh = gnss.fresh(t);
        // While GNSS is available, the calibrator learns the barometer's offset against the map.
        calibrator.onBaroAltitude(h, fresh);
        double offset = calibrator.baroOffset();
        if (fresh || Double.isNaN(offset)) {
            windowStartAltitude = Double.NaN;
            return; // GNSS pins the position better; the barometer is used inside tunnels
        }
        if (aid.baroVerticalSpeed) {
            verticalSpeedUpdate(h);
        }
        // Absolute altitude match: barometric altitude (minus its offset) should equal the map
        // elevation at the particle's position. On a slope this tells where along the road we are.
        double nu = 4;
        // Down-weight for correlated errors: n updates per correlation time carry about one update's information.
        double temper = 1.0 / Math.max(1, pf.baroCorrelationTime / pf.baroInterval);
        for (int i = 0; i < n; i++) {
            double e = (h - offset - edges[edge[i]].elevation(s[i])) / pf.sigmaBaro;
            logw[i] += -temper * (nu + 1) / 2 * Math.log1p(e * e / nu);
        }
        normaliseAndResample();
    }

    /**
     * Barometric vertical speed: the altitude change over a few seconds against each particle's
     * predicted change (distance travelled times map slope). It carries speed information on slopes
     * and is insensitive to slowly varying pressure offsets (weather, tunnel entry step), which the
     * absolute altitude match has to be down-weighted for.
     */
    private void verticalSpeedUpdate(double h) {
        // First call (e.g. right after entering the tunnel): just start a window.
        if (Double.isNaN(windowStartAltitude)) {
            windowStartAltitude = h;
            windowStartT = t;
            java.util.Arrays.fill(predDz, 0);
            return;
        }
        // Wait until the window is complete (5 s by default).
        if (t - windowStartT < pf.verticalSpeedWindow - 1e-6) {
            return;
        }
        // Measured climb over the window, compared with each particle's predicted climb.
        double dh = h - windowStartAltitude;
        double nu = 4;
        for (int i = 0; i < n; i++) {
            double e = (dh - predDz[i]) / pf.sigmaVerticalChange;
            logw[i] += -(nu + 1) / 2 * Math.log1p(e * e / nu);
        }
        // Start the next window from here.
        windowStartAltitude = h;
        windowStartT = t;
        java.util.Arrays.fill(predDz, 0);
    }

    /** A portal was detected: particles near any tunnel mouth gain weight. */
    private void portalUpdate() {
        double nu = 3;
        for (int i = 0; i < n; i++) {
            double best = Double.MAX_VALUE;
            double px = x(i), py = y(i);
            for (Portal p : net.portals()) {
                best = Math.min(best, Math.hypot(px - p.x(), py - p.y()));
            }
            // The light sensor lags, so the uncertainty grows with speed.
            double sigma = 8 + 0.5 * v[i];
            double e = best / sigma;
            logw[i] += -(nu + 1) / 2 * Math.log1p(e * e / nu);
        }
        normaliseAndResample();
    }

    /** A cell handover happened: particles near that cell's handover zone gain weight (coarse, 100 m). */
    private void cellUpdate(int cellId) {
        CellZone zone = null;
        for (CellZone z : net.cells()) {
            if (z.cellId() == cellId) {
                zone = z;
            }
        }
        if (zone == null) {
            return;
        }
        double nu = 3, sigma = 100;
        for (int i = 0; i < n; i++) {
            double e = Math.hypot(x(i) - zone.x(), y(i) - zone.y()) / sigma;
            logw[i] += -(nu + 1) / 2 * Math.log1p(e * e / nu);
        }
        normaliseAndResample();
    }

    /** BLE scan (option E): compare each heard beacon's signal strength with what each particle's distance predicts. */
    private void bleUpdate(BleScan scan) {
        SensorsConfig.BleConfig bc = ctx.scenario().sensors.ble;
        // dz = height difference between the wall-mounted beacon and the phone in the car.
        double nu = 4, sigma = 6, dz = bc.height - 1.2;
        for (int k = 0; k < scan.beaconIds().length; k++) {
            Beacon b = beaconsById.get(scan.beaconIds()[k]);
            if (b == null) {
                continue;
            }
            for (int i = 0; i < n; i++) {
                // Distance from the particle to the beacon's surveyed position.
                double dx = x(i) - b.mx(), dy = y(i) - b.my();
                double d = Math.max(0.5, Math.sqrt(dx * dx + dy * dy + dz * dz));
                // Log-distance path loss: signal drops by 10*n dB per tenfold distance.
                double expected = bc.txPower - 10 * bc.pathLossExponent * Math.log10(d);
                double e = (scan.rssi()[k] - expected) / sigma;
                logw[i] += -0.5 * (nu + 1) / 2 * Math.log1p(e * e / nu); // tempered: shadowing is correlated
            }
        }
        normaliseAndResample();
    }

    // ---------------------------------------------------------------------------------------
    // Propagation and IMU-based updates
    // ---------------------------------------------------------------------------------------

    @Override
    public void onVehicleImu(VehicleImu imu) {
        t = imu.t();
        calibrator.onImu(imu);
        if (imu.mountDisturbed() && initialised) {
            // The phone moved: the residual tracking error (~1% gyro scale times the rotation)
            // changes the gravity leak into the forward axis by up to ~0.05 m/s^2, so the calibrated
            // bias no longer holds exactly. Widen it and let the map decide.
            for (int i = 0; i < n; i++) {
                ba[i] += rng.gaussian(0.05);
                ks[i] = 1 + rng.gaussian(pf.gyroScaleSigma);
            }
        }
        if (!initialised) {
            lastBlockT = imu.t();
            return;
        }
        // Collect 10 IMU samples (0.1 s) into one block: moving 1000 particles at 100 Hz is not
        // needed, and averaging also reduces vibration noise.
        blockAcc += imu.aFwd();
        lastAccelScale = imu.accelScale();
        blockYaw += imu.yawRate();
        blockLat += imu.aLat();
        blockStationary += imu.stationary() ? 1 : 0;
        if (++blockCount < BLOCK) {
            return;
        }
        double dt = Double.isNaN(lastBlockT) ? BLOCK * 0.01 : imu.t() - lastBlockT;
        lastBlockT = imu.t();
        double aF = blockAcc / blockCount, yaw = blockYaw / blockCount, lat = blockLat / blockCount;
        // The block counts as "stopped" if most of its samples said so.
        boolean stationary = blockStationary * 2 > blockCount;
        blockAcc = blockYaw = blockLat = 0;
        blockCount = blockStationary = 0;
        propagate(dt, aF);

        winYaw += yaw;
        winLat += lat;
        winBlocks++;
        boolean updated = false;

        if (aid.zupt) {
            if (stationary) {
                // Zero-velocity update: the car is stopped, so particles with speed far from zero lose weight.
                movingBlocks = 0;
                for (int i = 0; i < n; i++) {
                    double e = v[i] / SIGMA_ZUPT;
                    logw[i] += -0.5 * e * e;
                }
                updated = true;
            } else if (++movingBlocks > 20) {
                // Clearly moving for 2 s: hypotheses that think the car is stopped lose weight.
                for (int i = 0; i < n; i++) {
                    if (v[i] < 0.5) {
                        double e = (0.5 - v[i]) / 0.25;
                        logw[i] += -0.5 * e * e * 0.1;
                    }
                }
                updated = true;
            }
        }

        // End of a curvature window: compare the measured yaw rate (and lateral acceleration) with
        // what each particle predicts from its speed and the map curvature where it is.
        if (winBlocks == window) {
            double my = winYaw / window - calibrator.gyroBias(), ml = winLat / window - calibrator.lateralBias();
            for (int i = 0; i < n; i++) {
                // Predicted yaw rate = speed x curvature, scaled by this particle's gyro-scale guess.
                double py = ks[i] * predYaw[i] / window, pl = predLat[i] / window;
                if (aid.curvature) {
                    // Allowed mismatch: a fixed part plus a part proportional to the turn rate.
                    double sy = pf.sigmaYaw + pf.sigmaYawRelative * Math.abs(py);
                    double e = (my - py) / sy;
                    logw[i] += -0.5 * e * e;
                }
                if (aid.lateralAccel) {
                    double sl = pf.sigmaLat + pf.sigmaLatRelative * Math.abs(pl);
                    double e = (ml - pl) / sl;
                    logw[i] += -0.5 * e * e;
                }
                predYaw[i] = 0;
                predLat[i] = 0;
            }
            updated |= aid.curvature || aid.lateralAccel;
            winYaw = winLat = 0;
            winBlocks = 0;
        }

        // Once per second inside tunnels: a weak pull towards the speed limit / reported traffic speed.
        if (aid.speedPrior && !gnss.fresh(t) && !stationary && t - lastPriorT >= 1) {
            lastPriorT = t;
            for (int i = 0; i < n; i++) {
                double expected = net.expectedSpeed(new MapPosition(edge[i], s[i]));
                // Wide on purpose: repeated every second, it should only matter over ~30 s.
                double sigma = (4 + 0.2 * expected) * Math.sqrt(PRIOR_DECORRELATION);
                double e = (v[i] - expected) / sigma;
                logw[i] += -0.5 * e * e;
            }
            updated = true;
        }
        if (updated) {
            normaliseAndResample();
        }
    }

    /** Moves every particle forward by one block (dt ~ 0.1 s) using the measured forward acceleration. */
    private void propagate(double dt, double aF) {
        double sqdt = Math.sqrt(dt);
        double pFollow = cfg.pFollow;
        double scale = 1 + lastAccelScale;
        for (int i = 0; i < n; i++) {
            Edge e = edges[edge[i]];
            double g = e.grade(s[i]);
            // True acceleration = measured forward acceleration, minus this particle's bias guess,
            // minus gravity along the slope at this particle's position, corrected for the scale.
            double a = (aF - ba[i] - MathUtil.G * g / Math.sqrt(1 + g * g)) / scale;
            // Speed may go slightly negative in the state: clamping at zero would make particles
            // with a too-high accelerometer bias look perfectly stopped, and stop updates would
            // then select a biased bias. Position only ever moves forward.
            double vn = Math.max(-2, v[i] + a * dt + rng.gaussian(pf.sigmaV * sqdt));
            double ds = Math.max(0, 0.5 * (v[i] + vn)) * dt;
            s[i] += ds;
            // Climb predicted for the vertical-speed check: distance travelled times sin(slope).
            predDz[i] += ds * g / Math.sqrt(1 + g * g);
            v[i] = vn;
            accel[i] = a;
            ba[i] += rng.gaussian(pf.sigmaBias * sqdt);       // the bias drifts slowly
            // Past the end of the edge: continue on the next one, choosing a branch at a fork.
            while (s[i] > e.length) {
                if (e.isLeaf()) {
                    // End of the road network: stop here.
                    s[i] = e.length;
                    v[i] = 0;
                    break;
                }
                double rest = s[i] - e.length;
                int routeChild = ctx.route().next(e.id);
                int other = e.children().get(0) == routeChild ? e.children().get(1) : e.children().get(0);
                // Most particles follow the planned route; some take the other branch, so the filter
                // can notice when the driver does.
                edge[i] = rng.chance(pFollow) ? routeChild : other;
                e = edges[edge[i]];
                s[i] = rest;
            }
            // Accumulate the yaw rate (v x curvature) and lateral acceleration (v^2 x curvature) the
            // map predicts at the particle's new position, for the next curvature comparison.
            double k = e.curvature(s[i]);
            predYaw[i] += v[i] * k;
            predLat[i] += v[i] * v[i] * k;
        }
    }

    // ---------------------------------------------------------------------------------------
    // Weights and resampling
    // ---------------------------------------------------------------------------------------

    /** Normalises the weights to sum to one and resamples when too few particles carry the weight. */
    private void normaliseAndResample() {
        // Subtract the largest log-weight before exponentiating, to avoid numerical underflow.
        double max = -Double.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            max = Math.max(max, logw[i]);
        }
        double sum = 0;
        double[] w = new double[n];
        for (int i = 0; i < n; i++) {
            w[i] = Math.exp(logw[i] - max);
            sum += w[i];
        }
        double sumSq = 0;
        for (int i = 0; i < n; i++) {
            w[i] /= sum;
            sumSq += w[i] * w[i];
            logw[i] = Math.log(Math.max(w[i], 1e-300));
        }
        trackForks(w);
        // 1 / sum(w^2) is the "effective number of particles". Below half of n, resample.
        if (1.0 / sumSq < n / 2.0) {
            resample(w);
        }
    }

    /**
     * Systematic resampling: draws a new cloud where likely particles are copied several times and
     * unlikely ones disappear. Small random jitter keeps the copies from being identical.
     */
    private void resample(double[] w) {
        int[] ne = new int[n];
        double[] ns = new double[n], nv = new double[n], nba = new double[n], nks = new double[n], nko = new double[n];
        double[] nacc = new double[n];
        double[] ndz = new double[n];
        double[] npy = new double[n], npl = new double[n];
        // Evenly spaced picks through the cumulative weights, starting at a random offset.
        double step = 1.0 / n;
        double u = rng.uniform() * step;
        double c = w[0];
        int j = 0;
        for (int i = 0; i < n; i++) {
            double target = u + i * step;
            while (target > c && j < n - 1) {
                c += w[++j];
            }
            // New particle i is a (slightly jittered) copy of old particle j.
            ne[i] = edge[j];
            ns[i] = MathUtil.clamp(s[j] + rng.gaussian(0.3), 0, edges[edge[j]].length);
            nv[i] = v[j] + rng.gaussian(0.03);
            nba[i] = ba[j] + rng.gaussian(0.001);
            nks[i] = ks[j] + rng.gaussian(0.0005);
            nko[i] = ko[j] + rng.gaussian(0.0002);
            nacc[i] = accel[j];
            ndz[i] = predDz[j];
            npy[i] = predYaw[j];
            npl[i] = predLat[j];
        }
        edge = ne;
        s = ns;
        v = nv;
        ba = nba;
        ks = nks;
        ko = nko;
        accel = nacc;
        predDz = ndz;
        // Curvature window accumulators travel with the particles.
        System.arraycopy(npy, 0, predYaw, 0, n);
        System.arraycopy(npl, 0, predLat, 0, n);
        // After resampling all particles are equally likely again.
        double lw = -Math.log(n);
        for (int i = 0; i < n; i++) {
            logw[i] = lw;
        }
        guardBranches();
    }

    /** Records when a noticeable share of the particles first passes each fork. */
    private void trackForks(double[] w) {
        for (Edge e : edges) {
            if (e.isLeaf() || forkPassed.containsKey(e.id)) {
                continue;
            }
            // Weight of particles already past the end of edge e (i.e. past its fork).
            double beyond = 0;
            for (int i = 0; i < n; i++) {
                if (edge[i] != e.id && net.isAncestorOrSelf(e.id, edge[i])) {
                    beyond += w[i];
                }
            }
            if (beyond > 0.2) {
                forkPassed.put(e.id, t);
            }
        }
    }

    /**
     * Keeps a minimum share of particles on each branch for a few seconds after a fork, so the
     * correct branch is not lost before the curvature evidence (the ramp's curve) arrives.
     */
    private void guardBranches() {
        for (Map.Entry<Integer, Double> fork : forkPassed.entrySet()) {
            // Only during the guard time after the cloud reached this fork.
            if (t - fork.getValue() > cfg.branchGuardTime) {
                continue;
            }
            Edge parent = edges[fork.getKey()];
            int c0 = parent.children().get(0), c1 = parent.children().get(1);
            // Count particles on each of the two branches.
            int n0 = 0, n1 = 0;
            for (int i = 0; i < n; i++) {
                if (edge[i] == c0) {
                    n0++;
                } else if (edge[i] == c1) {
                    n1++;
                }
            }
            int min = (int) Math.ceil(cfg.branchMinShare * n);
            // Too few particles past the fork yet to judge.
            if (n0 + n1 < 0.3 * n) {
                continue;
            }
            // Top up the weaker branch by moving particles over from the stronger one.
            if (n0 < min) {
                move(c1, c0, min - n0);
            } else if (n1 < min) {
                move(c0, c1, min - n1);
            }
        }
    }

    /** Moves {@code count} random particles from one branch to the other, at the same distance past the fork. */
    private void move(int from, int to, int count) {
        for (int i = 0; i < n && count > 0; i++) {
            int k = rng.nextInt(n);
            if (edge[k] == from) {
                edge[k] = to;
                s[k] = Math.min(s[k], edges[to].length);
                count--;
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // Output
    // ---------------------------------------------------------------------------------------

    /** Map x coordinate of particle i. */
    private double x(int i) {
        return edges[edge[i]].x(s[i]);
    }

    /** Map y coordinate of particle i. */
    private double y(int i) {
        return edges[edge[i]].y(s[i]);
    }

    /** Normalised particle weights (summing to one), from the stored log-weights. */
    private double[] weights() {
        double[] w = new double[n];
        double max = -Double.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            max = Math.max(max, logw[i]);
        }
        double sum = 0;
        for (int i = 0; i < n; i++) {
            w[i] = Math.exp(logw[i] - max);
            sum += w[i];
        }
        for (int i = 0; i < n; i++) {
            w[i] /= sum;
        }
        return w;
    }

    /** Most probable leaf: at each fork, follow the child subtree with more weight. */
    private int bestLeaf(double[] w) {
        // Total particle weight on each edge.
        double[] edgeW = new double[edges.length];
        for (int i = 0; i < n; i++) {
            edgeW[edge[i]] += w[i];
        }
        // Walk from the start of the road: at each fork take the branch whose subtree holds more weight.
        int e = 0;
        while (!edges[e].isLeaf()) {
            int best = -1;
            double bestW = 0;
            for (int c : edges[e].children()) {
                double m = 0;
                for (Edge d : edges) {
                    if (net.isAncestorOrSelf(c, d.id)) {
                        m += edgeW[d.id];
                    }
                }
                if (m > bestW) {
                    bestW = m;
                    best = c;
                }
            }
            // No particle beyond this fork yet: assume the planned route.
            e = best >= 0 ? best : ctx.route().next(e);
        }
        return e;
    }

    @Override
    public Estimate estimate() {
        if (!initialised) {
            return Estimate.invalid(t);
        }
        double[] w = weights();
        int leaf = bestLeaf(w);
        // Weighted mean position along the most probable path (particles on other branches are left
        // out), weighted mean speed, and the probability of being off the planned route.
        double sw = 0, mean = 0, meanV = 0, offRoute = 0;
        for (int i = 0; i < n; i++) {
            if (!ctx.route().contains(edge[i])) {
                offRoute += w[i];
            }
            if (net.isAncestorOrSelf(edge[i], leaf)) {
                sw += w[i];
                mean += w[i] * (edges[edge[i]].pathStart + s[i]);
            }
            meanV += w[i] * v[i];
        }
        mean /= sw;
        // Spread of the cloud = the estimate's uncertainty (standard deviation).
        double var = 0, varV = 0;
        for (int i = 0; i < n; i++) {
            if (net.isAncestorOrSelf(edge[i], leaf)) {
                double d = edges[edge[i]].pathStart + s[i] - mean;
                var += w[i] * d * d;
            }
            double dv = v[i] - meanV;
            varV += w[i] * dv * dv;
        }
        var /= sw;
        MapPosition p = net.positionOnPath(leaf, mean);

        // A thinned-out copy of the cloud (every n/150-th particle) for the map display.
        int m = snapshots ? Math.min(SNAPSHOT, n) : 0;
        int[] se = new int[m];
        float[] ss = new float[m], swt = new float[m];
        for (int k = 0; k < m; k++) {
            int i = (int) ((long) k * n / m);
            se[k] = edge[i];
            ss[k] = (float) s[i];
            swt[k] = (float) w[i];
        }
        return new Estimate(t, true, p.edge(), p.s(), leaf, mean, Math.sqrt(var), Math.max(0, meanV), Math.sqrt(varV),
                offRoute, gnss.mode(t), snapshots ? new ParticleSnapshot(se, ss, swt) : null);
    }

    @Override
    public double probRemainingBelow(double targetOffset, double d) {
        if (!initialised) {
            return 0;
        }
        // Share of the cloud (by weight) that is already within d metres of the target, or past it.
        double[] w = weights();
        double p = 0;
        int routeLeaf = ctx.route().leaf();
        for (int i = 0; i < n; i++) {
            if (!ctx.route().contains(edge[i])) {
                // Off the route: past the maneuver only if it left the route at (or after) the
                // maneuver's fork. A particle on an earlier, wrong exit never reaches it.
                if (net.forkBetween(edge[i], routeLeaf) >= targetOffset - 1) {
                    p += w[i];
                }
            } else if (targetOffset - (edges[edge[i]].pathStart + s[i]) <= d) {
                p += w[i];
            }
        }
        return p;
    }

    @Override
    public double probOnSubtree(int root) {
        if (!initialised) {
            return 0;
        }
        // Weight of particles inside the subtree starting at root, plus those not yet at that fork.
        double[] w = weights();
        double p = 0;
        for (int i = 0; i < n; i++) {
            if (net.isAncestorOrSelf(root, edge[i]) || net.isAncestorOrSelf(edge[i], root)) {
                p += w[i];
            }
        }
        return p;
    }

    @Override
    public double probOnPath(int leaf) {
        if (!initialised) {
            return 0;
        }
        // Weight of particles that lie somewhere on the path from the start to this leaf.
        double[] w = weights();
        double p = 0;
        for (int i = 0; i < n; i++) {
            if (net.isAncestorOrSelf(edge[i], leaf)) {
                p += w[i];
            }
        }
        return p;
    }

    @Override
    public double gradeAtEstimate() {
        if (!initialised) {
            return Double.NaN;
        }
        Estimate e = estimate();
        return edges[e.edge()].grade(e.s());
    }

    /** Mean accelerometer bias of the particle cloud (for diagnostics). */
    public double meanAccelBias() {
        double[] w = weights();
        double m = 0;
        for (int i = 0; i < n; i++) {
            m += w[i] * ba[i];
        }
        return m;
    }
}
