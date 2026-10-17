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

    private int[] edge;
    private double[] s, v, ba, ks, ko, accel, logw;
    private final BiasCalibrator calibrator;
    private double[] predYaw, predLat;
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
        switch (event) {
            case GnssFix fix -> onGnss(fix);
            case BaroSample b -> onBaro(b);
            case LightSample l -> {
                if (portals.onLight(l) && initialised && aid.portal) {
                    portalUpdate();
                }
            }
            case CellChange c -> {
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
                if (initialised && ctx.useObd() && gnssSpeedSum > 200 && !gnss.fresh(t)) {
                    // Calibrated speedometer scale; its residual error (ko, per particle) is
                    // systematic, so it is carried as a nuisance state rather than treated as noise.
                    double scale = obdSum / gnssSpeedSum;
                    double sigma = 0.25;
                    double latency = ctx.scenario().sensors.obd.latency; // assumed calibrated per car
                    for (int i = 0; i < n; i++) {
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
        if (!aid.gnss || !gnss.accept(fix)) {
            return;
        }
        calibrator.onFix(fix);
        if (!Double.isNaN(lastObd) && fix.speed() > 5) {
            obdSum += lastObd;
            gnssSpeedSum += fix.speed();
        }
        double sigma = Math.max(1.5, fix.accuracy() / 1.5);
        if (!initialised) {
            initialise(fix, sigma);
            return;
        }
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

    private void initialise(GnssFix fix, double sigma) {
        double[] near = net.nearest(fix.x(), fix.y());
        for (int i = 0; i < n; i++) {
            edge[i] = (int) near[0];
            s[i] = MathUtil.clamp(near[1] + rng.gaussian(sigma), 0, edges[edge[i]].length);
            v[i] = Math.max(0, fix.speed() + rng.gaussian(0.3));
            ba[i] = rng.gaussian(0.2);
            ks[i] = 1 + rng.gaussian(pf.gyroScaleSigma);
            ko[i] = rng.gaussian(0.005);
            logw[i] = -Math.log(n);
        }
        initialised = true;
    }

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
        calibrator.onBaroAltitude(h, fresh);
        double offset = calibrator.baroOffset();
        if (fresh || Double.isNaN(offset)) {
            return; // GNSS pins the position better; the barometer is used inside tunnels
        }
        double nu = 4;
        // Down-weight for correlated errors: n updates per correlation time carry about one update's information.
        double temper = 1.0 / Math.max(1, pf.baroCorrelationTime / pf.baroInterval);
        for (int i = 0; i < n; i++) {
            double e = (h - offset - edges[edge[i]].elevation(s[i])) / pf.sigmaBaro;
            logw[i] += -temper * (nu + 1) / 2 * Math.log1p(e * e / nu);
        }
        normaliseAndResample();
    }

    private void portalUpdate() {
        double nu = 3;
        for (int i = 0; i < n; i++) {
            double best = Double.MAX_VALUE;
            double px = x(i), py = y(i);
            for (Portal p : net.portals()) {
                best = Math.min(best, Math.hypot(px - p.x(), py - p.y()));
            }
            double sigma = 8 + 0.5 * v[i];
            double e = best / sigma;
            logw[i] += -(nu + 1) / 2 * Math.log1p(e * e / nu);
        }
        normaliseAndResample();
    }

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

    private void bleUpdate(BleScan scan) {
        SensorsConfig.BleConfig bc = ctx.scenario().sensors.ble;
        double nu = 4, sigma = 6, dz = bc.height - 1.2;
        for (int k = 0; k < scan.beaconIds().length; k++) {
            Beacon b = beaconsById.get(scan.beaconIds()[k]);
            if (b == null) {
                continue;
            }
            for (int i = 0; i < n; i++) {
                double dx = x(i) - b.mx(), dy = y(i) - b.my();
                double d = Math.max(0.5, Math.sqrt(dx * dx + dy * dy + dz * dz));
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
            // The phone moved: small residual misalignment changes the gravity leak into the
            // forward axis, so the calibrated bias no longer holds. Widen it and let the map decide.
            for (int i = 0; i < n; i++) {
                ba[i] += rng.gaussian(0.1);
                ks[i] = 1 + rng.gaussian(pf.gyroScaleSigma);
            }
        }
        if (!initialised) {
            lastBlockT = imu.t();
            return;
        }
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

        if (winBlocks == window) {
            double my = winYaw / window - calibrator.gyroBias(), ml = winLat / window - calibrator.lateralBias();
            for (int i = 0; i < n; i++) {
                double py = ks[i] * predYaw[i] / window, pl = predLat[i] / window;
                if (aid.curvature) {
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

        if (aid.speedPrior && !gnss.fresh(t) && !stationary && t - lastPriorT >= 1) {
            lastPriorT = t;
            for (int i = 0; i < n; i++) {
                double expected = net.expectedSpeed(new MapPosition(edge[i], s[i]));
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

    private void propagate(double dt, double aF) {
        double sqdt = Math.sqrt(dt);
        double pFollow = cfg.pFollow;
        double scale = 1 + lastAccelScale;
        for (int i = 0; i < n; i++) {
            Edge e = edges[edge[i]];
            double g = e.grade(s[i]);
            double a = (aF - ba[i] - MathUtil.G * g / Math.sqrt(1 + g * g)) / scale;
            // Speed may go slightly negative in the state: clamping at zero would make particles
            // with a too-high accelerometer bias look perfectly stopped, and stop updates would
            // then select a biased bias. Position only ever moves forward.
            double vn = Math.max(-2, v[i] + a * dt + rng.gaussian(pf.sigmaV * sqdt));
            s[i] += Math.max(0, 0.5 * (v[i] + vn)) * dt;
            v[i] = vn;
            accel[i] = a;
            ba[i] += rng.gaussian(pf.sigmaBias * sqdt);
            while (s[i] > e.length) {
                if (e.isLeaf()) {
                    s[i] = e.length;
                    v[i] = 0;
                    break;
                }
                double rest = s[i] - e.length;
                int routeChild = ctx.route().next(e.id);
                int other = e.children().get(0) == routeChild ? e.children().get(1) : e.children().get(0);
                edge[i] = rng.chance(pFollow) ? routeChild : other;
                e = edges[edge[i]];
                s[i] = rest;
            }
            double k = e.curvature(s[i]);
            predYaw[i] += v[i] * k;
            predLat[i] += v[i] * v[i] * k;
        }
    }

    // ---------------------------------------------------------------------------------------
    // Weights and resampling
    // ---------------------------------------------------------------------------------------

    private void normaliseAndResample() {
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
        if (1.0 / sumSq < n / 2.0) {
            resample(w);
        }
    }

    private void resample(double[] w) {
        int[] ne = new int[n];
        double[] ns = new double[n], nv = new double[n], nba = new double[n], nks = new double[n], nko = new double[n];
        double[] nacc = new double[n];
        double[] npy = new double[n], npl = new double[n];
        double step = 1.0 / n;
        double u = rng.uniform() * step;
        double c = w[0];
        int j = 0;
        for (int i = 0; i < n; i++) {
            double target = u + i * step;
            while (target > c && j < n - 1) {
                c += w[++j];
            }
            ne[i] = edge[j];
            ns[i] = MathUtil.clamp(s[j] + rng.gaussian(0.3), 0, edges[edge[j]].length);
            nv[i] = v[j] + rng.gaussian(0.03);
            nba[i] = ba[j] + rng.gaussian(0.001);
            nks[i] = ks[j] + rng.gaussian(0.0005);
            nko[i] = ko[j] + rng.gaussian(0.0002);
            nacc[i] = accel[j];
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
        // Curvature window accumulators travel with the particles.
        System.arraycopy(npy, 0, predYaw, 0, n);
        System.arraycopy(npl, 0, predLat, 0, n);
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
            if (t - fork.getValue() > cfg.branchGuardTime) {
                continue;
            }
            Edge parent = edges[fork.getKey()];
            int c0 = parent.children().get(0), c1 = parent.children().get(1);
            int n0 = 0, n1 = 0;
            for (int i = 0; i < n; i++) {
                if (edge[i] == c0) {
                    n0++;
                } else if (edge[i] == c1) {
                    n1++;
                }
            }
            int min = (int) Math.ceil(cfg.branchMinShare * n);
            if (n0 + n1 < 0.3 * n) {
                continue;
            }
            if (n0 < min) {
                move(c1, c0, min - n0);
            } else if (n1 < min) {
                move(c0, c1, min - n1);
            }
        }
    }

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

    private double x(int i) {
        return edges[edge[i]].x(s[i]);
    }

    private double y(int i) {
        return edges[edge[i]].y(s[i]);
    }

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
        double[] edgeW = new double[edges.length];
        for (int i = 0; i < n; i++) {
            edgeW[edge[i]] += w[i];
        }
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
        double[] w = weights();
        double p = 0;
        for (int i = 0; i < n; i++) {
            if (!ctx.route().contains(edge[i])) {
                p += w[i];
            } else if (targetOffset - (edges[edge[i]].pathStart + s[i]) <= d) {
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
        double[] w = weights();
        double p = 0;
        for (int i = 0; i < n; i++) {
            if (net.isAncestorOrSelf(edge[i], leaf)) {
                p += w[i];
            }
        }
        return p;
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
