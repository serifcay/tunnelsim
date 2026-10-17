package tunnelsim.estimator;

import tunnelsim.align.VehicleImu;
import tunnelsim.map.Features.CellZone;
import tunnelsim.map.Features.Portal;
import tunnelsim.map.MapPosition;
import tunnelsim.sensor.SensorEvent;
import tunnelsim.sensor.SensorEvent.CellChange;
import tunnelsim.sensor.SensorEvent.GnssFix;
import tunnelsim.sensor.SensorEvent.LightSample;
import tunnelsim.util.MathUtil;

/**
 * Approach C: extended Kalman filter on the planned route with state [offset, speed, accel bias].
 * Propagates with the forward accelerometer minus the gravity component of the map grade;
 * corrected by GNSS position and speed, zero-velocity updates, a weak traffic/limit speed prior,
 * light-based portal detection and cell handovers. It follows the route at forks and does no
 * curvature matching, which is strongly non-linear and multi-modal; that is what D adds.
 */
public final class MapAidedEkf extends PathEstimator {
    private static final double Q_OFFSET = 0.05 * 0.05;
    private static final double Q_SPEED = 0.12 * 0.12;
    private static final double Q_BIAS = 0.002 * 0.002;
    private static final double GATE = 25;      // NIS gate (5 sigma)
    private static final double PRIOR_DECORRELATION = 30; // s

    private final double[] x = new double[3];
    private final double[][] p = new double[3][3];
    private final PortalDetector portals = new PortalDetector();
    private int rejected;
    private double lastImuT = Double.NaN;
    private double lastPriorT = Double.NEGATIVE_INFINITY;

    public MapAidedEkf(EstimatorContext ctx) {
        super(ctx);
    }

    @Override
    public String id() {
        return "C";
    }

    @Override
    public void onSensor(SensorEvent event) {
        t = event.t();
        switch (event) {
            case GnssFix fix -> {
                if (!ctx.aiding().gnss || !gnss.accept(fix)) {
                    return;
                }
                double[] m = matchFix(fix);
                if (m == null) {
                    return;
                }
                double inflate = gnss.fixesSinceOutage() <= 5 ? 3 : 1;
                double r = Math.pow(fix.accuracy() / 1.5 * inflate, 2) + m[1] * m[1] * 0.1;
                if (!initialised) {
                    x[0] = m[0];
                    x[1] = fix.speed();
                    x[2] = 0;
                    p[0][0] = r;
                    p[1][1] = 0.5 * 0.5;
                    p[2][2] = 0.2 * 0.2;
                    initialised = true;
                    return;
                }
                if (!update(new double[]{1, 0, 0}, m[0] - x[0], r, true)) {
                    if (++rejected >= 3) {
                        // Consistent rejections mean the filter, not the fixes, is wrong: re-anchor.
                        x[0] = m[0];
                        p[0][0] = r;
                        p[0][1] = p[1][0] = p[0][2] = p[2][0] = 0;
                        rejected = 0;
                    }
                } else {
                    rejected = 0;
                }
                double sv = Math.max(0.1, fix.speedAccuracy()) * inflate;
                update(new double[]{0, 1, 0}, fix.speed() - x[1], sv * sv, true);
            }
            case LightSample light -> {
                if (portals.onLight(light) && initialised && ctx.aiding().portal) {
                    double portalOffset = nearestPortalOffset();
                    if (!Double.isNaN(portalOffset)) {
                        double sigma = 8 + 0.5 * x[1];
                        update(new double[]{1, 0, 0}, portalOffset - x[0], sigma * sigma, true);
                    }
                }
            }
            case CellChange cell -> {
                if (initialised && ctx.aiding().cell) {
                    for (CellZone z : net.cells()) {
                        if (z.cellId() == cell.cellId() && ctx.net().isAncestorOrSelf(z.position().edge(), leaf)) {
                            update(new double[]{1, 0, 0}, net.pathOffset(z.position()) - x[0], 100 * 100, true);
                        }
                    }
                }
            }
            default -> {
            }
        }
    }

    @Override
    public void onVehicleImu(VehicleImu imu) {
        double dt = Double.isNaN(lastImuT) ? 0 : imu.t() - lastImuT;
        lastImuT = imu.t();
        t = imu.t();
        if (!initialised || dt <= 0) {
            return;
        }
        if (imu.mountDisturbed()) {
            p[2][2] += 0.1 * 0.1; // phone moved: the calibrated bias no longer holds
        }
        MapPosition pos = position(x[0]);
        double g = net.edge(pos.edge()).grade(pos.s());
        double a = (imu.aFwd() - x[2] - MathUtil.G * g / Math.sqrt(1 + g * g)) / (1 + imu.accelScale());
        // No clamp at zero speed here (see RoadParticleFilter.propagate); position only moves forward.
        double v = Math.max(-2, x[1] + a * dt);
        x[0] = clampOffset(x[0] + Math.max(0, 0.5 * (x[1] + v)) * dt);
        x[1] = v;
        // P = F P F' + Q, F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
        double[][] f = {{1, dt, -0.5 * dt * dt}, {0, 1, -dt}, {0, 0, 1}};
        double[][] fp = mul(f, p);
        double[][] n = mul(fp, transpose(f));
        n[0][0] += Q_OFFSET * dt;
        n[1][1] += Q_SPEED * dt;
        n[2][2] += Q_BIAS * dt;
        copy(n, p);

        if (imu.stationary() && ctx.aiding().zupt) {
            update(new double[]{0, 1, 0}, -x[1], 0.05 * 0.05, false);
        }
        if (!gnss.fresh(t) && ctx.aiding().speedPrior && !imu.stationary() && t - lastPriorT >= 1) {
            lastPriorT = t;
            double expected = net.expectedSpeed(pos);
            double sigma = (4 + 0.2 * expected) * Math.sqrt(PRIOR_DECORRELATION);
            update(new double[]{0, 1, 0}, expected - x[1], sigma * sigma, false);
        }
    }

    private double nearestPortalOffset() {
        double best = Double.NaN, bestD = Double.MAX_VALUE;
        for (Portal portal : net.portals()) {
            if (!net.onCommonPath(portal.position().edge(), leaf)) {
                continue;
            }
            double off = net.pathOffset(portal.position());
            double d = Math.abs(off - x[0]);
            if (d < bestD) {
                bestD = d;
                best = off;
            }
        }
        return bestD < 1000 ? best : Double.NaN;
    }

    /** Scalar Kalman update; returns false if gated out. */
    private boolean update(double[] h, double innovation, double r, boolean gate) {
        double[] ph = new double[3];
        for (int i = 0; i < 3; i++) {
            ph[i] = p[i][0] * h[0] + p[i][1] * h[1] + p[i][2] * h[2];
        }
        double s = h[0] * ph[0] + h[1] * ph[1] + h[2] * ph[2] + r;
        if (gate && innovation * innovation / s > GATE) {
            return false;
        }
        double[] k = {ph[0] / s, ph[1] / s, ph[2] / s};
        for (int i = 0; i < 3; i++) {
            x[i] += k[i] * innovation;
        }
        double[][] n = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                n[i][j] = p[i][j] - k[i] * ph[j];
            }
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                p[i][j] = 0.5 * (n[i][j] + n[j][i]);
            }
        }
        x[1] = Math.max(-2, x[1]);
        x[0] = clampOffset(x[0]);
        return true;
    }

    @Override
    public Estimate estimate() {
        return build(x[0], Math.sqrt(Math.max(0, p[0][0])), Math.max(0, x[1]), Math.sqrt(Math.max(0, p[1][1])));
    }

    @Override
    public double probRemainingBelow(double targetOffset, double d) {
        return gaussRemaining(x[0], Math.sqrt(Math.max(0, p[0][0])), targetOffset, d);
    }

    public double accelBias() {
        return x[2];
    }

    private static double[][] mul(double[][] a, double[][] b) {
        double[][] r = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
        }
        return r;
    }

    private static double[][] transpose(double[][] a) {
        double[][] r = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[i][j] = a[j][i];
            }
        }
        return r;
    }

    private static void copy(double[][] src, double[][] dst) {
        for (int i = 0; i < 3; i++) {
            System.arraycopy(src[i], 0, dst[i], 0, 3);
        }
    }
}
