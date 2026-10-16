package tunnelsim.estimator;

import tunnelsim.align.VehicleImu;
import tunnelsim.map.Edge;
import tunnelsim.map.RoadNetwork;
import tunnelsim.sensor.SensorEvent.GnssFix;
import tunnelsim.util.MathUtil;

/**
 * Calibrates IMU biases while GNSS is available, for the particle filter to inherit at tunnel entry.
 * <ul>
 *   <li>Forward accelerometer bias: a Kalman filter on [speed, bias], propagated with the forward
 *       accelerometer minus map-grade gravity and corrected by GNSS Doppler speed.</li>
 *   <li>Gyro yaw bias and lateral accelerometer bias: low-pass of measured minus map-predicted
 *       yaw rate (v*k) and lateral acceleration (v^2*k) at the GNSS-matched position.</li>
 *   <li>Barometer offset (weather, sensor offset, phone height): low-pass of barometric altitude
 *       minus map elevation at the GNSS-matched position.</li>
 * </ul>
 * Static parameters are a known weak spot of particle filters (resampling discards what was learned),
 * which is why this is done by a small Kalman filter instead.
 */
final class BiasCalibrator {
    private static final double Q_V = 0.12 * 0.12;
    private static final double Q_B = 0.0005 * 0.0005;
    private static final double TAU = 60;

    private final RoadNetwork net;
    private double v, ba;
    private double pvv = 1, pvb = 0, pbb = 0.2 * 0.2;
    private double bg, bl;
    private double grade, curvature, elevation = Double.NaN;
    private double baroOffset = Double.NaN;
    private boolean ready;
    private double lastT = Double.NaN;
    private double sumYaw, sumLat;
    private int count;
    private double lastFixT = Double.NaN;
    private int updates;

    BiasCalibrator(RoadNetwork net) {
        this.net = net;
    }

    void onImu(VehicleImu imu) {
        double dt = Double.isNaN(lastT) ? 0 : imu.t() - lastT;
        lastT = imu.t();
        if (!ready || dt <= 0) {
            return;
        }
        double a = imu.aFwd() - ba - MathUtil.G * grade / Math.sqrt(1 + grade * grade);
        v = v + a * dt;
        // F = [[1, -dt], [0, 1]]
        double nvv = pvv - 2 * dt * pvb + dt * dt * pbb + Q_V * dt;
        double nvb = pvb - dt * pbb;
        pvv = nvv;
        pvb = nvb;
        pbb += Q_B * dt;
        if (imu.stationary()) {
            update(-v, 0.05 * 0.05);
        }
        sumYaw += imu.yawRate();
        sumLat += imu.aLat();
        count++;
    }

    void onFix(GnssFix fix) {
        double[] near = net.nearest(fix.x(), fix.y());
        Edge e = net.edge((int) near[0]);
        grade = e.grade(near[1]);
        elevation = e.elevation(near[1]);
        double k = e.curvature(near[1]);
        if (!ready) {
            v = fix.speed();
            ready = true;
        } else {
            double sv = Math.max(0.1, fix.speedAccuracy());
            update(fix.speed() - v, sv * sv);
            if (count > 0 && fix.t() - lastFixT < 1.5) {
                double g = Math.min(1, (fix.t() - lastFixT) / TAU);
                double vm = fix.speed();
                bg += (sumYaw / count - vm * 0.5 * (k + curvature) - bg) * g;
                bl += (sumLat / count - vm * vm * 0.5 * (k + curvature) - bl) * g;
            }
            updates++;
        }
        curvature = k;
        lastFixT = fix.t();
        sumYaw = sumLat = 0;
        count = 0;
    }

    private void update(double innovation, double r) {
        double s = pvv + r;
        double kv = pvv / s, kb = pvb / s;
        v += kv * innovation;
        ba += kb * innovation;
        double nvv = pvv - kv * pvv, nvb = pvb - kv * pvb, nbb = pbb - kb * pvb;
        pvv = nvv;
        pvb = nvb;
        pbb = nbb;
    }

    /** Barometric altitude averaged over ~0.5 s, while GNSS is available. */
    void onBaroAltitude(double h, boolean gnssFresh) {
        if (!gnssFresh || Double.isNaN(elevation)) {
            return;
        }
        double off = h - elevation;
        baroOffset = Double.isNaN(baroOffset) ? off : baroOffset + (off - baroOffset) * 0.05;
    }

    double baroOffset() {
        return baroOffset;
    }

    boolean ready() {
        return ready && updates >= 10;
    }

    double accelBias() {
        return ba;
    }

    double accelBiasSigma() {
        return Math.sqrt(Math.max(0, pbb));
    }

    double gyroBias() {
        return bg;
    }

    double lateralBias() {
        return bl;
    }
}
