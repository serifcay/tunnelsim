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
 * which is why this is done by a small Kalman filter instead. The filter only runs while GNSS is
 * fresh: inside tunnels its map grade (taken at the last fix) would be wrong.
 */
final class BiasCalibrator {
    private static final double Q_V = 0.12 * 0.12;      // how much the speed may wander per second
    private static final double Q_B = 0.0005 * 0.0005;  // how much the bias may drift per second
    private static final double TAU = 60;               // averaging time of the gyro/lateral bias [s]
    private static final double FRESH = 2.0;            // GNSS older than this counts as lost [s]

    private final RoadNetwork net;
    // Kalman state: speed v and accelerometer bias ba, with their covariance (pvv, pvb, pbb).
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

    /** Prediction step: integrate the forward accelerometer into speed. */
    void onImu(VehicleImu imu) {
        double dt = Double.isNaN(lastT) ? 0 : imu.t() - lastT;
        lastT = imu.t();
        // After phone handling the calibrated bias is less certain.
        if (imu.mountDisturbed()) {
            pbb += 0.05 * 0.05;
        }
        if (!ready || dt <= 0 || imu.t() - lastFixT > FRESH) {
            return;
        }
        // Acceleration = measured forward force - bias - gravity along the slope, corrected for scale.
        double a = (imu.aFwd() - ba - MathUtil.G * grade / Math.sqrt(1 + grade * grade)) / (1 + imu.accelScale());
        v = v + a * dt;
        // F = [[1, -dt], [0, 1]]
        // Covariance prediction P = F P F' + Q, written out for the 2x2 case.
        double nvv = pvv - 2 * dt * pvb + dt * dt * pbb + Q_V * dt;
        double nvb = pvb - dt * pbb;
        pvv = nvv;
        pvb = nvb;
        pbb += Q_B * dt;
        // Stopped car: speed is known to be zero, which also pins down the bias.
        if (imu.stationary()) {
            update(-v, 0.05 * 0.05);
        }
        // Sums for the gyro and lateral bias estimates (averaged per GNSS interval).
        sumYaw += imu.yawRate();
        sumLat += imu.aLat();
        count++;
    }

    /** Correction step with a GNSS fix; also refreshes the map values (grade, curvature, elevation) there. */
    void onFix(GnssFix fix) {
        double[] near = net.nearest(fix.x(), fix.y());
        Edge e = net.edge((int) near[0]);
        grade = e.grade(near[1]);
        elevation = e.elevation(near[1]);
        double k = e.curvature(near[1]);
        if (!ready) {
            v = fix.speed();
            ready = true;
        } else if (fix.t() - lastFixT > FRESH) {
            // Back from an outage: restart the speed state, keep the calibration.
            v = fix.speed();
            pvv = 1;
            pvb = 0;
        } else {
            // Kalman update with the GNSS Doppler speed.
            double sv = Math.max(0.1, fix.speedAccuracy());
            update(fix.speed() - v, sv * sv);
            if (count > 0 && fix.t() - lastFixT < 1.5) {
                // Gyro bias = measured yaw rate - (speed x map curvature); lateral bias likewise with
                // speed^2 x curvature. Averaged slowly (gain g) over about TAU seconds.
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

    /** Scalar Kalman update of the speed state (H = [1, 0]); the bias is corrected through its correlation. */
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
        // Offset = barometric altitude - map elevation, smoothed slowly.
        double off = h - elevation;
        baroOffset = Double.isNaN(baroOffset) ? off : baroOffset + (off - baroOffset) * 0.05;
    }

    double baroOffset() {
        return baroOffset;
    }

    /** True once enough GNSS fixes have been used for the calibration to be meaningful. */
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
