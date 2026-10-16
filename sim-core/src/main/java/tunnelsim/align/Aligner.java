package tunnelsim.align;

import tunnelsim.sensor.SensorEvent.GnssFix;
import tunnelsim.sensor.SensorEvent.ImuSample;
import tunnelsim.util.Rot;

import java.util.function.DoubleFunction;

/**
 * Estimates how the phone sits in the car and rotates IMU samples into the car frame.
 * <ul>
 *   <li><b>Up</b>: the specific force when the car is stopped (or cruising steadily with GNSS).</li>
 *   <li><b>Forward</b>: regression of horizontal acceleration on GNSS speed changes. The intercept
 *       absorbs accelerometer bias; lateral (curve) acceleration is uncorrelated with speed changes.</li>
 *   <li><b>Handling</b>: when the gyro shows a rotation far faster than a car can turn, the phone is
 *       moving in its mount; the estimated axes are rotated along with the gyro.</li>
 * </ul>
 * In TRUTH mode the true mount is used, to separate alignment errors from estimator errors.
 */
public final class Aligner {
    private static final double SPIKE_RATE = 0.35; // rad/s, well above any car yaw rate

    private final DoubleFunction<double[]> trueMount;
    private final StationaryDetector detector = new StationaryDetector();

    // Car axes expressed in phone coordinates.
    private double[] fwd = {0, 0, -1};
    private double[] left = {-1, 0, 0};
    private double[] up = {0, 1, 0};
    private boolean upInitialised;
    private boolean forwardEstimated;

    // Low-pass specific force (phone frame) and accumulation between GNSS fixes.
    private final double[] lowPass = new double[3];
    private boolean lowPassInit;
    private final double[] intervalSum = new double[3];
    private double intervalYaw;
    private int intervalCount;
    private GnssFix prevFix;
    private double lastImuT = Double.NaN;

    // Exponentially forgetting regression sums: x = dv/dt, h = horizontal specific force.
    private double sw, sx, sxx;
    private final double[] sh = new double[3];
    private final double[] sxh = new double[3];

    public Aligner(DoubleFunction<double[]> trueMount) {
        this.trueMount = trueMount;
    }

    public boolean forwardEstimated() {
        return trueMount != null || forwardEstimated;
    }

    public VehicleImu onImu(ImuSample s) {
        double dt = Double.isNaN(lastImuT) ? 0.01 : Math.max(1e-4, s.t() - lastImuT);
        lastImuT = s.t();
        double[] f = {s.ax(), s.ay(), s.az()};
        double[] w = {s.gx(), s.gy(), s.gz()};
        boolean stationary = detector.update(f[0], f[1], f[2], w[0], w[1], w[2]);

        if (trueMount != null) {
            double[] m = trueMount.apply(s.t());
            fwd = new double[]{m[0], m[3], m[6]};
            left = new double[]{m[1], m[4], m[7]};
            up = new double[]{m[2], m[5], m[8]};
        } else {
            double rate = Rot.norm(w);
            if (rate > SPIKE_RATE) {
                rotateAll(w, dt);
            }
            double tau = stationary ? 0.5 : 2.0;
            if (!lowPassInit) {
                System.arraycopy(f, 0, lowPass, 0, 3);
                lowPassInit = true;
            } else {
                for (int k = 0; k < 3; k++) {
                    lowPass[k] += (f[k] - lowPass[k]) * Math.min(1, dt / tau);
                }
            }
            if (stationary && rate < 0.03) {
                updateUp(lowPass, upInitialised ? 0.02 : 1.0);
            }
            for (int k = 0; k < 3; k++) {
                intervalSum[k] += f[k];
            }
            intervalYaw += Rot.dot(w, up);
            intervalCount++;
        }

        return new VehicleImu(s.t(), Rot.dot(f, fwd), Rot.dot(f, left), Rot.dot(f, up),
                Rot.dot(w, fwd), Rot.dot(w, left), Rot.dot(w, up), stationary);
    }

    public void onGnss(GnssFix fix) {
        if (trueMount != null) {
            return;
        }
        // Only good fixes: near tunnel portals the Doppler speed is noisy, and on a curve that noise
        // multiplies the lateral acceleration and rotates the forward estimate.
        boolean good = fix.accuracy() < 8 && fix.speedAccuracy() < 0.3
                && (prevFix == null || prevFix.accuracy() < 8 && prevFix.speedAccuracy() < 0.3);
        if (prevFix != null && good && intervalCount > 0 && fix.t() - prevFix.t() < 1.5) {
            double dtFix = fix.t() - prevFix.t();
            double dvdt = (fix.speed() - prevFix.speed()) / dtFix;
            double[] mean = Rot.scale(intervalSum, 1.0 / intervalCount);
            double[] h = Rot.sub(mean, Rot.scale(up, Rot.dot(mean, up)));
            // Quasi-static cruising refines "up" slowly.
            if (Math.abs(dvdt) < 0.1 && fix.speed() > 5 && Math.abs(Rot.dot(h, left)) < 0.2) {
                updateUp(mean, 0.05);
            }
            // Straight road only: in curves the lateral acceleration times random speed changes
            // leaks into the regression.
            boolean straight = Math.abs(intervalYaw / intervalCount) < 0.01;
            if (straight && (fix.speed() > 1 || prevFix.speed() > 1)) {
                double forget = Math.exp(-dtFix / 300.0);
                sw = sw * forget + 1;
                sx = sx * forget + dvdt;
                sxx = sxx * forget + dvdt * dvdt;
                for (int k = 0; k < 3; k++) {
                    sh[k] = sh[k] * forget + h[k];
                    sxh[k] = sxh[k] * forget + dvdt * h[k];
                }
                double varX = sxx - sx * sx / sw;
                if (varX > 8) {
                    double[] cov = new double[3];
                    for (int k = 0; k < 3; k++) {
                        cov[k] = sxh[k] - sx * sh[k] / sw;
                    }
                    double[] f = Rot.sub(cov, Rot.scale(up, Rot.dot(cov, up)));
                    if (Rot.norm(f) > 1e-6) {
                        fwd = Rot.normalize(f);
                        left = Rot.cross(up, fwd);
                        forwardEstimated = true;
                    }
                }
            }
        }
        prevFix = fix;
        intervalSum[0] = intervalSum[1] = intervalSum[2] = 0;
        intervalYaw = 0;
        intervalCount = 0;
    }

    /** Blends the up axis towards the direction of a (quasi-static) specific force. */
    private void updateUp(double[] f, double gain) {
        double[] u = Rot.normalize(f);
        double[] blended = Rot.normalize(new double[]{
            up[0] + gain * (u[0] - up[0]), up[1] + gain * (u[1] - up[1]), up[2] + gain * (u[2] - up[2])});
        up = blended;
        upInitialised = true;
        fwd = Rot.normalize(Rot.sub(fwd, Rot.scale(up, Rot.dot(fwd, up))));
        left = Rot.cross(up, fwd);
    }

    /** The phone rotated by w*dt relative to the car: car axes appear to rotate the other way. */
    private void rotateAll(double[] w, double dt) {
        double angle = Rot.norm(w) * dt;
        double[] r = Rot.axisAngle(w, -angle);
        fwd = Rot.apply(r, fwd);
        up = Rot.normalize(Rot.apply(r, up));
        fwd = Rot.normalize(Rot.sub(fwd, Rot.scale(up, Rot.dot(fwd, up))));
        left = Rot.cross(up, fwd);
        double[] a = Rot.apply(r, sh), b = Rot.apply(r, sxh), c = Rot.apply(r, lowPass);
        System.arraycopy(a, 0, sh, 0, 3);
        System.arraycopy(b, 0, sxh, 0, 3);
        System.arraycopy(c, 0, lowPass, 0, 3);
    }

    /** Current car-to-phone rotation estimate (columns are car axes in phone coordinates). */
    public double[] mount() {
        return new double[]{fwd[0], left[0], up[0], fwd[1], left[1], up[1], fwd[2], left[2], up[2]};
    }
}
