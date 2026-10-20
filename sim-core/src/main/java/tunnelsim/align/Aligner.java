package tunnelsim.align;

import tunnelsim.sensor.SensorEvent.GnssFix;
import tunnelsim.sensor.SensorEvent.ImuSample;
import tunnelsim.util.Rot;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

/**
 * Estimates how the phone sits in the car and rotates IMU samples into the car frame.
 * <ul>
 *   <li><b>Up</b>: the specific force when the car is stopped (or, if it never stops, cruising steadily
 *       with GNSS), estimated once and then <b>frozen</b>. The phone does not move relative to the car
 *       unless it is handled, so later re-estimates only add noise: deceleration still in the
 *       low-pass at the moment of a stop, and the slope of the road where the car stopped. A slope
 *       absorbed into "up" is then subtracted again from the map by the estimators (counted twice);
 *       a constant tilt, in contrast, is absorbed by the accelerometer-bias calibration.</li>
 *   <li><b>Forward</b>: regression of horizontal acceleration on GNSS speed changes. The intercept
 *       absorbs accelerometer bias; lateral (curve) acceleration is uncorrelated with speed changes.</li>
 *   <li><b>Effective forward scale</b>: the regression slope magnitude is (1 + scale), where scale
 *       combines the accelerometer's scale error with the car's pitch under acceleration (squat and
 *       dive leak ~3% of the acceleration as gravity into the forward axis). It is computed in the
 *       phone frame, so it does not depend on the forward estimate still converging, and corrected
 *       for GNSS speed noise in the regressor, which would otherwise bias it low. With little
 *       acceleration to learn from the slope is noisy, so it is shrunk towards 0 by its own
 *       statistical uncertainty (prior: 0 +- 3%).</li>
 *   <li><b>Handling</b>: when the gyro shows a rotation far faster than a car can turn, the phone is
 *       moving in its mount; the estimated axes are rotated along with the gyro. At the next stop
 *       "up" is re-estimated from gravity <b>with the road slope removed</b>: the slope comes from the
 *       map at the best estimator's position (a stop on a slope must not be absorbed into "up").
 *       Without a slope source the gyro-tracked alignment is kept.</li>
 * </ul>
 * In TRUTH mode the true mount is used, to separate alignment errors from estimator errors.
 */
public final class Aligner {
    private static final double SPIKE_RATE = 0.35; // rad/s, well above any car yaw rate

    private final DoubleFunction<double[]> trueMount;
    private final StationaryDetector detector = new StationaryDetector();

    // Car axes expressed in phone coordinates.
    // Starting guess: a portrait phone facing the driver (phone z points backwards, x to the right).
    private double[] fwd = {0, 0, -1};
    private double[] left = {-1, 0, 0};
    private double[] up = {0, 1, 0};
    private boolean upInitialised;
    private boolean upLocked;
    private int upSamples;
    private boolean forwardEstimated;
    private static final int UP_STATIONARY_SAMPLES = 300; // 3 s at 100 Hz
    private static final int UP_QUASI_STATIC_FIXES = 15;

    // Low-pass specific force (phone frame) and accumulation between GNSS fixes.
    private final double[] lowPass = new double[3];
    private boolean lowPassInit;
    private final double[] intervalSum = new double[3];
    private double intervalYaw;
    private int intervalCount;
    private GnssFix prevFix;
    private double lastImuT = Double.NaN;
    private boolean inSpike;

    // Exponentially forgetting regression sums: x = dv/dt, h = horizontal specific force.
    private static final double SCALE_PRIOR_SIGMA = 0.03;
    private double sw, sx, sxx, snn, shh;
    private double scale;
    private boolean scaleEstimated;
    private final double[] sh = new double[3];
    private final double[] sxh = new double[3];

    private final boolean useScale;
    private boolean slopeAware = true;
    private DoubleSupplier gradeProvider;
    private boolean handled;

    public Aligner(DoubleFunction<double[]> trueMount) {
        this(trueMount, true);
    }

    /** Road grade at the car's estimated position, used to re-align at stops after handling. */
    public void setGradeProvider(DoubleSupplier provider, boolean slopeAware) {
        this.gradeProvider = provider;
        this.slopeAware = slopeAware;
    }

    /** @param useScale report the learned forward scale to the estimators (false: always 0) */
    public Aligner(DoubleFunction<double[]> trueMount, boolean useScale) {
        this.trueMount = trueMount;
        this.useScale = useScale;
    }

    public boolean forwardEstimated() {
        return trueMount != null || forwardEstimated;
    }

    /** Takes one raw IMU sample (phone axes) and returns it expressed in car axes. */
    public VehicleImu onImu(ImuSample s) {
        double dt = Double.isNaN(lastImuT) ? 0.01 : Math.max(1e-4, s.t() - lastImuT);
        lastImuT = s.t();
        double[] f = {s.ax(), s.ay(), s.az()};   // specific force (acceleration + gravity reaction)
        double[] w = {s.gx(), s.gy(), s.gz()};   // angular rate
        boolean stationary = detector.update(f[0], f[1], f[2], w[0], w[1], w[2]);

        boolean disturbed = false;
        if (trueMount != null) {
            // TRUTH mode: take the car axes straight from the simulator's true phone orientation.
            double[] m = trueMount.apply(s.t());
            fwd = new double[]{m[0], m[3], m[6]};
            left = new double[]{m[1], m[4], m[7]};
            up = new double[]{m[2], m[5], m[8]};
        } else {
            double rate = Rot.norm(w);
            // A rotation faster than any car can turn means the phone itself is moving in its holder:
            // rotate our estimate of the car axes along with it.
            if (rate > SPIKE_RATE) {
                rotateAll(w, dt);
                inSpike = true;
            } else if (inSpike) {
                inSpike = false;
                disturbed = true;
                // The phone was handled: allow "up" to be re-estimated at the next opportunity.
                upLocked = false;
                upSamples = 0;
                handled = true;
            }
            // Low-pass filter of the specific force: faster while stopped (it is then pure gravity).
            double tau = stationary ? 0.5 : 2.0;
            if (!lowPassInit) {
                System.arraycopy(f, 0, lowPass, 0, 3);
                lowPassInit = true;
            } else {
                for (int k = 0; k < 3; k++) {
                    lowPass[k] += (f[k] - lowPass[k]) * Math.min(1, dt / tau);
                }
            }
            // While stopped and not rotating, gravity points straight "up" in the car: learn it,
            // unless "up" is already locked.
            if (!upLocked && stationary && rate < 0.03) {
                double[] gravity = lowPass;
                boolean use = true;
                if (handled && slopeAware) {
                    double grade = gradeProvider == null ? Double.NaN : gradeProvider.getAsDouble();
                    if (Double.isNaN(grade)) {
                        use = false; // no slope source: keep the gyro-tracked alignment
                    } else {
                        // At rest on a slope the car body is pitched: gravity has a component g*sin(slope)
                        // along the car's forward axis. Remove it before taking the direction as "up".
                        double gs = Rot.norm(lowPass) * grade / Math.sqrt(1 + grade * grade);
                        gravity = Rot.sub(lowPass, Rot.scale(fwd, gs));
                    }
                }
                if (use) {
                    // First sample sets "up" directly; later ones blend in slowly.
                    updateUp(gravity, upInitialised ? 0.02 : 1.0);
                    if (++upSamples >= UP_STATIONARY_SAMPLES) {
                        upLocked = true;
                    }
                }
            }
            // Sums over the interval between two GNSS fixes, for the forward-axis regression.
            for (int k = 0; k < 3; k++) {
                intervalSum[k] += f[k];
            }
            intervalYaw += Rot.dot(w, up);
            intervalCount++;
        }

        // Project the sample onto the car axes: forward, left and up components.
        return new VehicleImu(s.t(), Rot.dot(f, fwd), Rot.dot(f, left), Rot.dot(f, up),
                Rot.dot(w, fwd), Rot.dot(w, left), Rot.dot(w, up), stationary, disturbed,
                accelScale());
    }

    /**
     * Uses each pair of consecutive GNSS fixes to learn which way is "forward": the horizontal part of
     * the measured acceleration that goes up and down together with the GNSS speed changes.
     */
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
            // Speed change according to GNSS, and the mean measured specific force over the same interval.
            double dvdt = (fix.speed() - prevFix.speed()) / dtFix;
            double[] mean = Rot.scale(intervalSum, 1.0 / intervalCount);
            // Horizontal part of it (the component along "up" removed).
            double[] h = Rot.sub(mean, Rot.scale(up, Rot.dot(mean, up)));
            // Fallback when the car has not stopped yet: quasi-static cruising.
            if (!upLocked && Math.abs(dvdt) < 0.1 && fix.speed() > 5 && Math.abs(Rot.dot(h, left)) < 0.2) {
                updateUp(mean, upInitialised ? 0.1 : 1.0);
                upSamples += UP_STATIONARY_SAMPLES / UP_QUASI_STATIC_FIXES;
                if (upSamples >= UP_STATIONARY_SAMPLES) {
                    upLocked = true;
                }
            }
            // Straight road only: in curves the lateral acceleration times random speed changes
            // leaks into the regression.
            boolean straight = Math.abs(intervalYaw / intervalCount) < 0.01;
            if (straight && (fix.speed() > 1 || prevFix.speed() > 1)) {
                // Running sums for a linear regression of h on dv/dt. Old data fades out over ~5
                // minutes, so the estimate can follow slow changes.
                double forget = Math.exp(-dtFix / 300.0);
                sw = sw * forget + 1;
                sx = sx * forget + dvdt;
                sxx = sxx * forget + dvdt * dvdt;
                // Variance that GNSS speed noise adds to dv/dt (errors in the regressor).
                double noise = (fix.speedAccuracy() * fix.speedAccuracy()
                        + prevFix.speedAccuracy() * prevFix.speedAccuracy()) / (dtFix * dtFix);
                snn = snn * forget + noise;
                for (int k = 0; k < 3; k++) {
                    sh[k] = sh[k] * forget + h[k];
                    sxh[k] = sxh[k] * forget + dvdt * h[k];
                }
                shh = shh * forget + Rot.dot(h, h);
                // Only solve once the car has accelerated/braked enough (variance of dv/dt).
                double varX = sxx - sx * sx / sw;
                if (varX > 8) {
                    // Covariance between dv/dt and the horizontal force: points in the forward direction.
                    double[] cov = new double[3];
                    for (int k = 0; k < 3; k++) {
                        cov[k] = sxh[k] - sx * sh[k] / sw;
                    }
                    double[] f = Rot.sub(cov, Rot.scale(up, Rot.dot(cov, up)));
                    if (Rot.norm(f) > 1e-6) {
                        fwd = Rot.normalize(f);
                        left = Rot.cross(up, fwd);
                        forwardEstimated = true;
                        // The length of the regression slope is (1 + scale): how much forward force
                        // the phone measures per unit of real acceleration.
                        double signal = varX - snn;
                        if (signal > 8 && sw > 5) {
                            double raw = Rot.norm(f) / signal - 1;
                            // Residual variance of the horizontal force not explained by dv/dt.
                            double total = shh - Rot.dot(sh, sh) / sw;
                            double explained = Rot.dot(f, f) / varX;
                            double residual = Math.max(1e-6, (total - explained) / (sw - 2));
                            double rawVar = residual / signal;
                            // Shrink towards zero when the estimate is uncertain (little acceleration seen).
                            double prior = SCALE_PRIOR_SIGMA * SCALE_PRIOR_SIGMA;
                            scale = Math.max(-0.1, Math.min(0.1, raw * prior / (prior + rawVar)));
                            scaleEstimated = true;
                        }
                    }
                }
            }
        }
        // Start a new interval at this fix.
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
        // Keep the three axes perpendicular: remove any "up" component from forward, rebuild left.
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
        // The regression sums and the low-pass are phone-frame vectors too: rotate them along.
        double[] a = Rot.apply(r, sh), b = Rot.apply(r, sxh), c = Rot.apply(r, lowPass);
        System.arraycopy(a, 0, sh, 0, 3);
        System.arraycopy(b, 0, sxh, 0, 3);
        System.arraycopy(c, 0, lowPass, 0, 3);
    }

    /** Effective forward accelerometer scale error (measured = (1 + scale) * true), 0 until known. */
    public double accelScale() {
        return useScale && scaleEstimated ? scale : 0;
    }

    /** Current car-to-phone rotation estimate (columns are car axes in phone coordinates). */
    public double[] mount() {
        return new double[]{fwd[0], left[0], up[0], fwd[1], left[1], up[1], fwd[2], left[2], up[2]};
    }
}
