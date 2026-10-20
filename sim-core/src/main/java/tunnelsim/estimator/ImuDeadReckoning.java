package tunnelsim.estimator;

import tunnelsim.align.VehicleImu;
import tunnelsim.sensor.SensorEvent;
import tunnelsim.sensor.SensorEvent.GnssFix;

/**
 * Approach B: integrate the forward accelerometer, with its bias calibrated against GNSS speed
 * changes before the tunnel. No map aiding beyond following the road: no grade correction, no
 * stop detection, no priors. Shows the classic quadratic drift and the effect of road grade.
 */
public final class ImuDeadReckoning extends PathEstimator {
    // Averaging time of the bias estimate [s].
    private static final double BIAS_TAU = 30;

    private double offset;
    private double speed;
    private double bias;
    private double sigmaFix;
    private double outageStart = Double.NaN;
    private GnssFix prevFix;
    private double accSum;
    private int accCount;
    private double lastImuT = Double.NaN;

    public ImuDeadReckoning(EstimatorContext ctx) {
        super(ctx);
    }

    @Override
    public String id() {
        return "B";
    }

    @Override
    public void onSensor(SensorEvent event) {
        t = event.t();
        if (event instanceof GnssFix fix && ctx.aiding().gnss && gnss.accept(fix)) {
            double[] m = matchFix(fix);
            if (m == null) {
                return;
            }
            // Bias = mean measured forward acceleration minus the acceleration GNSS saw (speed change),
            // averaged slowly. Note: no slope correction, on purpose; this is the naive baseline.
            if (prevFix != null && accCount > 0 && fix.t() - prevFix.t() < 1.5) {
                double dvdt = (fix.speed() - prevFix.speed()) / (fix.t() - prevFix.t());
                double innovation = accSum / accCount - dvdt;
                bias += (innovation - bias) * Math.min(1, (fix.t() - prevFix.t()) / BIAS_TAU);
            }
            prevFix = fix;
            accSum = 0;
            accCount = 0;
            offset = m[0];
            speed = fix.speed();
            sigmaFix = fix.accuracy() / 1.5;
            outageStart = Double.NaN;
            initialised = true;
        }
    }

    @Override
    public void onVehicleImu(VehicleImu imu) {
        double dt = Double.isNaN(lastImuT) ? 0 : imu.t() - lastImuT;
        lastImuT = imu.t();
        t = imu.t();
        // Sum of forward acceleration since the last fix, for the bias estimate.
        accSum += imu.aFwd();
        accCount++;
        if (!initialised) {
            return;
        }
        // Remember when GNSS was lost; the uncertainty grows from then on.
        if (!gnss.fresh(imu.t()) && Double.isNaN(outageStart)) {
            outageStart = imu.t();
        }
        // Integrate acceleration into speed and speed into distance (trapezoidal rule).
        double v = Math.max(0, speed + (imu.aFwd() - bias) / (1 + imu.accelScale()) * dt);
        offset = clampOffset(offset + 0.5 * (speed + v) * dt);
        speed = v;
    }

    private double sigma() {
        double tau = Double.isNaN(outageStart) ? 0 : t - outageStart;
        // Residual bias of ~0.05 m/s^2 plus 0.2 m/s initial speed error.
        return Math.sqrt(sigmaFix * sigmaFix + Math.pow(0.025 * tau * tau, 2) + Math.pow(0.2 * tau, 2));
    }

    @Override
    public Estimate estimate() {
        double tau = Double.isNaN(outageStart) ? 0 : t - outageStart;
        return build(offset, sigma(), speed, 0.2 + 0.05 * tau);
    }

    @Override
    public double probRemainingBelow(double targetOffset, double d) {
        return gaussRemaining(offset, sigma(), targetOffset, d);
    }
}
