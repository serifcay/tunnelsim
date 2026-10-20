package tunnelsim.estimator;

import tunnelsim.align.VehicleImu;
import tunnelsim.sensor.SensorEvent;
import tunnelsim.sensor.SensorEvent.GnssFix;

/**
 * Approach A: hold the last GNSS speed and keep moving along the route. Roughly what many
 * navigation apps do in a tunnel. Uncertainty grows with 10% of the distance travelled.
 */
public final class ConstantVelocityEstimator extends PathEstimator {
    private double offset;
    private double speed;
    // Uncertainty of the last fix, and distance driven since it (the uncertainty grows with it).
    private double sigmaFix;
    private double distanceSinceFix;
    private double lastT = Double.NaN;

    public ConstantVelocityEstimator(EstimatorContext ctx) {
        super(ctx);
    }

    @Override
    public String id() {
        return "A";
    }

    @Override
    public void onSensor(SensorEvent event) {
        // Move forward to this event's time first, then apply the event.
        advance(event.t());
        // A usable GNSS fix resets position and speed to the fix.
        if (event instanceof GnssFix fix && ctx.aiding().gnss && gnss.accept(fix)) {
            double[] m = matchFix(fix);
            if (m != null) {
                offset = m[0];
                speed = fix.speed();
                sigmaFix = fix.accuracy() / 1.5;
                distanceSinceFix = 0;
                lastT = fix.t();
                initialised = true;
            }
        }
    }

    @Override
    public void onVehicleImu(VehicleImu imu) {
        advance(imu.t());
    }

    /** Moves the estimate forward at the last known speed. */
    private void advance(double now) {
        if (initialised && !Double.isNaN(lastT)) {
            double d = speed * (now - lastT);
            offset = clampOffset(offset + d);
            distanceSinceFix += d;
        }
        lastT = now;
        t = now;
    }

    @Override
    public Estimate estimate() {
        // Uncertainty: the fix error combined with 10% of the distance driven since then.
        double sigma = Math.hypot(sigmaFix, 0.1 * distanceSinceFix);
        return build(offset, sigma, speed, 0.1 * speed + 0.2);
    }

    @Override
    public double probRemainingBelow(double targetOffset, double d) {
        return gaussRemaining(offset, Math.hypot(sigmaFix, 0.1 * distanceSinceFix), targetOffset, d);
    }
}
