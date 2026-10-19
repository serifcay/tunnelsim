package tunnelsim.estimator;

import tunnelsim.align.VehicleImu;
import tunnelsim.sensor.SensorEvent;

/**
 * A position estimator. It receives every raw sensor event and every car-frame IMU sample from
 * the alignment front-end, in timestamp order.
 */
public interface Estimator {
    String id();

    void onSensor(SensorEvent event);

    void onVehicleImu(VehicleImu imu);

    Estimate estimate();

    /** Probability that the remaining distance to {@code targetOffset} on the route is at most {@code d}. */
    double probRemainingBelow(double targetOffset, double d);

    /** Probability that the car is on the path ending in {@code leaf}. */
    double probOnPath(int leaf);

    /**
     * Probability that the car is in the subtree that starts at {@code edge} (the branch taken at
     * one fork), counting hypotheses that have not reached that fork yet as consistent. Unlike
     * {@link #probOnPath}, later forks do not matter.
     */
    double probOnSubtree(int edge);

    /** Map road grade at the current estimate, or NaN if unknown (used to re-align the phone at stops). */
    default double gradeAtEstimate() {
        return Double.NaN;
    }
}
