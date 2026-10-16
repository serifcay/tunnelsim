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
}
