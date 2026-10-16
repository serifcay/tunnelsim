package tunnelsim.align;

/**
 * IMU sample rotated into the car frame (x forward, y left, z up) by the alignment front-end.
 * {@code aFwd} still contains the gravity component along a sloped road; the estimators remove it.
 */
public record VehicleImu(double t, double aFwd, double aLat, double aUp, double rollRate, double pitchRate,
                         double yawRate, boolean stationary) {
}
