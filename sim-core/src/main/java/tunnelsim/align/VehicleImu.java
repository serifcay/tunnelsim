package tunnelsim.align;

/**
 * IMU sample rotated into the car frame (x forward, y left, z up) by the alignment front-end.
 * {@code aFwd} still contains the gravity component along a sloped road; the estimators remove it.
 * {@code mountDisturbed} is set once when the phone has just been moved in its mount: the
 * alignment was re-propagated with the gyro, but bias calibrations no longer hold exactly.
 * {@code accelScale} is the effective forward scale error: estimators use
 * a = (aFwd - bias - g*sin(slope)) / (1 + accelScale).
 */
public record VehicleImu(double t, double aFwd, double aLat, double aUp, double rollRate, double pitchRate,
                         double yawRate, boolean stationary, boolean mountDisturbed, double accelScale) {
}
