package tunnelsim.sensor;

/**
 * Everything the phone records, as timestamped events. Positions are in the local metric frame
 * (x east, y north); a real-device log would be converted into the same frame.
 */
public sealed interface SensorEvent {
    double t();

    /** Accelerometer [m/s^2] and gyroscope [rad/s] in the phone frame (x right, y up, z out of screen). */
    record ImuSample(double t, double ax, double ay, double az, double gx, double gy, double gz)
            implements SensorEvent {
    }

    /**
     * A GNSS fix. {@code accuracy} follows the Android convention: radius of 68% confidence [m].
     */
    record GnssFix(double t, double x, double y, double accuracy, double speed, double speedAccuracy,
                   double bearing) implements SensorEvent {
    }

    /** GNSS raw-measurement summary (Android only): satellites used and mean C/N0 [dB-Hz]. */
    record GnssStatus(double t, int satellites, double meanCn0) implements SensorEvent {
    }

    /** Barometric pressure [Pa]. */
    record BaroSample(double t, double pressure) implements SensorEvent {
    }

    /** Ambient light [lux] (Android only). */
    record LightSample(double t, double lux) implements SensorEvent {
    }

    /** Magnetic field in the phone frame [uT]. */
    record MagSample(double t, double mx, double my, double mz) implements SensorEvent {
    }

    /** Serving cell changed (Android only). */
    record CellChange(double t, int cellId) implements SensorEvent {
    }

    /** One BLE scan window: beacon ids heard and their RSSI [dBm]. Option E. */
    record BleScan(double t, int[] beaconIds, double[] rssi) implements SensorEvent {
    }

    /** Vehicle speed from OBD-II / Android Automotive [m/s]. Option E. */
    record ObdSpeed(double t, double speed) implements SensorEvent {
    }
}
