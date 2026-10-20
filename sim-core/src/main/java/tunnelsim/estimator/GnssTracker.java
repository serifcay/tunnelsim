package tunnelsim.estimator;

import tunnelsim.estimator.Estimate.Mode;
import tunnelsim.sensor.SensorEvent.GnssFix;

/** Tracks GNSS availability: fresh fixes, outages, and the recovery period after an outage. */
final class GnssTracker {
    // Fixes worse than this [m] are ignored; a fix is "fresh" for FRESH seconds; after an outage longer
    // than LONG_OUTAGE the next RECOVERY seconds count as recovering.
    static final double MAX_ACCURACY = 30;
    private static final double FRESH = 1.6;
    private static final double RECOVERY = 10;
    private static final double LONG_OUTAGE = 5;

    private double lastFixT = Double.NEGATIVE_INFINITY;
    private double recoveryStart = Double.NEGATIVE_INFINITY;
    private int fixesSinceOutage = Integer.MAX_VALUE;

    /** Registers a fix; returns false if it is too inaccurate to use. */
    boolean accept(GnssFix fix) {
        if (fix.accuracy() > MAX_ACCURACY) {
            return false;
        }
        // First fix after an outage (e.g. leaving a tunnel): start counting fixes again.
        if (fix.t() - lastFixT > LONG_OUTAGE && lastFixT > Double.NEGATIVE_INFINITY) {
            recoveryStart = fix.t();
            fixesSinceOutage = 0;
        }
        // Count fixes since the outage (stays at MAX_VALUE if there never was one).
        fixesSinceOutage = fixesSinceOutage == Integer.MAX_VALUE ? Integer.MAX_VALUE : fixesSinceOutage + 1;
        lastFixT = fix.t();
        return true;
    }

    /** True if a fix arrived within the last FRESH seconds. */
    boolean fresh(double t) {
        return t - lastFixT < FRESH;
    }

    /** Fixes received since the end of the last outage (large if there was none). */
    int fixesSinceOutage() {
        return fixesSinceOutage;
    }

    double lastFixT() {
        return lastFixT;
    }

    /**
     * What the estimate is based on right now: nothing yet, GNSS, dead reckoning, or recovering
     * after an outage.
     */
    Mode mode(double t) {
        if (lastFixT == Double.NEGATIVE_INFINITY) {
            return Mode.INIT;
        }
        if (!fresh(t)) {
            return Mode.DEAD_RECKONING;
        }
        return t - recoveryStart < RECOVERY ? Mode.RECOVERING : Mode.GNSS;
    }
}
