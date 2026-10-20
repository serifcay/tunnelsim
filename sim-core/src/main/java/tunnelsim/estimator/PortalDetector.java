package tunnelsim.estimator;

import tunnelsim.sensor.SensorEvent.LightSample;

import java.util.ArrayDeque;

/**
 * Detects passing a tunnel portal from the ambient light sensor: a change of more than a factor
 * of four within about a second. Works day (bright to dark) and night (dark to bright). Tunnel
 * lighting is graded over the entrance zone, so normal in-tunnel variation stays below threshold.
 */
final class PortalDetector {
    private static final double LOG_THRESHOLD = 0.6; // factor 4
    // Compare light levels over the last second; ignore new events for 5 s after one fired.
    private static final double WINDOW = 1.0;
    private static final double REFRACTORY = 5.0;

    private final ArrayDeque<double[]> history = new ArrayDeque<>();
    private double lastEvent = Double.NEGATIVE_INFINITY;

    /** Feeds one light sample; returns true when a portal is detected. */
    boolean onLight(LightSample s) {
        // Work in log10(lux): a factor change in light becomes a fixed difference.
        double log = Math.log10(Math.max(0.1, s.lux()));
        history.addLast(new double[]{s.t(), log});
        // Keep only the samples of the last second.
        while (history.size() > 1 && s.t() - history.peekFirst()[0] > WINDOW) {
            history.removeFirst();
        }
        // Range of light levels within that second.
        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        for (double[] h : history) {
            min = Math.min(min, h[1]);
            max = Math.max(max, h[1]);
        }
        // More than a factor 4 change within a second: the car just passed a portal.
        if (max - min > LOG_THRESHOLD && s.t() - lastEvent > REFRACTORY) {
            lastEvent = s.t();
            history.clear();
            return true;
        }
        return false;
    }
}
