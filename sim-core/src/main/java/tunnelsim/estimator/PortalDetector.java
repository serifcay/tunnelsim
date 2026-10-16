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
    private static final double WINDOW = 1.0;
    private static final double REFRACTORY = 5.0;

    private final ArrayDeque<double[]> history = new ArrayDeque<>();
    private double lastEvent = Double.NEGATIVE_INFINITY;

    boolean onLight(LightSample s) {
        double log = Math.log10(Math.max(0.1, s.lux()));
        history.addLast(new double[]{s.t(), log});
        while (history.size() > 1 && s.t() - history.peekFirst()[0] > WINDOW) {
            history.removeFirst();
        }
        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        for (double[] h : history) {
            min = Math.min(min, h[1]);
            max = Math.max(max, h[1]);
        }
        if (max - min > LOG_THRESHOLD && s.t() - lastEvent > REFRACTORY) {
            lastEvent = s.t();
            history.clear();
            return true;
        }
        return false;
    }
}
