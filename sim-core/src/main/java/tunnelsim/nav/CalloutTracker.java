package tunnelsim.nav;

import tunnelsim.estimator.Estimator;
import tunnelsim.map.Route;
import tunnelsim.scenario.ScenarioConfig.NavConfig;

import java.util.ArrayList;
import java.util.List;

/**
 * Fires the turn-by-turn callouts for the route's maneuver ("in 400 m, take the exit").
 * MEAN fires when the estimated remaining distance reaches D (median of the belief);
 * CONSERVATIVE fires as soon as there is a probability alpha that it already has, because a late
 * callout makes the driver miss the exit while an early one is only an annoyance.
 */
public final class CalloutTracker {
    /**
     * @param trueRemaining true distance to the maneuver when the callout fired [m]
     * @param fired         false if the car passed the maneuver before the callout fired
     */
    public record Callout(double distance, double t, double trueRemaining, double truthSpeed, boolean fired,
                          boolean late) {
        /** Seconds early (+) or late (-) relative to the ideal moment. */
        public double timingError() {
            return truthSpeed > 0.5 ? (trueRemaining - distance) / truthSpeed : 0;
        }
    }

    private final Route route;
    private final NavConfig cfg;
    private final List<Double> pending;
    private final List<Callout> callouts = new ArrayList<>();

    public CalloutTracker(Route route, NavConfig cfg) {
        this.route = route;
        this.cfg = cfg;
        this.pending = new ArrayList<>(cfg.calloutDistances);
        this.pending.sort((a, b) -> Double.compare(b, a));
    }

    public List<Callout> callouts() {
        return callouts;
    }

    /** @return callouts fired during this tick */
    public List<Callout> update(double t, Estimator est, int truthEdge, double truthOffset, double truthSpeed) {
        List<Callout> fired = new ArrayList<>();
        if (!route.hasManeuver()) {
            return fired;
        }
        if (route.leftBeforeManeuver(truthEdge)) {
            pending.clear(); // the driver took an earlier exit: these callouts no longer apply
            return fired;
        }
        double maneuver = route.maneuverOffset();
        double threshold = "MEAN".equalsIgnoreCase(cfg.calloutPolicy) ? 0.5 : cfg.calloutAlpha;
        while (!pending.isEmpty()) {
            double d = pending.get(0);
            double trueRemaining = maneuver - truthOffset;
            if (trueRemaining < 0) {
                Callout c = new Callout(d, t, trueRemaining, truthSpeed, false, true);
                callouts.add(c);
                fired.add(c);
                pending.remove(0);
                continue;
            }
            if (est.probRemainingBelow(maneuver, d) >= threshold) {
                Callout c = new Callout(d, t, trueRemaining, truthSpeed, true, trueRemaining < d - cfg.lateTolerance);
                callouts.add(c);
                fired.add(c);
                pending.remove(0);
                continue;
            }
            break;
        }
        return fired;
    }
}
