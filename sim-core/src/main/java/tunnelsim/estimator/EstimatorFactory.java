package tunnelsim.estimator;

import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.util.Rng;

import java.util.List;

/** Creates estimators by their short id (A, B, C, D, D+E). */
public final class EstimatorFactory {
    public static final List<String> ALL = List.of("A", "B", "C", "D", "D+E");

    private EstimatorFactory() {
    }

    /** Human-readable name of an estimator, for the UI. */
    public static String describe(String id) {
        return switch (id) {
            case "A" -> "Constant velocity";
            case "B" -> "IMU dead reckoning";
            case "C" -> "Map-aided EKF";
            case "D" -> "Road particle filter";
            // D+E is the same particle filter with option E (BLE beacons / OBD speed) enabled.
            case "D+E" -> "Particle filter + BLE/OBD";
            default -> id;
        };
    }

    public static Estimator create(String id, RoadNetwork net, Route route, ScenarioConfig cfg, long seed) {
        // Each estimator gets its own random stream, so adding or removing one does not change the others.
        Rng rng = Rng.derive(seed, "estimator/" + id);
        return switch (id) {
            case "A" -> new ConstantVelocityEstimator(new EstimatorContext(net, route, cfg, cfg.estimator, false, rng));
            case "B" -> new ImuDeadReckoning(new EstimatorContext(net, route, cfg, cfg.estimator, false, rng));
            case "C" -> new MapAidedEkf(new EstimatorContext(net, route, cfg, cfg.estimator, false, rng));
            case "D" -> new RoadParticleFilter(new EstimatorContext(net, route, cfg, cfg.estimator, false, rng), "D");
            case "D+E" -> new RoadParticleFilter(new EstimatorContext(net, route, cfg, cfg.estimator, true, rng), "D+E");
            default -> throw new IllegalArgumentException("Unknown estimator '" + id + "' (known: " + ALL + ")");
        };
    }
}
