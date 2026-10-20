package tunnelsim.engine;

import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.scenario.ScenarioConfig;

import java.util.List;

/**
 * Result of one scenario run: the scenario, the road network and route, one result per car, and the compute
 * time.
 */
public record RunResult(ScenarioConfig scenario, RoadNetwork net, Route route, List<CarResult> cars,
                        double computeSeconds) {
    /** Length of the run: the longest car trajectory [s]. */
    public double duration() {
        return cars.stream().mapToDouble(c -> c.truth.duration()).max().orElse(0);
    }
}
