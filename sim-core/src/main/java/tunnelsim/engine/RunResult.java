package tunnelsim.engine;

import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.scenario.ScenarioConfig;

import java.util.List;

public record RunResult(ScenarioConfig scenario, RoadNetwork net, Route route, List<CarResult> cars,
                        double computeSeconds) {
    public double duration() {
        return cars.stream().mapToDouble(c -> c.truth.duration()).max().orElse(0);
    }
}
