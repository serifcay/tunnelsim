package tunnelsim;

import org.junit.jupiter.api.Test;
import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.ScenarioLoader;
import tunnelsim.util.Rng;
import tunnelsim.vehicle.Driver;
import tunnelsim.vehicle.TruthTrajectory;

import java.nio.file.Path;

import static org.assertj.core.api.Assertions.assertThat;

class ScenarioLibraryTest {
    @Test
    void everyScenarioLoadsAndEveryCarReachesTheEndOfItsRoad() {
        for (Path file : ScenarioLoader.list(Path.of("..", "scenarios"))) {
            ScenarioConfig cfg = ScenarioLoader.load(file);
            RoadNetwork net = RoadNetwork.build(cfg, Rng.derive(cfg.seed, "map"));
            Route route = new Route(net, cfg.route.takeBranchAt);
            for (ScenarioConfig.CarConfig car : cfg.cars) {
                TruthTrajectory truth = new Driver(net, route, car, Rng.derive(cfg.seed, "car/" + car.id)).simulate(cfg.duration);
                int last = truth.size() - 1;
                System.out.printf("%-28s %-12s arrives at %6.1f s (%.0f of %.0f m)%n", cfg.name, car.id, truth.t(last),
                        truth.offset(last), net.pathEnd(truth.leaf));
                assertThat(truth.offset(last)).as(file + " " + car.id).isGreaterThan(net.pathEnd(truth.leaf) - 2);
            }
        }
    }
}
