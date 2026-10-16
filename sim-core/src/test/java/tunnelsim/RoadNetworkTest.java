package tunnelsim;

import org.junit.jupiter.api.Test;
import tunnelsim.map.MapPosition;
import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.util.Rng;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

class RoadNetworkTest {
    @Test
    void forkSplitsTheMainRoadAndErrorsAreMeasuredThroughTheFork() {
        ScenarioConfig cfg = TruthAndSensorsTest.scenario("s02-curved-exit.yaml");
        RoadNetwork net = RoadNetwork.build(cfg, Rng.derive(1, "map"));
        Route route = new Route(net, cfg.route.takeBranchAt);
        assertThat(net.edges()).hasSize(3); // main-0, main-1, ramp
        assertThat(net.edge(route.leaf()).ramp).isTrue();
        assertThat(route.maneuverOffset()).isCloseTo(3900, offset(1e-6));

        MapPosition onMain = net.positionOnMain(4000);
        MapPosition onRamp = net.positionOnPath(route.leaf(), 4050);
        RoadNetwork.PathError e = net.error(onRamp, onMain);
        assertThat(e.wrongBranch()).isTrue();
        assertThat(e.meters()).isCloseTo(150 + 100, offset(1e-6));

        MapPosition before = net.positionOnMain(3800);
        assertThat(net.error(before, onMain).meters()).isCloseTo(-200, offset(1e-6));
        assertThat(net.tunnelAt(net.positionOnMain(2000))).isEqualTo(0);
        assertThat(net.tunnelAt(net.positionOnPath(route.leaf(), 3900 + 300))).isEqualTo(-1);
        assertThat(net.gnssQuality(net.positionOnMain(500))).isEqualTo(1.0);
    }
}
