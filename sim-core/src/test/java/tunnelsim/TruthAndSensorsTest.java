package tunnelsim;

import org.junit.jupiter.api.Test;
import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.ScenarioLoader;
import tunnelsim.sensor.SensorEvent;
import tunnelsim.sensor.SensorLog;
import tunnelsim.sensor.SensorSuite;
import tunnelsim.util.MathUtil;
import tunnelsim.util.Rng;
import tunnelsim.vehicle.Driver;
import tunnelsim.vehicle.TruthTrajectory;

import java.nio.file.Path;

import static org.assertj.core.api.Assertions.assertThat;

class TruthAndSensorsTest {
    static ScenarioConfig scenario(String file) {
        return ScenarioLoader.load(Path.of("..", "scenarios", file));
    }

    @Test
    void carDrivesTheWholeRouteIncludingTheRamp() {
        ScenarioConfig cfg = scenario("s02-curved-exit.yaml");
        RoadNetwork net = RoadNetwork.build(cfg, Rng.derive(cfg.seed, "map"));
        Route route = new Route(net, cfg.route.takeBranchAt);
        TruthTrajectory truth = new Driver(net, route, cfg.cars.get(0), Rng.derive(cfg.seed, "car")).simulate(cfg.duration);

        assertThat(net.edge(truth.leaf).ramp).isTrue();
        int last = truth.size() - 1;
        assertThat(truth.offset(last)).isGreaterThan(net.pathEnd(truth.leaf) - 2);
        double vmax = 0;
        boolean inTunnel = false;
        for (int i = 0; i < truth.size(); i++) {
            vmax = Math.max(vmax, truth.v(i));
            inTunnel |= truth.tunnel(i) >= 0;
        }
        assertThat(vmax).isBetween(20.0, 25.0);
        assertThat(inTunnel).isTrue();
    }

    @Test
    void stationaryAccelerometerMeasuresGravityAndNoFixesInsideTunnel() {
        ScenarioConfig cfg = scenario("s02-curved-exit.yaml");
        RoadNetwork net = RoadNetwork.build(cfg, Rng.derive(cfg.seed, "map"));
        Route route = new Route(net, cfg.route.takeBranchAt);
        TruthTrajectory truth = new Driver(net, route, cfg.cars.get(0), Rng.derive(cfg.seed, "car")).simulate(cfg.duration);
        SensorLog log = SensorSuite.record(truth, net, cfg, 7);

        double sum = 0;
        int n = 0;
        for (SensorEvent.ImuSample s : log.of(SensorEvent.ImuSample.class)) {
            if (s.t() < 4) {
                sum += Math.sqrt(s.ax() * s.ax() + s.ay() * s.ay() + s.az() * s.az());
                n++;
            }
        }
        assertThat(sum / n).isCloseTo(MathUtil.G, org.assertj.core.data.Offset.offset(0.5));

        for (SensorEvent.GnssFix f : log.of(SensorEvent.GnssFix.class)) {
            int i = truth.indexAt(f.t());
            assertThat(truth.tunnel(i)).isEqualTo(-1);
        }
        assertThat(log.of(SensorEvent.GnssFix.class)).isNotEmpty();
        assertThat(log.of(SensorEvent.BaroSample.class)).isNotEmpty();
    }
}
