package tunnelsim;

import org.junit.jupiter.api.Test;
import tunnelsim.engine.RunResult;
import tunnelsim.engine.SimulationRun;
import tunnelsim.metrics.MonteCarlo;
import tunnelsim.metrics.TrackMetrics;
import tunnelsim.scenario.ScenarioConfig;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;

/** End-to-end behaviour of the estimators. Thresholds are loose regression guards, not targets. */
class EstimationTest {
    @Test
    void runsAreDeterministicForASeed() {
        ScenarioConfig cfg = TruthAndSensorsTest.scenario("s08-twin-tunnels.yaml");
        TrackMetrics a = SimulationRun.run(cfg).cars().get(0).metrics.get("D");
        TrackMetrics b = SimulationRun.run(cfg).cars().get(0).metrics.get("D");
        assertThat(a.rmsTunnel()).isEqualTo(b.rmsTunnel());
        cfg.seed++;
        TrackMetrics c = SimulationRun.run(cfg).cars().get(0).metrics.get("D");
        assertThat(c.rmsTunnel()).isNotEqualTo(a.rmsTunnel());
    }

    @Test
    void withoutTunnelsEveryEstimatorFollowsGnss() {
        ScenarioConfig cfg = TruthAndSensorsTest.scenario("s01-straight-flat.yaml");
        cfg.road.tunnels.clear();
        RunResult r = SimulationRun.run(cfg);
        r.cars().get(0).tracks.forEach((id, track) -> {
            double worst = track.samples.stream().filter(s -> s.t() > 30 && !Double.isNaN(s.error()))
                    .mapToDouble(s -> Math.abs(s.error())).max().orElse(0);
            assertThat(worst).as(id).isLessThan(15);
        });
    }

    @Test
    void particleFilterBeatsConstantVelocityAndCatchesTheMissedExit() {
        ScenarioConfig s2 = TruthAndSensorsTest.scenario("s02-curved-exit.yaml");
        s2.estimators = List.of("A", "D");
        MonteCarlo.Result r = MonteCarlo.run(s2, 8);
        assertThat(r.summaries().get("D").rms()[0]).isLessThan(r.summaries().get("A").rms()[0]);
        assertThat(r.summaries().get("D").callout()[0]).isLessThan(2.0);

        ScenarioConfig s6 = TruthAndSensorsTest.scenario("s06-missed-exit.yaml");
        s6.estimators = List.of("C", "D");
        MonteCarlo.Result m = MonteCarlo.run(s6, 8);
        assertThat(m.summaries().get("D").branch()[1]).isLessThan(8);
        assertThat(m.summaries().get("C").branch()[0]).isGreaterThan(20);
    }

    @Test
    void stopsOnSlopesDoNotCorruptTheAlignment() {
        // Re-estimating "up" at a stop on a slope used to count the slope twice (km-level errors).
        ScenarioConfig s11 = TruthAndSensorsTest.scenario("s11-stops-on-slopes.yaml");
        s11.estimators = List.of("C", "D");
        MonteCarlo.Result r = MonteCarlo.run(s11, 8);
        assertThat(r.summaries().get("D").max()[0]).isLessThan(150);
        assertThat(r.summaries().get("C").max()[0]).isLessThan(300);
    }

    @Test
    void phoneShiftInTheHolderIsTrackedByTheGyro() {
        // A 27 deg phone rotation mid-tunnel. When the simulated gyro under-reported the rotation,
        // the leftover tilt leaked gravity into the forward axis and D drifted back by ~100 m.
        ScenarioConfig s7 = TruthAndSensorsTest.scenario("s07-mount-shift.yaml");
        s7.estimators = List.of("D");
        MonteCarlo.Summary d = MonteCarlo.run(s7, 8).summaries().get("D");
        assertThat(d.rms()[0]).isLessThan(30);
        assertThat(d.within2Sigma()).isGreaterThan(0.85);
    }

    @Test
    void severalExitsInATunnelAreToldApart() {
        // Three exits; the route takes the middle one; car 2 takes exit 1 by mistake.
        ScenarioConfig s14 = TruthAndSensorsTest.scenario("s14-multi-exit-uphill.yaml");
        s14.estimators = List.of("C", "D");
        MonteCarlo.Result r = MonteCarlo.run(s14, 6);
        assertThat(r.summaries().get("D").branch()[1]).isLessThan(5);
        assertThat(r.summaries().get("D").callout()[0]).isLessThan(1.5);
        assertThat(r.summaries().get("D").lateFraction()).isEqualTo(0.0);
        assertThat(r.summaries().get("C").branch()[0]).isGreaterThan(8);
    }

    @Test
    void particleFilterIsRoughlyConsistent() {
        ScenarioConfig s3 = TruthAndSensorsTest.scenario("s03-underwater.yaml");
        s3.estimators = List.of("D");
        MonteCarlo.Summary d = MonteCarlo.run(s3, 8).summaries().get("D");
        assertThat(d.within2Sigma()).isGreaterThan(0.8);
    }
}
