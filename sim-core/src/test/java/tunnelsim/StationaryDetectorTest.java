package tunnelsim;

import org.junit.jupiter.api.Test;
import tunnelsim.align.StationaryDetector;
import tunnelsim.util.Rng;

import static org.assertj.core.api.Assertions.assertThat;

class StationaryDetectorTest {
    @Test
    void idleIsStationaryRoadVibrationIsNotAndPullingAwayReleasesImmediately() {
        StationaryDetector d = new StationaryDetector();
        Rng rng = new Rng(1);
        for (int i = 0; i < 200; i++) {
            double idle = 0.015 * Math.sin(2 * Math.PI * 24 * i * 0.01);
            d.update(rng.gaussian(0.012), 9.81 + idle + rng.gaussian(0.012), rng.gaussian(0.012), 0, 0, 0);
        }
        assertThat(d.stationary()).isTrue();
        // Pull away: forward acceleration ramps up at 2.5 m/s^3.
        int released = -1;
        for (int i = 0; i < 100 && released < 0; i++) {
            double a = Math.min(2, 2.5 * i * 0.01);
            if (!d.update(rng.gaussian(0.012), 9.81 + rng.gaussian(0.012), -a + rng.gaussian(0.012), 0, 0, 0)) {
                released = i;
            }
        }
        assertThat(released).isBetween(0, 12); // within ~0.1 s
        StationaryDetector m = new StationaryDetector();
        for (int i = 0; i < 200; i++) {
            m.update(rng.gaussian(0.06), 9.81 + rng.gaussian(0.12), rng.gaussian(0.06), 0, 0, 0);
        }
        assertThat(m.stationary()).isFalse();
    }
}
