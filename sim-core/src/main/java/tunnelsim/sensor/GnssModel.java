package tunnelsim.sensor;

import tunnelsim.map.RoadNetwork;
import tunnelsim.scenario.SensorsConfig.GnssConfig;
import tunnelsim.sensor.SensorEvent.GnssFix;
import tunnelsim.sensor.SensorEvent.GnssStatus;
import tunnelsim.util.GaussMarkov;
import tunnelsim.util.Rng;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * GNSS fixes and raw-measurement status. Open-sky errors are time-correlated (Gauss-Markov),
 * quality fades through the degraded zone around portals, there is no fix inside tunnels, and
 * after a tunnel the receiver needs a reacquisition delay and then suffers a decaying multipath
 * bias that its reported accuracy does not reflect.
 */
public final class GnssModel {
    private final GnssConfig c;
    private final RoadNetwork net;
    private final boolean raw;
    private final Rng rng;

    public GnssModel(GnssConfig c, RoadNetwork net, boolean raw, Rng rng) {
        this.c = c;
        this.net = net;
        this.raw = raw;
        this.rng = rng;
    }

    /** Produces the run's GNSS fixes (and, on Android, satellite/signal status) once per second. */
    public List<SensorEvent> generate(TruthTrajectory truth) {
        List<SensorEvent> out = new ArrayList<>();
        double period = 1.0 / c.rate;
        // Position errors in x and y that drift slowly (correlated over ~40 s), like real GNSS errors.
        GaussMarkov ex = new GaussMarkov(c.sigma, c.tau, rng);
        GaussMarkov ey = new GaussMarkov(c.sigma, c.tau, rng);
        double reacquireAt = -1;
        double mpStart = 0, mpX = 0, mpY = 0;
        int prevTunnel = -1;
        int lastIdx = 0;
        // Fixes don't arrive exactly on whole seconds: random phase.
        double phase = rng.uniform(0, period);

        for (double t = phase; t <= truth.duration(); t += period) {
            // A fix describes where the car was slightly earlier (receiver latency).
            int i = truth.indexAt(t - c.latency);
            // Detect tunnel exits between samples.
            for (int j = lastIdx; j <= truth.indexAt(t); j++) {
                int tun = truth.tunnel(j);
                if (prevTunnel >= 0 && tun < 0) {
                    // Leaving a tunnel: no fix for a few seconds, then a reflection error that fades out.
                    reacquireAt = truth.t(j) + rng.inRange(c.reacquireDelay);
                    double mag = rng.inRange(c.exitMultipath);
                    double dir = rng.uniform(0, 2 * Math.PI);
                    mpX = mag * Math.cos(dir);
                    mpY = mag * Math.sin(dir);
                    mpStart = reacquireAt;
                }
                prevTunnel = tun;
            }
            lastIdx = truth.indexAt(t) + 1;

            // Signal quality: 1 in open sky, lower near tunnel mouths, 0 inside.
            double q = net.gnssQuality(truth.position(i));
            double errX = ex.step(period), errY = ey.step(period);
            // Raw-measurement summary (Android only): satellite count and signal strength fall with quality.
            if (raw) {
                int sats = q <= 0 || t < reacquireAt ? 0 : (int) Math.max(0, Math.round(22 * q + rng.gaussian(1.5)));
                double cn0 = sats == 0 ? 0 : 22 + 18 * q + rng.gaussian(1.5);
                out.add(new GnssStatus(t, sats, cn0));
            }
            // Too little signal, or still reacquiring: no fix this second.
            if (q < c.minQuality || t < reacquireAt) {
                continue;
            }
            // Errors grow near portals (up to 4x); the exit reflection error decays exponentially.
            double degrade = 1 + 3 * (1 - q);
            double mp = Math.exp(-(t - mpStart) / c.exitMultipathTau);
            double x = truth.x(i) + errX * degrade + mpX * mp;
            double y = truth.y(i) + errY * degrade + mpY * mp;
            // The receiver's own accuracy claim (68% radius), sometimes optimistic, and it never includes the
            // reflection error.
            double accuracy = 1.5 * c.sigma * degrade * rng.inRange(c.reportedAccuracyFactor);
            double speedSigma = c.speedSigma * degrade;
            double speed = Math.max(0, truth.v(i) + rng.gaussian(speedSigma));
            double bearing = truth.heading(i) + rng.gaussian(speed > 1 ? 0.02 : 1.0);
            out.add(new GnssFix(t, x, y, accuracy, speed, speedSigma, bearing));
        }
        return out;
    }
}
