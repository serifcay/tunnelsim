package tunnelsim.sensor;

import tunnelsim.scenario.SensorsConfig.BaroConfig;
import tunnelsim.sensor.SensorEvent.BaroSample;
import tunnelsim.util.GaussMarkov;
import tunnelsim.util.MathUtil;
import tunnelsim.util.Rng;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Barometer: pressure from altitude, an unknown absolute offset (weather), slow drift, and tunnel
 * effects (entry pressure step, piston-effect fluctuation, ventilation/window spikes).
 */
public final class BaroModel {
    private final BaroConfig c;
    private final Rng rng;

    public BaroModel(BaroConfig c, Rng rng) {
        this.c = c;
        this.rng = rng;
    }

    public List<BaroSample> generate(TruthTrajectory truth) {
        List<BaroSample> out = new ArrayList<>();
        double period = 1.0 / c.rate;
        double weather = rng.uniform(-1500, 1500);
        GaussMarkov piston = new GaussMarkov(c.tunnelSigma, c.tunnelTau, rng);
        double step = 0;
        double spikeUntil = -1, spike = 0;
        int prevTunnel = -1;
        for (double t = 0; t <= truth.duration(); t += period) {
            int i = truth.indexAt(t);
            int tun = truth.tunnel(i);
            if (tun >= 0 && prevTunnel < 0) {
                step = rng.uniform(-c.entryStep, c.entryStep);
            }
            prevTunnel = tun;
            step *= Math.exp(-period / c.tunnelTau);
            weather += rng.gaussian(c.drift * Math.sqrt(period));
            double p = piston.step(period);
            if (t > spikeUntil && rng.chance(c.spikeRate * period)) {
                spikeUntil = t + rng.uniform(2, 5);
                spike = (rng.chance(0.5) ? 1 : -1) * c.spikeMagnitude * rng.uniform(0.5, 1.0);
            }
            double pressure = MathUtil.altitudeToPressure(truth.z(i) + 1.2)
                    + weather
                    + (tun >= 0 ? p : 0) + step
                    + (t <= spikeUntil ? spike : 0)
                    + rng.gaussian(c.noise);
            out.add(new BaroSample(t, pressure));
        }
        return out;
    }
}
