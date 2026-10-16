package tunnelsim.sensor;

import tunnelsim.scenario.SensorsConfig.MagConfig;
import tunnelsim.sensor.SensorEvent.MagSample;
import tunnelsim.util.Rng;
import tunnelsim.util.Rot;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Magnetometer: Earth's field plus the car's hard-iron offset, and inside tunnels a strong
 * position-dependent disturbance (rebar, cabling, passing vehicles). Modelled so the simulator
 * can show why magnetic heading is unusable in tunnels.
 */
public final class MagModel {
    private static final double[] EARTH = {0, 22, -42}; // uT, x east, y north, z up

    private final MagConfig c;
    private final Rng rng;

    public MagModel(MagConfig c, Rng rng) {
        this.c = c;
        this.rng = rng;
    }

    public List<MagSample> generate(TruthTrajectory truth) {
        List<MagSample> out = new ArrayList<>();
        double[] hardIron = {rng.gaussian(10), rng.gaussian(10), rng.gaussian(10)};
        // Spatial disturbance: a sum of sinusoids along the road with random wavelengths and phases.
        int terms = 6;
        double[][] wave = new double[terms][4];
        for (double[] w : wave) {
            w[0] = rng.uniform(3, 40);              // wavelength [m]
            w[1] = rng.uniform(0, 2 * Math.PI);     // phase
            w[2] = rng.nextInt(3);                  // axis
            w[3] = rng.gaussian(c.tunnelDisturbance / Math.sqrt(terms / 3.0));
        }
        double period = 1.0 / c.rate;
        for (double t = 0; t <= truth.duration(); t += period) {
            int i = truth.indexAt(t);
            double[] b = EARTH.clone();
            if (truth.tunnel(i) >= 0) {
                for (double[] w : wave) {
                    b[(int) w[2]] += w[3] * Math.sin(2 * Math.PI * truth.offset(i) / w[0] + w[1]);
                }
            }
            double[] rnb = Rot.bodyToNav(truth.heading(i), truth.pitch(i), truth.roll(i));
            double[] body = Rot.apply(Rot.transpose(rnb), b);
            for (int k = 0; k < 3; k++) {
                body[k] += hardIron[k];
            }
            double[] p = Rot.apply(truth.mountAt(t), body);
            out.add(new MagSample(t, p[0] + rng.gaussian(c.noise), p[1] + rng.gaussian(c.noise),
                    p[2] + rng.gaussian(c.noise)));
        }
        return out;
    }
}
