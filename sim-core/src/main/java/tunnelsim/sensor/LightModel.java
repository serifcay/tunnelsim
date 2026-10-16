package tunnelsim.sensor;

import tunnelsim.map.Features.Portal;
import tunnelsim.map.RoadNetwork;
import tunnelsim.scenario.ScenarioConfig.TunnelConfig;
import tunnelsim.scenario.SensorsConfig.LightConfig;
import tunnelsim.sensor.SensorEvent.LightSample;
import tunnelsim.util.Rng;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Ambient light at the phone's sensor: daylight (or night) outside, a brighter entrance zone and
 * dim interior inside, daylight leaking in near portals, a periodic ceiling-light pattern, and a
 * first-order sensor lag.
 */
public final class LightModel {
    private final LightConfig c;
    private final RoadNetwork net;
    private final Rng rng;

    public LightModel(LightConfig c, RoadNetwork net, Rng rng) {
        this.c = c;
        this.net = net;
        this.rng = rng;
    }

    public List<LightSample> generate(TruthTrajectory truth) {
        List<LightSample> out = new ArrayList<>();
        double outside = net.night() ? c.nightLux : rng.inRange(c.dayLux);
        double period = 1.0 / c.rate;
        double filtered = outside;
        double next = 0;
        for (int i = 0; i < truth.size(); i++) {
            double t = truth.t(i);
            double lux = trueLux(truth, i, outside);
            filtered += (lux - filtered) * truth.dt / c.lag;
            if (t >= next) {
                out.add(new LightSample(t, Math.max(0, filtered * (1 + rng.gaussian(c.noiseFraction)))));
                next += period;
            }
        }
        return out;
    }

    private double trueLux(TruthTrajectory truth, int i, double outside) {
        int tun = truth.tunnel(i);
        if (tun < 0) {
            return outside;
        }
        TunnelConfig tc = net.tunnels().get(tun);
        // Distance to the nearest portal of this tunnel.
        double d = Double.MAX_VALUE;
        for (Portal p : net.portals()) {
            if (p.tunnel() == tun) {
                d = Math.min(d, Math.hypot(truth.x(i) - p.x(), truth.y(i) - p.y()));
            }
        }
        double inside = c.tunnelLux;
        if (!net.night() && d < c.entranceLength) {
            double entrance = Math.max(c.tunnelLux, c.entranceFraction * outside);
            inside = entrance + (c.tunnelLux - entrance) * d / c.entranceLength;
        }
        if (tc.lightSpacing > 0) {
            inside *= 1 + c.lightModulation * Math.cos(2 * Math.PI * truth.offset(i) / tc.lightSpacing);
        }
        return inside + (outside - inside) * Math.exp(-d / c.daylightLeak);
    }
}
