package tunnelsim.sensor;

import tunnelsim.map.Features.Beacon;
import tunnelsim.map.Features.CellZone;
import tunnelsim.map.RoadNetwork;
import tunnelsim.scenario.SensorsConfig.BleConfig;
import tunnelsim.scenario.SensorsConfig.CellConfig;
import tunnelsim.scenario.SensorsConfig.ObdConfig;
import tunnelsim.sensor.SensorEvent.BleScan;
import tunnelsim.sensor.SensorEvent.CellChange;
import tunnelsim.sensor.SensorEvent.ObdSpeed;
import tunnelsim.util.GaussMarkov;
import tunnelsim.util.Rng;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Cell handovers, BLE beacon scans (option E) and OBD vehicle speed (option E). */
public final class RadioModels {
    private RadioModels() {
    }

    /** A handover event when the car passes each cell zone, with position jitter and misses. */
    public static List<CellChange> cell(TruthTrajectory truth, RoadNetwork net, CellConfig c, Rng rng) {
        List<CellChange> out = new ArrayList<>();
        for (CellZone z : net.cells()) {
            if (!net.isAncestorOrSelf(z.position().edge(), truth.leaf)) {
                continue;
            }
            double at = net.pathOffset(z.position()) + rng.gaussian(c.jitter);
            boolean missed = rng.chance(c.missProbability);
            if (missed) {
                continue;
            }
            for (int i = 0; i < truth.size(); i++) {
                if (truth.offset(i) >= at) {
                    out.add(new CellChange(truth.t(i), z.cellId()));
                    break;
                }
            }
        }
        out.sort((a, b) -> Double.compare(a.t(), b.t()));
        return out;
    }

    /** Periodic BLE scans: log-distance path loss with correlated shadowing per beacon. */
    public static List<BleScan> ble(TruthTrajectory truth, RoadNetwork net, BleConfig c, Rng rng) {
        List<BleScan> out = new ArrayList<>();
        Map<Integer, GaussMarkov> shadow = new HashMap<>();
        for (double t = rng.uniform(0, c.scanInterval); t <= truth.duration(); t += c.scanInterval) {
            int i = truth.indexAt(t);
            double px = truth.x(i), py = truth.y(i), pz = truth.z(i) + 1.2;
            List<Integer> ids = new ArrayList<>();
            List<Double> rssi = new ArrayList<>();
            for (Beacon b : net.beacons()) {
                if (b.failed() || Math.abs(b.x() - px) > 150 || Math.abs(b.y() - py) > 150) {
                    continue;
                }
                double d = Math.max(0.5, Math.sqrt(sq(b.x() - px) + sq(b.y() - py) + sq(b.z() - pz)));
                GaussMarkov sh = shadow.computeIfAbsent(b.id(), k -> new GaussMarkov(c.shadowing, c.shadowingTau, rng));
                double r = c.txPower - 10 * c.pathLossExponent * Math.log10(d) + sh.step(c.scanInterval);
                if (r > c.sensitivity && rng.chance(c.detectionProbability)) {
                    ids.add(b.id());
                    rssi.add(r);
                }
            }
            if (!ids.isEmpty()) {
                out.add(new BleScan(t, ids.stream().mapToInt(Integer::intValue).toArray(),
                        rssi.stream().mapToDouble(Double::doubleValue).toArray()));
            }
        }
        return out;
    }

    /** Speedometer reading with a scale error, quantisation and latency. */
    public static List<ObdSpeed> obd(TruthTrajectory truth, ObdConfig c, Rng rng) {
        List<ObdSpeed> out = new ArrayList<>();
        double scale = 1 + rng.inRange(c.scaleError);
        for (double t = c.latency; t <= truth.duration(); t += 1.0 / c.rate) {
            double v = truth.v(truth.indexAt(t - c.latency)) * scale;
            out.add(new ObdSpeed(t, Math.round(v / c.quantization) * c.quantization));
        }
        return out;
    }

    private static double sq(double x) {
        return x * x;
    }
}
