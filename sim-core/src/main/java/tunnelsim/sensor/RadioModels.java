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
            // Only zones on the road this car actually drives.
            if (!net.isAncestorOrSelf(z.position().edge(), truth.leaf)) {
                continue;
            }
            // The handover happens somewhere around the zone (+-60 m), and is sometimes missed entirely.
            double at = net.pathOffset(z.position()) + rng.gaussian(c.jitter);
            boolean missed = rng.chance(c.missProbability);
            if (missed) {
                continue;
            }
            for (int i = 0; i < truth.size(); i++) {
                // The event fires when the car first reaches that point.
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
        // Each beacon's signal fades up and down slowly (shadowing by other vehicles, walls), independently per
        // beacon.
        Map<Integer, GaussMarkov> shadow = new HashMap<>();
        for (double t = rng.uniform(0, c.scanInterval); t <= truth.duration(); t += c.scanInterval) {
            int i = truth.indexAt(t);
            // The phone, 1.2 m above the road.
            double px = truth.x(i), py = truth.y(i), pz = truth.z(i) + 1.2;
            List<Integer> ids = new ArrayList<>();
            List<Double> rssi = new ArrayList<>();
            for (Beacon b : net.beacons()) {
                // Dead beacons are never heard; beacons more than 150 m away are skipped (out of range anyway).
                if (b.failed() || Math.abs(b.x() - px) > 150 || Math.abs(b.y() - py) > 150) {
                    continue;
                }
                double d = Math.max(0.5, Math.sqrt(sq(b.x() - px) + sq(b.y() - py) + sq(b.z() - pz)));
                GaussMarkov sh = shadow.computeIfAbsent(b.id(), k -> new GaussMarkov(c.shadowing, c.shadowingTau, rng));
                // Received signal strength: drops with log-distance, plus the slow fading.
                double r = c.txPower - 10 * c.pathLossExponent * Math.log10(d) + sh.step(c.scanInterval);
                // Heard only above the phone's sensitivity, and even then not in every scan.
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
        // Speedometers read a few percent high (tyre wear and pressure); one fixed error per car.
        double scale = 1 + rng.inRange(c.scaleError);
        for (double t = c.latency; t <= truth.duration(); t += 1.0 / c.rate) {
            // The reading is a little old (latency) and rounded to 1 km/h.
            double v = truth.v(truth.indexAt(t - c.latency)) * scale;
            out.add(new ObdSpeed(t, Math.round(v / c.quantization) * c.quantization));
        }
        return out;
    }

    private static double sq(double x) {
        return x * x;
    }
}
