package tunnelsim.sensor;

import tunnelsim.map.RoadNetwork;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.SensorsConfig;
import tunnelsim.util.Rng;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.List;

/** Builds the full sensor log of one phone from a car's ground truth. */
public final class SensorSuite {
    private SensorSuite() {
    }

    public static SensorLog record(TruthTrajectory truth, RoadNetwork net, ScenarioConfig cfg, long seed) {
        SensorsConfig s = cfg.sensors;
        DeviceProfile device = DeviceProfile.valueOf(cfg.device);
        List<List<? extends SensorEvent>> streams = new ArrayList<>();
        streams.add(new ImuModel(s, device.imuQuality, Rng.derive(seed, "imu")).generate(truth));
        streams.add(new GnssModel(s.gnss, net, device.gnssRaw, Rng.derive(seed, "gnss")).generate(truth));
        if (device.baro && s.baro.enabled) {
            streams.add(new BaroModel(s.baro, Rng.derive(seed, "baro")).generate(truth));
        }
        if (device.light) {
            streams.add(new LightModel(s.light, net, Rng.derive(seed, "light")).generate(truth));
        }
        streams.add(new MagModel(s.mag, Rng.derive(seed, "mag")).generate(truth));
        if (device.cell) {
            streams.add(RadioModels.cell(truth, net, s.cell, Rng.derive(seed, "cell")));
        }
        if (cfg.options.ble) {
            streams.add(RadioModels.ble(truth, net, s.ble, Rng.derive(seed, "ble")));
        }
        if (cfg.options.obd) {
            streams.add(RadioModels.obd(truth, s.obd, Rng.derive(seed, "obd")));
        }
        if (s.timestampJitter > 0) {
            streams = jitter(streams, s.timestampJitter, Rng.derive(seed, "jitter"));
        }
        return new SensorLog(streams);
    }

    private static List<List<? extends SensorEvent>> jitter(List<List<? extends SensorEvent>> streams, double sigma, Rng rng) {
        List<List<? extends SensorEvent>> out = new ArrayList<>();
        for (List<? extends SensorEvent> stream : streams) {
            List<SensorEvent> j = new ArrayList<>();
            for (SensorEvent e : stream) {
                j.add(shift(e, rng.gaussian(sigma)));
            }
            out.add(j);
        }
        return out;
    }

    private static SensorEvent shift(SensorEvent e, double dt) {
        return switch (e) {
            case SensorEvent.ImuSample s -> new SensorEvent.ImuSample(s.t() + dt, s.ax(), s.ay(), s.az(), s.gx(), s.gy(), s.gz());
            case SensorEvent.GnssFix f -> new SensorEvent.GnssFix(f.t() + dt, f.x(), f.y(), f.accuracy(), f.speed(), f.speedAccuracy(), f.bearing());
            case SensorEvent.GnssStatus g -> new SensorEvent.GnssStatus(g.t() + dt, g.satellites(), g.meanCn0());
            case SensorEvent.BaroSample b -> new SensorEvent.BaroSample(b.t() + dt, b.pressure());
            case SensorEvent.LightSample l -> new SensorEvent.LightSample(l.t() + dt, l.lux());
            case SensorEvent.MagSample m -> new SensorEvent.MagSample(m.t() + dt, m.mx(), m.my(), m.mz());
            case SensorEvent.CellChange c -> new SensorEvent.CellChange(c.t() + dt, c.cellId());
            case SensorEvent.BleScan b -> new SensorEvent.BleScan(b.t() + dt, b.beaconIds(), b.rssi());
            case SensorEvent.ObdSpeed o -> new SensorEvent.ObdSpeed(o.t() + dt, o.speed());
        };
    }
}
