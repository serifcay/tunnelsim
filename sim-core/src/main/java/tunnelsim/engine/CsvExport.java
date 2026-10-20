package tunnelsim.engine;

import tunnelsim.sensor.SensorEvent;
import tunnelsim.vehicle.TruthTrajectory;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;

/**
 * Writes a run to CSV: ground truth, every sensor stream the phone recorded, every estimator's
 * output and the event log, one folder per car. The sensor files use the same layout a real phone
 * recording would be converted to.
 */
public final class CsvExport {
    private CsvExport() {
    }

    /** Writes the run to baseDir/<scenario>-seed<n>/<car>/*.csv and returns the scenario folder. */
    public static Path write(RunResult run, Path baseDir) {
        // Folder name from the scenario name (unsafe characters replaced) and the seed.
        String name = run.scenario().name.replaceAll("[^A-Za-z0-9_-]+", "_") + "-seed" + run.scenario().seed;
        Path dir = baseDir.resolve(name);
        for (CarResult car : run.cars()) {
            Path d = dir.resolve(car.car.id);
            try {
                Files.createDirectories(d);
                writeTruth(car.truth, d.resolve("truth.csv"));
                writeSensors(car, d);
                for (Track track : car.tracks.values()) {
                    writeTrack(track, d.resolve("estimate_" + track.estimatorId.replace("+", "plus") + ".csv"));
                }
                try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(d.resolve("events.csv")))) {
                    w.println("t,estimator,type,text");
                    for (RunEvent e : car.events) {
                        w.printf(Locale.ROOT, "%.2f,%s,%s,\"%s\"%n", e.t(), e.estimator() == null ? "" : e.estimator(),
                                e.type(), e.text().replace("\"", "'"));
                    }
                }
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }
        return dir;
    }

    /** True trajectory, thinned to 10 Hz (every 10th 100 Hz sample). */
    private static void writeTruth(TruthTrajectory t, Path file) throws IOException {
        try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(file))) {
            w.println("t,edge,s,path_offset,x,y,z,heading,pitch,roll,v,a,curvature,grade,tunnel");
            for (int i = 0; i < t.size(); i += 10) {
                w.printf(Locale.ROOT, "%.2f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.3f,%.3f,%.6f,%.4f,%d%n",
                        t.t(i), t.edge(i), t.s(i), t.offset(i), t.x(i), t.y(i), t.z(i), t.heading(i), t.pitch(i),
                        t.roll(i), t.v(i), t.a(i), t.curvature(i), t.grade(i), t.tunnel(i));
            }
        }
    }

    /** One estimator's 10 Hz output, with its error against the truth. */
    private static void writeTrack(Track track, Path file) throws IOException {
        try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(file))) {
            w.println("t,valid,mode,edge,s,path_offset,sigma,speed,sigma_speed,p_off_route,error,wrong_branch,display_error");
            for (Track.Sample s : track.samples) {
                var e = s.estimate();
                w.printf(Locale.ROOT, "%.2f,%b,%s,%d,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%b,%.2f%n", s.t(), e.valid(),
                        e.mode(), e.edge(), e.s(), e.offset(), e.sigma(), e.speed(), e.sigmaSpeed(), e.pOffRoute(),
                        s.error(), s.wrongBranch(), s.displayError());
            }
        }
    }

    /** One CSV per sensor type, in the layout a real phone recording would be converted to. */
    private static void writeSensors(CarResult car, Path d) throws IOException {
        try (PrintWriter imu = writer(d, "imu.csv", "t,ax,ay,az,gx,gy,gz");
             PrintWriter gnss = writer(d, "gnss.csv", "t,x,y,accuracy,speed,speed_accuracy,bearing");
             PrintWriter status = writer(d, "gnss_status.csv", "t,satellites,mean_cn0");
             PrintWriter baro = writer(d, "baro.csv", "t,pressure_pa");
             PrintWriter light = writer(d, "light.csv", "t,lux");
             PrintWriter mag = writer(d, "mag.csv", "t,mx,my,mz");
             PrintWriter cell = writer(d, "cell.csv", "t,cell_id");
             PrintWriter ble = writer(d, "ble.csv", "t,beacon_id,rssi");
             PrintWriter obd = writer(d, "obd.csv", "t,speed")) {
            for (SensorEvent e : car.log.events()) {
                switch (e) {
                    case SensorEvent.ImuSample s -> imu.printf(Locale.ROOT, "%.3f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f%n",
                            s.t(), s.ax(), s.ay(), s.az(), s.gx(), s.gy(), s.gz());
                    case SensorEvent.GnssFix f -> gnss.printf(Locale.ROOT, "%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.4f%n",
                            f.t(), f.x(), f.y(), f.accuracy(), f.speed(), f.speedAccuracy(), f.bearing());
                    case SensorEvent.GnssStatus g -> status.printf(Locale.ROOT, "%.3f,%d,%.1f%n", g.t(), g.satellites(), g.meanCn0());
                    case SensorEvent.BaroSample b -> baro.printf(Locale.ROOT, "%.3f,%.2f%n", b.t(), b.pressure());
                    case SensorEvent.LightSample l -> light.printf(Locale.ROOT, "%.3f,%.1f%n", l.t(), l.lux());
                    case SensorEvent.MagSample m -> mag.printf(Locale.ROOT, "%.3f,%.2f,%.2f,%.2f%n", m.t(), m.mx(), m.my(), m.mz());
                    case SensorEvent.CellChange c -> cell.printf(Locale.ROOT, "%.3f,%d%n", c.t(), c.cellId());
                    case SensorEvent.BleScan s -> {
                        for (int k = 0; k < s.beaconIds().length; k++) {
                            ble.printf(Locale.ROOT, "%.3f,%d,%.1f%n", s.t(), s.beaconIds()[k], s.rssi()[k]);
                        }
                    }
                    case SensorEvent.ObdSpeed o -> obd.printf(Locale.ROOT, "%.3f,%.3f%n", o.t(), o.speed());
                }
            }
        }
    }

    private static PrintWriter writer(Path dir, String name, String header) throws IOException {
        PrintWriter w = new PrintWriter(Files.newBufferedWriter(dir.resolve(name)));
        w.println(header);
        return w;
    }
}
