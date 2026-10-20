package tunnelsim.app;

import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.paint.Color;
import tunnelsim.engine.CarResult;
import tunnelsim.engine.Track;
import tunnelsim.sensor.SensorEvent;
import tunnelsim.sensor.SensorLog;
import tunnelsim.util.MathUtil;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.ToDoubleFunction;

/** Tabs with the estimation error and every sensor stream the phone recorded. */
final class ChartsPane extends TabPane {
    private final AppState state;
    private final List<TimeSeriesChart> charts = new ArrayList<>();
    private final TimeSeriesChart error = new TimeSeriesChart("Along-road error, band = claimed ±2σ", "m");
    private final TimeSeriesChart speed = new TimeSeriesChart("Speed", "m/s");
    private final TimeSeriesChart accel = new TimeSeriesChart("Accelerometer (phone frame)", "m/s²");
    private final TimeSeriesChart gyro = new TimeSeriesChart("Gyroscope (phone frame)", "deg/s");
    private final TimeSeriesChart baro = new TimeSeriesChart("Barometric altitude vs map", "m");
    private final TimeSeriesChart gnss = new TimeSeriesChart("GNSS", "");
    private final TimeSeriesChart light = new TimeSeriesChart("Ambient light", "lux, log");
    private final TimeSeriesChart mag = new TimeSeriesChart("Magnetometer", "µT");
    private final TimeSeriesChart optionE = new TimeSeriesChart("Option E", "");
    private boolean full;
    private double half = 30;

    ChartsPane(AppState state, DoubleConsumer onSeek) {
        this.state = state;
        add("Error", error);
        add("Speed", speed);
        add("Accel", accel);
        add("Gyro", gyro);
        add("Baro", baro);
        add("GNSS", gnss);
        add("Light", light);
        add("Mag", mag);
        add("BLE/OBD", optionE);
        light.setLog(true);
        charts.forEach(c -> c.onSeek(onSeek));
        setTabClosingPolicy(TabClosingPolicy.UNAVAILABLE);
        getSelectionModel().selectedItemProperty().addListener(o -> refreshCursor());
    }

    private void add(String name, TimeSeriesChart chart) {
        charts.add(chart);
        getTabs().add(new Tab(name, chart));
    }

    /** Applies the chosen time window to every chart. */
    void setWindow(boolean fullRun, double halfWidth) {
        full = fullRun;
        half = halfWidth;
        charts.forEach(c -> c.setWindow(full, half));
    }

    /** Moves the playhead on the visible chart only (hidden tabs are redrawn when shown). */
    void refreshCursor() {
        Tab tab = getSelectionModel().getSelectedItem();
        if (tab != null && tab.getContent() instanceof TimeSeriesChart c) {
            c.setCursor(state.time);
        }
    }

    /** Rebuilds every chart's series from the selected car's truth, sensor log and estimator tracks. */
    void resultChanged() {
        CarResult car = state.car();
        if (car == null) {
            return;
        }
        TruthTrajectory truth = car.truth;
        SensorLog log = car.log;
        List<double[]> tunnels = tunnelIntervals(truth);
        double end = truth.duration();

        // Error with claimed +-2 sigma (consistent estimators keep the line inside their band).
        List<TimeSeriesChart.Series> err = new ArrayList<>();
        List<TimeSeriesChart.Series> spd = new ArrayList<>();
        int n = truth.size();
        // Truth series (speed, yaw rate, elevation) thinned from 100 Hz to 10 Hz for drawing.
        int stride = 10;
        double[] tt = new double[(n + stride - 1) / stride], tv = new double[tt.length];
        for (int i = 0, k = 0; i < n && k < tt.length; i += stride, k++) {
            tt[k] = truth.t(i);
            tv[k] = truth.v(i);
        }
        spd.add(TimeSeriesChart.Series.line("truth", Palette.TRUTH, tt, tv));
        for (String id : state.visible) {
            Track track = car.tracks.get(id);
            if (track == null) {
                continue;
            }
            int m = track.samples.size();
            double[] t = new double[m], e = new double[m], lo = new double[m], hi = new double[m], v = new double[m];
            for (int k = 0; k < m; k++) {
                Track.Sample s = track.samples.get(k);
                t[k] = s.t();
                boolean ok = s.estimate().valid() && !Double.isNaN(s.error());
                e[k] = ok ? s.error() : Double.NaN;
                lo[k] = ok ? -2 * s.estimate().sigma() : Double.NaN;
                hi[k] = ok ? 2 * s.estimate().sigma() : Double.NaN;
                v[k] = s.estimate().valid() ? s.estimate().speed() : Double.NaN;
            }
            Color c = Palette.estimator(id);
            err.add(new TimeSeriesChart.Series(id, c, t, e, lo, hi, false));
            spd.add(TimeSeriesChart.Series.line(id, c, t, v));
        }
        error.setSeries(err, tunnels, end);
        speed.setSeries(spd, tunnels, end);

        // Raw accelerometer and gyroscope, in phone axes; the gyro chart also shows the car's true yaw rate for
        // comparison.
        List<SensorEvent.ImuSample> imu = log.of(SensorEvent.ImuSample.class);
        double[] it = map(imu, SensorEvent::t);
        accel.setSeries(List.of(
                TimeSeriesChart.Series.line("x", Color.web("#d1495b"), it, map(imu, SensorEvent.ImuSample::ax)),
                TimeSeriesChart.Series.line("y", Color.web("#2e9e4f"), it, map(imu, SensorEvent.ImuSample::ay)),
                TimeSeriesChart.Series.line("z", Color.web("#2f6fd1"), it, map(imu, SensorEvent.ImuSample::az))),
                tunnels, end);
        double[] trueYaw = new double[tt.length];
        for (int i = 0, k = 0; i < n && k < tt.length; i += stride, k++) {
            trueYaw[k] = Math.toDegrees(truth.v(i) * truth.curvature(i));
        }
        gyro.setSeries(List.of(
                TimeSeriesChart.Series.line("x", Color.web("#d1495b"), it, map(imu, s -> Math.toDegrees(s.gx()))),
                TimeSeriesChart.Series.line("y", Color.web("#2e9e4f"), it, map(imu, s -> Math.toDegrees(s.gy()))),
                TimeSeriesChart.Series.line("z", Color.web("#2f6fd1"), it, map(imu, s -> Math.toDegrees(s.gz()))),
                new TimeSeriesChart.Series("car yaw rate v·κ (truth)", Palette.TRUTH, tt, trueYaw, null, null, true)),
                tunnels, end);

        List<SensorEvent.BaroSample> bs = log.of(SensorEvent.BaroSample.class);
        if (!bs.isEmpty()) {
            // Barometer and true road elevation, both relative to their first value (the barometer's absolute
            // level is unknown).
            double h0 = MathUtil.pressureToAltitude(bs.get(0).pressure());
            double z0 = truth.z(0);
            double[] tz = new double[tt.length];
            for (int i = 0, k = 0; i < n && k < tt.length; i += stride, k++) {
                tz[k] = truth.z(i) - z0;
            }
            baro.setSeries(List.of(
                    TimeSeriesChart.Series.line("barometer (relative)", Color.web("#8e5cc4"), map(bs, SensorEvent::t),
                            map(bs, b -> MathUtil.pressureToAltitude(b.pressure()) - h0)),
                    new TimeSeriesChart.Series("road elevation (truth)", Palette.TRUTH, tt, tz, null, null, true)),
                    tunnels, end);
        } else {
            baro.setSeries(List.of(), tunnels, end);
        }

        // GNSS: reported accuracy (with breaks during outages), and on Android signal strength and satellite
        // count.
        List<SensorEvent.GnssFix> fixes = log.of(SensorEvent.GnssFix.class);
        List<SensorEvent.GnssStatus> status = log.of(SensorEvent.GnssStatus.class);
        List<TimeSeriesChart.Series> g = new ArrayList<>();
        g.add(TimeSeriesChart.Series.line("fix accuracy [m]", Palette.GNSS, gapped(map(fixes, SensorEvent::t)),
                gappedValues(map(fixes, SensorEvent::t), map(fixes, SensorEvent.GnssFix::accuracy))));
        if (!status.isEmpty()) {
            g.add(TimeSeriesChart.Series.line("C/N0 [dB-Hz]", Color.web("#2f6fd1"), map(status, SensorEvent::t),
                    map(status, SensorEvent.GnssStatus::meanCn0)));
            g.add(TimeSeriesChart.Series.line("satellites", Color.web("#2e9e4f"), map(status, SensorEvent::t),
                    map(status, s -> s.satellites())));
        }
        gnss.setSeries(g, tunnels, end);

        List<SensorEvent.LightSample> ls = log.of(SensorEvent.LightSample.class);
        light.setSeries(ls.isEmpty() ? List.of() : List.of(TimeSeriesChart.Series.line("lux", Color.web("#e0a400"),
                map(ls, SensorEvent::t), map(ls, SensorEvent.LightSample::lux))), tunnels, end);

        List<SensorEvent.MagSample> ms = log.of(SensorEvent.MagSample.class);
        double[] mt = map(ms, SensorEvent::t);
        mag.setSeries(List.of(
                TimeSeriesChart.Series.line("|B|", Palette.TRUTH, mt, map(ms, s -> Math.sqrt(s.mx() * s.mx() + s.my() * s.my() + s.mz() * s.mz()))),
                TimeSeriesChart.Series.line("x", Color.web("#d1495b"), mt, map(ms, SensorEvent.MagSample::mx)),
                TimeSeriesChart.Series.line("y", Color.web("#2e9e4f"), mt, map(ms, SensorEvent.MagSample::my)),
                TimeSeriesChart.Series.line("z", Color.web("#2f6fd1"), mt, map(ms, SensorEvent.MagSample::mz))),
                tunnels, end);

        List<TimeSeriesChart.Series> e = new ArrayList<>();
        List<SensorEvent.ObdSpeed> obd = log.of(SensorEvent.ObdSpeed.class);
        if (!obd.isEmpty()) {
            e.add(TimeSeriesChart.Series.line("OBD speed [m/s]", Color.web("#0f8b8d"), map(obd, SensorEvent::t),
                    map(obd, SensorEvent.ObdSpeed::speed)));
            e.add(TimeSeriesChart.Series.line("true speed", Palette.TRUTH, tt, tv));
        }
        List<SensorEvent.BleScan> ble = log.of(SensorEvent.BleScan.class);
        if (!ble.isEmpty()) {
            e.add(TimeSeriesChart.Series.line("strongest beacon [dBm]", Palette.BEACON, map(ble, SensorEvent::t),
                    map(ble, s -> java.util.Arrays.stream(s.rssi()).max().orElse(Double.NaN))));
            e.add(TimeSeriesChart.Series.line("beacons heard", Color.web("#8e5cc4"), map(ble, SensorEvent::t),
                    map(ble, s -> s.beaconIds().length)));
        }
        optionE.setSeries(e, tunnels, end);
        charts.forEach(c -> c.setWindow(full, half));
        refreshCursor();
    }

    /** Time intervals the car spent inside tunnels, for the grey shading. */
    private static List<double[]> tunnelIntervals(TruthTrajectory truth) {
        List<double[]> out = new ArrayList<>();
        double start = -1;
        for (int i = 0; i < truth.size(); i++) {
            boolean in = truth.tunnel(i) >= 0;
            if (in && start < 0) {
                start = truth.t(i);
            } else if (!in && start >= 0) {
                out.add(new double[]{start, truth.t(i)});
                start = -1;
            }
        }
        if (start >= 0) {
            out.add(new double[]{start, truth.duration()});
        }
        return out;
    }

    /** Extracts one number per event into an array (e.g. all timestamps). */
    private static <T> double[] map(List<T> list, ToDoubleFunction<T> f) {
        double[] out = new double[list.size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = f.applyAsDouble(list.get(i));
        }
        return out;
    }

    /** Inserts a NaN point into gaps longer than 3 s, so outages show as breaks. */
    private static double[] gapped(double[] t) {
        List<Double> out = new ArrayList<>();
        for (int i = 0; i < t.length; i++) {
            if (i > 0 && t[i] - t[i - 1] > 3) {
                out.add(t[i - 1] + 0.5);
            }
            out.add(t[i]);
        }
        return out.stream().mapToDouble(Double::doubleValue).toArray();
    }

    private static double[] gappedValues(double[] t, double[] v) {
        List<Double> out = new ArrayList<>();
        for (int i = 0; i < t.length; i++) {
            if (i > 0 && t[i] - t[i - 1] > 3) {
                out.add(Double.NaN);
            }
            out.add(v[i]);
        }
        return out.stream().mapToDouble(Double::doubleValue).toArray();
    }
}
