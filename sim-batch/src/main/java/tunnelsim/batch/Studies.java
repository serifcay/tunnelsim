package tunnelsim.batch;

import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.CategoryChart;
import org.knowm.xchart.CategoryChartBuilder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.XYSeries;
import org.knowm.xchart.style.Styler;
import org.knowm.xchart.style.lines.SeriesLines;
import org.knowm.xchart.style.markers.SeriesMarkers;
import tunnelsim.metrics.MonteCarlo;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.ScenarioConfig.SegmentConfig;
import tunnelsim.scenario.ScenarioConfig.TunnelConfig;
import tunnelsim.scenario.ScenarioLoader;

import java.awt.Color;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.Consumer;

/** Batch studies behind the write-up: CSV tables in results/, selected figures copied to docs/figures. */
final class Studies {
    private final int runs;
    private final Path out;
    private final Path docs = Path.of("docs", "figures");

    Studies(int runs, Path out) {
        this.runs = runs;
        this.out = out;
    }

    /** Same estimator colours as the app, so figures and screen look alike. */
    static Color color(String id) {
        return switch (id) {
            case "A" -> new Color(0xe0, 0x7a, 0x1f);
            case "B" -> new Color(0x8e, 0x5c, 0xc4);
            case "C" -> new Color(0x2f, 0x6f, 0xd1);
            case "D" -> new Color(0x2e, 0x9e, 0x4f);
            case "D+E" -> new Color(0x0f, 0x8b, 0x8d);
            default -> Color.GRAY;
        };
    }

    /** Monte Carlo per scenario: summary table and error-versus-time-in-tunnel curves. */
    void curves(List<Path> scenarios) {
        Path dir = dir("curves");
        try (PrintWriter csv = csv(dir.resolve("summary.csv"),
                "scenario,estimator,runs,rms_p50,rms_p95,max_p50,max_p95,exit_p50,exit_p95,callout_p50,callout_p95,late_fraction,branch_p50,branch_p95,within2sigma,nees,runtime_ms")) {
            for (Path file : scenarios) {
                ScenarioConfig cfg = ScenarioLoader.load(file);
                MonteCarlo.Result r = MonteCarlo.run(cfg, runs);
                System.out.print(MonteCarlo.format(r));
                for (MonteCarlo.Summary s : r.summaries().values()) {
                    csv.printf(Locale.ROOT, "\"%s\",%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.0f%n",
                            cfg.name, s.estimator(), s.runs(), s.rms()[0], s.rms()[1], s.max()[0], s.max()[1],
                            s.exit()[0], s.exit()[1], s.callout()[0], s.callout()[1], s.lateFraction(),
                            s.branch()[0], s.branch()[1], s.within2Sigma(), s.nees(), s.runtimeMs());
                }
                XYChart chart = new XYChartBuilder().width(900).height(520)
                        .title(cfg.name + ": along-road error vs time in tunnel (" + runs + " runs)")
                        .xAxisTitle("time since tunnel entry [s]").yAxisTitle("|error| [m], log scale").build();
                style(chart);
                chart.getStyler().setYAxisLogarithmic(true);
                for (MonteCarlo.Summary s : r.summaries().values()) {
                    List<Double> x = new ArrayList<>(), p50 = new ArrayList<>(), p95 = new ArrayList<>();
                    for (double[] b : s.errorCurve()) {
                        // Each bin is [time, median error, 95th percentile, number of runs still in the tunnel].
                        // Only plot times that at least half the runs reached, otherwise the tail is noise from a
                        // few long runs. Errors are clamped at 0.1 m so the log scale works.
                        if (b[3] >= runs / 2.0) {
                            x.add(b[0]);
                            p50.add(Math.max(0.1, b[1]));
                            p95.add(Math.max(0.1, b[2]));
                        }
                    }
                    if (x.size() < 2) {
                        continue;
                    }
                    // Solid line for the median, dashed line for the 95th percentile, in the estimator's colour.
                    XYSeries a = chart.addSeries(s.estimator() + " median", x, p50);
                    a.setLineColor(color(s.estimator()));
                    a.setMarker(SeriesMarkers.NONE);
                    XYSeries b = chart.addSeries(s.estimator() + " 95%", x, p95);
                    b.setLineColor(color(s.estimator()));
                    b.setLineStyle(SeriesLines.DASH_DASH);
                    b.setMarker(SeriesMarkers.NONE);
                }
                save(chart, dir, "error_" + slug(file));
            }
        }
    }

    /** Contribution of each aiding source to the particle filter. */
    void ablation(List<Path> scenarios) {
        Path dir = dir("ablation");
        // Each variant switches off one aiding source (or turns on an optional one) and is compared with "all".
        Map<String, Consumer<ScenarioConfig>> variants = new LinkedHashMap<>();
        variants.put("all", c -> { });
        variants.put("no curvature", c -> c.estimator.aiding.curvature = false);
        variants.put("no baro", c -> c.estimator.aiding.baro = false);
        variants.put("no vertical speed", c -> c.estimator.aiding.baroVerticalSpeed = false);
        variants.put("no stops", c -> c.estimator.aiding.zupt = false);
        variants.put("no speed prior", c -> c.estimator.aiding.speedPrior = false);
        variants.put("no portal", c -> c.estimator.aiding.portal = false);
        variants.put("no cell", c -> c.estimator.aiding.cell = false);
        variants.put("no accel scale", c -> c.estimator.accelScaleCalibration = false);
        variants.put("slope absorbed at stops", c -> c.estimator.slopeAwareRealignment = false);
        variants.put("+ lateral acc", c -> c.estimator.aiding.lateralAccel = true);
        CategoryChart chart = new CategoryChartBuilder().width(1000).height(520)
                .title("Particle filter ablation: median in-tunnel RMS error (" + runs + " runs)")
                .xAxisTitle("").yAxisTitle("RMS error [m]").build();
        style(chart);
        try (PrintWriter csv = csv(dir.resolve("ablation.csv"), "scenario,variant,rms_p50,rms_p95,max_p50,max_p95,exit_p50,within2sigma")) {
            for (Path file : scenarios) {
                List<String> names = new ArrayList<>();
                List<Number> values = new ArrayList<>();
                ScenarioConfig base = ScenarioLoader.load(file);
                for (Map.Entry<String, Consumer<ScenarioConfig>> v : variants.entrySet()) {
                    // Fresh copy per variant so changes don't leak into the next one; only the particle filter is
                    // run.
                    ScenarioConfig cfg = ScenarioLoader.copy(base);
                    cfg.estimators = new ArrayList<>(List.of("D"));
                    v.getValue().accept(cfg);
                    MonteCarlo.Summary s = MonteCarlo.run(cfg, runs).summaries().get("D");
                    csv.printf(Locale.ROOT, "\"%s\",%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f%n", base.name, v.getKey(),
                            s.rms()[0], s.rms()[1], s.max()[0], s.max()[1], s.exit()[0], s.within2Sigma());
                    System.out.printf(Locale.ROOT, "%-26s %-16s rms %6.1f / %6.1f  max %6.1f / %6.1f  in2sig %.2f%n",
                            base.name, v.getKey(), s.rms()[0], s.rms()[1], s.max()[0], s.max()[1], s.within2Sigma());
                    names.add(v.getKey());
                    values.add(s.rms()[0]);
                }
                chart.addSeries(base.name, names, values);
            }
        }
        save(chart, dir, "ablation");
    }

    /** Error at the tunnel exit versus tunnel length, on a featureless road and a curvy, hilly one. */
    void length() {
        Path dir = dir("length");
        XYChart chart = new XYChartBuilder().width(900).height(520)
                .title("Largest in-tunnel error vs tunnel length (median of " + runs + " runs)")
                .xAxisTitle("tunnel length [m]").yAxisTitle("max |error| in tunnel [m], log scale").build();
        style(chart);
        chart.getStyler().setYAxisLogarithmic(true);
        try (PrintWriter csv = csv(dir.resolve("length.csv"), "road,estimator,length,exit_p50,exit_p95,max_p50,max_p95")) {
            // Sweep tunnel length from 500 m to 5 km in 500 m steps, on a road with no features and on one full of
            // curves and slopes.
            for (String road : List.of("straight-flat", "curvy-hilly")) {
                Map<String, List<Double>> x = new LinkedHashMap<>(), y = new LinkedHashMap<>();
                for (double len = 500; len <= 5000; len += 500) {
                    ScenarioConfig cfg = generated(road, len);
                    MonteCarlo.Result r = MonteCarlo.run(cfg, runs);
                    for (MonteCarlo.Summary s : r.summaries().values()) {
                        csv.printf(Locale.ROOT, "%s,%s,%.0f,%.2f,%.2f,%.2f,%.2f%n", road, s.estimator(), len,
                                s.exit()[0], s.exit()[1], s.max()[0], s.max()[1]);
                        x.computeIfAbsent(s.estimator(), k -> new ArrayList<>()).add(len);
                        // Max in-tunnel error, not error at exit: the light-sensor portal fix resets C and D just before the exit.
                        y.computeIfAbsent(s.estimator(), k -> new ArrayList<>()).add(Math.max(0.1, s.max()[0]));
                    }
                    System.out.printf(Locale.ROOT, "%-14s %5.0f m  %s%n", road, len, r.summaries().values().stream()
                            .map(s -> String.format(Locale.ROOT, "%s max %.1f", s.estimator(), s.max()[0])).toList());
                }
                for (String est : x.keySet()) {
                    XYSeries s = chart.addSeries(est + " " + road, x.get(est), y.get(est));
                    s.setLineColor(color(est));
                    s.setMarkerColor(color(est));
                    // Dashed lines for the featureless road, solid for the curvy, hilly one.
                    if (road.equals("straight-flat")) {
                        s.setLineStyle(SeriesLines.DASH_DASH);
                    }
                }
            }
        }
        save(chart, dir, "max_error_vs_length");
    }

    /** What each phone profile loses. */
    void devices(List<Path> scenarios) {
        Path dir = dir("devices");
        try (PrintWriter csv = csv(dir.resolve("devices.csv"), "scenario,device,rms_p50,rms_p95,max_p95,exit_p50,callout_p50,within2sigma")) {
            for (Path file : scenarios) {
                // Same scenario on each phone profile (iOS apps get no light sensor or cell info, the budget phone
                // has no barometer and a noisier IMU).
                for (String device : List.of("GENERIC", "IOS", "LOW_END")) {
                    ScenarioConfig cfg = ScenarioLoader.load(file);
                    cfg.device = device;
                    cfg.estimators = new ArrayList<>(List.of("D"));
                    MonteCarlo.Summary s = MonteCarlo.run(cfg, runs).summaries().get("D");
                    csv.printf(Locale.ROOT, "\"%s\",%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f%n", cfg.name, device, s.rms()[0],
                            s.rms()[1], s.max()[1], s.exit()[0], s.callout()[0], s.within2Sigma());
                    System.out.printf(Locale.ROOT, "%-26s %-8s rms %6.1f / %6.1f  exit %6.1f  in2sig %.2f%n", cfg.name,
                            device, s.rms()[0], s.rms()[1], s.exit()[0], s.within2Sigma());
                }
            }
        }
    }

    /** Accuracy and cost against the number of particles. */
    void particles(Path scenario) {
        Path dir = dir("particles");
        try (PrintWriter csv = csv(dir.resolve("particles.csv"), "particles,rms_p50,rms_p95,max_p95,within2sigma,runtime_ms")) {
            // More particles cost more CPU; this shows where accuracy stops improving.
            for (int n : List.of(100, 300, 1000, 3000)) {
                ScenarioConfig cfg = ScenarioLoader.load(scenario);
                cfg.estimators = new ArrayList<>(List.of("D"));
                cfg.estimator.particles = n;
                MonteCarlo.Summary s = MonteCarlo.run(cfg, runs).summaries().get("D");
                csv.printf(Locale.ROOT, "%d,%.2f,%.2f,%.2f,%.3f,%.0f%n", n, s.rms()[0], s.rms()[1], s.max()[1],
                        s.within2Sigma(), s.runtimeMs());
                System.out.printf(Locale.ROOT, "N=%5d rms %6.1f / %6.1f  in2sig %.2f  %6.0f ms per run%n", n, s.rms()[0],
                        s.rms()[1], s.within2Sigma(), s.runtimeMs());
            }
        }
    }

    /** A generated road: 1.2 km approach, a tunnel of the given length, 800 m after. */
    static ScenarioConfig generated(String type, double length) {
        ScenarioConfig c = new ScenarioConfig();
        c.name = type + " " + (int) length + " m";
        c.seed = 100 + (long) length;
        c.estimators = new ArrayList<>(List.of("A", "C", "D"));
        c.road.main.add(segment("straight", 1200, null, 0, 0));
        // Repeat a 600 m block (ease into a bend, hold it, ease out, then a straight), alternating left/right
        // bends and going down for two blocks, then up for two, at 3% grade.
        if (type.equals("curvy-hilly")) {
            double done = 0;
            int k = 0;
            while (done < length) {
                double sign = k % 2 == 0 ? 1 : -1;
                double grade = (k % 4 < 2 ? -0.03 : 0.03);
                c.road.main.add(segment("clothoid", 100, null, sign * 0.0025, grade));
                c.road.main.add(segment("arc", 250, sign * 0.0025, 0, grade));
                c.road.main.add(segment("clothoid", 100, null, 0, grade));
                c.road.main.add(segment("straight", 150, null, 0, grade));
                done += 600;
                k++;
            }
        } else {
            c.road.main.add(segment("straight", length, null, 0, 0));
        }
        c.road.main.add(segment("straight", 1000, null, 0, 0));
        TunnelConfig t = new TunnelConfig();
        t.from = 1200;
        t.to = 1200 + length;
        c.road.tunnels.add(t);
        // Enough time to drive the whole road at about 15 m/s, plus a minute of slack.
        c.duration = (2200 + length) / 15 + 60;
        return ScenarioLoader.validate(c);
    }

    /** One road segment; curvature is in 1/m (positive turns left), grade is rise over run. */
    private static SegmentConfig segment(String type, double length, Double curvature, double curvatureEnd, double grade) {
        SegmentConfig s = new SegmentConfig();
        s.type = type;
        s.length = length;
        s.curvature = curvature;
        s.curvatureEnd = curvatureEnd;
        s.grade = grade;
        return s;
    }

    /** Shared look for line charts. */
    private static void style(XYChart chart) {
        chart.getStyler().setLegendPosition(Styler.LegendPosition.InsideNW);
        chart.getStyler().setChartBackgroundColor(Color.WHITE);
        chart.getStyler().setPlotGridLinesColor(new Color(0xe4, 0xe4, 0xe0));
        chart.getStyler().setMarkerSize(5);
    }

    /** Shared look for bar charts; labels are tilted so long variant names fit. */
    private static void style(CategoryChart chart) {
        chart.getStyler().setLegendPosition(Styler.LegendPosition.InsideNE);
        chart.getStyler().setChartBackgroundColor(Color.WHITE);
        chart.getStyler().setXAxisLabelRotation(20);
    }

    /** Creates results/<study> (and docs/figures) and returns the study folder. */
    private Path dir(String study) {
        Path d = out.resolve(study);
        try {
            Files.createDirectories(d);
            Files.createDirectories(docs);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return d;
    }

    /** Saves the chart as PNG in the study folder and copies it into docs/figures for the write-up. */
    private void save(Object chart, Path dir, String name) {
        try {
            Path file = dir.resolve(name);
            if (chart instanceof XYChart xy) {
                BitmapEncoder.saveBitmap(xy, file.toString(), BitmapEncoder.BitmapFormat.PNG);
            } else if (chart instanceof CategoryChart cc) {
                BitmapEncoder.saveBitmap(cc, file.toString(), BitmapEncoder.BitmapFormat.PNG);
            }
            Files.copy(dir.resolve(name + ".png"), docs.resolve(name + ".png"), StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /** Opens a CSV file and writes its header row. */
    private static PrintWriter csv(Path file, String header) {
        try {
            PrintWriter w = new PrintWriter(Files.newBufferedWriter(file));
            w.println(header);
            return w;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /** File name without the .yaml extension, used to name figures. */
    private static String slug(Path file) {
        return file.getFileName().toString().replace(".yaml", "");
    }
}
