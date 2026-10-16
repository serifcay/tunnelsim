package tunnelsim.metrics;

import tunnelsim.engine.CarResult;
import tunnelsim.engine.RunResult;
import tunnelsim.engine.SimulationRun;
import tunnelsim.engine.Track;
import tunnelsim.nav.CalloutTracker.Callout;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.ScenarioLoader;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.stream.IntStream;

/**
 * Runs a scenario with many seeds (car behaviour, sensor noise, phone mount and estimator randomness
 * all change) and aggregates the metrics per estimator. Runs execute in parallel; each run is
 * deterministic, so the aggregate is reproducible.
 */
public final class MonteCarlo {
    /** Width of the time-in-tunnel bins for error curves [s]. */
    public static final double BIN = 5;

    private MonteCarlo() {
    }

    public record Summary(String estimator, int runs, double[] rms, double[] max, double[] exit, double[] callout,
                          double lateFraction, double[] branch, double[] recovery, double within2Sigma,
                          double nees, double runtimeMs, List<double[]> errorByTime) {

        /** Error percentiles (50, 95) per time-in-tunnel bin: {binStart, p50, p95, count}. */
        public List<double[]> errorCurve() {
            return errorByTime;
        }
    }

    public record Result(String scenario, int runs, Map<String, Summary> summaries) {
    }

    /** What one run contributes to the aggregate (full results are dropped to bound memory). */
    private record Digest(String estimator, TrackMetrics metrics, List<Callout> callouts, Map<Integer, Double> binMax) {
    }

    public static Result run(ScenarioConfig base, int runs) {
        List<List<Digest>> digests = IntStream.range(0, runs).parallel().mapToObj(k -> {
            ScenarioConfig cfg = ScenarioLoader.copy(base);
            cfg.seed = base.seed * 7919 + k;
            RunResult r = SimulationRun.run(cfg, true);
            List<Digest> d = new ArrayList<>();
            for (CarResult car : r.cars()) {
                for (Map.Entry<String, TrackMetrics> en : car.metrics.entrySet()) {
                    Track track = car.tracks.get(en.getKey());
                    d.add(new Digest(en.getKey(), en.getValue(), List.copyOf(track.callouts), binErrors(track, car.truth)));
                }
            }
            return d;
        }).toList();

        Map<String, Summary> out = new LinkedHashMap<>();
        for (String est : base.estimators) {
            List<Double> rms = new ArrayList<>(), max = new ArrayList<>(), exit = new ArrayList<>(),
                    callout = new ArrayList<>(), branch = new ArrayList<>(), recovery = new ArrayList<>();
            int late = 0, callouts = 0;
            double within = 0, nees = 0, runtime = 0;
            int consistencyRuns = 0;
            Map<Integer, List<Double>> byBin = new LinkedHashMap<>();
            for (List<Digest> run : digests) {
                for (Digest dg : run) {
                    if (!dg.estimator().equals(est)) {
                        continue;
                    }
                    TrackMetrics m = dg.metrics();
                    add(rms, m.rmsTunnel());
                    add(max, m.maxTunnel());
                    for (double e : m.exitError()) {
                        add(exit, e);
                    }
                    for (Callout c : dg.callouts()) {
                        callouts++;
                        if (!c.fired() || c.late()) {
                            late++;
                        }
                        if (c.fired()) {
                            add(callout, Math.abs(c.timingError()));
                        }
                    }
                    add(branch, m.branchDecision());
                    add(recovery, m.recovery());
                    if (!Double.isNaN(m.within2Sigma())) {
                        within += m.within2Sigma();
                        nees += m.nees();
                        consistencyRuns++;
                    }
                    runtime += m.runtimeMs();
                    dg.binMax().forEach((b, e) -> byBin.computeIfAbsent(b, kk -> new ArrayList<>()).add(e));
                }
            }
            List<double[]> curve = new ArrayList<>();
            byBin.entrySet().stream().sorted(Map.Entry.comparingByKey()).forEach(en -> {
                double[] v = en.getValue().stream().mapToDouble(Double::doubleValue).sorted().toArray();
                curve.add(new double[]{en.getKey() * BIN, pct(v, 0.5), pct(v, 0.95), v.length});
            });
            out.put(est, new Summary(est, runs, stats(rms), stats(max), stats(exit), stats(callout),
                    callouts == 0 ? Double.NaN : (double) late / callouts, stats(branch), stats(recovery),
                    consistencyRuns == 0 ? Double.NaN : within / consistencyRuns,
                    consistencyRuns == 0 ? Double.NaN : nees / consistencyRuns,
                    runtime / Math.max(1, runs), curve));
        }
        return new Result(base.name, runs, out);
    }

    /** Largest absolute error per time-in-tunnel bin of one run. */
    private static Map<Integer, Double> binErrors(Track track, TruthTrajectory truth) {
        double entry = Double.NaN;
        int prevTunnel = -1;
        Map<Integer, Double> runMax = new LinkedHashMap<>();
        for (Track.Sample s : track.samples) {
            int tun = truth.tunnel(truth.indexAt(s.t()));
            if (tun >= 0 && prevTunnel < 0) {
                entry = s.t();
            }
            prevTunnel = tun;
            if (tun < 0 || Double.isNaN(entry) || Double.isNaN(s.error())) {
                continue;
            }
            int bin = (int) ((s.t() - entry) / BIN);
            runMax.merge(bin, Math.abs(s.error()), Math::max);
        }
        return runMax;
    }

    private static void add(List<Double> list, double v) {
        if (!Double.isNaN(v)) {
            list.add(v);
        }
    }

    /** {p50, p95, max, count}. */
    private static double[] stats(List<Double> values) {
        double[] v = values.stream().mapToDouble(Double::doubleValue).sorted().toArray();
        if (v.length == 0) {
            return new double[]{Double.NaN, Double.NaN, Double.NaN, 0};
        }
        return new double[]{pct(v, 0.5), pct(v, 0.95), v[v.length - 1], v.length};
    }

    private static double pct(double[] sorted, double p) {
        if (sorted.length == 0) {
            return Double.NaN;
        }
        return sorted[(int) Math.min(sorted.length - 1, Math.floor(p * sorted.length))];
    }

    public static String format(Result r) {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format(Locale.ROOT, "%s  (%d runs)%n", r.scenario(), r.runs()));
        sb.append(String.format(Locale.ROOT, "%-4s %15s %15s %15s %15s %6s %13s %7s %6s %6s%n", "est",
                "rmsTun p50/p95", "maxTun p50/p95", "exitErr p50/p95", "callout[s] p50/p95", "late",
                "branch p50/p95", "in2sig", "NEES", "ms"));
        for (Summary s : r.summaries().values()) {
            sb.append(String.format(Locale.ROOT, "%-4s %15s %15s %15s %15s %6s %13s %7s %6s %6.0f%n", s.estimator(),
                    pair(s.rms()), pair(s.max()), pair(s.exit()), pair(s.callout()), f(s.lateFraction()),
                    pair(s.branch()), f(s.within2Sigma()), f(s.nees()), s.runtimeMs()));
        }
        return sb.toString();
    }

    private static String pair(double[] st) {
        return f(st[0]) + " / " + f(st[1]);
    }

    private static String f(double v) {
        if (Double.isNaN(v)) {
            return "-";
        }
        return Math.abs(v) >= 100 ? String.format(Locale.ROOT, "%.0f", v) : String.format(Locale.ROOT, "%.1f", v);
    }

    static double[] sorted(double[] v) {
        double[] c = v.clone();
        Arrays.sort(c);
        return c;
    }
}
